"""
Async processing system for robot controller backend.
- Background task processing
- Task queue management
- Priority-based processing
- Resource management
"""

import asyncio
import time
import logging
from typing import Any, Callable, Dict, List, Optional, Union
from dataclasses import dataclass, field
from enum import Enum
from collections import deque
import threading

logger = logging.getLogger(__name__)

class TaskPriority(Enum):
    """Task priority levels"""
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4

@dataclass
class Task:
    """Task definition"""
    id: str
    func: Callable
    args: tuple = field(default_factory=tuple)
    kwargs: dict = field(default_factory=dict)
    priority: TaskPriority = TaskPriority.NORMAL
    created_at: float = field(default_factory=time.time)
    max_retries: int = 3
    retry_count: int = 0
    timeout: Optional[float] = None
    callback: Optional[Callable] = None
    error_callback: Optional[Callable] = None

class AsyncTaskProcessor:
    """Async task processor with priority queue"""
    
    def __init__(self, max_workers: int = 10, max_queue_size: int = 1000):
        self.max_workers = max_workers
        self.max_queue_size = max_queue_size
        self.task_queue = deque()
        self.workers = []
        self.running = False
        self.stats = {
            "tasks_processed": 0,
            "tasks_failed": 0,
            "tasks_retried": 0,
            "average_processing_time": 0
        }
        self.processing_times = deque(maxlen=1000)  # Keep last 1000 processing times
    
    async def start(self):
        """Start the task processor"""
        if self.running:
            return
        
        self.running = True
        logger.info(f"Starting async task processor with {self.max_workers} workers")
        
        # Start worker tasks
        for i in range(self.max_workers):
            worker = asyncio.create_task(self._worker(f"worker-{i}"))
            self.workers.append(worker)
    
    async def stop(self):
        """Stop the task processor"""
        if not self.running:
            return
        
        self.running = False
        logger.info("Stopping async task processor")
        
        # Cancel all workers
        for worker in self.workers:
            worker.cancel()
        
        # Wait for workers to finish
        await asyncio.gather(*self.workers, return_exceptions=True)
        self.workers.clear()
    
    async def submit_task(self, task: Task) -> bool:
        """Submit a task for processing"""
        if len(self.task_queue) >= self.max_queue_size:
            logger.warning("Task queue is full, rejecting task")
            return False
        
        # Insert task based on priority
        inserted = False
        for i, queued_task in enumerate(self.task_queue):
            if task.priority.value > queued_task.priority.value:
                self.task_queue.insert(i, task)
                inserted = True
                break
        
        if not inserted:
            self.task_queue.append(task)
        
        logger.debug(f"Task {task.id} submitted with priority {task.priority.name}")
        return True
    
    async def submit(
        self,
        func: Callable,
        *args,
        priority: TaskPriority = TaskPriority.NORMAL,
        task_id: Optional[str] = None,
        max_retries: int = 3,
        timeout: Optional[float] = None,
        callback: Optional[Callable] = None,
        error_callback: Optional[Callable] = None,
        **kwargs
    ) -> str:
        """Submit a function for async processing"""
        task_id = task_id or f"task-{int(time.time() * 1000)}"
        
        task = Task(
            id=task_id,
            func=func,
            args=args,
            kwargs=kwargs,
            priority=priority,
            max_retries=max_retries,
            timeout=timeout,
            callback=callback,
            error_callback=error_callback
        )
        
        success = await self.submit_task(task)
        if not success:
            raise RuntimeError("Failed to submit task")
        
        return task_id
    
    async def _worker(self, worker_name: str):
        """Worker coroutine"""
        logger.debug(f"Worker {worker_name} started")
        
        while self.running:
            try:
                # Get next task
                if not self.task_queue:
                    await asyncio.sleep(0.01)  # Small delay to prevent busy waiting
                    continue
                
                task = self.task_queue.popleft()
                await self._process_task(task, worker_name)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Worker {worker_name} error: {e}")
                await asyncio.sleep(0.1)  # Brief pause on error
        
        logger.debug(f"Worker {worker_name} stopped")
    
    async def _process_task(self, task: Task, worker_name: str):
        """Process a single task"""
        start_time = time.time()
        
        try:
            logger.debug(f"Worker {worker_name} processing task {task.id}")
            
            # Execute task with timeout if specified
            if task.timeout:
                result = await asyncio.wait_for(
                    self._execute_task(task),
                    timeout=task.timeout
                )
            else:
                result = await self._execute_task(task)
            
            # Update stats
            processing_time = time.time() - start_time
            self.processing_times.append(processing_time)
            self.stats["tasks_processed"] += 1
            
            # Call success callback
            if task.callback:
                try:
                    if asyncio.iscoroutinefunction(task.callback):
                        await task.callback(result)
                    else:
                        task.callback(result)
                except Exception as e:
                    logger.error(f"Callback error for task {task.id}: {e}")
            
            logger.debug(f"Task {task.id} completed in {processing_time:.3f}s")
            
        except asyncio.TimeoutError:
            logger.error(f"Task {task.id} timed out")
            await self._handle_task_error(task, TimeoutError("Task timed out"))
            
        except Exception as e:
            logger.error(f"Task {task.id} failed: {e}")
            await self._handle_task_error(task, e)
    
    async def _execute_task(self, task: Task):
        """Execute the task function"""
        if asyncio.iscoroutinefunction(task.func):
            return await task.func(*task.args, **task.kwargs)
        else:
            # Run sync function in thread pool
            loop = asyncio.get_event_loop()
            return await loop.run_in_executor(None, task.func, *task.args, **task.kwargs)
    
    async def _handle_task_error(self, task: Task, error: Exception):
        """Handle task error with retry logic"""
        self.stats["tasks_failed"] += 1
        
        # Retry if possible
        if task.retry_count < task.max_retries:
            task.retry_count += 1
            self.stats["tasks_retried"] += 1
            
            # Exponential backoff
            delay = min(2 ** task.retry_count, 60)  # Max 60 seconds
            await asyncio.sleep(delay)
            
            # Re-queue task with lower priority
            task.priority = TaskPriority.LOW
            await self.submit_task(task)
            
            logger.info(f"Task {task.id} retry {task.retry_count}/{task.max_retries}")
        else:
            # Call error callback
            if task.error_callback:
                try:
                    if asyncio.iscoroutinefunction(task.error_callback):
                        await task.error_callback(error)
                    else:
                        task.error_callback(error)
                except Exception as e:
                    logger.error(f"Error callback failed for task {task.id}: {e}")
    
    def get_stats(self) -> Dict[str, Any]:
        """Get processor statistics"""
        if self.processing_times:
            self.stats["average_processing_time"] = sum(self.processing_times) / len(self.processing_times)
        
        return {
            **self.stats,
            "queue_size": len(self.task_queue),
            "active_workers": len([w for w in self.workers if not w.done()]),
            "running": self.running
        }

# Background task manager
class BackgroundTaskManager:
    """Manager for background tasks"""
    
    def __init__(self):
        self.tasks = {}
        self.processor = AsyncTaskProcessor()
    
    async def start(self):
        """Start the background task manager"""
        await self.processor.start()
    
    async def stop(self):
        """Stop the background task manager"""
        await self.processor.stop()
    
    async def schedule_periodic(
        self,
        func: Callable,
        interval: float,
        task_name: str,
        *args,
        **kwargs
    ):
        """Schedule a periodic task"""
        async def periodic_task():
            while True:
                try:
                    await asyncio.sleep(interval)
                    await self.processor.submit(func, *args, **kwargs)
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Periodic task {task_name} error: {e}")
        
        task = asyncio.create_task(periodic_task())
        self.tasks[task_name] = task
        return task
    
    async def schedule_delayed(
        self,
        func: Callable,
        delay: float,
        task_name: str,
        *args,
        **kwargs
    ):
        """Schedule a delayed task"""
        async def delayed_task():
            await asyncio.sleep(delay)
            await self.processor.submit(func, *args, **kwargs)
        
        task = asyncio.create_task(delayed_task())
        self.tasks[task_name] = task
        return task
    
    def cancel_task(self, task_name: str):
        """Cancel a scheduled task"""
        if task_name in self.tasks:
            self.tasks[task_name].cancel()
            del self.tasks[task_name]

# Global instances
task_processor = AsyncTaskProcessor()
background_manager = BackgroundTaskManager()
