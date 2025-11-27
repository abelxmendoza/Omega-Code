"""
Advanced caching system for robot controller backend.
- Redis-based caching with fallback to memory
- TTL management and cache invalidation
- Cache warming and preloading
- Performance monitoring
"""

import time
import json
import pickle
import hashlib
from typing import Any, Optional, Dict, List, Callable
from functools import wraps
import logging

logger = logging.getLogger(__name__)

try:
    import redis
    REDIS_AVAILABLE = True
except ImportError:
    REDIS_AVAILABLE = False
    logger.warning("Redis not available, using memory cache")

class CacheManager:
    """Advanced cache manager with Redis and memory fallback"""
    
    def __init__(self, redis_url: str = "redis://localhost:6379", default_ttl: int = 300):
        self.default_ttl = default_ttl
        self.memory_cache = {}
        self.cache_stats = {
            "hits": 0,
            "misses": 0,
            "sets": 0,
            "deletes": 0
        }
        
        if REDIS_AVAILABLE:
            try:
                self.redis_client = redis.from_url(redis_url, decode_responses=True)
                self.redis_client.ping()  # Test connection
                self.use_redis = True
                logger.info("Redis cache initialized successfully")
            except Exception as e:
                logger.warning(f"Redis connection failed: {e}, using memory cache")
                self.use_redis = False
        else:
            self.use_redis = False
    
    def get(self, key: str) -> Optional[Any]:
        """Get value from cache"""
        try:
            if self.use_redis:
                value = self.redis_client.get(key)
                if value is not None:
                    self.cache_stats["hits"] += 1
                    return json.loads(value)
            else:
                if key in self.memory_cache:
                    cached_item = self.memory_cache[key]
                    if time.time() < cached_item["expires"]:
                        self.cache_stats["hits"] += 1
                        return cached_item["value"]
                    else:
                        del self.memory_cache[key]
            
            self.cache_stats["misses"] += 1
            return None
        except Exception as e:
            logger.error(f"Cache get error: {e}")
            return None
    
    def set(self, key: str, value: Any, ttl: Optional[int] = None) -> bool:
        """Set value in cache"""
        try:
            ttl = ttl or self.default_ttl
            
            if self.use_redis:
                self.redis_client.setex(key, ttl, json.dumps(value))
            else:
                self.memory_cache[key] = {
                    "value": value,
                    "expires": time.time() + ttl
                }
            
            self.cache_stats["sets"] += 1
            return True
        except Exception as e:
            logger.error(f"Cache set error: {e}")
            return False
    
    def delete(self, key: str) -> bool:
        """Delete value from cache"""
        try:
            if self.use_redis:
                self.redis_client.delete(key)
            else:
                self.memory_cache.pop(key, None)
            
            self.cache_stats["deletes"] += 1
            return True
        except Exception as e:
            logger.error(f"Cache delete error: {e}")
            return False
    
    def clear(self) -> bool:
        """Clear all cache"""
        try:
            if self.use_redis:
                self.redis_client.flushdb()
            else:
                self.memory_cache.clear()
            return True
        except Exception as e:
            logger.error(f"Cache clear error: {e}")
            return False
    
    def get_stats(self) -> Dict[str, Any]:
        """Get cache statistics"""
        hit_rate = 0
        total_requests = self.cache_stats["hits"] + self.cache_stats["misses"]
        if total_requests > 0:
            hit_rate = self.cache_stats["hits"] / total_requests
        
        return {
            **self.cache_stats,
            "hit_rate": hit_rate,
            "total_requests": total_requests,
            "use_redis": self.use_redis
        }

# Global cache instance
cache_manager = CacheManager()

def cached(ttl: int = 300, key_prefix: str = ""):
    """Decorator for caching function results"""
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            # Generate cache key
            key_data = {
                "func": func.__name__,
                "args": args,
                "kwargs": kwargs
            }
            key_hash = hashlib.md5(json.dumps(key_data, sort_keys=True).encode()).hexdigest()
            cache_key = f"{key_prefix}{func.__name__}:{key_hash}"
            
            # Try to get from cache
            cached_result = cache_manager.get(cache_key)
            if cached_result is not None:
                return cached_result
            
            # Execute function and cache result
            result = func(*args, **kwargs)
            cache_manager.set(cache_key, result, ttl)
            return result
        
        return wrapper
    return decorator

class CacheWarmer:
    """Cache warming utility"""
    
    def __init__(self, cache_manager: CacheManager):
        self.cache_manager = cache_manager
    
    async def warm_cache(self, warming_functions: List[Callable]):
        """Warm cache with predefined functions"""
        for func in warming_functions:
            try:
                if asyncio.iscoroutinefunction(func):
                    await func()
                else:
                    func()
            except Exception as e:
                logger.error(f"Cache warming error for {func.__name__}: {e}")
    
    def schedule_warming(self, warming_functions: List[Callable], interval: int = 300):
        """Schedule periodic cache warming"""
        import threading
        
        def warm_periodically():
            while True:
                time.sleep(interval)
                asyncio.run(self.warm_cache(warming_functions))
        
        thread = threading.Thread(target=warm_periodically, daemon=True)
        thread.start()

# Cache invalidation patterns
class CacheInvalidator:
    """Cache invalidation utility"""
    
    def __init__(self, cache_manager: CacheManager):
        self.cache_manager = cache_manager
    
    def invalidate_pattern(self, pattern: str):
        """Invalidate cache keys matching pattern"""
        if cache_manager.use_redis:
            keys = cache_manager.redis_client.keys(pattern)
            if keys:
                cache_manager.redis_client.delete(*keys)
        else:
            keys_to_delete = [key for key in cache_manager.memory_cache.keys() if pattern in key]
            for key in keys_to_delete:
                del cache_manager.memory_cache[key]
    
    def invalidate_by_tags(self, tags: List[str]):
        """Invalidate cache by tags"""
        for tag in tags:
            self.invalidate_pattern(f"tag:{tag}:*")
    
    def invalidate_function(self, func_name: str):
        """Invalidate all cached results for a function"""
        self.invalidate_pattern(f"{func_name}:*")

# Performance monitoring for cache
class CacheProfiler:
    """Cache performance profiler"""
    
    def __init__(self, cache_manager: CacheManager):
        self.cache_manager = cache_manager
        self.operation_times = []
    
    def profile_operation(self, operation: str, func: Callable):
        """Profile cache operation"""
        start_time = time.time()
        result = func()
        end_time = time.time()
        
        operation_time = end_time - start_time
        self.operation_times.append({
            "operation": operation,
            "time": operation_time,
            "timestamp": time.time()
        })
        
        return result
    
    def get_performance_report(self) -> Dict[str, Any]:
        """Get performance report"""
        if not self.operation_times:
            return {"message": "No operations profiled"}
        
        avg_time = sum(op["time"] for op in self.operation_times) / len(self.operation_times)
        max_time = max(op["time"] for op in self.operation_times)
        min_time = min(op["time"] for op in self.operation_times)
        
        return {
            "total_operations": len(self.operation_times),
            "average_time": avg_time,
            "max_time": max_time,
            "min_time": min_time,
            "cache_stats": self.cache_manager.get_stats()
        }
