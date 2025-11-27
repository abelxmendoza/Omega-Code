"""
Performance API endpoint for serving real-time metrics to the frontend.
- System metrics (CPU, memory, disk, network)
- Application metrics (response times, error rates)
- Cache statistics
- WebSocket connection health
"""

import asyncio
import json
import time
from typing import Dict, Any
from aiohttp import web, web_request
import logging

logger = logging.getLogger(__name__)

# Import optimization utilities
try:
    from utils.optimization.performance_monitor import performance_monitor, app_profiler
    from utils.optimization.cache_manager import cache_manager
    OPTIMIZATION_AVAILABLE = True
except ImportError:
    OPTIMIZATION_AVAILABLE = False
    logger.warning("Optimization utilities not available")

class PerformanceAPI:
    """Performance API handler"""
    
    def __init__(self):
        self.start_time = time.time()
        self.request_count = 0
        self.error_count = 0
    
    async def get_performance_metrics(self, request: web_request.Request) -> web.Response:
        """Get current performance metrics"""
        try:
            self.request_count += 1
            
            if not OPTIMIZATION_AVAILABLE:
                return web.json_response({
                    "error": "Performance monitoring not available",
                    "timestamp": int(time.time() * 1000)
                })
            
            # Get current metrics
            current_metrics = performance_monitor.get_current_metrics()
            performance_summary = performance_monitor.get_performance_summary()
            cache_stats = cache_manager.get_stats()
            app_profile = app_profiler.get_profile_report()
            
            # Calculate uptime
            uptime = time.time() - self.start_time
            
            # Calculate error rate
            error_rate = (self.error_count / self.request_count * 100) if self.request_count > 0 else 0
            
            response_data = {
                "timestamp": int(time.time() * 1000),
                "uptime": uptime,
                "system": {
                    "cpu_usage": current_metrics.cpu_usage if current_metrics else 0,
                    "memory_usage": current_metrics.memory_usage if current_metrics else 0,
                    "memory_percent": current_metrics.memory_percent if current_metrics else 0,
                    "disk_usage": current_metrics.disk_usage if current_metrics else 0,
                    "network_io": current_metrics.network_io if current_metrics else {"bytes_sent": 0, "bytes_recv": 0}
                },
                "application": {
                    "response_time": current_metrics.response_time if current_metrics else 0,
                    "error_rate": error_rate,
                    "throughput": current_metrics.throughput if current_metrics else 0,
                    "websocket_connections": current_metrics.websocket_connections if current_metrics else 0
                },
                "cache": {
                    "hits": cache_stats.get("hits", 0),
                    "misses": cache_stats.get("misses", 0),
                    "hit_rate": cache_stats.get("hit_rate", 0),
                    "total_requests": cache_stats.get("total_requests", 0),
                    "use_redis": cache_stats.get("use_redis", False)
                },
                "performance_summary": performance_summary,
                "app_profile": app_profile
            }
            
            return web.json_response(response_data)
            
        except Exception as e:
            self.error_count += 1
            logger.error(f"Error getting performance metrics: {e}")
            return web.json_response({
                "error": str(e),
                "timestamp": int(time.time() * 1000)
            }, status=500)
    
    async def get_cache_stats(self, request: web_request.Request) -> web.Response:
        """Get cache statistics"""
        try:
            if not OPTIMIZATION_AVAILABLE:
                return web.json_response({
                    "error": "Cache manager not available",
                    "timestamp": int(time.time() * 1000)
                })
            
            cache_stats = cache_manager.get_stats()
            return web.json_response({
                "cache_stats": cache_stats,
                "timestamp": int(time.time() * 1000)
            })
            
        except Exception as e:
            logger.error(f"Error getting cache stats: {e}")
            return web.json_response({
                "error": str(e),
                "timestamp": int(time.time() * 1000)
            }, status=500)
    
    async def clear_cache(self, request: web_request.Request) -> web.Response:
        """Clear cache (admin endpoint)"""
        try:
            if not OPTIMIZATION_AVAILABLE:
                return web.json_response({
                    "error": "Cache manager not available",
                    "timestamp": int(time.time() * 1000)
                })
            
            success = cache_manager.clear()
            return web.json_response({
                "success": success,
                "message": "Cache cleared successfully" if success else "Failed to clear cache",
                "timestamp": int(time.time() * 1000)
            })
            
        except Exception as e:
            logger.error(f"Error clearing cache: {e}")
            return web.json_response({
                "error": str(e),
                "timestamp": int(time.time() * 1000)
            }, status=500)
    
    async def get_system_info(self, request: web_request.Request) -> web.Response:
        """Get system information"""
        try:
            import platform
            import psutil
            
            system_info = {
                "platform": platform.platform(),
                "python_version": platform.python_version(),
                "cpu_count": psutil.cpu_count(),
                "memory_total": psutil.virtual_memory().total,
                "disk_total": psutil.disk_usage('/').total,
                "boot_time": psutil.boot_time(),
                "uptime": time.time() - self.start_time,
                "optimization_available": OPTIMIZATION_AVAILABLE
            }
            
            return web.json_response({
                "system_info": system_info,
                "timestamp": int(time.time() * 1000)
            })
            
        except Exception as e:
            logger.error(f"Error getting system info: {e}")
            return web.json_response({
                "error": str(e),
                "timestamp": int(time.time() * 1000)
            }, status=500)

# Create API instance
performance_api = PerformanceAPI()

# Create routes
def create_performance_routes(app: web.Application):
    """Create performance API routes"""
    app.router.add_get('/api/performance/metrics', performance_api.get_performance_metrics)
    app.router.add_get('/api/performance/cache', performance_api.get_cache_stats)
    app.router.add_post('/api/performance/cache/clear', performance_api.clear_cache)
    app.router.add_get('/api/performance/system', performance_api.get_system_info)

if __name__ == '__main__':
    # Test the API
    app = web.Application()
    create_performance_routes(app)
    
    async def test_endpoints():
        """Test the performance endpoints"""
        print("Testing Performance API endpoints...")
        
        # Test metrics endpoint
        request = web_request.Request('GET', '/api/performance/metrics')
        response = await performance_api.get_performance_metrics(request)
        print(f"Metrics endpoint status: {response.status}")
        
        # Test cache stats endpoint
        request = web_request.Request('GET', '/api/performance/cache')
        response = await performance_api.get_cache_stats(request)
        print(f"Cache stats endpoint status: {response.status}")
        
        # Test system info endpoint
        request = web_request.Request('GET', '/api/performance/system')
        response = await performance_api.get_system_info(request)
        print(f"System info endpoint status: {response.status}")
    
    asyncio.run(test_endpoints())
