"""
Performance tests for the robot controller backend.
Tests system performance under various load conditions.
"""

import pytest
import asyncio
import websockets
import json
import time
import statistics
from conftest import WebSocketTestClient, PerformanceMonitor

@pytest.mark.performance
class TestWebSocketPerformance:
    """Test WebSocket performance metrics."""
    
    async def test_message_latency(self):
        """Test message latency."""
        client = WebSocketTestClient("ws://localhost:8081/")
        await client.connect()
        
        try:
            monitor = PerformanceMonitor()
            latencies = []
            
            # Measure latency for multiple messages
            for i in range(20):
                monitor.start_timer("ping")
                response = await client.send_command("status")
                latency = monitor.end_timer("ping")
                latencies.append(latency)
            
            # Calculate statistics
            avg_latency = statistics.mean(latencies)
            max_latency = max(latencies)
            min_latency = min(latencies)
            
            print(f"Latency - Avg: {avg_latency:.3f}s, Max: {max_latency:.3f}s, Min: {min_latency:.3f}s")
            
            # Assertions
            assert avg_latency < 0.1  # Average latency should be under 100ms
            assert max_latency < 0.5  # Max latency should be under 500ms
            
        finally:
            await client.disconnect()
    
    async def test_throughput(self):
        """Test message throughput."""
        client = WebSocketTestClient("ws://localhost:8081/")
        await client.connect()
        
        try:
            start_time = time.time()
            message_count = 0
            
            # Send messages as fast as possible for 5 seconds
            end_time = start_time + 5
            while time.time() < end_time:
                await client.send_command("status")
                message_count += 1
            
            duration = time.time() - start_time
            throughput = message_count / duration
            
            print(f"Throughput: {throughput:.2f} messages/second")
            
            # Should handle at least 10 messages per second
            assert throughput > 10
            
        finally:
            await client.disconnect()
    
    async def test_concurrent_throughput(self):
        """Test throughput with concurrent connections."""
        clients = []
        
        try:
            # Create multiple connections
            for i in range(5):
                client = WebSocketTestClient("ws://localhost:8081/")
                await client.connect()
                clients.append(client)
            
            start_time = time.time()
            total_messages = 0
            
            # Send messages from all clients concurrently
            async def send_messages(client, count):
                nonlocal total_messages
                for _ in range(count):
                    await client.send_command("status")
                    total_messages += 1
            
            # Create tasks for each client
            tasks = []
            for client in clients:
                task = asyncio.create_task(send_messages(client, 10))
                tasks.append(task)
            
            # Wait for all tasks to complete
            await asyncio.gather(*tasks)
            
            duration = time.time() - start_time
            throughput = total_messages / duration
            
            print(f"Concurrent throughput: {throughput:.2f} messages/second")
            
            # Should handle at least 50 messages per second with 5 clients
            assert throughput > 50
            
        finally:
            # Clean up all connections
            for client in clients:
                await client.disconnect()

@pytest.mark.performance
class TestMotorControlPerformance:
    """Test motor control performance."""
    
    async def test_motor_command_latency(self):
        """Test motor command execution latency."""
        client = WebSocketTestClient("ws://localhost:8081/")
        await client.connect()
        
        try:
            monitor = PerformanceMonitor()
            latencies = []
            
            # Test different motor commands
            commands = [
                ("forward", {"speed": 1000}),
                ("backward", {"speed": 1000}),
                ("left", {"speed": 1000, "ratio": 0.5}),
                ("right", {"speed": 1000, "ratio": 0.5}),
                ("stop", {})
            ]
            
            for command, data in commands:
                monitor.start_timer(f"motor_{command}")
                response = await client.send_command(command, data)
                latency = monitor.end_timer(f"motor_{command}")
                latencies.append(latency)
                
                # Verify command was successful
                response_data = json.loads(response)
                assert response_data["status"] == "ok"
            
            avg_latency = statistics.mean(latencies)
            print(f"Motor command average latency: {avg_latency:.3f}s")
            
            # Motor commands should be fast
            assert avg_latency < 0.05  # Under 50ms
            
        finally:
            await client.disconnect()
    
    async def test_telemetry_update_frequency(self):
        """Test telemetry update frequency."""
        client = WebSocketTestClient("ws://localhost:8081/")
        await client.connect()
        
        try:
            # Start movement to generate telemetry
            await client.send_command("forward", {"speed": 1500})
            
            # Collect telemetry updates
            telemetry_times = []
            start_time = time.time()
            
            for i in range(10):
                response = await client.send_command("status")
                telemetry_times.append(time.time())
                await asyncio.sleep(0.1)
            
            # Calculate update intervals
            intervals = []
            for i in range(1, len(telemetry_times)):
                interval = telemetry_times[i] - telemetry_times[i-1]
                intervals.append(interval)
            
            avg_interval = statistics.mean(intervals)
            print(f"Telemetry update interval: {avg_interval:.3f}s")
            
            # Should update frequently
            assert avg_interval < 0.2  # Under 200ms
            
        finally:
            await client.send_command("stop")
            await client.disconnect()

@pytest.mark.performance
class TestServoControlPerformance:
    """Test servo control performance."""
    
    async def test_servo_command_latency(self):
        """Test servo command execution latency."""
        client = WebSocketTestClient("ws://localhost:8081/")
        await client.connect()
        
        try:
            monitor = PerformanceMonitor()
            latencies = []
            
            # Test servo commands
            servo_commands = [
                {"horizontal": 0, "vertical": 0},
                {"horizontal": 90, "vertical": 90},
                {"horizontal": 180, "vertical": 180},
                {"horizontal": 45, "vertical": 135}
            ]
            
            for servo_data in servo_commands:
                monitor.start_timer("servo")
                response = await client.send_command("servo", servo_data)
                latency = monitor.end_timer("servo")
                latencies.append(latency)
                
                # Verify command was successful
                response_data = json.loads(response)
                assert response_data["status"] == "ok"
            
            avg_latency = statistics.mean(latencies)
            print(f"Servo command average latency: {avg_latency:.3f}s")
            
            # Servo commands should be fast
            assert avg_latency < 0.05  # Under 50ms
            
        finally:
            await client.disconnect()
    
    async def test_servo_precision(self):
        """Test servo positioning precision."""
        client = WebSocketTestClient("ws://localhost:8081/")
        await client.connect()
        
        try:
            # Test precise positioning
            target_angles = [0, 30, 60, 90, 120, 150, 180]
            
            for angle in target_angles:
                await client.send_command("servo", {"horizontal": angle, "vertical": angle})
                
                # Get status to verify position
                response = await client.send_command("status")
                response_data = json.loads(response)
                
                servo_data = response_data["servo"]
                horizontal = servo_data["horizontal"]
                vertical = servo_data["vertical"]
                
                # Check precision (allow small tolerance)
                assert abs(horizontal - angle) < 5
                assert abs(vertical - angle) < 5
            
        finally:
            await client.disconnect()

@pytest.mark.performance
class TestSystemLoadPerformance:
    """Test system performance under load."""
    
    async def test_high_frequency_commands(self):
        """Test system under high frequency commands."""
        client = WebSocketTestClient("ws://localhost:8081/")
        await client.connect()
        
        try:
            start_time = time.time()
            command_count = 0
            error_count = 0
            
            # Send commands at high frequency for 10 seconds
            end_time = start_time + 10
            
            while time.time() < end_time:
                try:
                    await client.send_command("status")
                    command_count += 1
                except Exception as e:
                    error_count += 1
                    print(f"Command error: {e}")
                
                await asyncio.sleep(0.01)  # 100 commands per second
            
            duration = time.time() - start_time
            success_rate = (command_count / (command_count + error_count)) * 100
            
            print(f"High frequency test - Commands: {command_count}, Errors: {error_count}, Success rate: {success_rate:.1f}%")
            
            # Should maintain high success rate
            assert success_rate > 95
            
        finally:
            await client.disconnect()
    
    async def test_memory_usage(self):
        """Test memory usage over time."""
        import psutil
        import os
        
        process = psutil.Process(os.getpid())
        initial_memory = process.memory_info().rss / 1024 / 1024  # MB
        
        client = WebSocketTestClient("ws://localhost:8081/")
        await client.connect()
        
        try:
            # Run operations for 30 seconds
            start_time = time.time()
            end_time = start_time + 30
            
            while time.time() < end_time:
                await client.send_command("status")
                await asyncio.sleep(0.1)
            
            final_memory = process.memory_info().rss / 1024 / 1024  # MB
            memory_increase = final_memory - initial_memory
            
            print(f"Memory usage - Initial: {initial_memory:.1f}MB, Final: {final_memory:.1f}MB, Increase: {memory_increase:.1f}MB")
            
            # Memory increase should be reasonable
            assert memory_increase < 100  # Less than 100MB increase
            
        finally:
            await client.disconnect()
    
    async def test_error_recovery_performance(self):
        """Test error recovery performance."""
        client = WebSocketTestClient("ws://localhost:8081/")
        await client.connect()
        
        try:
            # Test error recovery
            error_commands = [
                ("invalid_command", {}),
                ("forward", {"speed": "invalid"}),
                ("servo", {"horizontal": "invalid"}),
                ("unknown", {"param": "value"})
            ]
            
            recovery_times = []
            
            for command, data in error_commands:
                start_time = time.time()
                
                # Send invalid command
                await client.send_command(command, data)
                
                # Send valid command to test recovery
                response = await client.send_command("status")
                recovery_time = time.time() - start_time
                recovery_times.append(recovery_time)
                
                # Verify recovery
                response_data = json.loads(response)
                assert response_data["status"] == "ok"
            
            avg_recovery_time = statistics.mean(recovery_times)
            print(f"Average error recovery time: {avg_recovery_time:.3f}s")
            
            # Recovery should be fast
            assert avg_recovery_time < 0.1  # Under 100ms
            
        finally:
            await client.disconnect()
