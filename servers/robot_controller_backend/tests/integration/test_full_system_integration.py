"""
Integration tests for the robot controller backend.
Tests the interaction between different components and services.
"""

import pytest
import asyncio
import websockets
import json
import time
from conftest import WebSocketTestClient, generate_motor_telemetry, generate_servo_telemetry

@pytest.mark.integration
class TestMovementWebSocketIntegration:
    """Test movement WebSocket server integration."""
    
    async def test_movement_websocket_connection(self, movement_websocket):
        """Test basic WebSocket connection."""
        assert movement_websocket is not None
        assert not movement_websocket.closed
    
    async def test_movement_commands(self, movement_websocket):
        """Test movement commands."""
        client = WebSocketTestClient(f"ws://localhost:8081/")
        await client.connect()
        
        try:
            # Test forward command
            response = await client.send_command("forward", {"speed": 1000})
            response_data = json.loads(response)
            assert response_data["status"] == "ok"
            assert response_data["action"] == "forward"
            
            # Test stop command
            response = await client.send_command("stop")
            response_data = json.loads(response)
            assert response_data["status"] == "ok"
            assert response_data["action"] == "stop"
            
        finally:
            await client.disconnect()
    
    async def test_motor_telemetry_integration(self, movement_websocket):
        """Test motor telemetry integration."""
        client = WebSocketTestClient(f"ws://localhost:8081/")
        await client.connect()
        
        try:
            # Send forward command to generate telemetry
            await client.send_command("forward", {"speed": 1500})
            
            # Request status to get telemetry
            response = await client.send_command("status")
            response_data = json.loads(response)
            
            assert "motors" in response_data
            motors = response_data["motors"]
            
            # Check that all 4 motors have telemetry
            expected_motors = ["frontLeft", "frontRight", "rearLeft", "rearRight"]
            for motor in expected_motors:
                assert motor in motors
                assert "speed" in motors[motor]
                assert "power" in motors[motor]
                assert "pwm" in motors[motor]
            
        finally:
            await client.disconnect()
    
    async def test_servo_telemetry_integration(self, movement_websocket):
        """Test servo telemetry integration."""
        client = WebSocketTestClient(f"ws://localhost:8081/")
        await client.connect()
        
        try:
            # Test servo commands
            response = await client.send_command("servo", {"horizontal": 90, "vertical": 45})
            response_data = json.loads(response)
            assert response_data["status"] == "ok"
            
            # Request status to get servo telemetry
            response = await client.send_command("status")
            response_data = json.loads(response)
            
            assert "servo" in response_data
            servo = response_data["servo"]
            assert "horizontal" in servo
            assert "vertical" in servo
            assert "min" in servo
            assert "max" in servo
            
        finally:
            await client.disconnect()

@pytest.mark.integration
class TestSensorWebSocketIntegration:
    """Test sensor WebSocket server integration."""
    
    async def test_sensor_websocket_connection(self, sensor_websocket):
        """Test basic sensor WebSocket connection."""
        assert sensor_websocket is not None
        assert not sensor_websocket.closed
    
    async def test_line_tracking_data(self, sensor_websocket):
        """Test line tracking sensor data."""
        client = WebSocketTestClient(f"ws://localhost:8090/")
        await client.connect()
        
        try:
            # Wait for sensor data
            message = await client.receive_message(timeout=5)
            assert message is not None
            
            # Check line tracking data structure
            if "lineTracking" in message:
                line_data = message["lineTracking"]
                assert "left" in line_data
                assert "center" in line_data
                assert "right" in line_data
                
        finally:
            await client.disconnect()

@pytest.mark.integration
class TestFullSystemIntegration:
    """Test full system integration."""
    
    async def test_movement_and_sensor_integration(self):
        """Test integration between movement and sensor systems."""
        movement_client = WebSocketTestClient("ws://localhost:8081/")
        sensor_client = WebSocketTestClient("ws://localhost:8090/")
        
        await movement_client.connect()
        await sensor_client.connect()
        
        try:
            # Test movement command
            response = await movement_client.send_command("forward", {"speed": 1000})
            response_data = json.loads(response)
            assert response_data["status"] == "ok"
            
            # Test sensor data reception
            sensor_message = await sensor_client.receive_message(timeout=5)
            assert sensor_message is not None
            
            # Test status request
            status_response = await movement_client.send_command("status")
            status_data = json.loads(status_response)
            assert "motors" in status_data
            assert "servo" in status_data
            
        finally:
            await movement_client.disconnect()
            await sensor_client.disconnect()
    
    async def test_error_handling_integration(self):
        """Test error handling across components."""
        movement_client = WebSocketTestClient("ws://localhost:8081/")
        await movement_client.connect()
        
        try:
            # Test invalid command
            response = await movement_client.send_command("invalid_command")
            response_data = json.loads(response)
            assert response_data["status"] == "error"
            
            # Test invalid parameters
            response = await movement_client.send_command("forward", {"speed": "invalid"})
            response_data = json.loads(response)
            # Should handle gracefully, not crash
            
        finally:
            await movement_client.disconnect()

@pytest.mark.integration
class TestPerformanceIntegration:
    """Test performance under load."""
    
    async def test_concurrent_connections(self):
        """Test multiple concurrent WebSocket connections."""
        clients = []
        
        try:
            # Create multiple connections
            for i in range(5):
                client = WebSocketTestClient("ws://localhost:8081/")
                await client.connect()
                clients.append(client)
            
            # Send commands from all clients
            tasks = []
            for client in clients:
                task = asyncio.create_task(
                    client.send_command("forward", {"speed": 1000})
                )
                tasks.append(task)
            
            # Wait for all commands to complete
            responses = await asyncio.gather(*tasks)
            
            # Check all responses are successful
            for response in responses:
                response_data = json.loads(response)
                assert response_data["status"] == "ok"
                
        finally:
            # Clean up all connections
            for client in clients:
                await client.disconnect()
    
    async def test_rapid_commands(self):
        """Test rapid command execution."""
        client = WebSocketTestClient("ws://localhost:8081/")
        await client.connect()
        
        try:
            start_time = time.time()
            
            # Send rapid commands
            for i in range(10):
                await client.send_command("forward", {"speed": 1000})
                await asyncio.sleep(0.1)
                await client.send_command("stop")
                await asyncio.sleep(0.1)
            
            end_time = time.time()
            duration = end_time - start_time
            
            # Should complete within reasonable time
            assert duration < 5.0
            
        finally:
            await client.disconnect()

@pytest.mark.integration
class TestDataConsistencyIntegration:
    """Test data consistency across components."""
    
    async def test_telemetry_consistency(self):
        """Test that telemetry data is consistent."""
        client = WebSocketTestClient("ws://localhost:8081/")
        await client.connect()
        
        try:
            # Send movement command
            await client.send_command("forward", {"speed": 2000})
            
            # Get status multiple times
            status1 = await client.send_command("status")
            await asyncio.sleep(0.1)
            status2 = await client.send_command("status")
            
            status1_data = json.loads(status1)
            status2_data = json.loads(status2)
            
            # Motor telemetry should be consistent
            motors1 = status1_data["motors"]
            motors2 = status2_data["motors"]
            
            for motor_name in ["frontLeft", "frontRight", "rearLeft", "rearRight"]:
                motor1 = motors1[motor_name]
                motor2 = motors2[motor_name]
                
                # PWM should be the same
                assert motor1["pwm"] == motor2["pwm"]
                
                # Speed and power should be similar (allowing for small variations)
                assert abs(motor1["speed"] - motor2["speed"]) < 10
                assert abs(motor1["power"] - motor2["power"]) < 5
            
        finally:
            await client.disconnect()
