const WebSocket = require('ws');

const ws = new WebSocket('wss://100.82.88.25:8080/ws', {
    rejectUnauthorized: false // Disable SSL check
});

ws.on('open', function open() {
    console.log('WebSocket connection established');

    // Example commands
    const commands = [
        { command: 'move-up', angle: 0, request_id: '1' },
        { command: 'move-down', angle: 0, request_id: '2' },
        { command: 'servo-horizontal', angle: 45, request_id: '3' },
        { command: 'ultrasonic-sensor', request_id: '4' },
        { command: 'line-tracking', request_id: '5' }
    ];

    commands.forEach(cmd => {
        ws.send(JSON.stringify(cmd));
    });
});

ws.on('message', function incoming(data) {
    const message = data.toString();
    console.log('Received message:', message);
});

ws.on('close', function close(code, reason) {
    console.log('WebSocket connection closed');
    console.log('Code:', code);
    console.log('Reason:', reason.toString());
});

ws.on('error', function error(err) {
    console.error('WebSocket error:', err);
});
