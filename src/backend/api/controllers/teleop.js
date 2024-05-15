const WebSocket = require('ws');
const config = require('config');

class TeleopController {

    constructor() {
        this.startTeleopWS = this.startTeleopWS.bind(this);
        this.onConnection = this.onConnection.bind(this);
        this.onMessage = this.onMessage.bind(this);
        this.onError = this.onError.bind(this);
        this.onClose = this.onClose.bind(this);
    }

    async onConnection(ws) {
        ws.on('message', data => this.onMessage(ws, data));
        ws.on('error', error => this.onError(ws, error));
        ws.on('close', () => this.onClose(ws));
        console.log('New WebSocket connection');
    }

    async onMessage(ws, data) {
        console.log(`Received message => ${data}`);
        ws.send('Message received');
    }

    async onError(ws, error) {
        console.log(`WebSocket error => ${error}`);
    }

    async onClose(ws) {
        console.log('WebSocket connection closed');
    }

    async startTeleopWS(req, res) {
        try {
            const wsPort = process.env.WS_PORT || config.get('server.teleop.port');
            const wsServer = new WebSocket.Server({ port: wsPort });

            wsServer.on('connection', this.onConnection);

            res.status(200).json({ message: 'WebSocket server started', port: wsPort });
        } catch (error) {
            console.error('Error starting WebSocket server:', error);
            res.status(500).json({ error: 'Internal Server Error' });
        }
    }
}

module.exports = new TeleopController();
