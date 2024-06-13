const WebSocket = require('ws');
const config = require('config');
const ROSLIB = require('roslib');

class CameraController {
    constructor() {
        this.startCameraWS = this.startCameraWS.bind(this);
        this.onConnection = this.onConnection.bind(this);
        this.onError = this.onError.bind(this);
        this.onClose = this.onClose.bind(this);
        this.socket = null;
        this.ros = null;
        this.cameraSubscriber = null;
    }

    startCameraController() {
        this.ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        this.ros.on('connection', () => {
            console.log('Connected to rosbridge server.');
            this.cameraSubscriber = new ROSLIB.Topic({
                ros: this.ros,
                name: '/camera_feed',
                messageType: 'sensor_msgs/CompressedImage'
            });

            this.cameraSubscriber.subscribe((msg) => this.video_frames_callback(msg));
            console.log('Camera controller started and subscribed to /camera_feed');
        });

        this.ros.on('error', (error) => {
            console.error('Error connecting to rosbridge server:', error);
        });

        this.ros.on('close', () => {
            console.log('Connection to rosbridge server closed.');
        });
    }

    async onConnection(ws) {
        ws.on('message', data => this.onMessage(ws, data));
        ws.on('error', error => this.onError(ws, error));
        ws.on('close', () => this.onClose(ws));
        console.log('Websocket connection established');

        if (this.ros != null) {
            console.log('Camera connection already established');
            return;
        }

        this.startCameraController();
    }

    async onMessage(ws, data) {
        console.log(`WebSocket message => ${data}`);
    }

    async onError(ws, error) {
        console.log(`WebSocket error => ${error}`);
    }

    async onClose(ws) {
        if (this.socket) {
            this.socket.close();
            this.socket = null;
        }
        if (this.cameraSubscriber) {
            this.cameraSubscriber.unsubscribe();
            this.cameraSubscriber = null;
        }
        if (this.ros) {
            this.ros.close();
            this.ros = null;
        }

        console.log('WebSocket connection closed');
    }

    video_frames_callback(msg) {
        if (!this.socket) {
            return console.log('WebSocket server not started');
        }

        console.log('Received video frame');
        this.socket.clients.forEach((ws) => ws.send(msg.data));
    }

    async startCameraWS(req, res) {
        try {
            const wsCameraPath = process.env.WS_CAMERA_PATH || config.get('server.camera.path');
            const wsPort = process.env.WS_PORT || config.get('server.camera.port');
            const host = req.get('host').split(':')[0];
            const wsURL = `ws://${host}:${wsPort}${wsCameraPath}`;

            if (this.socket) {
                res.status(200).json({
                    error: 'WebSocket server already started',
                    url: wsURL
                });
                return;
            }

            this.socket = new WebSocket.Server({
                path: wsCameraPath,
                port: wsPort
            });

            this.socket.on('connection', this.onConnection);

            res.status(200).json({
                message: 'WebSocket server started',
                url: wsURL
            });

            this.socket.on('listening', () => { });
        } catch (error) {
            console.error('Error starting WebSocket server:', error);
            res.status(500).json({ error: 'Internal Server Error' });
        }
    }
}

module.exports = new CameraController();
