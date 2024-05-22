const WebSocket = require('ws');
const config = require('config');
const rclnodejs = require('rclnodejs');

class CameraController {
    constructor() {
        this.startCameraWS = this.startCameraWS.bind(this);
        this.onConnection = this.onConnection.bind(this);
        this.onError = this.onError.bind(this);
        this.onClose = this.onClose.bind(this);
        this.socket = null;
        this.camera_node = null;
    }

    async startCameraController() {
        await rclnodejs.init();
        this.camera_node = new rclnodejs.Node('camera_node');
        this.cameraSubscriber = this.camera_node.createSubscription(
            'sensor_msgs/msg/CompressedImage',
            '/camera_feed',
            (msg) => this.video_frames_callback(msg)
        );
        console.log('Camera controller started');
        this.camera_node.spin();
    }

    async onConnection(ws) {
        ws.on('message', data => this.onMessage(ws, data));
        ws.on('error', error => this.onError(ws, error));
        ws.on('close', () => this.onClose(ws));
        console.log('WebSocket connection established');
        this.startCameraController();
    }


    async onError(ws, error) {
        console.log(`WebSocket error => ${error}`);
    }

    async onClose(ws) {
        this.socket.close();
        this.socket = null;
        this.teleop_node.destroy();
        rclnodejs.shutdown();
        this.teleop_node = null;

        console.log('WebSocket connection closed');
    }


    async video_frames_callback(msg) {
        if (!this.socket) {
            return console.log('WebSocket server not started');
        } 
        console.log('Received video frame');
        const imageData = Buffer.from(msg.data); // Convert data to Buffer
        const base64Image = imageData.toString('base64'); // Encode to Base64
        this.socket.clients.forEach((ws) => ws.send(base64Image));
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

            this.socket.on('listening', () => {});
        } catch (error) {
            console.error('Error starting WebSocket server:', error);
            res.status(500).json({ error: 'Internal Server Error' });
        }
    }
}

module.exports = new CameraController();
