const WebSocket = require('ws');
const config = require('config');
const rclnodejs = require('rclnodejs');

class CameraController {
    constructor() {
        this.startCameraWS = this.startCameraWS.bind(this);
        this.onConnection = this.onConnection.bind(this);
        // this.onError = this.onError.bind(this);
        // this.onClose = this.onClose.bind(this);
        this.socket = null;
        this.camera_node = null;
    }

    async startCameraController() {
        await rclnodejs.init();
        this.camera_node = new rclnodejs.Node('camera_node');
        this.cameraSubscriber = node.createSubscription(
            'std_msgs/msg/CompressedImage',
            '/video_frames',
            (msg) => this.video_frames_callback(msg)
        );
        console.log('Camera controller started');
        rclnodejs.spin(node);
    }

    async onConnection(ws) {
        // ws.on('message', data => this.onMessage(ws, data));
        // ws.on('error', error => this.onError(ws, error));
        // ws.on('close', () => this.onClose(ws));
        console.log('WebSocket connection established');
    }


    async video_frames_callback(msg) {
        console.log('Received video frame');
        // this.socket.send(msg);
    }
}

module.exports = new CameraController();
