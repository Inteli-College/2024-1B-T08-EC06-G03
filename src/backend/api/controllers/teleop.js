const WebSocket = require('ws');
const config = require('config');
const rclnodejs = require('rclnodejs');

class TeleopController {

    constructor() {
        this.startTeleopWS = this.startTeleopWS.bind(this);
        this.onConnection = this.onConnection.bind(this);
        this.onMessage = this.onMessage.bind(this);
        this.onError = this.onError.bind(this);
        this.onClose = this.onClose.bind(this);
        this.socket = null;
    }

    async onConnection(ws) {
        ws.on('message', data => this.onMessage(ws, data));
        ws.on('error', error => this.onError(ws, error));
        ws.on('close', () => this.onClose(ws));
        console.log('WebSocket connection established');

        await rclnodejs.init();
        const teleop_backend_node = new rclnodejs.Node('teleop_node');
        this.linear_speed_publisher = teleop_backend_node.createPublisher('std_msgs/msg/Float32', 'linear_speed');
        this.angular_speed_publisher = teleop_backend_node.createPublisher('std_msgs/msg/Float32', 'angular_speed');
        this.kill_button_publisher = teleop_backend_node.createPublisher('std_msgs/msg/Bool', 'kill_button');
        console.log('Teleop connection established');
    }

    async onMessage(ws, data) {
        const message = JSON.parse(data);

        if (message.linear_speed) {
            console.log('Publishing linear speed:', message.linear_speed);
            this.linear_speed_publisher.publish(message.linear_speed);
        }

        if (message.angular_speed) {
            console.log('Publishing angular speed:', message.angular_speed);
            this.angular_speed_publisher.publish(message.angular_speed);
        }

        if (message.kill_node) {
            console.log('Publishing kill node:', message.kill_node);
            this.kill_button_publisher.publish(message.kill_node);
        }

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

            if (this.socket) {
                res.status(400).json({ error: 'WebSocket server already started', port: wsPort });
                return;
            }

            this.socket = new WebSocket.Server({ port: wsPort });
            this.socket.on('connection', this.onConnection);

            res.status(200).json({ message: 'WebSocket server started', port: wsPort });
        } catch (error) {
            console.error('Error starting WebSocket server:', error);

            res.status(500).json({ error: 'Internal Server Error' });
        }
    }
}

module.exports = new TeleopController();
