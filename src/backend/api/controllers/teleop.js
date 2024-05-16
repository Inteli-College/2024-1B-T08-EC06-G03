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
        this.teleop_node = null;
    }

    async onConnection(ws) {
        ws.on('message', data => this.onMessage(ws, data));
        ws.on('error', error => this.onError(ws, error));
        ws.on('close', () => this.onClose(ws));
        console.log('WebSocket connection established');

        await rclnodejs.init();
        this.teleop_node = new rclnodejs.Node('teleop_node');
        this.linear_speed_publisher = this.teleop_node.createPublisher('std_msgs/msg/Float32', 'linear_speed');
        this.angular_speed_publisher = this.teleop_node.createPublisher('std_msgs/msg/Float32', 'angular_speed');
        this.kill_button_publisher = this.teleop_node.createPublisher('std_msgs/msg/Bool', 'kill_button');
        console.log('Teleop connection established');
    }

    async onMessage(ws, data) {
        let message = null;

        try {
            message = JSON.parse(data);
        } catch (error) {
            ws.send('Message was not in JSON');
            console.log(error);
            return;
        }

        if (message.linear_speed != null) {
            console.log('Publishing linear speed:', message.linear_speed);
            this.linear_speed_publisher.publish(message.linear_speed);
        }

        if (message.angular_speed != null) {
            console.log('Publishing angular speed:', message.angular_speed);
            this.angular_speed_publisher.publish(message.angular_speed);
        }

        if (message.kill_button != null) {
            console.log('Publishing kill button:', message.kill_button);
            this.kill_button_publisher.publish(message.kill_button);
        }

        ws.send('Message received');
    }

    async onError(ws, error) {
        console.log(`WebSocket error => ${error}`);
    }

    async onClose(ws) {
        this.socket.close();
        this.socket = null;
        this.teleop_node.destroy();
        rclnodejs.shutdown();

        console.log('WebSocket connection closed');
    }

    async startTeleopWS(req, res) {
        try {
            const wsTeleopPath = process.env.WS_TELEOP_PATH || config.get('server.teleop.path');
            const wsPort = process.env.WS_PORT || config.get('server.teleop.port');
            const wsURL = `ws://localhost:${wsPort}${wsTeleopPath}`;

            if (this.socket) {
                res.status(200).json({
                    error: 'WebSocket server already started',
                    url: wsURL
                });
                return;
            }


            this.socket = new WebSocket.Server({
                path: wsTeleopPath,
                port: wsPort
            });
            this.socket.on('connection', this.onConnection);

            res.status(200).json({
                message: 'WebSocket server started', url: wsURL
            });
        } catch (error) {
            console.error('Error starting WebSocket server:', error);
            res.status(500).json({ error: 'Internal Server Error' });
        }
    }
}

module.exports = new TeleopController();
