const WebSocket = require('ws');
const config = require('config');
const rclnodejs = require('rclnodejs');

class LidarController {
    constructor() {
        this.startLidarWS = this.startLidarWS.bind(this);
        this.onConnection = this.onConnection.bind(this);
        this.onError = this.onError.bind(this);
        this.onClose = this.onClose.bind(this);
        this.socket = null;
        this.lidar_node = null;
        this.threshold = 20;
        this.directionLabels = [
            "front",
            "front-right",
            "right",
            "back-right",
            "back",
            "back-left",
            "left",
            "front-left",
        ];
    }

    async startLidarController() {
        await rclnodejs.init();
        this.lidar_node = new rclnodejs.Node('lidar_node');
        this.lidarSubscriber = this.lidar_node.createSubscription(
            'std_msgs/msg/Float32MultiArray',
            '/obstacle_distances',
            (msg) => this.lidar_callback(msg)
        );
        console.log('Lidar controller started');
        this.lidar_node.spin();
    }

    async onConnection(ws) {
        ws.on('message', data => this.onMessage(ws, data));
        ws.on('error', error => this.onError(ws, error));
        ws.on('close', () => this.onClose(ws));
        console.log('WebSocket connection established');
        this.startLidarController();
    }

    async onError(ws, error) {
        console.log(`WebSocket error => ${error}`);
    }

    async onClose(ws) {
        this.socket.close();
        this.socket = null;
        this.lidar_node.destroy();
        rclnodejs.shutdown();
        this.lidar_node = null;

        console.log('WebSocket connection closed');
    }

    async lidar_callback(msg) {
        if (!this.socket) {
            return console.log('WebSocket server not started');
        }
        console.log('Received lidar data');
        console.log(msg)
        const obstacleStatus = this.checkObstacleStatus(msg.data);
        const labeledData = this.labelData(obstacleStatus);
        this.socket.clients.forEach((ws) => ws.send(JSON.stringify(labeledData)));
    }

    checkObstacleStatus(distances) {
        const obstacleThreshold = this.threshold;
        const obstacleStatus = distances.map(distance => distance < obstacleThreshold ? 1 : 0);
        return obstacleStatus;
    }

    labelData(data) {
        const labeledData = {};
        this.directionLabels.forEach((label, index) => {
            labeledData[label] = data[index];
        });
        return labeledData;
    }

    async startLidarWS(req, res) {
        try {
            const wsLidarPath = process.env.WS_LIDAR_PATH || config.get('server.lidar.path');
            const wsPort = process.env.WS_PORT || config.get('server.lidar.port');
            const host = req.get('host').split(':')[0];
            const wsURL = `ws://${host}:${wsPort}${wsLidarPath}`;

            if (this.socket) {
                res.status(200).json({
                    error: 'WebSocket server already started',
                    url: wsURL
                });
                return;
            }

            this.socket = new WebSocket.Server({
                path: wsLidarPath,
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

module.exports = new LidarController();
