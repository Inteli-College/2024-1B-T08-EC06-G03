const WebSocket = require('ws');
const config = require('config');
const rclnodejs = require('rclnodejs');

class TeleopController {
    constructor() {
        this.socket = null;
        this.teleopNode = null;
        this.linearSpeed = 0;
        this.angularSpeed = 0;
        this.obstacleThreshold = 20;
        this.directionLabels = [
            "front",
            "front-left",
            "left",
            "back-left",
            "back",
            "back-right",
            "right",
            "front-right",
        ];
        this.obstacleLocations = [];

        this.startTeleopWS = this.startTeleopWS.bind(this);
        this.onConnection = this.onConnection.bind(this);
        this.onMessage = this.onMessage.bind(this);
        this.onError = this.onError.bind(this);
        this.onClose = this.onClose.bind(this);
        this.lidarCallback = this.lidarCallback.bind(this);
    }

    async initializeTeleopNode() {
        try {
            if (!rclnodejs.init?.called) {
                console.log('Initializing rclnodejs');
                await rclnodejs.init();
            }
        } catch (error) {
            console.error('Error initializing rclnodejs:', error);
            return;
        }

        this.teleopNode = new rclnodejs.Node('teleop_node');
        this.linearSpeedPublisher = this.teleopNode.createPublisher('std_msgs/msg/Float32', 'linear_speed');
        this.angularSpeedPublisher = this.teleopNode.createPublisher('std_msgs/msg/Float32', 'angular_speed');
        this.killRobotClient = this.teleopNode.createClient('std_srvs/srv/Trigger', 'kill_robot_service');

        this.teleopNode.createTimer(50, () => {
            this.linearSpeed = this.validateLinearSpeed(this.linearSpeed);
            this.linearSpeedPublisher.publish(this.linearSpeed);
            this.angularSpeedPublisher.publish(this.angularSpeed);
            console.log(`Linear speed: ${this.linearSpeed}, Angular speed: ${this.angularSpeed}`);
        });

        this.lidarSubscriber = this.teleopNode.createSubscription(
            'std_msgs/msg/Float32MultiArray',
            '/obstacle_distances',
            this.lidarCallback
        );

        console.log('Teleop node initialized');
        this.teleopNode.spin();
    }

    async onConnection(ws) {
        ws.on('message', data => this.onMessage(ws, data));
        ws.on('error', error => this.onError(ws, error));
        ws.on('close', () => this.onClose(ws));
        console.log('WebSocket connection established');

        if (this.teleopNode) {
            console.log('Teleop node already initialized');
            return;
        }

        await this.initializeTeleopNode();
    }

    onMessage(ws, data) {
        let message;

        try {
            message = JSON.parse(data);
        } catch (error) {
            ws.send('Message was not in JSON');
            console.log('Invalid message format:', error);
            return;
        }

        if (message.linear_speed !== undefined) {
            this.linearSpeed = message.linear_speed;
        }

        if (message.angular_speed !== undefined) {
            this.angularSpeed = message.angular_speed;
        }

        if (message.kill_robot) {
            console.log('Calling kill robot service');
            this.killRobotClient.sendRequest({}, response => {
                console.log('Kill robot service response:', response);
                ws.send(`Kill button response: ${JSON.stringify(response)}`);
            });
        }

        ws.send('Message received');
    }

    onError(ws, error) {
        console.log(`WebSocket error => ${error}`);
    }

    async onClose(ws) {
        this.socket.close();
        this.socket = null;

        if (this.teleopNode) {
            this.teleopNode.destroy();
            rclnodejs.shutdown();
            this.teleopNode = null;
        }

        console.log('WebSocket connection closed');
    }

    lidarCallback(msg) {
        if (!this.socket) {
            console.log('WebSocket server not started');
            return;
        }

        const obstacleStatus = msg.data.map(distance => distance < this.obstacleThreshold ? 1 : 0);
        this.obstacleLocations = [];

        obstacleStatus.forEach((status, index) => {
            if (status === 1) {
                this.obstacleLocations.push(this.directionLabels[index]);
            }
        });

        console.log('Obstacle locations:', this.obstacleLocations);

        this.socket.clients.forEach(ws => ws.send(JSON.stringify({ obstacle: this.obstacleLocations })));
    }

    validateLinearSpeed(linearSpeed) {
        const obstaclesInFront = ['front', 'front-right', 'front-left'].some(direction => this.obstacleLocations.includes(direction));
        const obstaclesInBack = ['back', 'back-right', 'back-left'].some(direction => this.obstacleLocations.includes(direction));

        if (obstaclesInFront && linearSpeed > 0) {
            return 0;
        }

        if (obstaclesInBack && linearSpeed < 0) {
            return 0;
        }

        return linearSpeed;
    }

    async startTeleopWS(req, res) {
        try {
            const wsTeleopPath = process.env.WS_TELEOP_PATH || config.get('server.teleop.path');
            const wsPort = process.env.WS_PORT || config.get('server.teleop.port');
            const host = req.get('host').split(':')[0];
            const wsURL = `ws://${host}:${wsPort}${wsTeleopPath}`;

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
                message: 'WebSocket server started',
                url: wsURL
            });
        } catch (error) {
            console.error('Error starting WebSocket server:', error);
            res.status(500).json({ error: 'Internal Server Error' });
        }
    }
}

module.exports = new TeleopController();
