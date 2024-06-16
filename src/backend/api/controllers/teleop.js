const WebSocket = require('ws');
const config = require('config');
const ROSLIB = require('roslib');

class TeleopController {
    constructor() {
        this.startTeleopWS = this.startTeleopWS.bind(this);
        this.onConnection = this.onConnection.bind(this);
        this.onError = this.onError.bind(this);
        this.onClose = this.onClose.bind(this);
        this.socket = null;
        this.ros = null;
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

        this.lidarCallback = this.lidarCallback.bind(this);
        this.publishSpeeds = this.publishSpeeds.bind(this);
    }

    startTeleopController() {
        this.ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        this.ros.on('connection', () => {
            console.log('Connected to ROSBridge server.');
            this.lidarSubscriber = new ROSLIB.Topic({
                ros: this.ros,
                name: '/obstacle_distances',
                messageType: 'std_msgs/Float32MultiArray'
            });

            this.angularSpeedPublisher = new ROSLIB.Topic({
                ros: this.ros,
                name: '/angular_speed',
                messageType: 'std_msgs/Float32'
            });

            this.linearSpeedPublisher = new ROSLIB.Topic({
                ros: this.ros,
                name: '/linear_speed',
                messageType: 'std_msgs/Float32'
            });

            this.killRobotClient = new ROSLIB.Service({
                ros: this.ros,
                name: '/kill_robot_service',
                serviceType: 'std_srvs/srv/Trigger'
            });

            this.lidarSubscriber.subscribe(this.lidarCallback);

            console.log('Teleop controller started and subscribed to /obstacle_distances');

            setInterval(this.publishSpeeds, 100);
        });

        this.ros.on('error', (error) => {
            console.error('Error connecting to ROSBridge server:', error);
        });

        this.ros.on('close', () => {
            console.log('Connection to ROSBridge server closed.');
        });
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

    publishSpeeds() {
        if (this.ros == null) {
            console.error('ROS connection not established');
            return;
        }

        this.linearSpeed = this.validateLinearSpeed(this.linearSpeed);

        const linearSpeedMessage = new ROSLIB.Message({
            data: this.linearSpeed
        });
        const angularSpeedMessage = new ROSLIB.Message({
            data: this.angularSpeed
        });

        this.linearSpeedPublisher.publish(linearSpeedMessage);
        this.angularSpeedPublisher.publish(angularSpeedMessage);

        console.log(`Published speeds: linear=${this.linearSpeed}, angular=${this.angularSpeed}`);
    }

    async onConnection(ws) {
        ws.on('message', data => this.onMessage(ws, data));
        ws.on('error', error => this.onError(ws, error));
        ws.on('close', () => this.onClose(ws));
        console.log('WebSocket connection established');

        if (this.ros != null) {
            console.log('Teleop connection already established');
            return;
        }

        this.startTeleopController();
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
            const request = new ROSLIB.ServiceRequest({});
            this.killRobotClient.callService(request, (response) => {
                console.log('Kill robot service response:', response);
            });
        }
    }

    onError(ws, error) {
        console.log(`WebSocket error => ${error}`);
    }

    onClose(ws) {
        if (this.socket) {
            this.socket.close();
            this.socket = null;
        }
        console.log('WebSocket connection closed');
    }

    lidarCallback(msg) {
        if (!this.socket) {
            return console.log('WebSocket server not started');
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
