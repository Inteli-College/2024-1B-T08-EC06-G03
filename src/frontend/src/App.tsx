import React, { useEffect, useState, useCallback, useRef } from 'react';
import useWebSocket from 'react-use-websocket';
import Joystick from './components/Joystick'; // Adjust the path based on your project structure
import KillButton from './components/Kill'; // Import the KillButton component
import SnapButton from './components/Snap';
import HamburgerMenu from './components/HamburgerMenu';
import DetectionInterface from './components/DetectionInterface';
import ROSLIB from 'roslib'

const API_URL = `http://${window.location.hostname}:8000`;

type Direction =
    | 'front'
    | 'front-right'
    | 'right'
    | 'back-right'
    | 'back'
    | 'back-left'
    | 'left'
    | 'front-left';

const App: React.FC = () => {

    const [directions, setDirections] = useState<Direction[]>([]);
    const [teleopData, setTeleopData] = useState<any>({});
    const [teleopSocketUrl, setTeleopSocketUrl] = useState<string | null>(null);
    const [image, setImage] = useState<string | null>(null);

    const [fps, setFps] = useState<number>(0);
    const messageTimestamps = useRef<number[]>([]);

    const fetchData = useCallback(async () => {
        try {
            const response = await fetch(`${API_URL}/teleop/start`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({}),
            });
            const jsonData = await response.json();
            setTeleopData(jsonData);
        } catch (error) {
            console.error('Error fetching teleop data:', error);
        }
    }, []);

    const startCamera = () => {
        const ros = new ROSLIB.Ros({
            url: `ws://${window.location.hostname}:9090`
        });

        ros.on('connection', () => {
            console.log('Connected to rosbridge server.');
            const cameraSubscriber = new ROSLIB.Topic({
                ros: ros,
                name: '/camera_feed',
                messageType: 'sensor_msgs/CompressedImage'
            });

            cameraSubscriber.subscribe((msg: any) => {
                setImage(msg.data);

                const now = Date.now();
                messageTimestamps.current.push(now);
                const oneSecondAgo = now - 1000;
                messageTimestamps.current = messageTimestamps.current.filter(timestamp => timestamp >= oneSecondAgo);

                setFps(messageTimestamps.current.length);
            });

            console.log('Camera controller started and subscribed to /camera_feed');
        });

        ros.on('error', (error: Error) => {
            console.error('Error connecting to rosbridge server:', error);
        });

        ros.on('close', () => {
            console.log('Connection to rosbridge server closed.');
        });
    };

    useEffect(() => {
        fetchData();
    }, [fetchData]);

    useEffect(() => {
        startCamera();
        if (teleopData?.url) {
            setTeleopSocketUrl(teleopData.url);
        }
    }, [teleopData]);


    const teleopWebSocket = useWebSocket(teleopSocketUrl, {
        onOpen: () => console.log('WebSocket connection established.'),
        onClose: () => console.log('WebSocket connection closed.'),
        onError: (event) => console.error('WebSocket error:', event),
        onMessage: (event) => {
            console.log('WebSocket message:', event.data);
            try {
                const receivedDirections: Direction[] = JSON.parse(event.data).obstacle;
                setDirections(receivedDirections);
            } catch (error) {
                console.log('Error parsing WebSocket message:', error);
            }

        },
        shouldReconnect: (closeEvent) => true, // Will attempt to reconnect on all close events, such as server shutting down
    });


    useEffect(() => {
        if (teleopWebSocket.readyState === WebSocket.OPEN) {
            teleopWebSocket.sendMessage(JSON.stringify({ message: 'Hello, WebSocket!' }));
        }
    }, [teleopWebSocket.readyState, teleopWebSocket.sendMessage]);


    return (
    <div className='relative overflow-hidden h-screen flex flex-col items-center justify-center bg-gray-600'>
        <div className="relative w-full h-full flex items-center justify-center overflow-hidden">    
            <div className="relative h-full aspect-[4/3] object-cover bg-black">   
                <DetectionInterface directions={directions} />
                <img className="relative w-[640px] h-[480px] object-cover" src={`data:image/jpeg;base64,${image}`} />
            </div>
        </div>
        <div className="absolute top-0 left-0 p-4 flex flex-col items-start justify-start">
            <HamburgerMenu />
            <p className='text-red-600'>Fps: {fps}</p>
        </div>
        <div className="absolute bottom-20 left-20 p-4">
            <div className="relative">
                <KillButton sendMessage={teleopWebSocket.sendMessage} />
            </div>
        </div>
        <div className="absolute bottom-80 right-24 p-4">
            <div className="relative">
                <SnapButton sendMessage={teleopWebSocket.sendMessage} />
            </div>
        </div>
        <div className="absolute bottom-20 right-20 p-4">
            <div className="relative">
                <Joystick sendMessage={teleopWebSocket.sendMessage} />
            </div>
        </div>
    </div>
    );
};

export default App;
