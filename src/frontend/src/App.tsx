import React, { useEffect, useState, useCallback, useRef } from 'react';
import useWebSocket from 'react-use-websocket';
import Joystick from './components/Joystick'; // Adjust the path based on your project structure
import KillButton from './components/Kill'; // Import the KillButton component
import HamburgerMenu from './components/HamburgerMenu';

const API_URL = `http://${window.location.hostname}:8000`;

const App: React.FC = () => {
    const [teleopData, setTeleopData] = useState<any>({});
    const [teleopSocketUrl, setTeleopSocketUrl] = useState<string | null>(null);
    const [cameraData, setCameraData] = useState<any>({});
    const [cameraSocketUrl, setCameraSocketUrl] = useState<string | null>(null);
    const [image, setImage] = useState<string | null>(null);
    const [fps, setFps] = useState<number>(0);
    const messageTimestamps = useRef<number[]>([]);
    const now = Date.now(); // Capture the current timestamp

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

    const videoData = useCallback(async () => {
        try {
            const response = await fetch(`${API_URL}/camera/start`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({}),
            });
            const jsonData = await response.json();
            setCameraData(jsonData)
        } catch (error) {
            console.error('Error fetching camera:', error);
        }
    }, []);

    useEffect(() => {
        fetchData();
        videoData();
    }, [fetchData, videoData]);

    useEffect(() => {
        if (teleopData?.url) {
            setTeleopSocketUrl(teleopData.url);
        }
        if (cameraData?.url) {
            setCameraSocketUrl(cameraData.url);
        }
    }, [cameraData]);

    const teleopWebSocket = useWebSocket(teleopSocketUrl, {
        onOpen: () => console.log('WebSocket connection established.'),
        onClose: () => console.log('WebSocket connection closed.'),
        onError: (event) => console.error('WebSocket error:', event),
        onMessage: (event) => console.log('WebSocket message:', event.data),
        shouldReconnect: (closeEvent) => true, // Will attempt to reconnect on all close events, such as server shutting down
    });

    
    const cameraWebSocket = useWebSocket(cameraSocketUrl, {
        onOpen: () => console.log('WebSocket connection established.'),
        onClose: () => console.log('WebSocket connection closed.'),
        onError: (event) => console.error('WebSocket error:', event),
        onMessage: (event) => {setImage(event.data);// Add the current timestamp to the list
            messageTimestamps.current.push(now);
        
            // Remove timestamps older than one second
            const oneSecondAgo = now - 1000;
            messageTimestamps.current = messageTimestamps.current.filter(timestamp => timestamp >= oneSecondAgo);
        
            // Calculate FPS as the number of messages received in the last second
            setFps(messageTimestamps.current.length);
        },
        shouldReconnect: (closeEvent) => true, // Will attempt to reconnect on all close events, such as server shutting down
    });

    useEffect(() => {
        if (teleopWebSocket.readyState === WebSocket.OPEN) {
            teleopWebSocket.sendMessage(JSON.stringify({ message: 'Hello, WebSocket!' }));
        }
    }, [teleopWebSocket.readyState, teleopWebSocket.sendMessage]);


    useEffect(() => {
        if (cameraWebSocket.readyState === WebSocket.OPEN) {
            cameraWebSocket.sendMessage(JSON.stringify({ message: 'Hello, WebSocket!' }));
        }
    }, [cameraWebSocket.readyState, cameraWebSocket.sendMessage]);

    return (<>
        <img className="relative h-screen p-4" src= {`data:image/jpeg;base64,${image}`}></img>
            <div className="absolute top-0 left-0 p-4">
                <HamburgerMenu />
                <p>Fps: {fps}</p>
            </div>
            <div className="absolute bottom-0 left-0 flex items-start justify-start w-1/2 h-1/2 p-4">
                <div className="relative w-full h-full flex items-center justify-center">
                    <div className="absolute transform translate-x-[-9rem]">
                        <KillButton sendMessage={teleopWebSocket.sendMessage} />
                    </div>
                </div>
            </div>
            <div className="absolute bottom-0 right-0 flex items-end justify-end w-1/2 h-1/2 p-4">
                <div className="relative w-full h-full flex items-center justify-center">
                    <div className="absolute transform translate-x-full">
                        <Joystick sendMessage={teleopWebSocket.sendMessage} />
                    </div>
                </div>
            </div>
        </>
    );
};

export default App;
