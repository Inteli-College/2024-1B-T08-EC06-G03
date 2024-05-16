import React, { useEffect, useState, useCallback } from 'react';
import useWebSocket from 'react-use-websocket';
import Joystick from './components/Joystick'; // Adjust the path based on your project structure
import KillButton from './components/Kill'; // Import the KillButton component
import HamburgerMenu from './components/HamburgerMenu';

const API_URL = 'http://127.0.0.1:8000';

const App: React.FC = () => {
    const [teleopData, setTeleopData] = useState<any>({});
    const [socketUrl, setSocketUrl] = useState<string | null>(null);

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

    useEffect(() => {
        fetchData();
    }, [fetchData]);

    useEffect(() => {
        if (teleopData?.url) {
            setSocketUrl(teleopData.url);
        }
    }, [teleopData]);

    const { sendMessage, lastMessage, readyState } = useWebSocket(socketUrl, {
        onOpen: () => console.log('WebSocket connection established.'),
        onClose: () => console.log('WebSocket connection closed.'),
        onError: (event) => console.error('WebSocket error:', event),
        onMessage: (event) => console.log('WebSocket message received:', event),
        shouldReconnect: (closeEvent) => true, // Will attempt to reconnect on all close events, such as server shutting down
    });

    useEffect(() => {
        if (readyState === WebSocket.OPEN) {
            sendMessage(JSON.stringify({ message: 'Hello, WebSocket!' }));
        }
    }, [readyState, sendMessage]);

    return (
        <div className="relative h-screen bg-black p-4">
            <div className="absolute top-0 left-0 p-4">
                <HamburgerMenu />
            </div>
            <div className="absolute bottom-0 left-0 flex items-start justify-start w-1/2 h-1/2 p-4">
                <div className="relative w-full h-full flex items-center justify-center">
                    <div className="absolute transform translate-x-[-9rem]">
                        <KillButton />
                    </div>
                </div>
            </div>
            <div className="absolute bottom-0 right-0 flex items-end justify-end w-1/2 h-1/2 p-4">
                <div className="relative w-full h-full flex items-center justify-center">
                    <div className="absolute transform translate-x-full">
                        <Joystick />
                    </div>
                </div>
            </div>
        </div>
    );
};

export default App;
