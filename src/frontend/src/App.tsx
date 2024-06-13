import React, { useEffect, useState, useCallback, useRef } from 'react';
import useWebSocket from 'react-use-websocket';
import Joystick from './components/Joystick'; // Adjust the path based on your project structure
import KillButton from './components/Kill'; // Import the KillButton component
import SnapButton from './components/Snap';
import HamburgerMenu from './components/HamburgerMenu';
import DetectionInterface from './components/DetectionInterface';
import BatteryBar from './components/BatteryBar';

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
    const [cameraData, setCameraData] = useState<any>({});
    const [cameraSocketUrl, setCameraSocketUrl] = useState<string | null>(null);
    const [image, setImage] = useState<string | null>(null);
    const [batteryPercentage, setBatteryPercentage] = useState<number>(0.5); // Inicia com 50%

    useEffect(() => {
        const interval = setInterval(() => {
            setBatteryPercentage((prev) => {
            const newPercentage = prev + (Math.random() * 0.1 - 0.05); // Variação aleatória
            return Math.max(0, Math.min(1, newPercentage)); // Mantém entre 0 e 1
            });
        }, 2000); // Atualiza a cada 2 segundos
    
        return () => clearInterval(interval); // Limpa o intervalo ao desmontar
        }, []);
    

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
        <BatteryBar batteryPercentage={batteryPercentage} stroke-width="100"/>

    </div>
    );

    
};


export default App;
