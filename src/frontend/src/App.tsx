import React from 'react';
import useWebSocket from 'react-use-websocket';
import Joystick from './components/Joystick'; // Adjust the path based on your project structure
import KillButton from './components/Kill'; // Import the KillButton component
import HamburgerMenu from './components/HamburguerMenu'; // Import the HamburgerMenu component


const WS_URL = 'ws://127.0.0.1:8000';

const App: React.FC = () => {

  useWebSocket(WS_URL, {
    onOpen: () => {
      console.log('WebSocket connection established.');
    }
  });

  
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



