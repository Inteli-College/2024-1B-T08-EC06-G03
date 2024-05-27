import React from 'react';
import '../styles/kill.css';

interface KillButtonProps {
    sendMessage: (message: string) => void;
}

const KillButton: React.FC<KillButtonProps> = ({ sendMessage }) => {
    const handleClick = () => {
        sendMessage(JSON.stringify({ kill_robot: 'kill' }));
    };

    return (
        <button
            className="bg-red-500 text-2xl text-white font-bold rounded-full flex items-center justify-center transition-transform transform hover:bg-red-800"
            style={{ width: '7.5rem', height: '7.5rem' }} // 3/4 of 10rem (joystick's diameter is 10rem)
            onClick={handleClick}
        >
            KILL
        </button>
    );
};

export default KillButton;
