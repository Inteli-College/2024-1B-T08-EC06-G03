import React from 'react';
import '../styles/kill.css';
import CameraAltIcon from '@mui/icons-material/CameraAlt';

interface SnapButtonProps {
    sendMessage: (message: string) => void;
}

const SnapButton: React.FC<SnapButtonProps> = ({ sendMessage }) => {
    const handleClick = () => {
        sendMessage(JSON.stringify({ kill_robot: 'snap' }));
    };

    return (
        <button
            className="bg-gray-500 text-2xl text-white font-bold rounded-full flex items-center justify-center transition-transform transform hover:bg-gray-800"
            style={{ width: '7.5rem', height: '7.5rem' }} // 3/4 of 10rem (joystick's diameter is 10rem)
            onClick={handleClick}
        >
            <CameraAltIcon fontSize="large" />
        </button>
    );
};

export default SnapButton;
