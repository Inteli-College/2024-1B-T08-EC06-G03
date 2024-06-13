import React from 'react';
import '../styles/kill.css';
import { ToastContainer, toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';

interface KillButtonProps {
    sendMessage: (message: string) => void;
}

const KillButton: React.FC<KillButtonProps> = ({ sendMessage }) => {
    const notify = () => toast.error("O botão de matar processo foi acionado");
    const handleClick = () => {
        sendMessage(JSON.stringify({ kill_robot: 'kill' }));
    };

    return (
        <div>
            <button
                className="bg-red-500 text-2xl text-white font-bold rounded-full flex items-center justify-center transition-transform transform hover:bg-red-800"
                style={{ width: '7.5rem', height: '7.5rem' }} // 3/4 of 10rem (joystick's diameter is 10rem)
                onClick={() => {
                    handleClick();
                    notify();
                }}
            >
                KILL
            </button>
            <ToastContainer 
                position="bottom-center"
                autoClose={5000}
                hideProgressBar={false}
                newestOnTop={false}
                closeOnClick
                rtl={false}
                pauseOnFocusLoss
                draggable
                pauseOnHover
                theme="colored"
                />
            {/* Same as */}
        </div>

    );
};

export default KillButton;
