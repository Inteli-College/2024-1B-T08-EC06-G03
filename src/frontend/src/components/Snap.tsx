import React from 'react';
import '../styles/kill.css';
import CameraAltIcon from '@mui/icons-material/CameraAlt';
import { ToastContainer, toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';

interface SnapButtonProps {
    examinationId: string;
    image: string;
}

const SnapButton: React.FC<SnapButtonProps> = ({ examinationId, image }) => {
    const handleClick = () => {
        console.log(`Sending snap with id ${examinationId}`)

        fetch(`http://${window.location.hostname}:8000/api/tube-states`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                taken_at: Math.floor(Date.now() / 1000),
                examination_id: parseInt(examinationId),
                image,
            }),
        })
            .then(response => response.json())
            .then(result => {
                console.log('Fetch result:', result);
                if (result.error) {
                    toast.error(`${result.error}`);
                    return;
                }
                if (result.dirtness) {
                    toast.info("Análise completa: sujo!");
                    return;
                }
                toast.info("Análise completa: limpo!");

            })
            .catch(error => {
                console.error('Error:', error);
            });
        toast.info("Imagem enviada com sucesso!");
    };

    return (
        <>
            <button
                className="bg-gray-500 text-2xl text-white font-bold rounded-full flex items-center justify-center transition-transform transform hover:bg-gray-800"
                style={{ width: '7.5rem', height: '7.5rem' }} // 3/4 of 10rem (joystick's diameter is 10rem)
                onClick={handleClick}
            >
                <CameraAltIcon fontSize="large" />
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
        </>
    );
};

export default SnapButton;
