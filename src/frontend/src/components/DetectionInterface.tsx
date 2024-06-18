import React from 'react';

type Direction =
    | 'front'
    | 'front-right'
    | 'right'
    | 'back-right'
    | 'back'
    | 'back-left'
    | 'left'
    | 'front-left';

interface DetectionInterfaceProps {
    directions: Direction[];
}

const DetectionInterface: React.FC<DetectionInterfaceProps> = ({ directions }) => {
    const getIndicatorPosition = (direction: Direction) => {
        switch (direction) {
            case 'front':
                return 'absolute w-full h-16 bg-red-500 opacity-40 top-0 left-1/2 transform -translate-x-1/2';
            case 'front-right':
                return 'absolute w-0 h-0 opacity-40 border-b-[200px] border-b-transparent border-r-[200px] border-r-red-500 top-0 right-0';
            case 'right':
                return 'absolute w-16 h-full bg-red-500 opacity-40 top-1/2 right-0 transform -translate-y-1/2';
            case 'back-right':
                return 'absolute w-0 h-0 opacity-40 border-t-[200px] border-t-transparent border-r-[200px] border-r-red-500 bottom-0 right-0';
            case 'back':
                return 'absolute w-full h-16 bg-red-500 opacity-40 bottom-0 left-1/2 transform -translate-x-1/2';
            case 'back-left':
                return 'absolute w-0 h-0 opacity-40 border-t-[200px] border-t-transparent border-l-[200px] border-l-red-500 bottom-0 left-0';
            case 'left':
                return 'absolute w-16 h-full bg-red-500 opacity-40 top-1/2 left-0 transform -translate-y-1/2';
            case 'front-left':
                return 'absolute w-0 h-0 opacity-40 border-b-[200px] border-b-transparent border-l-[200px] border-l-red-500 top-0 left-0';
            default:
                return '';
        }
    };

    return (
        <>
            {directions.length != 0 && (
                <div>
                    {directions.map((direction, index) => (
                        <div
                            key={index}
                            className={`${getIndicatorPosition(direction)}`}
                        ></div>
                    ))}
                </div>
            )}
        </>
    );
};

export default DetectionInterface;
