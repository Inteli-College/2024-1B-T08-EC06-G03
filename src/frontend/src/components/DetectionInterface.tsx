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
        return 'top-0 left-1/2 transform -translate-x-1/2';
      case 'front-right':
        return 'top-0 right-0';
      case 'right':
        return 'top-1/2 right-0 transform -translate-y-1/2';
      case 'back-right':
        return 'bottom-0 right-0';
      case 'back':
        return 'bottom-0 left-1/2 transform -translate-x-1/2';
      case 'back-left':
        return 'bottom-0 left-0';
      case 'left':
        return 'top-1/2 left-0 transform -translate-y-1/2';
      case 'front-left':
        return 'top-0 left-0';
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
          className={`absolute w-20 h-20 bg-red-500 rounded-full ${getIndicatorPosition(direction)}`}
        ></div>
      ))}
    </div>
    )}
    </>
  );
};

export default DetectionInterface;
