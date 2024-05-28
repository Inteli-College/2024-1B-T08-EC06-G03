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
        return 'absolute w-3/4 h-96 bg-red-500 opacity-40 rounded-full -top-64 left-1/2 transform -translate-x-1/2';
      case 'front-right':
        return 'absolute w-96 h-96 bg-red-500 opacity-40 rounded-full -top-40 -right-40';
      case 'right':
        return 'absolute w-96 h-3/4 bg-red-500 opacity-40 rounded-full top-1/2 -right-64 transform -translate-y-1/2';
      case 'back-right':
        return 'absolute w-96 h-96 bg-red-500 opacity-40 rounded-full -bottom-40 -right-40';
      case 'back':
        return 'absolute w-3/4 h-96 bg-red-500 opacity-40 rounded-full -bottom-64 left-1/2 transform -translate-x-1/2';
      case 'back-left':
        return 'absolute w-96 h-96 bg-red-500 opacity-40 rounded-full -bottom-40 -left-40';
      case 'left':
        return 'absolute w-96 h-3/4 bg-red-500 opacity-40 rounded-full top-1/2 -left-64 transform -translate-y-1/2';
      case 'front-left':
        return 'absolute w-96 h-96 bg-red-500 opacity-40 rounded-full -top-40 -left-40';
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
