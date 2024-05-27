import React, { useState } from 'react';
import DetectionInterface from '../components/DetectionInterface';

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
  const [directions, setDirections] = useState<Direction[]>(['front']);

  const addDirection = (newDirection: Direction) => {
    if (!directions.includes(newDirection)) {
      setDirections([...directions, newDirection]);
    }
  };

  const removeDirection = (directionToRemove: Direction) => {
    setDirections(directions.filter(direction => direction !== directionToRemove));
  };

  return (
    <div className="flex flex-col items-center justify-center h-screen">
      <div className="relative w-64 h-64 border border-gray-300">
        <DetectionInterface directions={directions} />
      </div>
      <div className="mt-4">
        <select
          onChange={(e) => addDirection(e.target.value as Direction)}
          className="p-2 border border-gray-300 rounded"
        >
          <option value="" disabled selected>Add Direction</option>
          <option value="front">Front</option>
          <option value="front-right">Front Right</option>
          <option value="right">Right</option>
          <option value="back-right">Back Right</option>
          <option value="back">Back</option>
          <option value="back-left">Back Left</option>
          <option value="left">Left</option>
          <option value="front-left">Front Left</option>
        </select>
      </div>
      <div className="mt-4">
        {directions.map(direction => (
          <button
            key={direction}
            onClick={() => removeDirection(direction)}
            className="m-1 p-2 bg-red-500 text-white rounded"
          >
            {direction}
          </button>
        ))}
      </div>
    </div>
  );
};

export default App;
