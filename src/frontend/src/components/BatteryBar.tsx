import React from 'react';

interface BatteryBarProps {
  batteryPercentage: number;
}

const BatteryBar: React.FC<BatteryBarProps> = ({ batteryPercentage }) => {
  const batteryLevel = batteryPercentage * 100;

  const getBatteryColor = () => {
    if (batteryLevel > 65) return '#4caf50'; // Verde
    if (batteryLevel > 20) return '#ffeb3b'; // Amarelo
    return '#f44336'; // Vermelho
  };

  return (
    <div
      className="fixed top-2.5 right-2.5 z-50 text-center max-w-[150px] p-1.5"
    >
      <div
        className="w-[130px] bg-gray-300 h-[30px] rounded overflow-hidden mb-2.5 border border-black border-2"
      >
        <div
          className={`h-full transition-all duration-500 ease-in-out`}
          style={{
            width: `${batteryLevel}%`,
            backgroundColor: getBatteryColor(), // Continua utilizando estilo inline para cor dinâmica
          }}
        />
      </div>
      <p className="font-bold text-white m-0">
        {`${batteryLevel.toFixed(0)}%`}
      </p>
    </div>
  );}
  

export default BatteryBar;
