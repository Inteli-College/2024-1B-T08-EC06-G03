import React from 'react';

interface BatteryBarProps {
  batteryPercentage: number;
}

const BatteryBar: React.FC<BatteryBarProps> = ({ batteryPercentage }) => {
  const batteryLevel = batteryPercentage * 100;

  const getBatteryColor = () => {
    if (batteryLevel > 50) return '#4caf50'; // Verde
    if (batteryLevel > 20) return '#ffeb3b'; // Amarelo
    return '#f44336'; // Vermelho
  };

  return (
    <div style={{ width: '100%', maxWidth: '400px', margin: '0 auto', textAlign: 'center' }}>
      <div
        style={{
          width: '100%',
          backgroundColor: '#ccc',
          height: '30px',
          borderRadius: '5px',
          overflow: 'hidden',
          marginBottom: '10px',
        }}
      >
        <div
          style={{
            width: `${batteryLevel}%`,
            backgroundColor: getBatteryColor(),
            height: '100%',
            transition: 'width 0.5s ease-in-out',
          }}
        />
      </div>
      <p>{`Nível da bateria: ${batteryLevel.toFixed(0)}%`}</p>
    </div>
  );
};

export default BatteryBar;
