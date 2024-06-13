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
    <div style={{ 
      position: 'fixed', 
      top: '10px', 
      right: '10px', 
      zIndex: 1000, 
      textAlign: 'center',
      maxWidth: '150px',
      padding: '5px', // Espaçamento interno
    }}>
      <div
        style={{
          width: '100%',
          backgroundColor: '#ccc',
          height: '30px',
          borderRadius: '5px',
          overflow: 'hidden',
          marginBottom: '10px',
          border: '1px solid black', // Bordas da barra
          borderWidth: '2px', // Espessura da borda
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
      <p style={{ margin: 0, color: 'red' }}>{`Nível da bateria: ${batteryLevel.toFixed(0)}%`}</p> {/* Estiliza o texto em vermelho */}
    </div>
  );
};

export default BatteryBar;
