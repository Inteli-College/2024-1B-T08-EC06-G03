import React, { useEffect, useState } from 'react';
import 'chart.js/auto';

interface CleaningData {
  id: number;
  etapa: string;
  robot_id: number;
  reboiler_id: number;
  started_at: string;
  finished_at: string;
}

const CleaningTable: React.FC = () => {
  const [cleaningData, setCleaningData] = useState<CleaningData[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await fetch('http://localhost:8000/api/examinations'); // Ajuste a URL conforme necessário
        const data = await response.json();
        setCleaningData(data);
        setLoading(false);
      } catch (error) {
        console.error('Error fetching data:', error);
        setLoading(false);
      }
    };

    fetchData();
  }, []);

  return (
    <div className="flex-grow pb-44 pt-12 bg-gray-100">
      <div className="w-2/3 mx-auto bg-green-800 p-6 rounded-lg shadow-lg flex flex-col h-full">
        <h1 className="text-2xl text-white mb-4">Procedimentos de Limpeza</h1>
        <div className="bg-white p-4 rounded-lg flex-grow">
          {loading ? (
            <p>Loading...</p>
          ) : (
            <table className="w-full table-fixed h-full">
              <thead>
                <tr className="bg-green-800 text-white">
                  <th className="w-1/4 p-2">Etapa</th>
                  <th className="w-1/4 p-2">Robot ID</th>
                  <th className="w-1/4 p-2">Reboiler ID</th>
                  <th className="w-1/4 p-2">Started At</th>
                  <th className="w-1/4 p-2">Finished At</th>
                </tr>
              </thead>
              <tbody>
                {cleaningData.map((data) => (
                  <tr key={data.id} className="text-center">
                    <td className="p-2 border">{data.etapa}</td>
                    <td className="p-2 border">{data.robot_id}</td>
                    <td className="p-2 border">{data.reboiler_id}</td>
                    <td className="p-2 border">{new Date(data.started_at).toLocaleString()}</td>
                    <td className="p-2 border">{new Date(data.finished_at).toLocaleString()}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </div>
      </div>
    </div>
  );
};

export default CleaningTable;
