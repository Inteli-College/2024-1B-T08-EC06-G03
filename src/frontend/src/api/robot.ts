
import { Robot } from '../components/Columns';

export const getRobots = async (unit_id: number): Promise<Robot[] | string> => {
    try {
      console.log(unit_id)
    const robotResponse = await fetch(`http://localhost:8000/api/robots/unit/${unit_id}`, {
      method: 'GET',
      headers: {
        'Cache-Control': 'no-cache',
      },
    });

      if (!robotResponse.ok) {
        throw new Error('Failed to fetch data from one or both endpoints');
      }
      return await robotResponse.json();
    } 
    catch (error) {
      console.error('Error fetching data:', error);
    }
  }
