
import { Robot } from '../components/Columns';

export const getRobots = async (): Promise<Robot[] | string> => {
    try {
    const robotResponse = await fetch('http://localhost:8000/api/orders', {
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
      return error instanceof Error ? error.message : String(error);
    }
  }
