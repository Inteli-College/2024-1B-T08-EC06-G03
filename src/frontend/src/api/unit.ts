import { Unit } from '../components/Columns';

export const getUnities = async (): Promise<Unit[] | string> => {
  try {
    const unitResponse = await fetch(`http://localhost:8000/api/unities`, {
      method: 'GET',
      headers: {
        'Cache-Control': 'no-cache',
      },
    });

    if (!unitResponse.ok) {
      throw new Error('Failed to fetch data from one or both endpoints');
    }
    return await unitResponse.json();
  }
  catch (error) {
    return error instanceof Error ? error.message : String(error);
  }
}
