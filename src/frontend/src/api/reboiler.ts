import { Reboiler } from '../components/Columns';

export const getReboilers = async (unit_id: number): Promise<Reboiler[] | string> => {
  try {
    const ordersResponse = await fetch(`http://localhost:8000/api/reboilers/unit/${unit_id}`, {
      method: 'GET',
      headers: {
        'Cache-Control': 'no-cache',
      },
    });
    if (!ordersResponse.ok) {
      throw new Error('Failed to fetch data from one or both endpoints');
    }
    return ordersResponse.json()
  }
  catch (error) {
    return error instanceof Error ? error.message : String(error);
  }
}

export const createReboiler = async (reboiler: Reboiler): Promise<Reboiler | string> => {
  try {
    const createReboilerResponse = await fetch(`http://localhost:8000/api/reboilers/`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(
        {
          number: reboiler.number,
          unit_id: reboiler.unit_id,
        }
      ),
    });

    if (!createReboilerResponse.ok) {
      throw new Error('Failed to fetch data from one or both endpoints');
    }
    return await createReboilerResponse.json();
  }
  catch (error) {
    console.error('Error fetching data:', error);
    return error instanceof Error ? error.message : String(error);
  }
}