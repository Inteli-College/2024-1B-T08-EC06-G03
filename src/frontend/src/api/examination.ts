import { Examination } from '../components/Columns';

export const getExaminationByID = async (id: number): Promise<Examination | string> => {
  const API_URL = `http://${window.location.hostname}:8000`;

  try {
    const ordersResponse = await fetch(`${API_URL}/api/examinations/${id}`, {
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
