
import { Examination, Order } from '../components/Columns';

export const getOrders = async (unit_id: number): Promise<Order[] | string> => {
    try {
      const ordersResponse= await fetch(`http://localhost:8000/api/orders/unit/${unit_id}`, {
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
