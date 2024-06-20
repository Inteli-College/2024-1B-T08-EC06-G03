
import { Examination, Order } from '../components/Columns';

export const getOrders = async (): Promise<Order[] | string> => {
    try {
      const [ordersResponse, examinationsResponse] = await Promise.all([
        fetch('http://localhost:8000/api/orders', {
          method: 'GET',
          headers: {
            'Cache-Control': 'no-cache',
          },
        }),
        fetch('http://localhost:8000/api/examinations', {
          method: 'GET',
          headers: {
            'Cache-Control': 'no-cache',
          },
        }),
      ]);

      if (!ordersResponse.ok || !examinationsResponse.ok) {
        throw new Error('Failed to fetch data from one or both endpoints');
      }

      const orders = await ordersResponse.json();
      const examinations = await examinationsResponse.json();

      return orders.map((order: Order) => ({
        ...order,
        examinations: examinations.filter((examination: Examination) => examination.order_id === order.id),
      }));

    } catch (error) {
      return error instanceof Error ? error.message : String(error);
    }
  }
