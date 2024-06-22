import { Order, OrderWithDirtness } from '../components/Columns';

export const getOrders = async (unit_id: number): Promise<OrderWithDirtness[] | string> => {
  try {
    const ordersResponse = await fetch(`http://localhost:8000/api/orders/dirtness/${unit_id}`, {
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

export const createOrder = async (order: Order): Promise<Order | string> => {
  try {
    const createOrderResponse = await fetch(`http://localhost:8000/api/orders/`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(
        {
          status: order.status,
          robot_id: order.robot_id,
          reboiler_id: order.reboiler_id,
          started_at: order.started_at,
          finished_at: order.finished_at,
          Examinations: order.Examinations,
        }
      ),
    });

    if (!createOrderResponse.ok) {
      throw new Error('Failed to fetch data from one or both endpoints');
    }
    return await createOrderResponse.json();
  }
  catch (error) {
    return error instanceof Error ? error.message : String(error);
  }
}
