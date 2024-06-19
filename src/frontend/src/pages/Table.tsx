import React, { useEffect, useState } from 'react';
import { Plus } from 'lucide-react'; 
import {
  Tabs,
  TabsContent,
  TabsList,
  TabsTrigger,
} from '@/components/ui/tabs';
import { Examination, Order, columns } from '../components/Columns';
import { DataTable } from '../components/DataTable';
import { NewProcessDialog } from '@/components/NewProcessDialog';
import { UnitDropdown } from '@/components/UnitDropdown';
import { Button } from '@/components/ui/button';
import Navbar from '../components/Navbar';

const Table: React.FC = () => {
  const [data, setData] = useState<Order[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    async function fetchData() {
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

        const combinedData = orders.map((order: Order) => ({
          ...order,
          examinations: examinations.filter((examination: Examination) => examination.order_id === order.id),
        }));

        setData(combinedData);
        setLoading(false);
      } catch (error) {
        if (error instanceof Error) {
          setError(error.message);
        } else {
          setError(String(error));
        }
        setLoading(false);
      }
    }

    fetchData();
  }, []);

  if (loading) {
    return <div>Loading...</div>;
  }

  if (error) {
    return <div>Error fetching data: {error}</div>;
  }

  return (
    <div>
      <Navbar />
      <div className="container mx-auto py-10 pt-44">
        <div className="mb-4 flex justify-between items-center">
          <UnitDropdown />
          <Button variant="outline" className="w-10 h-10 flex justify-center items-center p-0">
            <Plus className="h-5 w-5" />
          </Button>
        </div>
        <Tabs defaultValue="procedimentos">
          <TabsList className="grid w-full grid-cols-3">
            <TabsTrigger value="procedimentos">Procedimentos</TabsTrigger>
            <TabsTrigger value="robos">Robôs</TabsTrigger>
            <TabsTrigger value="reboilers">Reboilers</TabsTrigger>
          </TabsList>
          <TabsContent value="procedimentos">
            <div className="flex justify-end mb-2 mt-6">
              <NewProcessDialog />
            </div>
            <DataTable columns={columns} data={data} />
          </TabsContent>
          <TabsContent value="robos">
            <div className="flex justify-end mb-2 mt-6">
              <NewProcessDialog />
            </div>
            <DataTable columns={columns} data={data} />
          </TabsContent>
          <TabsContent value="reboilers">
            <div className="flex justify-end mb-2 mt-6">
              <NewProcessDialog />
            </div>
            <DataTable columns={columns} data={data} />
          </TabsContent>
        </Tabs>
      </div>
    </div>
  );
};

export default Table;
