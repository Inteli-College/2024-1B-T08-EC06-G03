/// Table.tsx (or Table.js)
import React, { useEffect, useState } from 'react';
import {
  Tabs,
  TabsContent,
  TabsList,
  TabsTrigger,
} from "@/components/ui/tabs";
import { Process, columns } from "../components/Columns";
import { DataTable } from "../components/DataTable";
import { NewProcessDialog } from "@/components/NewProcessDialog"
import { UnitDropdown } from "@/components/UnitDropdown"
import Navbar from '../components/Navbar';
import App from "@/App.tsx"

async function getData(): Promise<Process[]> {
  // Fetch data from your API here.
  return [
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    {
      etapa: "pré",
      robo: "RoboBolin",
      reboiler: "pending",
      data: "2021-10-01T00:00:00Z",
      sujidade: 20,
    },
    // ...
  ];
}

const Table: React.FC = () => {
  const [data, setData] = useState<Process[]>([]);
  const [loading, setLoading] = useState<boolean>(true);

  useEffect(() => {
    async function fetchData() {
      const result = await getData();
      setData(result);
      setLoading(false);
    }
    fetchData();
  }, []);

  if (loading) {
    return <div>Loading...</div>;
  }

  return (
    <div>
      <Navbar />
      <div className="container mx-auto py-10 pt-32">
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
