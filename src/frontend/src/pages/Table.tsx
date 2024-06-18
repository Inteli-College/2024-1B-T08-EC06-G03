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
import SelectOptions from '@/components/Select-options';
import Navbar from '../components/Navbar';
import App from "@/App.tsx"
import Modal_template, { SubmitFunction } from "../components/Modal_template"
import { create } from 'domain';
import { Input } from '@/components/ui/input';

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


async function createRobot(data: any ) {
  // Fetch data from your API here.
  console.log(data)
}

async function createProcedure(data: any ) {
  // Fetch data from your API here.
  console.log(data)
}

async function createReboiler(data: any ) {
  // Fetch data from your API here.
  console.log(data)
}

function formDataToObject(formData: FormData): { [key: string]: any } {
  const data: { [key: string]: any } = {};
  formData.forEach((value, key) => {
    data[key] = value;
  });
  return data;
}

const Table: React.FC = () => {
  const [data, setData] = useState<Process[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [step, setStep] = useState<string>("");

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

  const newProcedure: SubmitFunction = (event) => {
    event.preventDefault();

    const formData = new FormData(event.currentTarget);
    formData.append("step", step);
    const data = formDataToObject(formData);
    createProcedure(data);
  }

  const newRobot: SubmitFunction = (event) => {
    event.preventDefault();
    const formData = new FormData(event.currentTarget);
    const data = formDataToObject(formData);
    createRobot(data);
  }

  const newReboiler: SubmitFunction = (event) => {
    event.preventDefault();
    const formData = new FormData(event.currentTarget);
    const data = formDataToObject(formData);
    createReboiler(data);
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
              <Modal_template 
                title={"Cadastrar procedimento"}
                button_label="cadastrar procedimento" 
                children={
                  <div>
                    <div>
                      <label>Etapa</label>
                      <br/>
                      <SelectOptions options={["pré-limpeza", "pós-limpeza"]} selected={step} setSelected={setStep}/>
                    </div>
                    <div>
                      <label>Reboiler</label>
                      <br/>
                      <Input type="text" name='reboiler' placeholder="Insira o reboiler a ser inspecionado" />
                      <br/>
                      <label>Robô</label>
                      <Input type="text" name="robot" placeholder="Insira o robô a ser inspecionado"/>
                      <br/>
                    </div>
                  </div>} 
                submit_action={newProcedure}
                isOpen={true}>
              </Modal_template>
            </div>
            <DataTable columns={columns} data={data} />
          </TabsContent>
          <TabsContent value="robos">
            <div className="flex justify-end mb-2 mt-6">
              <Modal_template 
                title={"Cadastrar robô"}
                button_label="cadastrar robô" 
                children={
                <div>
                  <div>
                    <label>Apelido</label>
                    <br/>
                    <Input type="text" name='nickname' placeholder="Insira o apelido do robô a ser cadastrado"/>
                    <br/>
                  </div>
                </div>
                } 
                submit_action={newRobot}
                isOpen={true}>
              </Modal_template>
            </div>
            <DataTable columns={columns} data={data} />
          </TabsContent>
          <TabsContent value="reboilers">
            <div className="flex justify-end mb-2 mt-6">
              <Modal_template 
                title={"Cadastrar reboiler"}
                button_label="Cadastrar reboiler" 
                children={
                <div>
                  <div>
                    <label>Número</label>
                    <br/>
                    <Input type="text" name='number' placeholder="Insira o número do reboiler ser cadastrado"/>
                    <br/>
                  </div>
                </div>
                } 
                submit_action={newReboiler}
                isOpen={true}>
              </Modal_template>
            </div>
            <DataTable columns={columns} data={data} />
          </TabsContent>
        </Tabs>
        
        
      </div>
    </div>
  );
};

export default Table;
