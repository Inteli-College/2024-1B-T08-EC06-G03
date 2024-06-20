import React, { useEffect, useState } from 'react';
import { Plus } from 'lucide-react'; 
import {
  Tabs,
  TabsContent,
  TabsList,
  TabsTrigger,
} from '@/components/ui/tabs';
import { Robot, Order, Reboiler, columnsExamination, columnsRobot, columnsReboiler } from '../components/Columns';
import { getOrders } from '@/api/orders';
import { getRobots, createRobot } from '@/api/robot';
import { getReboilers } from '@/api/reboiler';
import { DataTable } from '../components/DataTable';
import { NewProcessDialog } from '@/components/NewProcessDialog';
import  UnitDropdown from '../components/UnitDropdown';
import { Button } from '@/components/ui/button';
import Navbar from '../components/Navbar';
import SelectOptions from '@/components/Select-options';
import Modal_template, { SubmitFunction } from "../components/Modal_template"
import { create } from 'domain';
import { Input } from '@/components/ui/input';
import { toast } from 'react-toastify';
import { ColumnDef } from '@tanstack/react-table';


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
  return {...data};
}

const Table: React.FC = () => {
  const [data, setData] = useState<Order[] | Robot[] | Reboiler[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [step, setStep] = useState<string>("");
  const [tabSelected, setTabSelected] = useState<string>("procedimentos");
  const [typeTable, setTypeTable] = useState<ColumnDef<Order | Robot | Reboiler>[] | null>();
  const [unit, setUnit] = useState<number>(1);


  useEffect(() => {
    console.log(tabSelected)
    if (tabSelected === "procedimentos") {
      setLoading(true);
      setTypeTable(columnsExamination as ColumnDef<Order | Robot | Reboiler>[]);
      fetchOrders();
    }
    else if (tabSelected === "robos") {
      setLoading(true);
      setTypeTable(columnsRobot as ColumnDef<Order | Robot | Reboiler>[]);
      fetchRobots();
    }
    else if (tabSelected === "reboilers") {
      setLoading(true);
      setTypeTable(columnsReboiler as ColumnDef<Order | Robot | Reboiler>[]);
      fetchReboilers();
    }
  }, [tabSelected, unit]);

  const fetchOrders = async () => {
    try {
      const orders: Order[] | string = await getOrders(unit);
      if (typeof orders === "string") {
        setError(orders);
      } else {
        setData(orders);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
      setError('An error occurred while fetching data.'); 
    } finally {
      setLoading(false);
    }
  }; 

  const fetchRobots = async () => {
    try {
      const robots: Robot[] | string = await getRobots(unit);
      if (typeof robots === "string") {
        setError(robots);
      } else {
        setData(robots);
      }
      console.log(robots)
    } catch (error) {
      console.error('Error fetching data:', error);
      setError('An error occurred while fetching data.'); 
    } finally {
      setLoading(false);
    }
  }; 

  const registerRobot = async (robot : Robot)=> {
    try{
      const newrobot: Robot | string = await createRobot(robot);
      console.log(newrobot)    
    } catch (error) {
      console.error('Error fetching data:', error);
      setError('An error occurred while fetching data.'); 
    } finally {
      setLoading(false);      return robot
    }
  }; 


  const fetchReboilers = async () => {
    try {
      const reboilers: Reboiler[] | string = await getReboilers(unit);
      if (typeof reboilers === "string") {
        setError(reboilers);
      } else {
        setData(reboilers);
      }
      console.log(reboilers)    } catch (error) {
      console.error('Error fetching data:', error);
      setError('An error occurred while fetching data.'); 
    } finally {
      setLoading(false);
    }
  }; 

  useEffect(() => {
    setLoading(true);
    fetchOrders();
  }, []);



  if (error) {
    return <div>Error fetching data: {error}</div>;
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
    let newrobotdata = {
      nickname: formData.get("nickname") as string,
      unit_id: unit,
      id: null
    }
    console.log(newrobotdata)
    const robotRegistered = registerRobot(newrobotdata);
    if (robotRegistered !== null){
      toast.success("Robô cadastrado com sucesso")
    }
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
      <div className="container mx-auto py-10 pt-44">
        <div className="mb-4 flex justify-between items-center">
          <UnitDropdown selected={unit} setSelected={setUnit} />
          <Button variant="outline" className="w-10 h-10 flex justify-center items-center p-0">
            <Plus className="h-5 w-5" />
          </Button>
        </div>
        <Tabs defaultValue="procedimentos">
          <TabsList className="grid w-full grid-cols-3">
            <TabsTrigger value="procedimentos" onFocus={()=>setTabSelected("procedimentos")}>Procedimentos</TabsTrigger>
            <TabsTrigger value="robos" onFocus={()=>setTabSelected("robos")} >Robôs</TabsTrigger>
            <TabsTrigger value="reboilers" onFocus={()=>setTabSelected("reboilers")}>Reboilers</TabsTrigger>
          </TabsList>
          {loading && ( 
             <div><br/>Loading...</div>
          )}
          {!loading && (<>
          <TabsContent value="procedimentos">
            <div className="flex justify-end mb-2 mt-6">
              <Modal_template
                className= "new_procedure_modal" 
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
            <DataTable columns={typeTable} data={data} />
          </TabsContent>
          <TabsContent value="robos">
            <div className="flex justify-end mb-2 mt-6">
              <Modal_template
                className= "new_robot_modal"  
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
            <DataTable columns={typeTable} data={data} />
          </TabsContent>
          <TabsContent value="reboilers">
            <div className="flex justify-end mb-2 mt-6">
              <Modal_template
                className= "new_reboiler_modal" 
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
            <DataTable columns={typeTable} data={data} />
          </TabsContent>
          </>
          )}
        </Tabs>
      </div>
    </div>
  );
};

export default Table;
