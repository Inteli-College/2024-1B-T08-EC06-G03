import React, { useEffect, useState } from 'react';
import { Plus } from 'lucide-react';
import {
  Tabs,
  TabsContent,
  TabsList,
  TabsTrigger,
} from '@/components/ui/tabs';
import { Robot, Order, Reboiler, OrderWithDirtness, ExaminationWithDirtness, columnsExamination, columnsRobot, columnsReboiler } from '../components/Columns';
import { getOrders, createOrder } from '@/api/orders';
import { getRobots, createRobot } from '@/api/robot';
import { getReboilers, createReboiler } from '@/api/reboiler';
import { DataTable } from '../components/DataTable';
import UnitDropdown from '../components/UnitDropdown';
import RobotDropdown from '@/components/robotDropdown';
import ReboilerDropdown from '@/components/reboilerDropdown';
import { Button } from '@/components/ui/button';
import Navbar from '../components/Navbar';
import SelectOptions from '@/components/Select-options';
import Modal_template, { SubmitFunction } from "../components/Modal_template"
import { Input } from '@/components/ui/input';
import { toast } from 'react-toastify';
import { ColumnDef } from '@tanstack/react-table';


const Table: React.FC = () => {
  const [data, setData] = useState<Order[] | Robot[] | Reboiler[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [tabSelected, setTabSelected] = useState<string>("procedimentos");
  const [typeTable, setTypeTable] = useState<ColumnDef<OrderWithDirtness | Robot | Reboiler>[] | null>();
  const [unit, setUnit] = useState<number>(1);
  const [reboiler_selected, setReboilerSelected] = useState<number>(0);
  const [robot_selected, setRobotSelected] = useState<number>(1);

  useEffect(() => {
    if (tabSelected === "procedimentos") {
      setLoading(true);
      setTypeTable(columnsExamination as ColumnDef<OrderWithDirtness | Robot | Reboiler>[]);
      fetchOrders();
    }
    else if (tabSelected === "robos") {
      setLoading(true);
      setTypeTable(columnsRobot as ColumnDef<OrderWithDirtness | Robot | Reboiler>[]);
      fetchRobots();
    }
    else if (tabSelected === "reboilers") {
      setLoading(true);
      setTypeTable(columnsReboiler as ColumnDef<OrderWithDirtness | Robot | Reboiler>[]);
      fetchReboilers();
    }
  }, [tabSelected, unit]);

  const fetchOrders = async () => {
    try {
      const orders: OrderWithDirtness[] | string = await getOrders(unit);
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

  const registerOrder = async (order: Order) => {
    try {
      const neworder: Order | string = await createOrder(order);
      console.log(neworder)
    } catch (error) {
      console.error('Error fetching data:', error);
      setError('An error occurred while fetching data.');
    } finally {
      setLoading(false);
      return order
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

  const registerRobot = async (robot: Robot) => {
    try {
      const newrobot: Robot | string = await createRobot(robot);
      console.log(newrobot)
    } catch (error) {
      console.error('Error fetching data:', error);
      setError('An error occurred while fetching data.');
    } finally {
      setLoading(false);
      return robot
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
      console.log(reboilers)
    } catch (error) {
      console.error('Error fetching data:', error);
      setError('An error occurred while fetching data.');
    } finally {
      setLoading(false);
    }
  };

  const registerReboiler = async (reboiler: Reboiler) => {
    try {
      const newreboiler: Reboiler | string = await createReboiler(reboiler);
      console.log(newreboiler)
    } catch (error) {
      console.error('Error fetching data:', error);
      setError('An error occurred while fetching data.');
    } finally {
      setLoading(false);
      return reboiler
    }
  };

  useEffect(() => {
    setLoading(true);
    fetchOrders();
  }, []);



  if (error) {
    return <div>Error fetching data: {error}</div>;
  }

  const newOrder: SubmitFunction = (event) => {
    event.preventDefault();
    const formData = new FormData(event.currentTarget);
    console.log(formData)
    let neworderdata = {
      id: null,
      status: "active",
      robot_id: robot_selected,
      reboiler_id: reboiler_selected,
      Examinations: [],
      started_at: Math.floor(Date.now() / 1000),
      finished_at: null,
    };
    console.log(neworderdata)
    const robotRegistered = registerOrder(neworderdata);
    if (robotRegistered !== null) {
      toast.success("Procedimento cadastrado com sucesso")
    }
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
    if (robotRegistered !== null) {
      toast.success("Robô cadastrado com sucesso")
    }
  }

  const newReboiler: SubmitFunction = (event) => {
    event.preventDefault();
    const formData = new FormData(event.currentTarget);
    let newreboilerdata = {
      number: formData.get("number_reboiler") as unknown as number,
      unit_id: unit,
      id: null
    }
    console.log(newreboilerdata)
    const reboilerRegistered = registerReboiler(newreboilerdata);
    if (reboilerRegistered !== null) {
      toast.success("Reboiler cadastrado com sucesso")
    }
  }


  return (
    <div>
      <Navbar />
      <div className="container mx-auto py-10 pt-44">
        <div className="mb-4 flex justify-between items-center">
          <UnitDropdown selected={unit} setSelected={() => setUnit} />
        </div>
        <Tabs defaultValue="procedimentos">
          <TabsList className="grid w-full grid-cols-3">
            <TabsTrigger value="procedimentos" onFocus={() => setTabSelected("procedimentos")}>Procedimentos</TabsTrigger>
            <TabsTrigger value="robos" onFocus={() => setTabSelected("robos")} >Robôs</TabsTrigger>
            <TabsTrigger value="reboilers" onFocus={() => setTabSelected("reboilers")}>Reboilers</TabsTrigger>
          </TabsList>
          {loading && (
            <div><br />Loading...</div>
          )}
          {!loading && (<>
            <TabsContent value="procedimentos">
              <div className="flex justify-end mb-2 mt-6">
                <Modal_template
                  className="new_procedure_modal"
                  title={"Cadastrar procedimento"}
                  button_label="cadastrar procedimento"
                  children={
                    <div>
                      <div>
                        <label>Reboiler</label>
                        <br />
                        <ReboilerDropdown name="reboiler_number" unit={unit} selected={reboiler_selected} setSelected={(reboiler) => setReboilerSelected(reboiler as unknown as number)} />
                        <br />
                        <label>Robô</label>
                        <RobotDropdown name="robot_number" unit={unit} selected={robot_selected} setSelected={(robot) => setRobotSelected(robot as unknown as number)} />
                        <br />
                      </div>
                    </div>}
                  submit_action={newOrder}
                  isOpen={true}>
                </Modal_template>
              </div>
              <DataTable columns={typeTable} data={data} />
            </TabsContent>
            <TabsContent value="robos">
              <div className="flex justify-end mb-2 mt-6">
                <Modal_template
                  className="new_robot_modal"
                  title={"Cadastrar robô"}
                  button_label="cadastrar robô"
                  children={
                    <div>
                      <div>
                        <label>Apelido</label>
                        <br />
                        <Input
                          type="text"
                          name='nickname'
                          placeholder="Insira o apelido do robô a ser cadastrado"
                          onKeyPress={(e) => {
                            if (e.key === 'Enter') {
                              e.preventDefault();
                            }
                          }} />
                        <br />
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
                  className="new_reboiler_modal"
                  title={"Cadastrar reboiler"}
                  button_label="Cadastrar reboiler"
                  children={
                    <div>
                      <div>
                        <label>Número</label>
                        <br />
                        <Input type="number" name='number_reboiler' placeholder="Insira o número do reboiler ser cadastrado" />
                        <br />
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
