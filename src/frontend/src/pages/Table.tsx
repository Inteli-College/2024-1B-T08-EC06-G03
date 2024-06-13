import { Process, columns } from "../components/Columns"
import { DataTable } from "../components/DataTable"
 
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
    // ...
  ]
}
 
export default async function DemoPage() {
  const data = await getData()
 
  return (
    <div className="container mx-auto py-10">
      <DataTable columns={columns} data={data} />
    </div>
  )
}