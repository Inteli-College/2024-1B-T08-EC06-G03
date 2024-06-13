import { ColumnDef } from "@tanstack/react-table"

// This type is used to define the shape of our data.
// You can use a Zod schema here if you want.
export type Process = {
  etapa: "pré" | "pós"
  robo: string
  reboiler: string
  data: string
  sujidade: number
}

export const columns: ColumnDef<Process>[] = [
  {
    accessorKey: "etapa",
    header: "Etapa",
  },
  {
    accessorKey: "robo",
    header: "Robô",
  },
  {
    accessorKey: "reboiler",
    header: "Reboiler",
  },
  {
    accessorKey: "data",
    header: "Data",
  },
  {
    accessorKey: "sujidade",
    header: "Sujidade (%)",
  },
]
