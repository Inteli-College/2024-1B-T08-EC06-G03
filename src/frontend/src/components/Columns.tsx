import { ColumnDef } from "@tanstack/react-table"
import { MoreHorizontal } from "lucide-react"
 
import { Button } from "@/components/ui/button"
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu"

import { NewProcessDialog } from "./NewProcessDialog"


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
  {
    id: "actions",
    cell: ({ row }) => {
      const process = row.original
 
      return (
        <DropdownMenu>
          <DropdownMenuTrigger asChild>
            <Button variant="ghost" className="h-8 w-8 p-0">
              <span className="sr-only">Open menu</span>
              <MoreHorizontal className="h-4 w-4" />
            </Button>
          </DropdownMenuTrigger>
          <DropdownMenuContent align="end">
            <DropdownMenuLabel>Actions</DropdownMenuLabel>
            <DropdownMenuItem
              onClick={() => navigator.clipboard.writeText(process.robo)}
            >
              Copy payment ID
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            <DropdownMenuItem>View customer</DropdownMenuItem>
            <DropdownMenuItem>View payment details</DropdownMenuItem>
          </DropdownMenuContent>
        </DropdownMenu>
      )
    },
  },
]
