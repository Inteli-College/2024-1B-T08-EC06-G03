// Columns.ts

import { ColumnDef } from "@tanstack/react-table";
import { MoreHorizontal } from "lucide-react";

import { Button } from "@/components/ui/button";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";

export type Examination = {
  id: number;
  step: "Pré" | "Pós";
  started_at: number;
  finished_at: number;
  order_id: number;
};

export type Order = {
  id: number;
  status: string;
  robot_id: number;
  reboiler_id: number;
  started_at: number;
  finished_at: number;
  Examinations: Examination[];
};

export type Robot = {
  id: number;
  nickname: string;
  unit_id: number;
}

export type Reboiler = {
  id: number;
  number: number;
}

export type Unit = {
  id: number;
  city: string;
  state: string;
}

export type dropdown = {
  id: number;
  label: string;
  value: number;
}

export const columnsExamination: ColumnDef<Order>[] = [
  {
    accessorKey: "status",
    header: "Status",
  },
  {
    accessorKey: "started_at",
    header: "Data de Início",
    cell: ({ getValue }) => {
      const value = getValue() as number;
      return new Date(value * 1000).toLocaleString();
    },
  },
  {
    accessorKey: "finished_at",
    header: "Data de Fim",
    cell: ({ getValue }) => {
      const value = getValue() as number;
      return new Date(value * 1000).toLocaleString();
    },
  },
  {
    accessorKey: "robot_id",
    header: "Robô",
  },
  {
    accessorKey: "reboiler_id",
    header: "Reboiler",
  },
  {
    id: "actions",
    cell: ({ row }) => {
      const order = row.original;

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
            <DropdownMenuItem onClick={() => navigator.clipboard.writeText(order.id.toString())}>
              Copy Order ID
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            <DropdownMenuItem>View customer</DropdownMenuItem>
            <DropdownMenuItem>View order details</DropdownMenuItem>
          </DropdownMenuContent>
        </DropdownMenu>
      );
    },
  }
];
  


export const columnsRobot: ColumnDef<Robot>[] = [
  {
    accessorKey: "id",
    header: "ID",
  },
  {
    accessorKey: "nickname",
    header: "Nickname",
  },
  {
    accessorKey: "unit_id",
    header: "Unit ID",
  },{
    id: "actions",
    cell: ({ row }) => {
      const order = row.original;

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
            <DropdownMenuItem onClick={() => navigator.clipboard.writeText(order.id.toString())}>
              Copy Order ID
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            <DropdownMenuItem>View customer</DropdownMenuItem>
            <DropdownMenuItem>View order details</DropdownMenuItem>
          </DropdownMenuContent>
        </DropdownMenu>
      );
    },
  }
];