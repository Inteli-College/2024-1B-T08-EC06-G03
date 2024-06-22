import React from "react";
import {
  ColumnDef,
  flexRender,
  getCoreRowModel,
  getPaginationRowModel,
  useReactTable,
} from "@tanstack/react-table";

import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";

import { Accordion, AccordionItem, AccordionTrigger, AccordionContent } from "@/components/ui/accordion";
import { Button } from "@/components/ui/button";
import { Reboiler, Robot } from '../components/Columns';
import { Order, OrderWithDirtness } from '../components/Columns'; // Certifique-se de importar o tipo Order
import { Gamepad2 } from "lucide-react";
import { useNavigate } from "react-router-dom";

interface DataTableProps {
  columns: ColumnDef<Order | Robot>[];
  data: OrderWithDirtness[] | Robot[] | Reboiler[];
}

export function DataTable({
  columns,
  data,
}: DataTableProps) {
  const table = useReactTable({
    data,
    columns,
    getCoreRowModel: getCoreRowModel(),
    getPaginationRowModel: getPaginationRowModel(),
    initialState: { pagination: { pageSize: 7 } }, // Set page size to 7
  });

  const navigate = useNavigate();

  const handleControlClick = (examinationId: number) => {
    navigate(`/control/${examinationId}`);
  };

  return (
    <div className="rounded-md border bg-gray-100 p-4">
      {table.getRowModel().rows?.length ? (
        <div>
          <Table className="w-full text-left">
            <TableHeader>
              {table.getHeaderGroups().map((headerGroup) => (
                <TableRow key={headerGroup.id}>
                  {headerGroup.headers.map((header) => (
                    <TableHead key={header.id} className="p-2">
                      {header.isPlaceholder
                        ? null
                        : flexRender(
                          header.column.columnDef.header,
                          header.getContext()
                        )}

                    </TableHead>
                  ))}
                </TableRow>
              ))}
            </TableHeader>
          </Table>
          <div>
            {table.getRowModel().rows.some(row => 'Examinations' in row.original) ? (
              <Accordion type="single" collapsible>
                {table.getRowModel().rows.map((row) => (
                  'Examinations' in row.original ? (
                    <AccordionItem key={row.id} value={row.id}>
                      <AccordionTrigger className="w-full">
                        <Table className="w-full text-left">
                          <TableBody>
                            <TableRow
                              data-state={row.getIsSelected() && "selected"}
                              className="hover:bg-gray-200"
                            >
                              {row.getVisibleCells().map((cell) => (
                                <TableCell key={cell.id} className="p-2">
                                  {cell.column.id !== 'actions' && flexRender(
                                    cell.column.columnDef.cell,
                                    cell.getContext()
                                  )}
                                </TableCell>
                              ))}
                            </TableRow>
                          </TableBody>
                        </Table>
                      </AccordionTrigger>
                      <AccordionContent>
                        <Table className="w-full text-left">
                          <TableHeader>
                            <TableRow>
                              <TableHead className="p-2">Etapa</TableHead>
                              <TableHead className="p-2">Data</TableHead>
                              <TableHead className="p-2">Sujidade(%)</TableHead>
                              <TableHead className="p-2">Ações</TableHead>
                            </TableRow>
                          </TableHeader>
                          <TableBody>
                            {row.original.Examinations.filter((exam) => exam.step === "Pré")
                              .map((exam) => (
                                <TableRow key={exam.id}>
                                  <TableCell className="p-2">
                                    {exam.step}
                                  </TableCell>
                                  <TableCell className="p-2">
                                    {new Date(exam.started_at * 1000).toLocaleString()}
                                  </TableCell>
                                  <TableCell className="p-2">
                                    {exam.dirtness * 100}
                                  </TableCell>
                                  <TableCell className="p-2">
                                    <Button
                                      variant="ghost"
                                      className="h-8 w-8 p-0"
                                      onClick={() => handleControlClick(row.original.id as number)}
                                    >
                                      <Gamepad2 className="h-4 w-4" />
                                    </Button>
                                  </TableCell>
                                </TableRow>
                              ))}
                            {row.original.Examinations.filter((exam) => exam.step === "Pós")
                              .map((exam) => (
                                <TableRow key={exam.id}>
                                  <TableCell className="p-2">
                                    {exam.step}
                                  </TableCell>
                                  <TableCell className="p-2">
                                    {new Date(exam.started_at * 1000).toLocaleString()}
                                  </TableCell>
                                  <TableCell className="p-2">
                                    {exam.dirtness * 100}
                                  </TableCell>
                                  <TableCell className="p-2">
                                    <Button
                                      variant="ghost"
                                      className="h-8 w-8 p-0"
                                      onClick={() => handleControlClick(row.original.id as number)}
                                    >
                                      <Gamepad2 className="h-4 w-4" />
                                    </Button>
                                  </TableCell>
                                </TableRow>
                              ))}
                          </TableBody>
                        </Table>
                      </AccordionContent>
                    </AccordionItem>
                  ) : (
                    <Table className="w-full text-left" key={row.id}>
                      <TableBody>
                        <TableRow
                          data-state={row.getIsSelected() && "selected"}
                          className="hover:bg-gray-200"
                        >
                          {row.getVisibleCells().map((cell) => (
                            <TableCell key={cell.id} className="p-2">
                              {cell.column.id !== 'actions' && flexRender(
                                cell.column.columnDef.cell,
                                cell.getContext()
                              )}
                            </TableCell>
                          ))}
                        </TableRow>
                      </TableBody>
                    </Table>
                  )
                ))}
              </Accordion>
            ) : (
              table.getRowModel().rows.map((row) => (
                <Table className="w-full text-left" key={row.id}>
                  <TableBody>
                    <TableRow
                      data-state={row.getIsSelected() && "selected"}
                      className="hover:bg-gray-200">
                      {row.getVisibleCells().map((cell) => (
                        <TableCell key={cell.id} className="p-2">
                          {cell.column.id !== 'actions' && flexRender(
                            cell.column.columnDef.cell,
                            cell.getContext())}
                        </TableCell>
                      ))}
                    </TableRow>
                  </TableBody>
                </Table>
              ))
            )}
          </div>
        </div>
      ) : (
        <div className="h-24 text-center">No results.</div>
      )}
      <div className="flex items-center justify-end space-x-2 py-4">
        <Button
          variant="outline"
          size="sm"
          onClick={() => table.previousPage()}
          disabled={!table.getCanPreviousPage()}
        >
          Previous
        </Button>
        <Button
          variant="outline"
          size="sm"
          onClick={() => table.nextPage()}
          disabled={!table.getCanNextPage()}
        >
          Next
        </Button>
      </div>
    </div>
  );
}
