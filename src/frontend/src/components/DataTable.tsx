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
import { Order } from '../components/Columns'; // Certifique-se de importar o tipo Order

interface DataTableProps {
  columns: ColumnDef<Order, any>[];
  data: Order[];
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

  return (
    <div className="rounded-md border bg-gray-100 p-4">
      <Table className="w-full text-left">
        <TableHeader>
          {table.getHeaderGroups().map((headerGroup) => (
            <TableRow key={headerGroup.id}>
              {headerGroup.headers.map((header) => (
                <TableHead key={header.id} className="p-2 border-b">
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
      <Accordion type="single" collapsible>
        {table.getRowModel().rows?.length ? (
          table.getRowModel().rows.map((row) => (
            <AccordionItem key={row.id} value={row.id}>
              <AccordionTrigger className="w-full">
                <Table className="w-full text-left">
                  <TableBody>
                    <TableRow
                      data-state={row.getIsSelected() && "selected"}
                      className="hover:bg-gray-200"
                    >
                      {row.getVisibleCells().map((cell) => (
                        <TableCell key={cell.id} className="p-2 border-b">
                          {flexRender(
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
                      <TableHead className="p-2 border-b">Etapa</TableHead>
                      <TableHead className="p-2 border-b">Data</TableHead>
                    </TableRow>
                  </TableHeader>
                  <TableBody>
                    {row.original.examinations
                      .filter((exam) => exam.step === "Pré")
                      .map((exam) => (
                        <TableRow key={exam.id}>
                          <TableCell className="p-2 border-b">
                            {exam.step}
                          </TableCell>
                          <TableCell className="p-2 border-b">
                            {new Date(exam.started_at * 1000).toLocaleString()}
                          </TableCell>
                        </TableRow>
                      ))}
                    {row.original.examinations
                      .filter((exam) => exam.step === "Pós")
                      .map((exam) => (
                        <TableRow key={exam.id}>
                          <TableCell className="p-2 border-b">
                            {exam.step}
                          </TableCell>
                          <TableCell className="p-2 border-b">
                            {new Date(exam.started_at * 1000).toLocaleString()}
                          </TableCell>
                        </TableRow>
                      ))}
                  </TableBody>
                </Table>
              </AccordionContent>
            </AccordionItem>
          ))
        ) : (
          <div className="h-24 text-center">No results.</div>
        )}
      </Accordion>
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
