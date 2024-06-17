import { Button } from "@/components/ui/button";
import {
  Dialog,
  DialogClose,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";

export function NewProcessDialog() {
  return (
    <Dialog>
      <DialogTrigger asChild>
        <Button variant="outline" className="justify-end">Novo Processo</Button>
      </DialogTrigger>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Novo Processo</DialogTitle>
          <DialogDescription>
            Preencha os campos abaixo para criar um novo processo.
          </DialogDescription>
        </DialogHeader>
        <div className="space-y-4">
          <div>
            <Label htmlFor="campo1" className="pb-6">Campo 1</Label>
            <Input id="campo1" placeholder="Digite o valor do Campo 1" />
          </div>
          <div>
            <Label htmlFor="campo2" className="pb-6">Campo 2</Label>
            <Input id="campo2" placeholder="Digite o valor do Campo 2" />
          </div>
          <div>
            <Label htmlFor="campo3" className="pb-6">Campo 3</Label>
            <Input id="campo3" placeholder="Digite o valor do Campo 3" />
          </div>
          <div>
            <Label htmlFor="campo4" className="pb-6">Campo 4</Label>
            <Input id="campo4" placeholder="Digite o valor do Campo 4" />
          </div>
        </div>
        <DialogFooter className="sm:justify-end">
          <Button type="submit">
            Criar Processo
          </Button>
          <DialogClose asChild>
            <Button type="button" variant="secondary">
              Fechar
            </Button>
          </DialogClose>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}
