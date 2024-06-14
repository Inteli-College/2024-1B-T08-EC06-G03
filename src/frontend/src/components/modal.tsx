import React from "react";
import {Dialog,DialogContent,DialogHeader,DialogTitle,DialogTrigger,DialogFooter} from "@/components/ui/dialog"
import { Button } from "@/components/ui/button"

export type SubmitFunction = (event: React.MouseEvent<HTMLButtonElement, MouseEvent>) => void;

interface ModalProps {
    button_label: string;
    title: string;
    children: React.ReactNode;
    isOpen: boolean;
    submit_action: SubmitFunction;
    };

export const Modal_template: React.FC<ModalProps> = ({ button_label, title, children, submit_action}) => {
    return (
        <Dialog>
      <DialogTrigger asChild>
        <Button variant="outline">{button_label}</Button>
      </DialogTrigger>
      <DialogContent className="sm:max-w-[425px]">
        <DialogHeader>
          <DialogTitle>{title}</DialogTitle>
        </DialogHeader>
        {children}
        <DialogFooter>
          <Button type="submit" onSubmit={submit_action}>Enviar</Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
    );
}

