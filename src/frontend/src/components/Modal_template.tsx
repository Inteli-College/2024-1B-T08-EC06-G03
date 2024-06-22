import React from "react";
import {Dialog,DialogContent,DialogHeader,DialogTitle,DialogTrigger,DialogFooter} from "@/components/ui/dialog"
import { Button } from "@/components/ui/button"

export type SubmitFunction = (event: React.FormEvent<HTMLFormElement>) => void;

interface ModalProps {
    className: string;
    button_label: string;
    title: string;
    children: React.ReactNode;
    isOpen: boolean;
    submit_action: SubmitFunction;
    };

const Modal_template: React.FC<ModalProps> = ({ button_label, title, children, submit_action}) => {
    return (
        <Dialog>
      <DialogTrigger asChild>
        <Button variant="outline">{button_label}</Button>
      </DialogTrigger>
      <DialogContent className="sm:max-w-[425px]">
        <DialogHeader>
          <DialogTitle>{title}</DialogTitle>
        </DialogHeader>
        <form onSubmit={submit_action}>
            {children}
            <DialogFooter>
            <DialogTrigger asChild>
            <Button type="submit">Enviar</Button>
            </DialogTrigger>
        </DialogFooter>
        </form>
      </DialogContent>
    </Dialog>
    );
}

export default Modal_template;

