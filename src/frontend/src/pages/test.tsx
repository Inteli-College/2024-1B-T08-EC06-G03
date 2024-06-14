import { Modal_template, SubmitFunction } from "@/components/modal";

const Teste = () => {
    const handleSubmit: SubmitFunction = (event) => {
        event.preventDefault();
        // Lógica de submissão do formulário
        console.log('enviou');
      };

    return (
        <div>
        <Modal_template 
        title="Modal Title" 
        button_label="abrir modal" 
        children={<div>lalala</div>} 
        submit_action={handleSubmit}
        isOpen={true}>
            </Modal_template>
        </div>
    );
}

export default Teste;