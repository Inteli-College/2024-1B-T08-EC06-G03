const prisma = require('../models/prismaClient');

const getAllExaminations = async (req, res) => {
    try {
        const examinations = await prisma.examination.findMany();
        res.json(examinations);
    } catch (error) {
        res.status(500).json({ error: 'Error fetching examinations' });
    }
};

const getExaminationById = async (req, res) => {
    const { id } = req.params;
    try {
        const examination = await prisma.examination.findUnique({
            where: { id: parseInt(id) }
        });
        if (examination) {
            res.json(examination);
        } else {
            res.status(404).json({ error: 'Examination not found' });
        }
    } catch (error) {
        res.status(500).json({ error: 'Error fetching examination' });
    }
};

const getAllImagesByExaminationId = async (req, res) => {
    const { id } = req.params;
    try {
        const examinations = await prisma.tubeState.findMany({
            where: { id: parseInt(id) },
            include: {
                TubeStates:{ 
                    include:{
                        image: false
                    }
                }
            }
        });
        if(examinations.TubeStates.length > 0){
            res.json(examinations)
        }
        else{
            res.status(404).json({ error: 'No tube states were found at that examination' });
        }
    }catch (error) {
        res.status(500).json({ error: "Internal Server Error" });
    }
}

const getAllTubeStatesByExaminationId = async (req, res) => {
    const { id } = req.params;
    try{
        const examination = await prisma.examination.findUnique({
            where: { id: parseInt(id) },
            include: {
                TubeStates:{ 
                    include:{
                        image: false
                    }
                }
            }
        });
        if(examination.TubeStates.length > 0){
            res.json(examination)
        }
        else{
            res.status(404).json({ error: 'No tube states were found at that examination' });
        }
    }
    catch (error) {
        console.log(error)  
        res.status(500).json({ error: "Internal Server Error" });
    }
}

const createExamination = async (req, res) => {
    const { started_at, finished_at, order_id } = req.body;
    let step = ""
    try {
        const examinationExist = await prisma.examination.findMany({
            where: {
                order_id: order_id
            }
        });
        if (examinationExist.length !== 0) {
            if (examinationExist.length > 1) {
                res.status(400).json({ error: 'Examinations already exists' });
                return;
            }
            step = "Pós"
        }else{
            step = "Pré"
        }
        const newExamination = await prisma.examination.create({
            data: {
                step: step,
                started_at: started_at ?  started_at : null,
                finished_at: finished_at ? finished_at : null,
                order_id: order_id
            }
        });
        res.status(201).json(newExamination);
    } catch (error) {
        console.log(error)
        res.status(500).json({ error: "Internal Server Error" });
    }
};

const updateExamination = async (req, res) => {
    const { id } = req.params;
    const { step, started_at, finished_at, order_id } = req.body;
    try {
        const updatedExamination = await prisma.examination.update({
            where: { id: parseInt(id) },
            data: {
                step: step,
                started_at: started_at,
                finished_at: finished_at,
                order_id: order_id
            }
        });
        res.json(updatedExamination);
    } catch (error) {
        res.status(500).json({ error: 'Error updating examination' + error });
    }
};

const deleteExamination = async (req, res) => {
    const { id } = req.params;
    try {
        await prisma.examination.delete({
            where: { id: parseInt(id) }
        });
        res.status(204).json("Examination deleted successfully");
    } catch (error) {
        console.log(error)
        res.status(500).json({ error: 'Error deleting examination' });
    }
};

module.exports = {
    getAllExaminations,
    getAllTubeStatesByExaminationId,
    getAllImagesByExaminationId,
    getExaminationById,
    createExamination,
    updateExamination,
    deleteExamination
};
