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
        const images = await prisma.tubeState.findMany({
            where: { examination_id: parseInt(id) },
            include: {
                image: true
            }
        });
        if(images.length > 0){
            res.json(images)
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
        const tubeStates = await prisma.tubeState.findMany({
            where: { examination_id: parseInt(id) },
            include: {
                image: false
            }
        });
        if(tubeStates.length > 0){
            res.json(tubeStates)
        }
        else{
            res.status(404).json({ error: 'No tube states were found at that examination' });
        }
    }
    catch (error) {
        res.status(500).json({ error: "Internal Server Error" });
    }
}

const createExamination = async (req, res) => {
    const { etapa, robot_id, reboiler_id, started_at, finished_at } = req.body;
    try {
        const newExamination = await prisma.examination.create({
            data: {
                etapa,
                robot_id,
                reboiler_id,
                started_at: started_at ?  started_at : null,
                finished_at: finished_at ? finished_at : null
            }
        });
        res.status(201).json(newExamination);
    } catch (error) {
        res.status(500).json({ error: "Internal Server Error" });
    }
};

const updateExamination = async (req, res) => {
    const { id } = req.params;
    const { etapa, robot_id, reboiler_id, started_at, finished_at } = req.body;
    try {
        const updatedExamination = await prisma.examination.update({
            where: { id: parseInt(id) },
            data: {
                etapa: etapa,
                robot_id: robot_id,
                reboiler_id: reboiler_id,
                started_at: started_at,
                finished_at: finished_at
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
