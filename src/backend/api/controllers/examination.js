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

const createExamination = async (req, res) => {
    const { etapa, robot_id, reboiler_id, started_at, finished_at } = req.body;
    try {
        const newExamination = await prisma.examination.create({
            data: {
                etapa,
                robot_id,
                reboiler_id,
                started_at: started_at ? new Date(started_at) : null,
                finished_at: finished_at ? new Date(finished_at) : null
            }
        });
        res.status(201).json(newExamination);
    } catch (error) {
        res.status(500).json({ error: 'Error creating examination' });
    }
};

const updateExamination = async (req, res) => {
    const { id } = req.params;
    const { etapa, robot_id, reboiler_id, started_at, finished_at } = req.body;
    try {
        const updatedExamination = await prisma.examination.update({
            where: { id: parseInt(id) },
            data: {
                etapa,
                robot_id,
                reboiler_id,
                started_at: started_at ? new Date(started_at) : null,
                finished_at: finished_at ? new Date(finished_at) : null
            }
        });
        res.json(updatedExamination);
    } catch (error) {
        res.status(500).json({ error: 'Error updating examination' });
    }
};

const deleteExamination = async (req, res) => {
    const { id } = req.params;
    try {
        await prisma.examination.delete({
            where: { id: parseInt(id) }
        });
        res.status(204).send();
    } catch (error) {
        res.status(500).json({ error: 'Error deleting examination' });
    }
};

module.exports = {
    getAllExaminations,
    getExaminationById,
    createExamination,
    updateExamination,
    deleteExamination
};
