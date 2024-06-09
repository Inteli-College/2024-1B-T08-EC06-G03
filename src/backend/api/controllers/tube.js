const prisma = require('../models/prismaClient');

const getAllTubes = async (req, res) => {
    try {
        const tubes = await prisma.tube.findMany();
        res.json(tubes);
    } catch (error) {
        res.status(500).json({ error: 'Error fetching tubes' });
    }
};

const getTubeById = async (req, res) => {
    const { id } = req.params;
    try {
        const tube = await prisma.tube.findUnique({
            where: { id: parseInt(id) }
        });
        if (tube) {
            res.json(tube);
        } else {
            res.status(404).json({ error: 'Tube not found' });
        }
    } catch (error) {
        res.status(500).json({ error: 'Error fetching tube' });
    }
};

const createTube = async (req, res) => {
    const {reboiler_id, position_column, position_row  } = req.body;
    try {
        const newTube = await prisma.tube.create({
            data: {
                reboiler_id: reboiler_id,
                position_column: position_column, 
                position_row: position_row   
            }
        });
        res.status(201).json({message: "Tubo criado com sucesso"});
    } catch (error) {
        res.status(500).json({ error: 'Error creating tube' });
    }
};

const updateTube = async (req, res) => {
    const { id } = req.params;
    const { reboiler_id, position } = req.body;
    try {
        const updatedTube = await prisma.tube.update({
            where: { id: parseInt(id) },
            data: { reboiler_id, position }
        });
        res.json(updatedTube);
    } catch (error) {
        res.status(500).json({ error: 'Error updating tube' });
    }
};

const deleteTube = async (req, res) => {
    const { id } = req.params;
    try {
        await prisma.tube.delete({
            where: { id: parseInt(id) }
        });
        res.status(204).send();
    } catch (error) {
        res.status(500).json({ error: 'Error deleting tube' });
    }
};

module.exports = {
    getAllTubes,
    getTubeById,
    createTube,
    updateTube,
    deleteTube
};
