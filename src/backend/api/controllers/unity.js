const prisma = require('../models/prismaClient');

const getAllUnity = async (req, res) => {
    try {
        const unity = await prisma.unity.findMany();
        res.json(unity);
    } catch (error) {
        res.status(500).json({ error: 'Error fetching unity' });
    }
};

const getUnityById = async (req, res) => {
    const { id } = req.params;
    try {
        const unity = await prisma.unity.findUnique({
            where: { id: parseInt(id) }
        });
        if (unity) {
            res.json(unity);
        } else {
            res.status(404).json({ error: 'Unity not found' });
        }
    } catch (error) {
        res.status(500).json({ error: 'Error fetching unity' });
    }
};

const createUnity = async (req, res) => {
    const { city, state } = req.body;
    try {
        const newUnity = await prisma.unity.create({
            data: {
                city,
                state
            }
        });
        res.status(201).json(newUnity);
    } catch (error) {
        res.status(500).json({ error: 'Error creating unity' });
    }
};

const updateUnity = async (req, res) => {
    const { id } = req.params;
    const { city, state } = req.body;
    try {
        const updatedUnity = await prisma.unity.update({
            where: { id: parseInt(id) },
            data: { city, state }
        });
        res.json(updatedUnity);
    } catch (error) {
        res.status(500).json({ error: 'Error updating unity' });
    }
};

const deleteUnity = async (req, res) => {
    const { id } = req.params;
    try {
        await prisma.unity.delete({
            where: { id: parseInt(id) }
        });
        res.status(204).send();
    } catch (error) {
        res.status(500).json({ error: 'Error deleting unity' });
    }
};

module.exports = {
    getAllUnity,
    getUnityById,
    createUnity,
    updateUnity,
    deleteUnity
};
