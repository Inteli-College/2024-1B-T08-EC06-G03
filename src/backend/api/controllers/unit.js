const prisma = require('../models/prismaClient');

const getAllUnits = async (req, res) => {
    try {
        const unit = await prisma.unit.findMany();
        res.json(unit);
    } catch (error) {
        res.status(500).json({ error: 'Error fetching unit' });
    }
};

const getUnitById = async (req, res) => {
    const { id } = req.params;
    try {
        const unit = await prisma.unit.findUnique({
            where: { id: parseInt(id) }
        });
        if (unit) {
            res.json(unit);
        } else {
            res.status(404).json({ error: 'Unity not found' });
        }
    } catch (error) {
        res.status(500).json({ error: 'Error fetching unit' });
    }
};

const createUnit = async (req, res) => {
    const { city, state } = req.body;
    try {
        const newUnity = await prisma.unit.create({
            data: {
                city,
                state
            }
        });
        res.status(201).json(newUnity);
    } catch (error) {
        res.status(500).json({ error: 'Error creating unit' });
    }
};

const updateUnit = async (req, res) => {
    const { id } = req.params;
    const { city, state } = req.body;
    try {
        const updatedUnity = await prisma.unit.update({
            where: { id: parseInt(id) },
            data: { city, state }
        });
        res.json(updatedUnity);
    } catch (error) {
        res.status(500).json({ error: 'Error updating unit' });
    }
};

const deleteUnit = async (req, res) => {
    const { id } = req.params;
    try {
        await prisma.unit.delete({
            where: { id: parseInt(id) }
        });
        res.status(204).send();
    } catch (error) {
        res.status(500).json({ error: 'Error deleting unit' });
    }
};

module.exports = {
    getAllUnits,
    getUnitById,
    createUnit,
    updateUnit,
    deleteUnit
};
