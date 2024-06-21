const prisma = require('../models/prismaClient');

const getAllReboilers = async (req, res) => {
    try {
        const reboilers = await prisma.reboiler.findMany();
        res.json(reboilers);
    } catch (error) {
        res.status(500).json({ error: 'Error fetching reboilers' });
    }
};

const getReboilerById = async (req, res) => {
    const { id } = req.params;
    try {
        const reboiler = await prisma.reboiler.findUnique({
            where: { id: parseInt(id) }
        });
        if (reboiler) {
            res.json(reboiler);
        } else {
            res.status(404).json({ error: 'Reboiler not found' });
        }
    } catch (error) {
        res.status(500).json({ error: 'Error fetching reboiler' });
    }
};

const getReboilerByUnitId = async (req, res) => {
    const {id } = req.params;
    try{
        const reboiler = await prisma.reboiler.findMany({
            where: {unit_id: parseInt(id)}
        });
        if(reboiler){
            res.json(reboiler);
        }else{
            res.status(404).json({error: 'No reboilers were found at that unit'});
        }
    }
    catch(error){
        res.status(500).json({error: 'Error fetching reboiler'});
    }
}


const createReboiler = async (req, res) => {
    const { number, unit_id } = req.body;
    try {
        const newReboiler = await prisma.reboiler.create({
            data: {
            number: parseInt(number),
            unit_id: parseInt(unit_id)
            }
        });
        res.status(201).json(newReboiler);
    } catch (error) {
        console.log(error);
        res.status(500).json({ error: 'Error creating reboiler' });
    }
};

const updateReboiler = async (req, res) => {
    const { id } = req.params;
    const { number, unit_id } = req.body;
    try {
        const updatedReboiler = await prisma.reboiler.update({
            where: { id: parseInt(id) },
            data: { number, unit_id }
        });
        res.json(updatedReboiler);
    } catch (error) {
        res.status(500).json({ error: 'Error updating reboiler' });
    }
};

const deleteReboiler = async (req, res) => {
    const { id } = req.params;
    try {
        await prisma.reboiler.delete({
            where: { id: parseInt(id) }
        });
        res.status(204).send();
    } catch (error) {
        res.status(500).json({ error: 'Error deleting reboiler' });
    }
};

module.exports = {
    getAllReboilers,
    getReboilerById,
    getReboilerByUnitId,
    createReboiler,
    updateReboiler,
    deleteReboiler
};
