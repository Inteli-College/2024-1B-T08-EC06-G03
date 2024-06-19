const prisma = require('../models/prismaClient');
const detector = require('../service/detector');

const getAllTubeStates = async (req, res) => {
    try {
        const tubeStates = await prisma.tubeState.findMany();
        res.json(tubeStates);
    } catch (error) {
        res.status(500).json({ error: 'Error fetching tube states' });
    }
};

const getTubeStateById = async (req, res) => {
    const { id } = req.params;
    try {
        const tubeState = await prisma.tubeState.findUnique({
            where: { id: parseInt(id) }
        });
        if (tubeState) {
            res.json(tubeState);
        } else {
            res.status(404).json({ error: 'Tube state not found' });
        }
    } catch (error) {
        res.status(500).json({ error: 'Error fetching tube state' });
    }
};

const createTubeState = async (req, res) => {
    const { image, examination_id, taken_at } = req.body;
    const response =  await detector.analyse(image);
    const  image_analysed = response.base64InferedImg;
    const dirtness =  response.dirtDetected;
    try{
        const imageCreated = await prisma.image.create({
            data: {
                image: image_analysed,
                taken_at
            }
        });
        const newTubeState = await prisma.tubeState.create({
            data: {
                dirtness,
                image_id: imageCreated.id,
                examination_id,
            }
        });
        res.status(201).json({image_analysed, dirtness});
    }
    catch (error) {
        console.log(error);
        res.status(500).json({ error: 'Error creating image' });
    }
};

const updateTubeState = async (req, res) => {
    const { id } = req.params;
    const { dirtness, image_id, examination_id } = req.body;
    try {
        const updatedTubeState = await prisma.tubeState.update({
            where: { id: parseInt(id) },
            data: {
                dirtness,
                image_id,
                examination_id
            }
        });
        res.json(updatedTubeState);
    } catch (error) {
        res.status(500).json({ error: 'Error updating tube state' });
    }
};

const deleteTubeState = async (req, res) => {
    const { id } = req.params;
    try {
        await prisma.tubeState.delete({
            where: { id: parseInt(id) }
        });
        res.status(204).send();
    } catch (error) {
        res.status(500).json({ error: 'Error deleting tube state' });
    }
};

module.exports = {
    getAllTubeStates,
    getTubeStateById,
    createTubeState,
    updateTubeState,
    deleteTubeState
};
