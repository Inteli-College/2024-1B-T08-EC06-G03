const prisma = require('../models/prismaClient');

const getAllImages = async (req, res) => {
    try {
        const images = await prisma.image.findMany();
        res.json(images);
    } catch (error) {
        res.status(500).json({ error: 'Error fetching images' });
    }
};

const getImageById = async (req, res) => {
    const { id } = req.params;
    try {
        const image = await prisma.image.findUnique({
            where: { id: parseInt(id) }
        });
        if (image) {
            res.json(image);
        } else {
            res.status(404).json({ error: 'Image not found' });
        }
    } catch (error) {
        res.status(500).json({ error: 'Error fetching image' });
    }
};

const createImage = async (req, res) => {
    const { taken_at, image } = req.body;
    try {
        const newImage = await prisma.image.create({
            data: {
                image: image,
                taken_at: taken_at
            }
        });
        res.status(201).json(newImage);
    } catch (error) {
        res.status(500).json({ error: 'Error creating image' });
    }
};

const updateImage = async (req, res) => {
    const { id } = req.params;
    const { image, taken_at } = req.body;
    try {
        const updatedImage = await prisma.image.update({
            where: { id: parseInt(id) },
            data: { taken_at: taken_at,
                image: image
             }
        });
        res.json(updatedImage);
    } catch (error) {
        res.status(500).json({ error: 'Error updating image' });
    }
};

const deleteImage = async (req, res) => {
    const { id } = req.params;
    try {
        await prisma.image.delete({
            where: { id: parseInt(id) }
        });
        res.status(204).send();
    } catch (error) {
        res.status(500).json({ error: 'Error deleting image' });
    }
};

module.exports = {
    getAllImages,
    getImageById,
    createImage,
    updateImage,
    deleteImage
};
