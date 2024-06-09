const prisma = require('../models/prismaClient');

const getAllRobots = async (req, res) => {
  try {
    const robots = await prisma.robot.findMany();
    res.json(robots);
  } catch (error) {
    res.status(500).json({ error: 'Error fetching robots' });
  }
};

const getRobotById = async (req, res) => {
  const { id } = req.params;
  try {
    const robot = await prisma.robot.findUnique({
      where: { id: parseInt(id) }
    });
    if (robot) {
      res.json(robot);
    } else {
      res.status(404).json({ error: 'Robot not found' });
    }
  } catch (error) {
    res.status(500).json({ error: 'Error fetching robot' });
  }
};

const createRobot = async (req, res) => {
  const { last_manufactured } = req.body;
  try {
    const newRobot = await prisma.robot.create({
      data: {
        last_manufactured: last_manufactured
      }
    });
    res.status(201).json(newRobot);
  } catch (error) {
    res.status(500).json({ error: 'Error creating robot' });
  }
};

const updateRobot = async (req, res) => {
  const { id } = req.params;
  const { last_manufactured } = req.body;
  try {
    const updatedRobot = await prisma.robot.update({
      where: { id: parseInt(id) },
      data: { last_manufactured: new Date(last_manufactured) }
    });
    res.json(updatedRobot);
  } catch (error) {
    res.status(500).json({ error: 'Error updating robot' });
  }
};

const deleteRobot = async (req, res) => {
  const { id } = req.params;
  try {
    await prisma.robot.delete({
      where: { id: parseInt(id) }
    });
    res.status(204).send();
  } catch (error) {
    res.status(500).json({ error: 'Error deleting robot' });
  }
};

module.exports = {
  getAllRobots,
  getRobotById,
  createRobot,
  updateRobot,
  deleteRobot
};
