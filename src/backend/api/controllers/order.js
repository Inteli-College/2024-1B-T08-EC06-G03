const prisma = require('../models/prismaClient');

const getAllOrders = async (req, res) => {
	try {
		const orders = await prisma.order.findMany();
		res.json(orders);
	} catch (error) {
		res.status(500).json({ error: 'Error fetching examinations' })
	}
};

const getOrdersById = async (req, res) => {
	const { id } = req.params;
	try {
		const order = await prisma.order.findUnique({
			where: { id: parseInt(id) }
		});
		if (order) {
			res.json(order);
		} else {
			res.status(404).json({ error: 'Examination not found' });
		} 
	} catch (error) {
		res.status(500).json({ error: 'Error fetching order '});
	}
};	

const getOrderByUnitId = async (req, res) => {
	const { unitId } = req.params; // Assuming unitId is passed in the request params
	
	try {
		const orders = await prisma.order.findMany({
		where: {
			reboiler: {
			unit: { id: parseInt(unitId) }, // Join with Reboiler and Unit tables
			},
		},
		include: {
			Examinations: true, // Include examination details
		},
		});
		if (orders.length > 0) {
		res.json(orders);
		} else {
		res.json({ message: 'No orders found for this unit' }); // Informative message
		}
	} catch (error) {
		console.log(error);
		res.status(500).json({ error: 'Error fetching orders' });
	}
};	  

const createOrder = async (req, res) => {
	const { status, robot_id, reboiler_id, started_at, finished_at } = req.body;
	try {
		const newOrder = await prisma.order.create({
			data: {
				status: status,
				started_at: started_at,
				finished_at: 0,
				robot: { connect: { id: parseInt(robot_id) } },
				reboiler: { connect: { id: parseInt(reboiler_id) } },
				Examinations: { create: { step: "pré", started_at: started_at, finished_at: 0} },
			}
		});
		res.status(201).json(newOrder);
	} catch (error) {
		console.log(error);
		res.status(500).json({ error: "Internal Server Error" });
	}
};

const updateOrder = async (req, res) => {
	const { id } = req.params;
	const { status, robot_id, reboiler_id, started_at, finished_at } = req.body;
	try {
		const updatedOrder = await prisma.order.update({
			where: { id: parseInt(id) },
			data: {
				status: status,
				robot_id: robot_id,
				reboiler_id: reboiler_id,
				started_at: started_at,
				finished_at: finished_at
			}
		});
		res.json(updatedOrder);
	} catch (error) {
		res.status(500).json({ error: "Error updating order" + error });
	}
};

const deleteOrder = async (req, res) => {
	const { id } = req.params;
	try {
		await prisma.order.delete({
			where: { id: parseInt(id) }
		});
		res.status(204).json("Order deleted successfully");
	} catch (error) {
		res.status(500).json({ error: "Error deleting order" });
	}
};

module.exports = {
	getAllOrders,
	getOrdersById,
	getOrderByUnitId,
	createOrder,
	updateOrder,
	deleteOrder
};
