const prisma = require('../models/prismaClient');
const { get } = require('../routes/examination');

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

const getOrdersWithDirtness = async (req, res) => {

	const { unitId } = req.params; // Assuming unitId is passed in the request params
    try {
        // Fetch all orders along with their examinations
        const orders = await prisma.order.findMany({
			where: {
				reboiler: {
				unit: { id: parseInt(unitId) }, // Join with Reboiler and Unit tables
				},
			},
            include: {
                Examinations: {
                    include: {
                        TubeStates: true
                    }
                }
            }
        });

        // Transform the orders to include the dirtness property in each examination
        const ordersWithDirtness = orders.map(order => {
            const examinationsWithDirtness = order.Examinations.map(examination => {
                const totalDirtness = examination.TubeStates.reduce((total, tubeState) => total + tubeState.dirtness, 0);
                const dirtness = examination.TubeStates.length > 0 ? totalDirtness / examination.TubeStates.length : 0;

                return {
                    id: examination.id,
                    dirtness: parseFloat(dirtness.toFixed(2)), // Add dirtness and format to 2 decimal places
                    step: examination.step,
                    started_at: examination.started_at,
                    finished_at: examination.finished_at,
                    order_id: examination.order_id
                };
            });

            return {
                id: order.id,
                status: order.status,
                robot_id: order.robot_id,
                reboiler_id: order.reboiler_id,
                started_at: order.started_at,
                finished_at: order.finished_at,
                Examinations: examinationsWithDirtness
            };
        });

        res.json(ordersWithDirtness);
    } catch (error) {
        console.error(error);
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
	getOrdersWithDirtness,
	createOrder,
	updateOrder,
	deleteOrder
};
