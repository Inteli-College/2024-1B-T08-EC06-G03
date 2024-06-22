const express = require('express');
const router = express.Router();
const orderController = require('../controllers/order');

router.get('/', orderController.getAllOrders);
router.get('/:id', orderController.getOrdersById);
router.get('/unit/:unitId', orderController.getOrderByUnitId);
router.get('/dirtness/:unitId', orderController.getOrdersWithDirtness);
router.post('/', orderController.createOrder);
router.put('/:id', orderController.updateOrder);
router.delete('/:id', orderController.deleteOrder);

module.exports = router;

