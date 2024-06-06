const express = require('express');
const router = express.Router();
const unitController = require('../controllers/unit');

router.get('/unit', unitController.getAllUnits);
router.get('/unit/:id', unitController.getUnitById);
router.post('/unit', unitController.createUnit);
router.put('/unit/:id', unitController.updateUnit);
router.delete('/unit/:id', unitController.deleteUnit);
 
module.exports = router;
