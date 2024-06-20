const express = require('express');
const router = express.Router();
const reboilerController = require('../controllers/reboiler');

router.get('/', reboilerController.getAllReboilers);
router.get('/:id', reboilerController.getReboilerById);
router.get('/unit/:id', reboilerController.getReboilerByUnitId);
router.post('/', reboilerController.createReboiler);
router.put('/:id', reboilerController.updateReboiler);
router.delete('/:id', reboilerController.deleteReboiler);
 
module.exports = router;
