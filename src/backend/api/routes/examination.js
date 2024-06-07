const express = require('express');
const router = express.Router();
const examinationController = require('../controllers/examination');

router.get('/', examinationController.getAllExaminations);
router.get('/image/:id', examinationController.getAllImagesByExaminationId);
router.get('/tube-state/:id',examinationController.getAllTubeStatesByExaminationId);
router.get('/:id', examinationController.getExaminationById);
router.post('/', examinationController.createExamination);
router.put('/:id', examinationController.updateExamination);
router.delete('/:id', examinationController.deleteExamination);
 
module.exports = router;
