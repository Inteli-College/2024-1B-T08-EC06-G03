const express = require('express');
const router = express.Router();
const examinationController = require('../controllers/examination');

router.get('/examination', examinationController.getAllExaminations);
router.get('/examination/:id', examinationController.getExaminationById);
router.post('/examination', examinationController.createExamination);
router.put('/examination/:id', examinationController.updateExamination);
router.delete('/examination/:id', examinationController.deleteExamination);
 
module.exports = router;
