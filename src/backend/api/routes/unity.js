const express = require('express');
const router = express.Router();
const unityController = require('../controllers/unity');

router.get('/unity', unityController.getAllUnity);
router.get('/unity/:id', unityController.getUnityById);
router.post('/unity', unityController.createUnity);
router.put('/unity/:id', unityController.updateUnity);
router.delete('/unity/:id', unityController.deleteUnity);
 
module.exports = router;
