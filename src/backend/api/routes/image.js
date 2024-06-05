const express = require('express');
const router = express.Router();
const imageController = require('../controllers/image');

router.get('/image', imageController.getAllImages);
router.get('/image/:id', imageController.getImageById);
router.post('/image', imageController.createImage);
router.put('/image/:id', imageController.updateImage);
router.delete('/image/:id', imageController.deleteImage);
 
module.exports = router;
