const express = require('express');
const router = express.Router();
const imageController = require('../controllers/image');

router.get('/', imageController.getAllImages);
router.get('/:id', imageController.getImageById);
router.post('/', imageController.createImage);
router.put('/:id', imageController.updateImage);
router.delete('/:id', imageController.deleteImage);
 
module.exports = router;
