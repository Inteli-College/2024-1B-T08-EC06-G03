require('dotenv').config();

const config = require('config');
const host = config.get('services.analyse.host');
const port = config.get('services.analyse.port');
const route = host + ':' + port; 

class Detector {
    static async analyse(image) {
        try {
            const response = await fetch(`${route}/infer`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ base64_img: image })
            });
            const data = await response.json();
            return {
                dirtDetected: data.dirt_detected,
                base64InferedImg: data.base64_infered_img
            };
        } catch (error) {
            console.error('Error:', error);
            throw error;
        }
    }
}

module.exports = Detector;