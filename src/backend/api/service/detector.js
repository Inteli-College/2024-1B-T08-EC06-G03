class Detector {
    constructor() {
    }

    async analyse(image) {
        try {
            const response = await fetch('localhost:3000/infer', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ image })
            });
            const data = await response.json();
            return data;
        } catch (error) {
            console.error('Error:', error);
            throw error;
        }
    }
}