document.addEventListener("DOMContentLoaded", function() {
    var buttons = document.querySelectorAll('.direction-button');

    buttons.forEach(function(button) {
        button.addEventListener('click', function() {
            var direction = button.id;
            console.log(direction);
        });
    });

    document.addEventListener('keydown', function(event) {
        var directionMap = {
            'ArrowUp': 'north',
            'ArrowDown': 'south',
            'ArrowLeft': 'west',
            'ArrowRight': 'east'
        };

        if (event.key in directionMap) {
            var direction = directionMap[event.key];
            console.log(direction);
        }
    });
});
