#!/bin/bash

# Executa ao sair do script
trap "echo 'Limpando... (apagando venv)'; rm -rf $(pwd)/src/venv" EXIT

cd src

python3 -m venv venv

source venv/bin/activate

pip install flask

cd workspace

# Pega o caminho da instaância do venv
FLASK_PATH=$(pip show flask | grep "Location:" | awk '{print $2}')

# Se não achar
if [ -z "$FLASK_PATH" ]; then
    echo "Typer not found. Please make sure it is installed."
    exit 1
fi

# Exporta a variável de ambiente PYTHONPATH com o caminho do venv
export PYTHONPATH="$PYTHONPATH:$FLASK_PATH"


echo "Updated PYTHONPATH: $PYTHONPATH"

# Builda e roda o ros2
colcon build

source install/local_setup.bash

ros2 run robot_navigation bot