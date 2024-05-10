import rclpy
from geometry_msgs.msg import Twist
from flask import Flask, request, jsonify, render_template
from flask_cors import CORS

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})  # Enable CORS for all routes

rclpy.init()
node = rclpy.create_node("move_robot_publisher")
pub = node.create_publisher(Twist, "/cmd_vel", 10)

last_command = Twist()
killed = False


def move_robot(direction):
    """
    Move the robot in the specified direction

    :param direction: The direction in which the robot should move
    :type direction: str
    """

    global killed, last_command

    if killed:
        return

    msg = last_command

    match direction:
        case "front":
            msg.linear.x = -0.2
        case "back":
            msg.linear.x = 0.2
        case "left":
            msg.angular.z = 0.9
        case "right":
            msg.angular.z = -0.9
        case _:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            pass

    node.get_logger().info(f"Sending movement command: {msg}")
    pub.publish(msg)
    last_command = msg


def kill_robot():
    """
    Stop the robot from moving
    """
    global killed

    move_robot("stop")
    killed = True


@app.route("/move", methods=["POST"])
def move():
    """
    Move the robot in the specified direction

    :param direction: The direction in which the robot should move

    :return: JSON response with the status of the movement command
    :rtype: flask.Response
    """
    data = request.json
    print(data)
    if data is None or len(data) == 0:
        data = request.json

    direction = data.get("direction")

    if not direction:
        return jsonify({"error": "Missing 'direction' in payload"}), 400

    move_robot(direction)
    return jsonify({"status": "Movement command executed"}), 200


@app.route("/kill")
def kill():
    """
    Stop the robot from moving

    :return: JSON response with the status of the kill command
    :rtype: flask.Response
    """
    kill_robot()
    return jsonify({"status": "Robot killed"}), 200


@app.route("/reset")
def reset():
    """
    Reset the robot to its initial state

    :return: JSON response with the status of the reset command
    :rtype: flask.Response
    """
    global killed
    killed = False
    return jsonify({"status": "Robot reset"}), 200


def main():
    app.run(host="0.0.0.0", port=5000, debug=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
