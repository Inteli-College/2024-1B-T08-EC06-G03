import "../styles/joystick.css";
import React, { useState, useEffect, useRef } from "react";

interface JoystickProps {
    sendMessage: (message: string) => void;
}

const Joystick: React.FC<JoystickProps> = ({ sendMessage }) => {
    const [isDragging, setIsDragging] = useState(false);
    const [position, setPosition] = useState({ x: 0, y: 0 });
    const joystickRef = useRef<HTMLDivElement>(null);
    const innerCircleRef = useRef<HTMLDivElement>(null);

    const sendJoystickData = (linear: number, angular: number) => {
        linear = Math.round(linear);
        angular = Math.round(angular);

        sendMessage(JSON.stringify({ "linear_speed": linear }));
        sendMessage(JSON.stringify({ "angular_speed": angular }));
        console.log("linear: " + linear);
        console.log("angular: " + angular);
    };

    const handleMouseDown = () => {
        setIsDragging(true);
    };

    const handleMouseUp = () => {
        setIsDragging(false);
        setPosition({ x: 0, y: 0 });
        sendJoystickData(0, 0);
    };

    const handleMouseMove = (event: MouseEvent) => {
        handleDrag(event.clientX, event.clientY);
    };

    const handleTouchStart = (event: TouchEvent) => {
        const touch = event.touches[0];
        handleDrag(touch.clientX, touch.clientY);
        setIsDragging(true);
    };

    const handleTouchMove = (event: TouchEvent) => {
        const touch = event.touches[0];
        handleDrag(touch.clientX, touch.clientY);
    };

    const handleDrag = (clientX: number, clientY: number) => {
        if (!isDragging || !joystickRef.current || !innerCircleRef.current) {
            return;
        }

        const rect = joystickRef.current.getBoundingClientRect();
        const innerRect = innerCircleRef.current.getBoundingClientRect();
        const maxRadius = rect.width / 2 + innerRect.width / 20;

        let x = clientX - rect.left - rect.width / 2;
        let y = clientY - rect.top - rect.height / 2;

        // Limit the movement to the bounds of the outer circle plus 50% of the inner circle's radius
        const distance = Math.sqrt(x * x + y * y);
        if (distance > maxRadius) {
            x = (x / distance) * maxRadius;
            y = (y / distance) * maxRadius;
        }

        setPosition({ x, y });

        // Calculate angle and radius percentage
        let angular = (x / maxRadius) * 100;
        let linear = -(y / maxRadius) * 100;

        if (Math.abs(angular) < 25) {
            angular = 0;
        }
        if (Math.abs(linear) < 20) {
            linear = 0;
        }

        sendJoystickData(linear, angular);
    };

    useEffect(() => {
        document.addEventListener("mousemove", handleMouseMove);
        document.addEventListener("mouseup", handleMouseUp);
        document.addEventListener("touchmove", handleTouchMove);
        document.addEventListener("touchend", handleMouseUp);
        return () => {
            document.removeEventListener("mousemove", handleMouseMove);
            document.removeEventListener("mouseup", handleMouseUp);
            document.removeEventListener("touchmove", handleTouchMove);
            document.removeEventListener("touchend", handleMouseUp);
        };
    }, [isDragging]);

    return (
        <div
            ref={joystickRef}
            onMouseDown={handleMouseDown}
            onTouchStart={handleTouchStart}
            className="relative w-40 h-40 rounded-full"
            style={{
                backgroundColor: "rgba(217, 217, 217, 0.5)",
                touchAction: "none",
            }}
        >
            <div
                ref={innerCircleRef}
                className="absolute w-16 h-16 rounded-full"
                style={{
                    backgroundColor: "#D9D9D9",
                    transform: `translate(${position.x}px, ${position.y}px)`,
                    transition: isDragging ? "none" : "transform 0.2s",
                    left: "50%",
                    top: "50%",
                    marginLeft: "-2rem", // Half of inner circle's width
                    marginTop: "-2rem", // Half of inner circle's height
                }}
            />
        </div>
    );
};

export default Joystick;
