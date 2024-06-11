import React, { useState } from "react";

const Switch: React.FC = () => {
    const [isOn, setIsOn] = useState(false);

    const toggleSwitch = () => {
        setIsOn(!isOn);
    };

    return (
        <div
            onClick={toggleSwitch}
            className={`flex items-center w-16 h-8 p-1 rounded-full cursor-pointer transition-colors duration-300 ${isOn ? "bg-green-500" : "bg-gray-300"
                }`}
        >
            <div
                className={`w-6 h-6 bg-white rounded-full shadow-md transform transition-transform duration-300 ${isOn ? "translate-x-8" : "translate-x-0"
                    }`}
            ></div>
        </div>
    );
};

export default Switch;
