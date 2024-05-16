import React, { useState, useEffect, useRef } from 'react';
import Switch from './Switch';

const HamburgerMenu: React.FC = () => {
    const [isOpen, setIsOpen] = useState(false);
    const menuRef = useRef<HTMLDivElement>(null);
    const buttonRef = useRef<HTMLDivElement>(null);

    const toggleMenu = () => {
        setIsOpen(prevState => !prevState);
    };

    const handleClickOutside = (event: MouseEvent) => {
        if (menuRef.current && !menuRef.current.contains(event.target as Node) && buttonRef.current && !buttonRef.current.contains(event.target as Node)) {
            setIsOpen(false);
        }
    };

    useEffect(() => {
        if (isOpen) {
            document.addEventListener('mousedown', handleClickOutside);
        } else {
            document.removeEventListener('mousedown', handleClickOutside);
        }
        return () => {
            document.removeEventListener('mousedown', handleClickOutside);
        };
    }, [isOpen]);

    return (
        <>
            <div className="flex items-center">
                <div
                    ref={buttonRef}
                    className={`flex flex-col items-center justify-center w-12 h-12 rounded cursor-pointer z-20 relative transform transition-transform duration-300 ${isOpen ? 'rotate-90' : ''}`}
                    onClick={toggleMenu}
                >
                    <div className="w-8 h-1 bg-white mb-1"></div>
                    <div className="w-8 h-1 bg-white mb-1"></div>
                    <div className="w-8 h-1 bg-white"></div>
                </div>
                {isOpen && (
                    <span className="text-white text-2xl">Funcionalidades</span>
                )}
            </div>
            <div
                ref={menuRef}
                className={`fixed top-0 left-0 h-full transition-transform duration-300 ${isOpen ? 'translate-x-0' : '-translate-x-full'}`}
                style={{ width: '270px', backgroundColor: 'rgba(217, 217, 217, 0.4)', paddingTop: '3rem' }}
            >
                {/* Space for menu items */}
                <ul className="p-4 mt-10">
                    <li className="p-2 text-2xl text-white flex items-center space-x-6">
                        <span>Status:</span>
                        <Switch />
                    </li>
                    <li className="p-2 text-2xl text-white flex items-center space-x-6">
                        <span>Modo:</span>
                        <Switch />
                    </li>
                </ul>
            </div>
        </>
    );
};

export default HamburgerMenu;
