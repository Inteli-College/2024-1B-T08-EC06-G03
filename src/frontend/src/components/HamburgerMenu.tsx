import React, { useState, useEffect, useRef } from 'react';
import { getOrders } from '@/api/orders';
import Switch from './Switch';
import { Button } from '@/components/ui/button';
import { useNavigate } from 'react-router-dom';
import { getExaminationByID } from '@/api/examination';
import { Examination } from './Columns';

interface HamburgerMenuProps {
    examinationId: string;
}

const HamburgerMenu: React.FC<HamburgerMenuProps> = ({examinationId}) => {
    const [isOpen, setIsOpen] = useState(false);
    const menuRef = useRef<HTMLDivElement>(null);
    const buttonRef = useRef<HTMLDivElement>(null);
    const navigate = useNavigate();
    const [order_id, setOrder] = useState<number >(0);


    const fetchOrderId = async () => {
        const response: Examination | string = await getExaminationByID(parseInt(examinationId));
        console.log(response);
        if (typeof response === 'string') {
            console.error('Failed to fetch examination:', response);
            return;
        }
        setOrder(response.order_id);
    }



    const toggleMenu = () => {
        setIsOpen(prevState => !prevState);
    };

    useEffect( () => {
        fetchOrderId();
        console.log('examinationId:', examinationId)
     }, []);

    
    async function updateOrderStatus(order_id: number, status: string) {
      
        try {
          const response = await fetch(`http://localhost:8000/api/orders/${order_id}`, {
            method: 'PUT',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({ status }),
          });
      
          if (!response.ok) {
            throw new Error(`Failed to update order with id ${order_id}`);
          }
      
          const result = await response.json();
          console.log('Order updated successfully:', result);
          return result;
        } catch (error) {
          console.error('Error updating order:', error);
          throw error;
        }
      }

      const handleUpdateStatus = async () => {
        const status = 'inactive';
    
        try {
        const result = await updateOrderStatus(order_id, status);
          console.log('Update result:', result);
          // Navegar ou fazer algo após a atualização bem-sucedida
          navigate('/');
        } catch (error) {
          console.error('Failed to update order status:', error);
        }
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
                        <button onClick={handleUpdateStatus} className='text-xl font-bold border rounded-lg p-2 hover:text-red-600'> 
                            Encerrar Examinação
                        </button>
                    </li>
                </ul>
            </div>
        </>
    );
};

export default HamburgerMenu;
