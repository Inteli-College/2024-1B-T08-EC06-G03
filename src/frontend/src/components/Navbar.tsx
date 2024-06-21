import React from 'react';

const Navbar: React.FC = () => {
  return (
    <nav className="bg-green-800 p-2 fixed w-full top-0 z-10">
      <div className="max-w-7xl mx-auto px-2 sm:px-6 lg:px-8">
        <div className="relative flex items-center justify-between h-16">
          <div className="flex flex-row items-center">
            <img className="h-10 w-10" src="/logo2.png" alt="Workflow" />
            <h1 className="text-white text-2xl font-bold">Desencana!</h1>
          </div>
        </div>
      </div>
    </nav>
  );
};

export default Navbar;
