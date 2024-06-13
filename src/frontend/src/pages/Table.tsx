import React from 'react';
import Navbar from '../components/Navbar';
import Table from '../components/Table';

const App: React.FC = () => {
    return (
        <div className="h-full bg-gray-100">
            <Navbar />
            <main className="pt-32 flex-grow">
                <Table />
            </main>
        </div>
    );
}

export default App;
