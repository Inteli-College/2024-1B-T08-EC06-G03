import React from 'react';
import Navbar from '../components/Navbar';
import {Table, TableHead, TableHeader} from "../components/ui/table"

const App: React.FC = () => {
  return (
    <div>
      <Navbar/>
      <div className='pt-32 max-w-4xl mx-auto'>
        <div className='border rounded'>
          <Table>
            <TableHeader>
              <TableHead>ID</TableHead>
              <TableHead>PRODUTO</TableHead>
            </TableHeader>
          </Table>
        </div>
      </div>
    </div>
    
  );
}

export default App;

