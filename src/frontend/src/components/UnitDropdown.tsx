import * as React from 'react';
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectLabel,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';

export function UnitDropdown() {
  const [options, setOptions] = React.useState([]);

  React.useEffect(() => {
    async function getOptions() {
      try {
        const response = await fetch('https://api.example.com/units'); // Replace with your actual endpoint
        const data = await response.json();
        setOptions(data);
      } catch (error) {
        console.error('Error fetching options:', error);
      }
    }

    getOptions();
  }, []);

  return (
    <Select>
      <SelectTrigger className="w-[300px]">
        <SelectValue placeholder="Select a unit" />
      </SelectTrigger>
      <SelectContent>
        <SelectGroup>
          <SelectLabel>Units</SelectLabel>
          {options.map((option) => (
            <SelectItem key={option.id} value={option.value}>
              {option.label}
            </SelectItem>
          ))}
        </SelectGroup>
      </SelectContent>
    </Select>
  );
}
