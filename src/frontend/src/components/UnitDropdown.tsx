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
import { getUnities } from '@/api/unit';
import { Unit } from '@/components/Columns';

export function UnitDropdown() {
  const [options, setOptions] = React.useState<Unit[] | [] >([]);
  const [error, setError] = React.useState<string | null>(null);

  React.useEffect(() => {
    async function getOptions() {
    try{
      const data: Unit[] | string   = await getUnities();
      if (typeof data === "string") {
        setError(data);
      } else {
        setOptions(data);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
      setError('An error occurred while fetching data.'); 
    }  finally{}}
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
