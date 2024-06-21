import * as React from 'react';
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import { getReboilers } from '@/api/reboiler';
import { Reboiler, dropdown } from '@/components/Columns';

interface SelectOptionsProps {
  name: string;
  selected: number;
  setSelected: (value: string) => void;
  unit: number;
}

const ReboilerDropdown: React.FC<SelectOptionsProps> = ({ setSelected, unit}) => {
  const [options, setOptions] = React.useState<dropdown[] | [] >([]);
  const [error, setError] = React.useState<string | null>(null);

  React.useEffect(() => {
    async function getOptions() {
    try{
      const data: Reboiler[] | string   = await getReboilers(unit);
      if (typeof data === "string") {
        setError(data);
      } else {
        const transformedOptions = data.map((reboiler) => ({
          id: reboiler.id as number,
          label: reboiler.number as unknown as string,
          value: reboiler.id as number,
        }));
        setOptions(transformedOptions);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
      setError('An error occurred while fetching data.'); 
    }  finally{}}
    getOptions();
  }, []);

  return (
    <Select onValueChange={setSelected}>
      <SelectTrigger className="w-[300px]">
        <SelectValue placeholder="Selecione um reboiler" />
      </SelectTrigger>
      <SelectContent>
        <SelectGroup>
          {options.map((option) => (
            <SelectItem key={option.id} value={option.value.toString()}>
              {option.label}
            </SelectItem>
          ))}
        </SelectGroup>
      </SelectContent>
    </Select>
  );
}

export default ReboilerDropdown;