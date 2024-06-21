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
import { Unit, dropdown } from '@/components/Columns';
import { Value } from '@radix-ui/react-select';

interface SelectOptionsProps {
  selected: number;
  setSelected: (value: string) => void;
}

const UnitDropdown: React.FC<SelectOptionsProps> = ({ setSelected }) => {
  const [options, setOptions] = React.useState<dropdown[] | []>([]);
  const [error, setError] = React.useState<string | null>(null);

  React.useEffect(() => {
    async function getOptions() {
      try {
        const data: Unit[] | string = await getUnities();
        if (typeof data === "string") {
          setError(data);
        } else {
          const transformedOptions = data.map((unit) => ({
            id: unit.id,
            label: unit.city + " - " + unit.state,
            value: unit.id,
          }));
          setOptions(transformedOptions);
        }
      } catch (error) {
        console.error('Error fetching data:', error);
        setError('An error occurred while fetching data.');
      } finally { }
    }
    getOptions();
  }, []);

  return (
    <Select onValueChange={setSelected(Number(Value))}>
      <SelectTrigger className="w-[300px]">
        <SelectValue placeholder="Select a unit" />
      </SelectTrigger>
      <SelectContent>
        <SelectGroup>
          <SelectLabel>Units</SelectLabel>
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

export default UnitDropdown;