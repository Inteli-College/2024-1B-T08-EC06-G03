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
import { getRobots } from '@/api/robot';
import { Robot, dropdown } from '@/components/Columns';

interface SelectOptionsProps {
  name: string;
  selected: number;
  setSelected: (value: string) => void;
  unit: number;
}

const RobotDropdown: React.FC<SelectOptionsProps> = ({ setSelected, unit }) => {
  const [options, setOptions] = React.useState<dropdown[] | []>([]);
  const [error, setError] = React.useState<string | null>(null);

  React.useEffect(() => {
    async function getOptions() {
      try {
        const data: Robot[] | string = await getRobots(unit);
        if (typeof data === "string") {
          setError(data);
        } else {
          const transformedOptions = data.map((robot) => ({
            id: robot.id as number,
            label: robot.nickname,
            value: robot.id as number,
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
    <Select onValueChange={setSelected}>
      <SelectTrigger className="w-[300px]">
        <SelectValue placeholder="Selecione um robô" />
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

export default RobotDropdown;