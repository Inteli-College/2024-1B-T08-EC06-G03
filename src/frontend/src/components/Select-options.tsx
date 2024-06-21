import React from "react"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"

interface SelectOptionsProps {
    options: string[];
    selected: string | number;
    
    setSelected: (value: string) => void;
}

const SelectOptions: React.FC<SelectOptionsProps> = ({ options, selected, setSelected }) => {
    return (
        <Select onValueChange={setSelected}>
            <SelectTrigger>
                <SelectValue>{selected}</SelectValue>
            </SelectTrigger>
            <SelectContent>
                {options.map((option) => (
                    <SelectItem key={option} value={option}>
                        {option}
                    </SelectItem>
                ))}
            </SelectContent>
        </Select>
    )
}

export default SelectOptions;