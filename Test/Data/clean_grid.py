# converts exponents to clean

def process_occupancy_grid(input_file: str, output_file: str) -> None:
    """
    Process an occupancy grid file to convert all numeric values to -1, 0, 50, or 100.
    
    Parameters:
        input_file (str): Path to the input file containing the occupancy grid.
        output_file (str): Path to save the processed output file.
    """
    # Define the mapping function
    def map_value(value: float) -> int:
        if value == -1.0:
            return -1
        elif value == 0.0:
            return 0
        elif value == 50.0:
            return 50
        elif value == 100.0:
            return 100
        else:
            raise ValueError(f"Unexpected value in the grid: {value}")
    
    # Read, process, and write the file
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            # Convert scientific notation to float, map values, and write back as integers
            cleaned_line = " ".join(str(map_value(float(value))) for value in line.split())
            outfile.write(cleaned_line + "\n")
    
    print(f"Processed file saved to {output_file}")

# Example usage
input_path = "fully_mapped_grid_with_person.txt"  # Replace with your input file path
output_path = "cleaned_fully_mapped_grid_with_person.txt"  # Replace with your desired output file path

process_occupancy_grid(input_path, output_path)
