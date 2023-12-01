# Open the file
with open('SWV_Trace_Log.txt', 'r') as file:
    lines = file.readlines()

# Initialize variables
data_1_cycles = []
data_2_cycles = []
data_3_cycles = []

# Iterate through the lines and extract necessary data
for line in lines:
    values = line.split(';')
    if len(values) >= 6 and values[1] == '"ITM Port 31"':
        trace_type = values[1].strip('"')
        data = int(values[2].strip('"'))
        cycles = int(values[3].strip('"'))

        if trace_type == 'ITM Port 31':
            if data == 1:
                data_1_cycles.append(cycles)
            elif data == 2:
                data_2_cycles.append(cycles)
            elif data == 3:
                data_3_cycles.append(cycles)

# Calculate averages
avg_data_1_to_2_cycles = sum(data_2_cycles) / len(data_2_cycles) - sum(data_1_cycles) / len(data_1_cycles)
avg_data_2_to_3_cycles = sum(data_3_cycles) / len(data_3_cycles) - sum(data_2_cycles) / len(data_2_cycles)

# Print results
print(f"Average cycles for the FFT: {int(avg_data_1_to_2_cycles)}")
print(f"Average cycles for the inverse FFT: {int(avg_data_2_to_3_cycles)}")
