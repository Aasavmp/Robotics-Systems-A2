# import pandas as pd
# import re
# import math

# # Load the Excel file into a DataFrame
# df = pd.read_excel('Results Waypoint 2.xlsx')

# # Extract the first 10 waypoints from the first two columns
# waypoints = df.iloc[:10, :2].values

# # Store these waypoints in a variable
# waypoints_variable = waypoints

# # Function to parse the header and create a dictionary
# def extract_parameters_from_header(text):
#     searchtype = text.split()[0]
#     speed = re.search(r'Speed (\d+)', text)
#     amplitude = re.search(r'A(\d+\.\d+)', text)
#     wavelength = re.search(r'W(\d+\.\d+)', text)
#     return {
#         'searchtype': searchtype,
#         'speed': int(speed.group(1)) if speed else None,
#         'amplitude': float(amplitude.group(1)) if amplitude else None,
#         'wavelength': float(wavelength.group(1)) if wavelength else None
#     }





# # Function to process the experiment runs from the 3rd column onwards
# def process_experiment_runs(df):
#     experiments = {}
#     col = 2  # Start from the third column (index 2)
#     while col < len(df.columns):
#         header_text = str(df.columns[col])
#         # Only process if header is not purely numeric and not blank, and check if the column itself is not blank
#         if not header_text.replace('.', '', 1).isdigit() and header_text.strip() and pd.notna(df.iloc[0, col]):
#             param_dict = extract_parameters_from_header(header_text)
#             dict_key = f"{param_dict['searchtype']}_{param_dict['speed']}_{param_dict['amplitude']}_{param_dict['wavelength']}"
#             experiments[dict_key] = {}
#             run_index = 1
#             next_col = col  # Start from the current header column
#             while next_col < len(df.columns) and pd.notna(df.iloc[0, next_col]):
#                 run_key = f"run{run_index}"
#                 xy_values = df.iloc[1:11, next_col].dropna().tolist()  # Assume the next 10 rows contain the data
#                 experiments[dict_key][run_key] = xy_values
#                 run_index += 1
#                 next_col += 1
#             col = next_col  # Move the outer loop's index to the column after the last non-blank column
#         else:
#             col += 1
#     return experiments

# # Apply the function to process experiment runs
# experiment_data = process_experiment_runs(df)

# # Output the processed data for verification
# print(experiment_data)
import pandas as pd
import re
import math

# Load the Excel file into a DataFrame
df = pd.read_excel('Results Waypoint 2.xlsx')

# Extract the first 10 waypoints from the first two columns and convert to list of tuples
waypoints = [tuple(map(float, x)) for x in df.iloc[:10, :2].values]

# Function to calculate Euclidean distance between two points
def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Function to parse the header and create a dictionary
def extract_parameters_from_header(text):
    searchtype = text.split()[0]
    speed = re.search(r'Speed (\d+)', text)
    amplitude = re.search(r'A(\d+\.\d+)', text)
    wavelength = re.search(r'W(\d+\.\d+)', text)
    return {
        'searchtype': searchtype,
        'speed': int(speed.group(1)) if speed else None,
        'amplitude': float(amplitude.group(1)) if amplitude else None,
        'wavelength': float(wavelength.group(1)) if wavelength else None
    }

# Function to process the experiment runs from the 3rd column onwards
def process_experiment_runs(df):
    experiments = {}
    col = 2  # Start from the third column (index 2)
    while col < len(df.columns):
        header_text = str(df.columns[col])
        # Only process if header is not purely numeric and not blank, and check if the column itself is not blank
        if not header_text.replace('.', '', 1).isdigit() and header_text.strip() and pd.notna(df.iloc[0, col]):
            param_dict = extract_parameters_from_header(header_text)
            dict_key = f"{param_dict['searchtype']}_{param_dict['speed']}_{param_dict['amplitude']}_{param_dict['wavelength']}"
            experiments[dict_key] = {}
            run_index = 1
            next_col = col  # Start from the current header column
            while next_col < len(df.columns) and pd.notna(df.iloc[0, next_col]):
                run_key = f"run{run_index}"
                xy_values = df.iloc[0:15, next_col].dropna().tolist()  # Assume the next 15 rows contain the data
                time_values = df.iloc[15:16, next_col].dropna().tolist()  # Assume the next row contains the time data
                # Ensure xy_values are floats
                original_xy_values = [tuple(map(float, xy.split(','))) for xy in xy_values if ',' in xy]

                # Filter xy_values based on distance to waypoints
                filtered_xy_values = [xy for xy in original_xy_values if any(distance(xy, wp) <= 0.025 for wp in waypoints)]
                experiments[dict_key][run_key] = filtered_xy_values
#
                # store original coordinates and number of waypoints found
                # experiments[dict_key][f'{run_key}_original_coordinates'] = original_xy_values
                # store filtered number of waypoints found
                # experiments[dict_key][f'{run_key}_num_waypoints_found'] = len(filtered_xy_values)
                # store time values
                experiments[dict_key][f'{run_key}_time_values'] = time_values
                run_index += 1
                next_col += 1
            col = next_col  # Move the outer loop's index to the column after the last non-blank column
        else:
            col += 1
    return experiments

# Function to process the experiment runs and compare the distance to waypoints
def calculate_processed_data(df, waypoints):
    # Dictionary to store the processedhfd data
    waypoints_distance_data = {}
    waypoints_number_found_data = {}
    run_accuracy_data = {}
    time_data = {}
    time_per_waypoint_data = {}

    # Get the experimental data
    experiment_data = process_experiment_runs(df)
    # Loop through the experiment data
    for key, value in experiment_data.items():
        # Store the distance to waypoints for each run in the experiment
        waypoints_distance_data[key] = {}
        waypoints_number_found_data[key] = []
        time_data[key] = []
        run_accuracy_data[key] = []
        time_per_waypoint_data[key] = []

        # Start run index
        run_index = 1
        # Loop through each run in the experiment
        for run_key, run_values in value.items():
            # Store the distance to waypoints

            # If the run_key is the same as the name of the filtered runs store those waypoints
            if run_key == f'run{run_index}':
                waypoints_distance_data[key][run_key] = []
                # Loop through the run values and calculate the distance to waypoints
                for xy in run_values:
                    for wp in waypoints:
                        dist = distance(xy, wp)
                        if dist <= 0.025:
                            waypoints_distance_data[key][run_key].append(dist)

                # Store the number of waypoints found
                waypoints_number_found_data[key].append(len(waypoints_distance_data[key][run_key]))

                # Calculate the average distance to waypoints
                average_distance = sum(waypoints_distance_data[key][run_key]) / len(waypoints_distance_data[key][run_key])
                run_accuracy_data[key].append(average_distance)
                        
            # If the run_key is the same as the name of the time values store those time values
            if run_key == f'run{run_index}_time_values':
                time_data[key].append(run_values[0])  

                # Calculate the time per waypoint
                time_per_waypoint_data[key].append(run_values[0] / waypoints_number_found_data[key][-1])
            
                # Increment the run index
                run_index += 1

        
    # return all the data
    return waypoints_distance_data, waypoints_number_found_data, run_accuracy_data, time_data, time_per_waypoint_data

# Apply the function to process experiment runs
experiment_data = process_experiment_runs(df)

waypoints_distance_data, waypoints_number_found_data, run_accuracy_data, time_data, time_per_waypoint_data = calculate_processed_data(df, waypoints)

print(experiment_data)

# Output the processed data for verification
# print(experiment_data)