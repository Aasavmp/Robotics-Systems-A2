# # Output the processed data for verification
# print(experiment_data)
import pandas as pd
import re
import math
import numpy as np
import matplotlib.pyplot as plt

# Load the Excel file into a DataFrame
df_one = pd.read_excel('Results Waypoint 1.xlsx')
df_two = pd.read_excel('Results Waypoint 2.xlsx')

# Extract the first 10 waypoints from the first two columns and convert to list of tuples
waypoints_one = [tuple(map(float, x)) for x in df_one.iloc[:10, :2].values]
waypoints_two = [tuple(map(float, x)) for x in df_two.iloc[:10, :2].values]

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
def process_experiment_runs(df, waypoints):
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
    waypoints_found = {}
    waypoints_distance_data = {}
    waypoints_number_found_data = {}
    run_accuracy_data = {}
    run_accuracy_data_firstpoint = {}
    run_accuracy_data_lastpoint = {}
    time_data = {}
    time_per_waypoint_data = {}

    waypoints_distance_data_x = {}
    waypoints_distance_data_y = {}
    run_accuracy_data_x = {}
    run_accuracy_data_y = {}
    run_accuracy_data_x_firstpoint = {}
    run_accuracy_data_y_firstpoint = {}
    run_accuracy_data_x_lastpoint = {}
    run_accuracy_data_y_lastpoint = {}

    # Get the experimental data
    experiment_data = process_experiment_runs(df, waypoints)
    # Loop through the experiment data
    for key, value in experiment_data.items():
        # Store the distance to waypoints for each run in the experiment
        waypoints_found[key] = {}
        waypoints_distance_data[key] = {}
        waypoints_number_found_data[key] = []
        time_data[key] = []
        run_accuracy_data[key] = []
        run_accuracy_data_firstpoint[key] = []
        run_accuracy_data_lastpoint[key] = []
        time_per_waypoint_data[key] = []

        waypoints_distance_data_x[key] = {}
        waypoints_distance_data_y[key] = {}

        run_accuracy_data_x[key] = []
        run_accuracy_data_y[key] = []
        run_accuracy_data_x_firstpoint[key] = []
        run_accuracy_data_y_firstpoint[key] = []
        run_accuracy_data_x_lastpoint[key] = []
        run_accuracy_data_y_lastpoint[key] = []

        # Start run index
        run_index = 1
        # Loop through each run in the experiment
        for run_key, run_values in value.items():
            # Store the distance to waypoints

            # If the run_key is the same as the name of the filtered runs store those waypoints
            if run_key == f'run{run_index}':
                waypoints_found[key][run_key] = run_values
                waypoints_distance_data[key][run_key] = []

                waypoints_distance_data_x[key][run_key] = []
                waypoints_distance_data_y[key][run_key] = []
                # Loop through the run values and calculate the distance to waypoints
                for xy in run_values:
                    for wp in waypoints:
                        dist = distance(xy, wp)
                        if dist <= 0.025:
                            # Store the distance to waypoints
                            waypoints_distance_data[key][run_key].append(dist)

                            # Store the x and y distances to waypoints
                            waypoints_distance_data_x[key][run_key].append(abs(xy[0] - wp[0]))
                            waypoints_distance_data_y[key][run_key].append(abs(xy[1] - wp[1]))

                # Store the number of waypoints found
                waypoints_number_found_data[key].append(len(waypoints_distance_data[key][run_key]))

                # Store the distance to the first and last waypoints
                run_accuracy_data_firstpoint[key].append(waypoints_distance_data[key][run_key][0])
                run_accuracy_data_lastpoint[key].append(waypoints_distance_data[key][run_key][-1])

                # Calculate the average distance to waypoints
                average_distance = np.mean(waypoints_distance_data[key][run_key])
                run_accuracy_data[key].append(average_distance)

                # Calculate the average x and y distance to waypoints
                run_accuracy_data_x[key].append(np.mean(waypoints_distance_data_x[key][run_key]))
                run_accuracy_data_y[key].append(np.mean(waypoints_distance_data_y[key][run_key]))

                # Calculate the average x and y distance to first waypoint
                run_accuracy_data_x_firstpoint[key].append(waypoints_distance_data_x[key][run_key][0])
                run_accuracy_data_y_firstpoint[key].append(waypoints_distance_data_y[key][run_key][0])

                # Calculate the average x and y distance to last waypoint
                run_accuracy_data_x_lastpoint[key].append(waypoints_distance_data_x[key][run_key][-1])
                run_accuracy_data_y_lastpoint[key].append(waypoints_distance_data_y[key][run_key][-1])
                        
            # If the run_key is the same as the name of the time values store those time values
            if run_key == f'run{run_index}_time_values':
                time_data[key].append(run_values[0])  

                # Calculate the time per waypoint
                time_per_waypoint_data[key].append(run_values[0] / waypoints_number_found_data[key][-1])
            
                # Increment the run index
                run_index += 1

        
    # return all the data
    return waypoints_found, waypoints_distance_data, waypoints_number_found_data, run_accuracy_data, run_accuracy_data_firstpoint, run_accuracy_data_lastpoint, time_data, time_per_waypoint_data, run_accuracy_data_x, run_accuracy_data_y, waypoints_distance_data_x, waypoints_distance_data_y, run_accuracy_data_x_firstpoint, run_accuracy_data_y_firstpoint, run_accuracy_data_x_lastpoint, run_accuracy_data_y_lastpoint

# Function to box plot the data
def box_plot_data(data, title, x_label, y_label):
    max_length = max(len(lst) for lst in data.values())
    for key, lst in data.items():
        if len(lst) < max_length:
            data[key] = lst + [np.nan] * (max_length - len(lst))
    # Create a DataFrame from the data
    df = pd.DataFrame(data)
    # Create a box plot
    ax = df.boxplot(rot=0)
    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    # Fix the y axis to start from 0
    ax.set_ylim(bottom=0)
    # Display the plot
    plt.show()

# Function to plot both files on the same graph
def plot_both_files(data_one, data_two, title, x_label, y_label):
    max_length = max(len(lst) for lst in data_one.values())
    for key, lst in data_one.items():
        if len(lst) < max_length:
            data_one[key] = lst + [np.nan] * (max_length - len(lst))
    max_length = max(len(lst) for lst in data_two.values())
    for key, lst in data_two.items():
        if len(lst) < max_length:
            data_two[key] = lst + [np.nan] * (max_length - len(lst))
    # Create a DataFrame from the data
    df_one = pd.DataFrame(data_one)
    df_two = pd.DataFrame(data_two)
    # Create a box plot for both files on the same graph but with different colors
    ax = df_one.boxplot(rot=0, positions=np.array(range(len(df_one.columns))) * 2.0, patch_artist=True, boxprops=dict(facecolor='blue'))
    df_two.boxplot(rot=0, positions=np.array(range(len(df_two.columns))) * 2.0 + 0.5, patch_artist=True, boxprops=dict(facecolor='red'))
    ax.set_title(title)

    # Set the labels at the middle of two boxes
    ax.set_xticks(np.array(range(len(df_one.columns))) * 2.0 + 0.25)
    ax.set_xticklabels([key for key in data_one.keys()])
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)

    # Make the labels vertical
    plt.xticks(rotation=90)

    # Fix the y axis to start from 0
    ax.set_ylim(bottom=0)
    # Display the plot
    plt.show()

waypoints_found, waypoints_distance_data, waypoints_number_found_data, run_accuracy_data, run_accuracy_data_firstpoint, run_accuracy_data_lastpoint, time_data,time_per_waypoint_data, run_accuracy_data_x, run_accuracy_data_y, waypoints_distance_data_x, waypoints_distance_data_y, run_accuracy_data_x_firstpoint, run_accuracy_data_y_firstpoint, run_accuracy_data_x_lastpoint, run_accuracy_data_y_lastpoint = calculate_processed_data(df_one, waypoints_one)
waypoints_found_two, waypoints_distance_data_two, waypoints_number_found_data_two, run_accuracy_data_two, run_accuracy_data_firstpoint_two, run_accuracy_data_lastpoint_two, time_data_two, time_per_waypoint_data_two, run_accuracy_data_x_two, run_accuracy_data_y_two, waypoints_distance_data_x_two, waypoints_distance_data_y_two, run_accuracy_data_x_firstpoint_two, run_accuracy_data_y_firstpoint_two, run_accuracy_data_x_lastpoint_two, run_accuracy_data_y_lastpoint_two = calculate_processed_data(df_two, waypoints_two)

# lot_both_files(run_accuracy_data, run_accuracy_data_two, 'Average Distance to Waypoints', 'Experiment', 'Average Distance to Waypoints') 
# plot_both_files(time_data, time_data_two, 'Time to Complete Experiment', 'Experiment', 'Time (s)')
# plot_both_files(time_per_waypoint_data, time_per_waypoint_data_two, 'Time per Waypoint', 'Experiment', 'Time per Waypoint (s)')
# plot_both_files(run_accuracy_data_firstpoint, run_accuracy_data_firstpoint_two, 'Distance to First Waypoint', 'Experiment', 'Distance to First Waypoint')
# plot_both_files(run_accuracy_data_lastpoint, run_accuracy_data_lastpoint_two, 'Distance to Last Waypoint', 'Experiment', 'Distance to Last Waypoint')
# plot_both_files(run_accuracy_data_x, run_accuracy_data_x_two, 'Average X Distance to Waypoints', 'Experiment', 'Average X Distance to Waypoints')
# plot_both_files(run_accuracy_data_y, run_accuracy_data_y_two, 'Average Y Distance to Waypoints', 'Experiment', 'Average Y Distance to Waypoints')
# box_plot_data(waypoints_number_found_data, 'Number of Waypoints Found', 'Experiment', 'Number of Waypoints Found')
# box_plot_data(run_accuracy_data, 'Average Distance to Waypoints', 'Experiment', 'Average Distance to Waypoints')
# box_plot_data(time_data, 'Time to Complete Experiment', 'Experiment', 'Time (s)')
# box_plot_data(time_per_waypoint_data, 'Time per Waypoint', 'Experiment', 'Time per Waypoint (s)')
# box_plot_data(run_accuracy_data_firstpoint, 'Distance to First Waypoint', 'Experiment', 'Distance to First Waypoint')
# box_plot_data(run_accuracy_data_lastpoint, 'Distance to Last Waypoint', 'Experiment', 'Distance to Last Waypoint')
# box_plot_data(run_accuracy_data_x, 'Average X Distance to Waypoints', 'Experiment', 'Average X Distance to Waypoints')
# box_plot_data(run_accuracy_data_y, 'Average Y Distance to Waypoints', 'Experiment', 'Average Y Distance to Waypoints')
# box_plot_data(run_accuracy_data_x_firstpoint, 'X Distance to First Waypoint', 'Experiment', 'X Distance to First Waypoint')
# box_plot_data(run_accuracy_data_y_firstpoint, 'Y Distance to First Waypoint', 'Experiment', 'Y Distance to First Waypoint')
# box_plot_data(run_accuracy_data_x_lastpoint, 'X Distance to Last Waypoint', 'Experiment', 'X Distance to Last Waypoint')
# box_plot_data(run_accuracy_data_y_lastpoint, 'Y Distance to Last Waypoint', 'Experiment', 'Y Distance to Last Waypoint')

# Plot the average distance to waypoints of each run
# for key, value in waypoints_number_found_data.items():
#     plt.plot(value, label=key)
# plt.legend(title='Experiment')
# plt.title('Average Distance to Waypoints')
# plt.xlabel('Run')
# plt.ylabel('Average Distance to Waypoints')
# plt.show()

# Output the processed data for verification
# print(experiment_data)