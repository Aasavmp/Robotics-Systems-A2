import pandas as pd
import re

# Load the Excel file into a DataFrame
df = pd.read_excel('Results Waypoint 2.xlsx')

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



# # Function to loop through each column header and create a dictionary for each header with text
# def create_dicts_from_column_headers(df):
#     header_dicts = {}
#     for header in df.columns:
#         header_text = str(header)
#         # Skip headers that are purely numeric
#         if not header_text.replace('.', '', 1).isdigit():
#             # Extract the parameters from the header text
#             param_dict = extract_parameters_from_header(header_text)
#             if param_dict['speed'] is not None:
#                 # Construct the key name only for headers with parameters found
#                 dict_key = f"{param_dict['searchtype']}_{param_dict['speed']}_{param_dict['amplitude']}_{param_dict['wavelength']}"
#                 header_dicts[dict_key] = param_dict
#     return header_dicts

# # Apply the function to create dictionaries from the column headers
# header_dicts = create_dicts_from_column_headers(df)

# Output the dictionaries for verification
# print(header_dicts)
# 


# Process the columns with text headers
def process_experiment_runs(df):
    experiments = {}
    for header in df.columns:
        header_text = str(header)
        if not header_text.replace('.', '', 1).isdigit():
            param_dict = extract_parameters_from_header(header_text)
            dict_key = f"{param_dict['searchtype']}_{param_dict['speed']}_{param_dict['amplitude']}_{param_dict['wavelength']}"
            # Initialise a dictionary for this experiment if it does not exist
            if dict_key not in experiments:
                experiments[dict_key] = {}
            # Loop to get the 10 xy pairs for each run
            for i in range(1, df[header].count(), 10):
                run_key = f"run{int((i-1)/10) + 1}"
                xy_values = df[header].iloc[i:i+10].tolist()
                # Store xy values in the experiment dictionary under the appropriate run key
                experiments[dict_key][run_key] = xy_values
    return experiments

# Apply the function to process experiment runs
experiment_data = process_experiment_runs(df)

# Output the processed data for verification
print(experiment_data)