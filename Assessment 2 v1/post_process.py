import pandas as pd
import re

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

# Load the Excel file into a DataFrame
df = pd.read_excel('Results Waypoint 2.xlsx')

# Function to loop through each column header and create a dictionary for each header with text
def create_dicts_from_column_headers(df):
    header_dicts = {}
    for header in df.columns:
        header_text = str(header)
        # Skip headers that are purely numeric
        if not header_text.replace('.', '', 1).isdigit():
            # Extract the parameters from the header text
            param_dict = extract_parameters_from_header(header_text)
            if param_dict['speed'] is not None:
                # Construct the key name only for headers with parameters found
                dict_key = f"{param_dict['searchtype']}_{param_dict['speed']}_{param_dict['amplitude']}_{param_dict['wavelength']}"
                header_dicts[dict_key] = param_dict
    return header_dicts

# Apply the function to create dictionaries from the column headers
header_dicts = create_dicts_from_column_headers(df)

# Output the dictionaries for verification
print(header_dicts)

