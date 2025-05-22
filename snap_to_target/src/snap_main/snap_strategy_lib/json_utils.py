#!/usr/bin/env python3
import json 
import os 
import csv

DEFAULT_PATH = '<your_default_path>'

def read_json_file(file_path, root): 

    data = None 
    try:
        # Opening JSON file
        with open(file_path, 'r') as f:
            # returns JSON object as a dictionary
            data = json.load(f)[root]
            
    except FileNotFoundError:
        print("Error: The file was not found.")
    except json.JSONDecodeError:
        print("Error: There was an error decoding the JSON data.")
    except KeyError:
        print("Error: The key %s was not found in the JSON data.", root)
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    return data 

def json_root_keys_to_list(file_path, root): 
    data = read_json_file(file_path, root)
    return list(data.keys())

def name_to_id_dict(dict, root): 
    return {item['name']: item['id'] for item in dict.get(root).values()}

def id_to_name_dict(dict, root): 
    return {item['id']: item['name'] for item in dict.get(root).values()}

def generate_csv(file_name, header_list, data_list, ids_enabled=True,
                 file_path=DEFAULT_PATH): 
    """
    Generates .csv file with given header and data list to the current 
    working directory.

    Args:
        header_list (List): List of column names as strings 
        data_list (List): List that includes data rows of same size 
                            [[col1, col2, ..., colN],..]. The amount of
                            columns is calculated from the elements in one row
    """

    if len(header_list) != len(data_list[0]): 
        print("Error: the header list contains more columns than the data \
                list")
        return
    
    if ids_enabled: 
        header_list.insert(0,'ID') 
        for i in range(1, len(data_list)+1): 
            data_list[i-1].insert(0,i)
    
    # if the filename doesn't include the post-fix, add it 
    if file_name.find(".csv") == -1:
        file_name += ".csv" 

    file_path = os.path.join(file_path, file_name)    
    os.makedirs(os.path.dirname(file_path),exist_ok=True)
    
    try: 
        with open(file_path, 'w') as f: 
            write = csv.writer(f)
            write.writerow(header_list)
            write.writerows(data_list)
    except csv.Error as e:
        print("Error in writing .csv file")

    print("File saved successfully at path %s" % file_path)
