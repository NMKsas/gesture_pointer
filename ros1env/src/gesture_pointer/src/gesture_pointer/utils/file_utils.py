#!/usr/bin/env python3
# Util functions for generating and reading .csv files 
import csv
import os 

def generate_csv(file_name, file_path, header_list, data_list, 
                 ids_enabled=True): 
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
                list.")
        print("Given header list:")
        print(header_list)
        print("First data row:")
        print(data_list)
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


def read_corners(file_name, file_path, is_header=True): 
    """
    Read corners from a .csv file. By default, the (x,y,z) coordinates for 
    each corner are defined in columns (2,3,4) of each .csv row. 

    Args:
        file_name (str): The file name.
        path (str): The file path. 
        is_header (bool, optional): True, when the data includes header that 
                                    should be skipped; false otherwise. 

    Returns:
        List: List of corners, [(x,y,z), ... , (x,y,z)], None if file not found
    """

    corners = []
    
    try: 
        with open(file_path + "/" + file_name , newline='') as csvfile:
            reader = csv.reader(csvfile)

            # skip the header if needed
            if is_header:
                next(reader)
            
            # read the x,y,z coordinates from default columns 
            for row in reader:
                x, y, z = float(row[2]), float(row[3]), float(row[4]) 
                corners.append([x,y,z])
    except FileNotFoundError as e: 
        print("File not found.")
        return None 
    
    # Convert lists to numpy arrays
    return corners
