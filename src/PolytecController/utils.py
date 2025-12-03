"""
Author: Maxime Leurquin
Date: Feb-2024
Description: A few utils used in other classes
"""
import os
import numpy as np
from typing import List
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import List

def make_border(ax, color, **kwargs):
    """
    Adds a colored border to the given Axes object.
    
    Parameters:
    ax (matplotlib.axes.Axes): The Axes object to which the border will be added.
    color (str): The color of the border.
    **kwargs: Additional keyword arguments for customization.
        - border_width (int, optional): The width of the border. Default is 10.
    """

    border_width = kwargs.get('border_width', 10)
    rect = patches.Rectangle(
        (0, 0), 1, 1, 
        transform=ax.transAxes,
        linewidth=border_width, 
        edgecolor=color, 
        facecolor='none', 
        clip_on=False
    )
    
    ax.add_patch(rect)
    return

def find_common_string(list1:List[str], list2:List[str])->List[str]:
    """
    Find the strings that are present in both list1 and list2.
    """
    set1 = set(list1)
    set2 = set(list2)
    return list(set1 & set2)

def relative_to_absolute(filepath):
    #most of polytec functions only accept absolute paths
    if not os.path.isabs(filepath):
        filepath=os.path.abspath(filepath)
    return filepath

def change_file_extension(filename, new_extension):
    name, extension = os.path.splitext(filename)
    new_filename = f"{name}.{new_extension}"
    return new_filename

def make_dir(filepath):
    #if given a filepath, create the folder to that filepath
    #if given a folder, create that folder.
    if os.path.splitext(filepath)[1]=="": #the given filepath is a folder
        os.makedirs(filepath,exist_ok=True)
    else: #the given filepath is a file
        directory = os.path.dirname(filepath)
        os.makedirs(directory,exist_ok=True)
    return

def postpend_filename(filepath,text):
     #adds text at the end of the filename
     name, extension = os.path.splitext(filepath)
     return f"{name}{text}{extension}"

def is_even(num:int)->bool:
    cond=(num % 2) ==0
    return cond

def uninterleave_data(interleaved_data):
    #given [[a,b,a,b],[a,b,a,b],...]] return [a,a,...], [b,b,...]
    if not is_even(interleaved_data.shape[1]):
        raise RuntimeError(f"The interleaving data in the svd file is missing one element: shape={interleaved_data.shape}. This is strange")
    a = interleaved_data[:,::2]#select every second element, starting from the first
    b = interleaved_data[:,1::2] #select every second element, starting from the second.
    return a,b