"""
Author: Maxime Leurquin
Date: Feb-2024
Description: Functions used both in Polyfile.py and PSV.py
"""
import numpy as np
from collections import Counter
from typing import List,Tuple, Any
from .enumerations import PTCScanStatus
from .utils import find_common_string

def get_scanpoints_status_statistics(infos):
    """
    Return a Counter with the number of scanpoints for each status number
    eg: Counter({1: 1500, 16: 300, 128: 100, 516: 50})
    """
    measpoints=infos.meas_points.meas_points_list
    scanstatuses=[measpoint.scan_status for measpoint in measpoints]
    counts=Counter(scanstatuses)
    return counts

def translate_status(status: int) -> List[str]:
    """
    given a status number, return what it means. 
    A single status number can mean multiple statuses which is why we return a list of str
    """
    result = []
    if status==0:
        return ["ptcScanStatusNotMeasured"] #else it returns ptcScanStatusNone which can causes problems.
    for key, value in PTCScanStatus.items():
        if status & value:
            result.append(key)
    return result

def find_statuses_matches(statuses:List[int],target_statuses:List[str])->np.ndarray[Any, np.dtype[np.bool_]]:
    """
    Given a list of statuses numbers return which ones match at least one of the given target statuses
    eg: statuses=[516,1,1,16]
    target_statuses=["ptcScanStatusDisabled","ptcScanStatusOverrange"]
    -->[True,False,False,True]
    """
    results=np.ones_like(statuses,dtype=bool)
    for i,status_int in enumerate(statuses):
        statuses_str=translate_status(status_int)#this is a list
        is_match=len(find_common_string(statuses_str,target_statuses))>=1
        results[i]=is_match
    return results

def check_measurement_validity(statuses:List[int])->np.ndarray[Any, np.dtype[np.bool_]]:
    """
    Return true if the status is not invalid
    Remember that one status may mean multiple things, eg: 516 means ptcScanStatusInterpolate AND ptcScanStatusOverrange
    eg.: check_measurement_validity([1,516])-->[True,False]
    """
    INVALID_STATUSES=["ptcScanStatusOverrange","pctScanStatusInterpolate","ptcScanStatusInterpolationFailed",
                      "ptcScanStatusInvalidated","ptcScanStatusVideoTriangulationFailed","ptcScanStatusNone",
                      "ptcScanStatusNotMeasured","ptcScanStatusDisabled"]
    return ~find_statuses_matches(statuses=statuses,target_statuses=INVALID_STATUSES)

def get_scanpoints_statuses(infos)->List[int]:
    """
    Return the status number of each scanpoint
    eg: return a list of ints: eg: 16,128,2,1,516
    the number can mean multiple status at the same time: 516 means interpolate & overrange
    """
    measpoints=infos.meas_points.meas_points_list
    scanstatuses=[measpoint.scan_status for measpoint in measpoints]
    return scanstatuses

def get_scanpoints(infos,category:str,ref_frame:str)->np.ndarray[Any, np.dtype[np.floating]]:
    """
    Retrieves scan points based on the specified category and reference frame.
    A scan point is a point where the laser stops and takes a measurement for some time.
    Scan points are returned as a list of coordinates in the format: [[x1, y1], [x2, y2], [x3, y3], ...]

    :param infos: polytec infos object (casted, not in COM form).
    :param category: str, category of scan points to retrieve. Options: All, Enabled, Disabled, Interpolated, Overrange, Valid.
    :param ref_frame: str, reference frame for the coordinates. Options: raw, video, pixel.
                      - raw: the raw coordinates stored in the file. The origin of the coordinate system is the point at
                        which the laser beam is emitted in 0° direction from the front of the scanning head. See PSV Software manual p 4.14: Scanning Head Control: 2D Point 
                      - video: the video coordinates.
                      - pixel: (0,0) is the top left corner of the image stored in the svd file. Coordinates are given in pixels. This is useful to overlay points on the image.
                      - For more info on reference frames read the Theory Manual section 14.3:Coordinate Systems
    :return: array-like, scan points based on the specified category and reference frame.
    """
    match ref_frame:
        case "raw":
            scanpoints=infos.meas_points.coordinates_to_array
            scanpoints=np.array(scanpoints).reshape(-1, 3)
        case "video"|"pixel":
            scanpoints=infos.meas_points.video_coordinates_to_array
            scanpoints=np.array(scanpoints).reshape(-1, 2)
            if ref_frame=="pixel":
                scanpoints=convert_Video2Image_coordinates(infos,scanpoints)
        case _:
            raise ValueError(f"{ref_frame=} but expected raw/video/pixel")
    scanstatuses=get_scanpoints_statuses(infos)
    match category:
        case "All":
            return scanpoints
        case "Enabled":
            mask=find_statuses_matches(scanstatuses,target_statuses=["ptcScanStatusDisabled"])
            return scanpoints[~mask]
        case "Disabled"|"Valid"|"Overrange"|"Interpolate"|"Hidden"|"InterpolationFailed"|"Invalidated"|"InvalidFrames"|"NotReachable"|"Optimal"|"VTFailed":
            mask=find_statuses_matches(scanstatuses,target_statuses=[f"ptcScanStatus{category}"])
            return scanpoints[mask]
        case _:
            raise ValueError(f"{category=} but expected All/Enabled/{list(PTCScanStatus.keys())}")

def get_image_size(infos)->Tuple[int,int]:
    """
    Retrieves the size of the image
    :param infos: polytec infos object (casted, not in COM form).
    :return: tuple (imgx, imgy), size of the image.
    """
    assert infos.has_video_mapping_info,"File does not have VideoMappingInfo"
    camera_size=infos.video_mapping_info.camera_size
    imgx=camera_size.image_x
    imgy=camera_size.image_y
    return imgx,imgy

def convert_Video2Image_coordinates(infos,video_points:np.ndarray)->np.ndarray[Any, np.dtype[np.floating]]:
    """
    Convert video coordinates to image coordinates
    video_points:list of points to convert. Ex: [[x1,y1],..[xn,yn]]
    Useful to show scanpoints on the microscope image
    """
    imgx,imgy=get_image_size(infos)
    left,top,right,bottom=infos.meas_points.video_rect #the video rectangle defining the video coordinate system
    scale_image_x = right-left
    scale_image_y = top-bottom
    
    video_points = np.array(video_points)
    ptx = imgx * (video_points[:, 0] - left) / scale_image_x
    pty = imgy - (imgy * (video_points[:, 1] - bottom) / scale_image_y)
    transformed_points = np.column_stack([ptx, pty])
    return transformed_points

def convert_Image2Video_coordinates(infos,img_points:np.ndarray)->np.ndarray[Any, np.dtype[np.floating]]:
    """
    Convert back image coordinates to video coordinates
    """
    imgx,imgy=get_image_size(infos)
    left,top,right,bottom=infos.meas_points.video_rect
    scale_image_x = right-left
    scale_image_y = top-bottom
    
    img_points=np.array(img_points)
    video_x=img_points[:,0]*(scale_image_x/imgx) + left
    video_y=-(img_points[:,1]-imgy)*(scale_image_y/imgy) + bottom
    transformed_points = np.column_stack([video_x, video_y])
    return transformed_points

def transform_scanpoints_for_cropped_image(scanpoints,cropbox):
    """
    When the image is cropped the pixel coordinates of the scanpoints need to be adjusted.
    scanpoints format: [[x1, y1], [x2, y2], [x3, y3], ...]
    """
    left, upper, right, lower = cropbox # Extract the crop parameters
    transformed_scanpoints = [[x - left, y - lower] for x, y in scanpoints]
    return np.array(transformed_scanpoints)