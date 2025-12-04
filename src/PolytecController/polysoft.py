"""
Author: Maxime Leurquin
Date: Feb-2024
Description: Class to control the software that controls polytec Laser Doppler Vibrometers like the MSA500-MSA600-Vibroflex
Support for Polytec Scan Viewer (PSV) and MSV and VibSoft
Reference: Basic Engine Manual.chm which is found in the installation folder of psv.
"""
import time
import os
import shutil
import cv2
from win32com.client import Dispatch #, gencache
import numpy as np
from pypylon import pylon
import threading

from .enumerations import (
    PTCSettings, PTCAcqState, PTCApplicationMode, PTCScanMode, 
    PTCScanCaps, PTCSnapshotType, PTCWindowType, PTCFileFormat, PTCAcqStartMode
)
from .classes import Infos
from . import polytec_common
from . import utils as ut

def create_temporary_folder(folder_name="temp"):
    temp_path = os.path.join(os.getcwd(), folder_name)
    if not os.path.exists(temp_path):
        os.makedirs(temp_path)
    return temp_path

class Polytec_software():
    def __init__(self,namespace:str):
        self.namespace=namespace
        match namespace:
            case "PSV":
                #acq_instance=Dispatch(f"{namespace}.AcquisitionInstance")
                #self.app=acq_instance.GetApplication(True)# app is launched if not already running
                self.app= Dispatch(f'{namespace}.Application')
            case "VibSoft":
                #create the instance if it not already done, otherwise link the opened instance
                self.app = Dispatch(f'{namespace}.Application')
            case _:
                raise ValueError(f"namespace should be either PSV or VibSoft but was {namespace=}")
        
        self.temp_folder=create_temporary_folder(f"{namespace}_temp_folder")
        time.sleep(5)#wait for software to finish launching.
        return

    def __enter__(self):
        self.open()
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return

    def clear_temp_directory(self):
        if self.temp_folder is not None:
            shutil.rmtree(self.temp_folder)
            self.temp_folder=None
        return

    def open(self):
        self.app.Activate()
        return

    def close(self):
        self.wait_for_scan_end()
        self.clear_temp_directory()
        self.app.Quit()
        return
    
    def wait_for_scan_end(self):
        while(self.app.Acquisition.State !=PTCAcqState["ptcAcqStateStopped"]):
            time.sleep(1)
        return
    
    def save_current_settings(self,savepath:str):
        savepath=ut.relative_to_absolute(savepath)
        self.app.Settings.Save(savepath)
        return

    def load_settings_from_file(self,settings_filepath:str): 
        settings_filepath=ut.relative_to_absolute(settings_filepath)
        self.app.Settings.Load(settings_filepath,PTCSettings["ptcSettingsAll"])
        return

class Psv(Polytec_software): #should work for MSV as well.
    def __init__(self):
        super().__init__('PSV')
        self.switch_to_acquisition_mode()
        self.infos=Infos.from_com(self.app.Acquisition.Infos)

    def switch_to_acquisition_mode(self):
        if self.app.Mode!=PTCApplicationMode["ptcApplicationModeAcquisition"]:
            #self.app.Quit()
            acq_instance=Dispatch(f"{self.namespace}.AcquisitionInstance")
            self.app=acq_instance.GetApplication(True)
        else:
            print("app is already in acquisition mode")
        return True

    def trigger_scan(self,savepath:str,scanMode=PTCScanMode["ptcScanAll"],scanCaps=PTCScanCaps["ptcScanCapsNone"]):
        savepath=ut.relative_to_absolute(savepath)
        ut.make_dir(savepath)
        self.app.Acquisition.ScanFileName=savepath
        self.app.Acquisition.ScanEx(scanMode,scanCaps)
        return
    
    def set_laser_intensity(infos,laser_intensity:int):
        raise NotImplementedError("Not possible to do in PSV10.3, maybe in PSV10.4 when it is released.")
        scanhead_control=infos.scanHeadDevicesInfo.scanHeadDevices[0].scanHeadControl
        scanhead_control.WriteLaserIntensityData(laser_intensity)
        return
    
    def export_snapshot(self,savepath:str):
        _, extension = os.path.splitext(savepath)
        if extension not in [".png",".jpg"]:
            raise ValueError("image extension must be png or jpg")
        savepath=ut.relative_to_absolute(savepath)
        ut.make_dir(savepath)
        self.app.Acquisition.ExportVideoSnapshotEx(savepath,PTCSnapshotType["ptcSnapshotTypeMainCamera"])
        return
    
    def get_scanpoints(self,category:str="All",ref_frame:str="pixel")->np.ndarray:
        return polytec_common.get_scanpoints(self.infos,category,ref_frame)
    
    def get_scanpoints_status_statistics(self):
        return polytec_common.get_scanpoints_status_statistics(self.infos)
    
    def log_current_settings(self,log_filepath:str):
        text=str(Infos.from_com(self.app.Acquisition.Infos))
        with open(log_filepath, 'a') as file:
            file.write("PSV settings:")
            file.write(text + '\n')
        return
    
    def control_zstage(self):
        #Unused for now, but if we ever want to move the objective positioner manually
        #stage name: P-725K169 ---> E712
        infos=Infos.from_com(self.app.Acquisition.Infos)
        zstage_control=infos.scan_head_devices_info.scan_head_devices[0].z_stage_control
        print(f"{zstage_control.connected=}")
        #zstage_control.StartMoveTo
        pass
    
    def init_objective_positioner(self):
        raise NotImplementedError("function does not work yet")
        infos=Infos.from_com(self.app.Acquisition.Infos)
        scanHeadControl=infos.scan_head_devices_info.scan_head_devices[0].scan_head_control
        scanHeadControl.Init()#TODO: does not work, need an input parameter but idk what
        return
    
    def autofocus(self,search_full_range:bool=True):
        """
        Only works when the objective positioner is installed.
        COMMON ISSUE:
        If the positionner returns to its default position before each scan, go to the psv menu:
        setup/preferences-> change objective positionner from auto to None, apply then set it back to auto and retry.
        """
        self.app.Acquisition.AutoFocusLaser(-1,search_full_range) #True to search full range, False to check near current pos
        return
    
class Vibsoft(Polytec_software):
    def __init__(self):
        super().__init__('VibSoft')

    def is_laser_on(self):
        #returns true if laser is on, false if it is off.
        laser_on=self.app.Acquisition.GetLaserOn(-1)
        return laser_on
    
    def set_laser_state(self,state:str):
        #turn laser on or off
        match state:
            case "ON":
                self.app.Acquisition.SetLaserOn(-1,True,False)
            case "OFF":
                self.app.Acquisition.SetLaserOn(-1,False,False)
            case _:
                raise ValueError(f"expected state ON or OFF but got {state}")
        return

    def save_data(self,savepath:str):
        if len(self.app.Windows)>=3:
            for window in self.app.Windows:
                if window.Type==PTCWindowType["ptcWindowTypeAnalyzer"] and window.Caption!="Analyzer *":
                    window.Close()
        for window in self.app.Windows:
            print(window.Caption)
            if window.Type==PTCWindowType["ptcWindowTypeAnalyzer"] and window.Caption=="Analyzer *":
                window.AnalyzerView.Autoscale()
                window.AnalyzerView.Export(savepath,PTCFileFormat["ptcFileFormatAnalyzer"])
                break
                #window.Close()
        return

    def trigger_scan(self,savepath:str):
        savepath=ut.relative_to_absolute(savepath)
        ut.make_dir(savepath)
        if not self.is_laser_on():
            self.set_laser_state("ON")
        self.app.Acquisition.Start(PTCAcqStartMode["ptcAcqStartSingle"])
        self.wait_for_scan_end()
        self.save_data(savepath)
        return
    

class Pylon():
    """
    !!! THE CLASS WILL NOT WORK IF PYLON IS OPENED
    """
    # Software to control Basler cameras (like the one used by the vibroflex)
    def __enter__(self):
        self.tl_factory = pylon.TlFactory.GetInstance()
        devices = self.tl_factory.EnumerateDevices()
        
        if not devices:
            raise RuntimeError("No camera devices found.")
        
        try:
            self.camera = pylon.InstantCamera(self.tl_factory.CreateFirstDevice())
            self.camera.Open()
        except Exception as e:
            raise RuntimeError(f"Failed to open camera: {e}. Make sure pylon viewer app is closed and retry.")
        
        # Shared buffer to store the latest frame (allows screenshots while live camera view is open)
        self.latest_frame = None
        self.lock = threading.Lock()

        # Variables to store the ROI coordinates
        self.roi_start = None
        self.roi_end = None
        self.is_dragging = False  # To track mouse dragging
        self.zoomed_in = False  # To track zoom state

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if hasattr(self, 'camera') and self.camera.IsOpen():
            self.camera.Close()

    def select_roi(self, event, x, y, flags, param):
        """Mouse callback to select ROI for zooming in the live view window."""
        if event == cv2.EVENT_LBUTTONDOWN:
            # Start the ROI selection on mouse click
            self.roi_start = (x, y)
            self.is_dragging = True
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.is_dragging:
                # Update the ROI end position while dragging
                self.roi_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            # Finalize the ROI selection on mouse release
            self.is_dragging = False
            self.roi_end = (x, y)
            self.zoomed_in = True

    def start_live_view(self):
        """Streams the live footage from the camera to a window in which you can zoom.
        This is useful since you cannot open pylon viewer and use the pypylon API at the same time.
        """
        if not self.camera.IsGrabbing():
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly, pylon.GrabLoop_ProvidedByUser)

        # Create an OpenCV window to show the live footage
        cv2.namedWindow("Live Camera Feed", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Live Camera Feed", self.select_roi)  # Set mouse callback for ROI selection
        while True:
            # Grab a frame
            grab_result = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            
            if grab_result.GrabSucceeded():
                # Convert the image to a format compatible with OpenCV
                converter = pylon.ImageFormatConverter()
                converter.OutputPixelFormat = pylon.PixelType_BGR8packed  # Use BGR format for OpenCV
                img = converter.Convert(grab_result).GetArray()

                # Store the frame in a shared buffer with a lock
                with self.lock:
                    self.latest_frame = img

                # Show a live update of the selection (if dragging)
                if self.roi_start and self.roi_end and self.is_dragging:
                    x1, y1 = self.roi_start
                    x2, y2 = self.roi_end
                    img_copy = img.copy()
                    # Draw the rectangle (live preview of the selection)
                    cv2.rectangle(img_copy, (x1, y1), (x2, y2), (0, 255, 0), 8)
                    cv2.imshow("Live Camera Feed", img_copy)
                elif self.roi_start and self.roi_end:
                    # Zoom: If ROI is finished, crop the image and display
                    x1, y1 = self.roi_start
                    x2, y2 = self.roi_end
                    if x2 > x1 and y2 > y1:
                        img = img[y1:y2, x1:x2]
                        self.zoomed_in = True  # Mark as zoomed in
                    cv2.imshow("Live Camera Feed", img)

                elif not self.zoomed_in:
                    # If no zoom is active, show the full image
                    cv2.imshow("Live Camera Feed", img)

            grab_result.Release()
            key = cv2.waitKey(1) & 0xFF
            if key== ord('q'):#quit if q is pressed
                break
            elif key == ord('r'):
                self.reset_zoom()
        cv2.destroyAllWindows()
        return

    def reset_zoom(self):
        """Reset the zoom to show the full image."""
        self.roi_start = None
        self.roi_end = None
        self.zoomed_in = False
        return

    def load_settings(self, settings: str):
        """Load camera settings from a .pfs file"""
        if os.path.exists(settings):
            try:
                pylon.FeaturePersistence.Load(settings, self.camera.GetNodeMap(), True)
            except pylon.GenericException as e:
                raise RuntimeError(f"Failed to load settings: {e.GetDescription()}")
        else:
            raise FileNotFoundError(f"Settings file not found: {settings}")

    def export_snapshot_in_thread(self, savepath: str):
        """If you want to take a picture while start_live_view() is running
        This method runs in a separate thread to export snapshots while the live feed continues.
        """
        def helper(savepath):
            with self.lock:
                if self.latest_frame is not None:
                    # Save the latest frame to the specified path
                    cv2.imwrite(savepath, self.latest_frame)
                    return savepath
                else:
                    raise RuntimeError("No frame available for snapshot.")
        threading.Thread(target=helper, args=(savepath,)).start()
        return

    def export_snapshot(self, savepath: str):
        """If you want to take a picture while start_live_view() is NOT running"""
        if not self.camera.IsGrabbing():
            self.camera.StartGrabbing()
        
        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed  # Use "BGR8" for color images

        grab_result = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grab_result.GrabSucceeded():
            # Convert the image to a format compatible with OpenCV
            img = converter.Convert(grab_result).GetArray()
            cv2.imwrite(savepath, img)
        grab_result.Release()
        return savepath