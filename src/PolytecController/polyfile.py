"""
Author: Maxime Leurquin
Date: Nov-2023
Description: A collection of classes to help reading the contents of .svd files.
Reference: Basic Engine Manual.chm which is found in the installation folder of psv.
"""
import io
import os
import shutil
import re
import warnings
from win32com.client import Dispatch
import numpy as np
import matplotlib.pyplot as plt
import PIL.Image
from typing import Tuple
from packaging import version

from .enumerations import (
    PTCDomainType,
    PTCScanStatus,
    PTCDisplayType,
    PTCGraphicFormatType,
    PTCFileID,
    PTCVibrationDirection
)
from .classes import (
    Version,
    SummaryInfo,
    Infos,
    SignalDesc,
    PointDomain,
    DOF,
    USD,
    unpack_collection
)
from . import polytec_common
from . import utils as ut

class PolyFile:
    def __init__(self, filepath):
        self.filepath=filepath
        self.filename=os.path.basename(filepath)
        
        self.file = Dispatch('PolyFile.PolyFile') #see PolyFile object in manual
        self.file.Open(filepath)
        self.version=Version.from_com(self.file.Version)
        self.readOnly=self.file.ReadOnly
        self.summaryInfo=SummaryInfo.from_com(self.file.SummaryInfo)
        self.infos = Infos.from_com(self.file.Infos)
        
        #just check if the filename extension is not .set:
        if not self.filename.endswith(".set"):
            self.pointDomains = unpack_collection(self.file.GetPointDomains(), PointDomain)
            self.domain = self.get_domain()

    def get_laser_intensity(self) -> float:
        """
        Retrieve the laser intensity setting from files
        
        NOTE: The laser intensity is stored in the comment section (summaryInfo.subject) of files
        created by PSV 10.3.1 or later on MSA-I-600 sensor head systems, for measurements that are MANUALLY launched
        Hopefully laser intensity information will be added for measurements launched by scripting in PSV10.4
        
        Returns:
            float: Laser intensity value in dB
            
        Raises:
            ValueError: If the file is created with a version below PSV 10.3.1
            ValueError: If the sensor head is not MSA-I-600
            ValueError: If laser intensity information cannot be found in the file
        """
        # Check file version compatibility
        file_version = version.parse(self.version.program_version_string)
        if file_version < version.parse("10.3.1"):
            raise ValueError(
                f"Laser intensity data only available on files created with PSV 10.3.1 or above."
                f"Current file version: {self.version.program_version_string}"
            )
        
        # Check sensor head compatibility
        if self.infos.hardware.sensor_head_names != "MSA-I-600":
            raise ValueError(
                f"Laser intensity data is only available for MSA-I-600 systems. "
                f"Current sensor head: {self.infos.hardware.sensor_head_names}"
            )
        
        # Extract laser intensity from subject field
        pattern = r'Laser intensity:\s*(-?\d+(?:\.\d+)?)\s*dB'
        match = re.search(pattern, self.summaryInfo.subject)
        
        if not match:
            #Likely the file does not contain laser information because the measurement was launched by scripting and not manually.
            raise ValueError(
                f"Laser intensity information not found in file. "
                f"Subject field content: '{self.summaryInfo.subject}'"
            )
        
        return float(match.group(1))
    
    def get_measurement_time(self,return_type:str)->str|float:
        """
        return the file creation time written in the svd file
        return_type: str or float.
            If float: returns the number of seconds that elapsed since the epoch (1-jan-1970)
        """
        match return_type:
            case "str":
                measurement_time=self.version.file_time.strftime('%d-%b-%Y %H:%M:%S %Z')
            case "float":
                measurement_time=self.version.file_time.timestamp() #number of seconds since 1-jan-1970
            case _:
                raise ValueError()
        return measurement_time

    def get_measurement_info(self)->Tuple[str,str]:
        """
        return the file creation time written in the svd file as a string.
        return the program that made the file and its full version as a string.
        """
        measurement_time=self.get_measurement_time(return_type="str")
        measurement_program=f"{self.version.program_id_string} {self.version.program_version_string}"
        return measurement_time,measurement_program
    
    def get_domain(self)->str:
        activeProperties=self.infos.acquisition_info_modes.active_properties
        if not self.infos.acquisition_info_modes.active_mode and activeProperties.has_time_properties:
            domain="Time"
        elif self.infos.acquisition_info_modes.active_mode and activeProperties.has_fft_properties:
            domain="FFT"
        else: 
            raise ValueError("Error while trying to determine domain type")
        return domain
    
    def get_sampling_frequency(self,domain:str)->int:
        match domain:
            case "Time":
                fs=self.infos.acquisition_info_modes.active_properties.time_properties.sample_frequency
            case "FFT":
                fs=self.infos.acquisition_info_modes.active_properties.fft_properties.sample_frequency
            case _:
                assert False,f"Wrong argument: expected either Time or FFT but got {domain}"
        return fs
    
    def print_available_data(self)->None:
        """
        Print all the available point data in this svd file.
        Get the point data using the get_point_data function
        """
        for pointDomain in self.pointDomains:
            for channel in pointDomain.channels:
                for signal in channel.signals:
                    for display in signal.displays:
                        print(f"PointDomain: {pointDomain.type}, channel {channel.name}, signal: {signal.name},display: {display.name}")

    def get_scanpoints(self,category:str="All",ref_frame:str="pixel"):
        return polytec_common.get_scanpoints(self.infos,category,ref_frame) #[[x1, y1], [x2, y2], [x3, y3], ...]
    
    def get_scanpoints_statuses(self):
        return polytec_common.get_scanpoints_statuses(self.infos)

    def plot_scanpoints(self,**kwargs)->None:
        """
        Plot the scanpoints stored in the svd file on the given ax.
        kwargs:
            ax: matplotlib axis to plot on. If None, a new figure and axis will be created.
            savepath: str or None. If not None, the figure will be saved to this path.
            category: str. Category of scan points to plot. Options: All, Enabled, Disabled, Interpolated, Overrange, Valid.
            label_scanpoints: bool. If True, each scanpoint will be labeled with its index.
            label_scanstatus: bool. If True, each scanpoint will be labeled with its scan status code.
            title: str. Title of the plot.
            ref_frame: str. Reference frame for the coordinates. Options: raw, video, pixel.
            legend: bool. If True, a legend will be added to the plot.
            add_border: bool. If True, a colored border will be added to the plot based on the worst status of the points.
            border_width: int. Width of the border if add_border is True.
            fontsize_title: int. Font size of the title.
            s: int. Size of the scatter points.
            label_scanpoint_fontsize: int. Font size of the scanpoint labels.
            cropbox: tuple or None. If not None, should be (left, upper, right, lower) bounds of the cropped image.
        """
        options = {"ax": None,"savepath": None,"category":"All","label_scanpoints": False,"label_scanstatus":False,
                   "title":self.filename,"ref_frame":"pixel","legend":False,"add_border":False,"border_width":20,"fontsize_title":32,
                   "s":10,"label_scanpoint_fontsize":12,"cropbox":None}
        options.update(kwargs)
        fig, ax = plt.subplots(figsize=(10,10)) if options["ax"] is None else (options["ax"].get_figure(),options["ax"])
        ax.set_title(options["title"],fontsize=options["fontsize_title"])

        #Point can be multiple statuses at the same time, therefore bad idea to have a different color for each status.
        STATUSES_COLORS={"Valid":"lime", "Optimal":"lime","Overrange":"orange","Interpolate":"orange","Invalidated":"red","Disabled":"red","NotReachable":"red","Hidden":"red","VideoTriangulationFailed":"red","InterpolationFailed":"red","None":"red","NotMeasured":"red"}
        statuses=self.get_scanpoints_statuses()
        scanpoints=self.get_scanpoints(category="All",ref_frame=options["ref_frame"]) #[[x1, y1], [x2, y2], [x3, y3], ...]
        if options["cropbox"] is not None:
            scanpoints=polytec_common.transform_scanpoints_for_cropped_image(scanpoints,options["cropbox"])
        colors=[]
        for i,(point,status) in enumerate(zip(scanpoints,statuses)):
            translated_status=polytec_common.translate_status(status) #translate status code to strings
            clean_translated_status=[st.replace("ptcScanStatus", "") for st in translated_status]
            #check that at least 1 of the translated status is in the requested category to see if we plot the point or not
            if bool(set(clean_translated_status) & set(options['category'])) or options['category']=="All":
                common=ut.find_common_string(clean_translated_status,list(STATUSES_COLORS.keys()))
                color=STATUSES_COLORS[common[0]]
                colors.append(color)
                ax.scatter(point[0],point[1],color=color,s=options["s"])
                text_label=""
                if options["label_scanpoints"]:
                    text_label+=f"{i+1}"
                if options["label_scanstatus"]:
                    text_label+=f" c:{status}"
                ax.annotate(text_label, (point[0],point[1]),color=color,va='bottom',fontsize=options["label_scanpoint_fontsize"]) #i+1 to match polytec's indexing in PSV
        #Add a colored border to the image, don't add border when everything is fine
        if options['add_border']:
            if "red" in colors:
                ut.make_border(ax, color="red", border_width=options["border_width"])
            elif "orange" in colors: #yellow does not have enough contrast
                ut.make_border(ax, color="orange", border_width=options["border_width"])

        if options["legend"]:
            ax.legend()
        if options["savepath"] is not None:
            ut.make_dir(options["savepath"])
            fig.savefig(options["savepath"],bbox_inches="tight")
            plt.close(fig)
        return 
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.close_file()

    def close_file(self):
        if self.file.IsOpen:
            self.file.Close()
        return
    
    def get_point_data(self, domain_name: str, channel_name: str, signal_name: str, display_name: str,
                      point_index: int=-1, frame: int = 0) -> Tuple[np.ndarray, np.ndarray, USD]:
        """
        Gets original or user defined data from a polytec file.
        This function matches Polytec's implementation.
        3D Data cannot be read directly.

        :param domain_name: the name of the domain, e.g. 'FFT' or 'Time'
        :param channel_name: the name of the channel, e.g. 'Vib' or 'Ref1' or 'Vib & Ref1' or 'Vib X' or 'Vib Y' or 'Vib Z'
        :param signal_name: the name of the signal, e.g. 'Velocity' or 'Displacement'
        :param display_name:  the name of the display, e.g. 'Real' or 'Magnitude' or 'Samples'
            If the display name is 'Real & Imag.' the data is returned as complex values.
            Note: display types that with two parts(e.g. "Mag. & Phase") other than "Real & Imag." are not supported!
        :param point_index: the (0-based) index of the point to get the coordinates from.
            If point is -1 the coordinates of all points will be returned. XYZ will contain the data of
            point i at row index i.
        :param frame: the frame number of the data. for data acquired in MultiFrame
        mode, 0 is the averaged frame and 1-n are the other frames. For user
        defined datasets the frame number is in the range 1-n where n is the
        number of frames in the user defined dataset. For all other data,
        use frame number 0
        :return: tuple of:
            x, the x-axis values of the data
            y, the data. columns correspond to the x-axis, rows to the point
               index. for point = 0: rows for points that have no data are set to zeros.
            usd, an object describing the signal
        """
        file = self.file
        point_domain = file.GetPointDomains().Item(domain_name)
        channel = point_domain.Channels.Item(channel_name)
        signal = channel.Signals.Item(signal_name)
        display = signal.Displays.Item(display_name)

        signal_desc = signal.Description

        x_axis, y_axis = signal_desc.XAxis, signal_desc.YAxis

        if point_domain.Type == PTCDomainType["ptcDomain3rdOctave"]:
            x = [x_axis.GetMidX(i) for i in range(x_axis.MaxCount)]
        else:
            step = ((x_axis.Max - x_axis.Min) / (x_axis.MaxCount-1))
            x = np.arange(x_axis.Min, x_axis.Max+step, step)
        
        # Setup usd
        response_dofs = signal_desc.ResponseDOFs
        reference_dofs = signal_desc.ReferenceDOFs

        usd = USD(
            name=signal_desc.Name, complex=signal_desc.Complex, data_type=signal_desc.DataType,
            domain_type=signal_desc.DomainType, function_type=signal_desc.FunctionType,
            power_signal=signal_desc.PowerSignal,
            is_3d=response_dofs.Count > 0 and response_dofs.Direction == PTCVibrationDirection["ptc3DVector"],
            db_reference=signal_desc.DbReference, x_name=x_axis.Name, x_unit=x_axis.Unit, x_min=x_axis.Min,
            x_max=x_axis.Max, x_count=x_axis.MaxCount, y_name=y_axis.Name, y_unit=y_axis.Unit, y_min=y_axis.Min,
            y_max=y_axis.Max,
            response_dofs=[] if response_dofs.Count == 0 else [DOF.from_com(dof) for dof in response_dofs],
            reference_dofs=[] if reference_dofs.Count == 0 else [DOF.from_com(dof) for dof in reference_dofs]
        )

        data_points = point_domain.DataPoints

        # if the signal is complex, two numbers are needed per entry (real and imag)
        # Note: display types that with two parts(e.g. "Mag. & Phase") other than "Real & Imag." are not supported!
        data_cnt = len(x) * 2 if signal_desc.Complex and display.Type == PTCDisplayType["ptcDisplayRealImag"] else len(x)

        if point_index == -1:
            # Get all points
            y = np.zeros((data_points.Count, data_cnt))
            for i, data_point in enumerate(data_points):
                y_point = np.array([float(val) for val in data_point.GetData(display, frame)])
                if len(y_point) == 0:
                    y[i] = np.zeros(data_cnt)
                else:
                    y[i] = y_point
        else:
            # Get specific point
            data_point = data_points.Item(point_index+1)  # point_index is 0-based, Item() is 1-based
            y = np.array(data_point.GetData(display, frame))

        # handle complex data
        if signal_desc.Complex and display.Type == PTCDisplayType["ptcDisplayRealImag"]:
            real_data = y[:, ::2] if point_index == -1 else y[::2]  # even indices
            imag_data = y[:, 1::2] if point_index == -1 else y[1::2]  # odd indices
            y = real_data + 1j * imag_data

        return np.array(x), np.array(y), usd
    
    def find_disabled_points_indexes(self)->np.ndarray:
        """
        Return a an array of boolean values indicating which points are disabled.
        True means the point is disabled.
        """
        if filetype=="ptcFileIDVibSoftFile":#does not make sense to disable points in pvb files since they only contain one point.
            raise RuntimeError("Function not available for pvb files")
        file = self.file
        filetype=self.version.file_id_string
        pointdomains = file.GetPointDomains()
        pointdomain = pointdomains.Item(self.domain)
        datapoints = pointdomain.DataPoints
        disabled=np.zeros(datapoints.Count,dtype=bool)
        for i in range(1, datapoints.Count + 1):
            datapoint = datapoints.Item(i)
            disabled[i-1]=polytec_common.find_statuses_matches([datapoint.MeasPoint.ScanStatus],["ptcScanStatusDisabled"])[0]
        return disabled

class PolySettings(PolyFile):
    def __init__(self, set_path):
        super().__init__(set_path)
        self.npoints = self.infos.meas_points.count #number of measurement points
    
    def print_available_data(self):
        raise NotImplementedError("Function not available for Settings file")

    def modify_scanpoints(self,new_sett_path:str,new_scanpoints:np.ndarray)->None:
        """
        Create a new settings file with modified scanpoint video coordinates compared to this file
        This file will not be modified. The new file will be saved to new_sett_path
        New scanpoints should be given in the same order as those contained in this file.
        """
        self.close_file() #cannot have 2 svd files open at same time
        ut.make_dir(new_sett_path)
        shutil.copy(self.filepath, new_sett_path)
        with PolySettings(new_sett_path) as new_sett:
            new_sett.file.ReadOnly=False
            measpoints=new_sett.file.Infos.MeasPoints
            assert len(measpoints)==len(new_scanpoints),"Polytec does not allow adding new scanpoints, only modifying"
            for measPoint,new_scanpoint in zip(measpoints,new_scanpoints):
                #print(f"{new_scanpoint=}")
                measPoint.SetVideoXY(new_scanpoint[0],new_scanpoint[1])
            new_sett.file.Save()
        self.file.Open(self.filepath)
        return
        
class Pvd(PolyFile):
    def __init__(self, pvd_path:str):
        super().__init__(pvd_path)
        if self.version.file_id==PTCFileID["ptcFileIDVibSoftFile"]:
            self.pointDomains=unpack_collection(self.file.GetPointDomains(), PointDomain)
            self.domain=self.get_domain() #FFT or time
    
class Svd(PolyFile):
    def __init__(self, svd_path:str):
        super().__init__(svd_path)
        if self.version.file_id==PTCFileID["ptcFileIDPSVFile"]:
            self.bandDomains=self.file.GetBandDomains() 
            self.pointAverageDomains=self.file.GetPointAverageDomains()
            self.pointDomains=unpack_collection(self.file.GetPointDomains(), PointDomain)

            self.npoints = self.infos.meas_points.count #number of measurement points
            self.domain=self.get_domain() #FFT or time

    def get_image(self,cropbox=None)->np.ndarray:
        """
        Get the image saved in the svd file. It can then be shown with ax.imshow(image)
        cropbox:optional left, upper, right, lower bound of the cropped image we want
        """
        assert self.infos.has_video_bitmap,"file does not have image information"
        assert self.infos.video_bitmap is not None, "video bitmap data is not available"
        imgx,imgy=self.get_image_size()
        img_data = self.infos.video_bitmap.image(PTCGraphicFormatType["ptcGraphicFormatPNG"], imgx, imgy)[0]
        with io.BytesIO(img_data) as stream:
            image = np.array(PIL.Image.open(stream))

        if cropbox is not None:
            left, upper, right, lower = cropbox  # Extract the crop parameters
            # Ensure crop box is within image bounds
            if not ((0<=left<right<=imgx) and (0<=lower<upper<=imgy)):
                raise ValueError("Invalid crop box dimensions.")
            image = image[lower:upper, left:right]
        return image
    
    def plot_image(self,**kwargs)->np.ndarray:
        options = {"show_scanpoints": True,"ax": None,"savepath": None,"title":self.filename,"cropbox":None}
        #Merge defaults and kwargs. the value associated with the key in kwargs takes precedence over the value associated with the same key in defaults
        options.update(kwargs)
        fig, ax = plt.subplots(figsize=(10,10)) if options["ax"] is None else (options["ax"].get_figure(),options["ax"])
        image=self.get_image(options["cropbox"])
        ax.imshow(image)
        if options['show_scanpoints']:
            options["ax"] = ax if options["ax"] is None else options["ax"]
            options["ref_frame"]="pixel"
            self.plot_scanpoints(**options)
        elif options["savepath"] is not None:
            ut.make_dir(options["savepath"])
            fig.savefig(options["savepath"],bbox_inches="tight")
            plt.close(fig)
        return image

    def get_image_size(self)->Tuple[int,int]:
        return polytec_common.get_image_size(self.infos)
    
    def get_scanpoints_status_statistics(self):
        return polytec_common.get_scanpoints_status_statistics(self.infos)
    
