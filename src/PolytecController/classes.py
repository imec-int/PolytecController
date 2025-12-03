"""
Author: Maxime Leurquin
Date: Dec-2025
Description: Objects defined by polytec using modern Python dataclasses. 
It's useful to have them in python to print them and see what's hiding in a .svd file.
Reference: Object Model Reference.chm which is found in the installation folder of psv.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Any, Tuple
import pywintypes
import warnings

from .enumerations import (
    get_key_from_value,
    PTCFileID,
    PTCProgramID,
    PTCSignalEnhancementMode,
    PTCAverageType,
    PTCVibrationDirection,
    PTCInputCoupling,
    PTCPhysicalQuantity,
    PTCChannelType,
    PTCWindowFunction,
    PTCScanStatus,
    PTCDomainType,
    PTCLensCalibrationCapsType
)

def unpack_collection(collection_com, obj_type):
    """
    Unpacks a collection of objects into a list of specified object type.
    :param collection_com: COM object, input collection.
    :param obj_type: type, type of objects in the collection.
    :return: list, list of objects of the specified type.
    """
    obj_list = []
    if collection_com.Count == 0:
        return []
    for i in range(1, collection_com.Count + 1):
        obj_list.append(obj_type.from_com(collection_com.Item(i)))
    return obj_list

def unpack_zero_based_collection(collection_com, obj_type):
    """
    Most of the collections are 1 indexed, but some are zero based...Annoying.
    Unpacks a collection of objects into a list of specified object type.
    :param collection_com: COM object, input collection.
    :param obj_type: type, type of objects in the collection.
    :return: list, list of objects of the specified type.
    """
    obj_list = []
    if collection_com.Count == 0:
        return []
    for i in range(0, collection_com.Count):
        obj_list.append(obj_type.from_com(collection_com.Item(i)))
    return obj_list

class BaseClass:
    """
    Base class for other classes to inherit from.
    Provides a method __str__ to generate a string representation of the object with its attributes.
    """
    def __str__(self, indent=0):
        attributes = vars(self)
        class_name = self.__class__.__name__
        info_str = f"{class_name}(\n"

        for key, value in attributes.items():
            value_str = self._format_value(value, indent)
            info_str += f"{'    ' * (indent + 1)}{key}={value_str},\n"

        info_str += '    ' * indent + ")"
        return info_str

    def _format_value(self, value, indent):
        """Determine how to format the value based on its type."""
        if isinstance(value, BaseClass):
            return self._format_base_class(value, indent)
        elif isinstance(value, list):
            return self._format_list(value, indent)
        else:
            return repr(value)

    def _format_base_class(self, value, indent):
        """Format a value if it's an instance of BaseClass."""
        return value.__str__(indent + 1).replace('\n', '\n' + '    ' * (indent + 1))

    def _format_list(self, value, indent):
        """Format a value if it's a list."""
        if len(value) == 0:
            return "Empty list"
        elif len(value) < 5:
            # Format each element in the list if the list has less than 5 items
            elements_str = [self._format_base_class(item, indent) if isinstance(item, BaseClass) else repr(item) for item in value]
            formatted_elements = ',\n'.join(f"{'    ' * (indent + 2)}{elem}" for elem in elements_str)
            return f"[\n{formatted_elements}\n{'    ' * (indent + 1)}]"
        else:
            return f"List containing {len(value)} {type(value[0]).__name__}"

@dataclass
class DOF(BaseClass):
    """Degree of Freedom class (copied from polytec python example)"""
    channel_name: str
    direction: int  # enum
    node: int
    node_description: str
    quantity: str
    unit: str

    @classmethod
    def from_com(cls, com_dof):
        return cls(
            channel_name=com_dof.ChannelName,
            direction=com_dof.Direction,
            node=com_dof.Node,
            node_description=com_dof.NodeDescription,
            quantity=com_dof.Quantity,
            unit=com_dof.Unit
        )

@dataclass
class USD(BaseClass):
    """User Signal Description class (copied from polytec python example)"""
    name: str
    complex: bool
    data_type: int  # enum
    domain_type: int  # enum
    function_type: int  # enum
    power_signal: bool
    is_3d: bool
    db_reference: float
    x_name: str
    x_unit: str
    x_min: float
    x_max: float
    x_count: int
    y_name: str
    y_unit: str
    y_min: float
    y_max: float
    response_dofs: List[DOF]
    reference_dofs: List[DOF]

@dataclass
class Infos(BaseClass):
    """Main information container for file.Infos"""
    # Boolean flags for available information
    has_acquisition_info_modes: bool
    has_alignments: bool
    has_camera_settings: bool
    has_db_references: bool
    has_elements: bool
    has_hardware: bool
    has_meas_points: bool
    has_measurement_locations: bool
    #has_position_device: bool
    #has_positioning_stage: bool
    has_profiles: bool
    has_scan_head_devices_info: bool
    has_textures: bool
    has_vibrometers: bool
    has_video_bitmap: bool
    has_video_mapping_info: bool
    
    # Actual data objects (None if not available)
    acquisition_info_modes: Optional[AcquisitionInfoModes]
    alignments: Optional[Alignments]
    camera_settings: Optional[Any]
    db_references : Optional[Any]
    elements: Optional[Any]
    hardware: Optional[Hardware]
    meas_points: Optional[MeasPoints]
    measurement_locations: Optional[Any]
    #position_device: Optional[Any]
    profiles: Optional[Any]
    scan_head_devices_info: Optional[ScanHeadDevicesInfo]
    textures: Optional[Any]
    vibrometers: Optional[Any]
    video_bitmap: Optional[VideoBitmap]
    video_mapping_info: Optional[VideoMappingInfo]

    @classmethod
    def from_com(cls, infos_com):
        # Extract all boolean flags first, using getattr for optional ones
        has_acquisition_info_modes = infos_com.HasAcquisitionInfoModes
        has_alignments = infos_com.HasAlignments
        has_camera_settings = infos_com.HasCameraSettings
        has_db_references = infos_com.HasDbReferences
        has_elements = infos_com.HasElements
        has_hardware = infos_com.HasHardware
        has_meas_points = infos_com.HasMeasPoints
        has_measurement_locations = getattr(infos_com, 'HasMeasurementLocations', False)
        #has_position_device = getattr(infos_com, 'HasPositionDevice', False)
        has_profiles = infos_com.HasProfiles
        has_scan_head_devices_info = infos_com.HasScanHeadDevicesInfo
        has_textures = getattr(infos_com, 'HasTextures', False)
        has_vibrometers = infos_com.HasVibrometers
        has_video_bitmap = infos_com.HasVideoBitmap
        has_video_mapping_info = infos_com.HasVideoMappingInfo
        
        return cls(
            has_acquisition_info_modes=has_acquisition_info_modes,
            has_alignments=has_alignments,
            has_camera_settings=has_camera_settings,
            has_db_references=has_db_references,
            has_elements=has_elements,
            has_hardware=has_hardware,
            has_meas_points=has_meas_points,
            has_measurement_locations=has_measurement_locations,
            #has_position_device=has_position_device,
            #has_positioning_stage=getattr(infos_com, 'HasPositioningStage', False),
            has_profiles=has_profiles,
            has_scan_head_devices_info=has_scan_head_devices_info,
            has_textures=has_textures,
            has_vibrometers=has_vibrometers,
            has_video_bitmap=has_video_bitmap,
            has_video_mapping_info=has_video_mapping_info,
            
            acquisition_info_modes=AcquisitionInfoModes.from_com(infos_com.AcquisitionInfoModes) if has_acquisition_info_modes else None,
            alignments=Alignments.from_com(infos_com.Alignments) if has_alignments else None,
            camera_settings=infos_com.CameraSettings if has_camera_settings else None,
            db_references=infos_com.DbReferences if has_db_references else None,
            elements=infos_com.Elements if has_elements else None,
            hardware=Hardware.from_com(infos_com.Hardware) if has_hardware else None,
            meas_points=MeasPoints.from_com(infos_com.MeasPoints) if has_meas_points else None,
            measurement_locations=getattr(infos_com, 'MeasurementLocations', None) if has_measurement_locations else None,
            #position_device=getattr(infos_com, 'PositionDevice', None) if has_position_device else None,
            profiles=infos_com.Profiles if has_profiles else None,
            scan_head_devices_info=ScanHeadDevicesInfo.from_com(infos_com.ScanHeadDevicesInfo) if has_scan_head_devices_info else None,
            textures=getattr(infos_com, 'Textures', None) if has_textures else None,
            vibrometers=infos_com.Vibrometers if has_vibrometers else None,
            video_bitmap=VideoBitmap.from_com(infos_com.VideoBitmap) if has_video_bitmap else None,
            video_mapping_info=VideoMappingInfo.from_com(infos_com.VideoMappingInfo) if has_video_mapping_info else None
        )

@dataclass
class ScanHeadDevicesInfo(BaseClass):
    """Information about scan head devices."""
    scan_head_name: str
    scan_head_type: str
    scan_head_devices: List[ScanHeadDevice]

    @classmethod
    def from_com(cls, scan_head_devices_info_com):
        return cls(
            scan_head_name=scan_head_devices_info_com.ScanHeadName,
            scan_head_type=scan_head_devices_info_com.ScanHeadType,
            scan_head_devices=unpack_collection(scan_head_devices_info_com.ScanHeadDevices, ScanHeadDevice)
        )

@dataclass
class ScanHeadDevice(BaseClass):
    """Scan head device configuration and controls."""
    caps: Any
    distance_sensor_control: DistanceSensorControl
    focus: ScanHeadLaserFocus
    lens_magnification: Any
    pan_tilt_head: Any
    roll_angle: float
    scan_head_control: Any
    scanner: ScanHeadScanner
    sensor_head_name: str
    sensor_head_names: Any
    video: ScanHeadVideo
    z_stage_control: ZStageControl

    @classmethod
    def from_com(cls, scan_head_device_com):
        return cls(
            caps=scan_head_device_com.Caps,
            distance_sensor_control=DistanceSensorControl.from_com(scan_head_device_com.DistanceSensorControl),
            focus=ScanHeadLaserFocus.from_com(scan_head_device_com.Focus),
            lens_magnification=scan_head_device_com.LensMagnification,
            pan_tilt_head=scan_head_device_com.PanTiltHead,
            roll_angle=scan_head_device_com.RollAngle,
            scan_head_control=scan_head_device_com.ScanHeadControl,
            scanner=ScanHeadScanner.from_com(scan_head_device_com.Scanner),
            sensor_head_name=scan_head_device_com.SensorHeadName,
            sensor_head_names=scan_head_device_com.SensorHeadNames,
            video=ScanHeadVideo.from_com(scan_head_device_com.Video),
            z_stage_control=ZStageControl.from_com(scan_head_device_com.ZStageControl)
        )

@dataclass
class ZStageControl(BaseClass):
    """Z-stage control information and settings."""
    connected: bool
    # Optional fields that are only available when connected
    movement_in_progress: Optional[bool]
    position: Optional[float]
    position_max: Optional[float]
    position_min: Optional[float]
    position_reached: Optional[bool]
    p_term: Optional[float]

    @classmethod
    def from_com(cls, z_stage_control_com):
        connected = z_stage_control_com.Connected
        return cls(
            connected=connected,
            movement_in_progress=z_stage_control_com.MovementInProgress if connected else None,
            position=z_stage_control_com.Position if connected else None,
            position_max=z_stage_control_com.PositionMax if connected else None,
            position_min=z_stage_control_com.PositionMin if connected else None,
            position_reached=z_stage_control_com.PositionReached if connected else None,
            p_term=z_stage_control_com.PTerm if connected else None
        )

@dataclass
class ScanHeadLaserFocus(BaseClass):
    """Laser focus settings and parameters."""
    depth_of_focus_factor: float
    focal_length: float
    focus_command_time: float
    focus_infinite_pos: float
    focus_sharpness: float
    focus_step_width: float
    # lens_to_lens_distance: obsolete
    lens_to_mirror_distance: float
    min_focus_distance: float
    time_per_focus_step: float

    @classmethod
    def from_com(cls, scan_head_laser_focus_com):
        return cls(
            depth_of_focus_factor=scan_head_laser_focus_com.DepthOfFocusFactor,
            focal_length=scan_head_laser_focus_com.FocalLength,
            focus_command_time=scan_head_laser_focus_com.FocusCommandTime,
            focus_infinite_pos=scan_head_laser_focus_com.FocusInfinitePos,
            focus_sharpness=scan_head_laser_focus_com.FocusSharpness,
            focus_step_width=scan_head_laser_focus_com.FocusStepWidth,
            lens_to_mirror_distance=scan_head_laser_focus_com.LensToMirrorDistance,
            min_focus_distance=scan_head_laser_focus_com.MinFocusDistance,
            time_per_focus_step=scan_head_laser_focus_com.TimePerFocusStep
        )

@dataclass
class ScanHeadScanner(BaseClass):
    """Scanner configuration and properties."""
    cos_correction_active_x: bool
    cos_correction_active_y: bool
    cos_correction_possible_x: bool
    cos_correction_possible_y: bool
    inverted_x: bool
    inverted_y: bool
    mirror_distance: float
    preferred_scan_direction: Any
    quantity: Any
    quantity_max_x: float
    quantity_max_y: float
    swapped_xy: bool

    @classmethod
    def from_com(cls, scan_head_scanner_com):
        return cls(
            cos_correction_active_x=scan_head_scanner_com.CosCorrectionActiveX,
            cos_correction_active_y=scan_head_scanner_com.CosCorrectionActiveY,
            cos_correction_possible_x=scan_head_scanner_com.CosCorrectionPossibleX,
            cos_correction_possible_y=scan_head_scanner_com.CosCorrectionPossibleY,
            inverted_x=scan_head_scanner_com.InvertedX,
            inverted_y=scan_head_scanner_com.InvertedY,
            mirror_distance=scan_head_scanner_com.MirrorDistance,
            preferred_scan_direction=scan_head_scanner_com.preferredScanDirection,
            quantity=scan_head_scanner_com.Quantity,
            quantity_max_x=scan_head_scanner_com.QuantityMaxX,
            quantity_max_y=scan_head_scanner_com.QuantityMaxY,
            swapped_xy=scan_head_scanner_com.SwappedXY
        )

@dataclass
class ScanHeadVideo(BaseClass):
    """Video head configuration."""
    flipped_x: bool
    flipped_y: bool
    type: Any

    @classmethod
    def from_com(cls, scan_head_video_com):
        return cls(
            flipped_x=scan_head_video_com.FlippedX,
            flipped_y=scan_head_video_com.FlippedY,
            type=scan_head_video_com.Type
        )

@dataclass
class DistanceSensorControl(BaseClass):
    """Distance sensor control and information."""
    type: Any
    name: str
    connected: bool
    # Optional fields only available when connected
    filter_on: Optional[bool]
    firmware_version: Optional[str]
    laser_on: Optional[bool]
    range_finder_enabled: Optional[bool]
    remote_filter: Optional[Any]
    signal_level: Optional[float]
    signal_level_threshold_max: Optional[float]
    signal_level_threshold_min: Optional[float]
    update_signal_level: Optional[bool]

    @classmethod
    def from_com(cls, distance_sensor_control_com):
        connected = distance_sensor_control_com.Connected
        return cls(
            type=distance_sensor_control_com.Type,
            name=distance_sensor_control_com.Name,
            connected=connected,
            filter_on=distance_sensor_control_com.FilterOn if connected else None,
            firmware_version=distance_sensor_control_com.FirmwareVersion if connected else None,
            laser_on=distance_sensor_control_com.LaserOn if connected else None,
            range_finder_enabled=distance_sensor_control_com.RangeFinderEnabled if connected else None,
            remote_filter=distance_sensor_control_com.RemoteFilter if connected else None,
            signal_level=distance_sensor_control_com.SignalLevel if connected else None,
            signal_level_threshold_max=distance_sensor_control_com.SignalLevelThresholdMax if connected else None,
            signal_level_threshold_min=distance_sensor_control_com.SignalLevelThresholdMin if connected else None,
            update_signal_level=distance_sensor_control_com.UpdateSignalLevel if connected else None
        )

@dataclass
class Alignments(BaseClass):
    """Alignment information container for file.Infos.Alignments"""
    alignments_2d: List[Alignment2D]
    alignments_3d: Optional[List[Alignment3D]]
    alignments_camera: Optional[List[AlignmentCamera]]
    video_rect: Optional[Tuple[int, int, int, int]]  # left, top, right, bottom

    @classmethod
    def from_com(cls, alignments_com):
        # Use getattr for optional attributes that might not exist in all versions
        alignments_3d_attr = getattr(alignments_com, 'Alignments3D', None)
        alignments_camera_attr = getattr(alignments_com, 'AlignmentsCamera', None)
        get_video_rect_method = getattr(alignments_com, 'GetVideoRect', None)
        
        return cls(
            alignments_2d=unpack_collection(alignments_com.Alignments2D, Alignment2D),
            alignments_3d=unpack_collection(alignments_3d_attr, Alignment3D) if alignments_3d_attr is not None else None,
            alignments_camera=unpack_collection(alignments_camera_attr, AlignmentCamera) if alignments_camera_attr is not None else None,
            video_rect=get_video_rect_method() if get_video_rect_method is not None else None
        )

@dataclass
class AlignmentCamera(BaseClass):
    """Camera alignment information."""
    align_3d_points: List[Align3DPoint]
    aspect_ratio: float
    cam_pos_x: float
    cam_pos_y: float
    cam_pos_z: float
    cam_vec_x: float
    cam_vec_y: float
    cam_vec_z: float
    current_quality: float
    mirror: bool
    persistent: bool
    target_quality: float
    valid: bool
    view_angle: float
    view_up_x: float
    view_up_y: float
    view_up_z: float

    @classmethod
    def from_com(cls, alignments_camera_com):
        return cls(
            align_3d_points=unpack_collection(alignments_camera_com.Align3DPoints, Align3DPoint),
            aspect_ratio=alignments_camera_com.AspectRatio,
            cam_pos_x=alignments_camera_com.CamPosX,
            cam_pos_y=alignments_camera_com.CamPosY,
            cam_pos_z=alignments_camera_com.CamPosZ,
            cam_vec_x=alignments_camera_com.CamVecX,
            cam_vec_y=alignments_camera_com.CamVecY,
            cam_vec_z=alignments_camera_com.CamVecZ,
            current_quality=alignments_camera_com.CurrentQuality,
            mirror=alignments_camera_com.Mirror,
            persistent=alignments_camera_com.Persistent,
            target_quality=alignments_camera_com.TargetQuality,
            valid=alignments_camera_com.Valid,
            view_angle=alignments_camera_com.ViewAngle,
            view_up_x=alignments_camera_com.ViewUpX,
            view_up_y=alignments_camera_com.ViewUpY,
            view_up_z=alignments_camera_com.ViewUpZ
        )

@dataclass
class Alignment2D(BaseClass):
    """2D alignment information."""
    align_2d_points: List[Align2DPoint]
    scan_head_distance: float
    scan_head_type: str
    scanner_quantity: Any
    valid: bool

    @classmethod
    def from_com(cls, alignments_2d_com):
        return cls(
            align_2d_points=unpack_collection(alignments_2d_com.Align2DPoints, Align2DPoint),
            scan_head_distance=alignments_2d_com.ScanHeadDistance,
            scan_head_type=alignments_2d_com.ScanHeadType,
            scanner_quantity=alignments_2d_com.ScannerQuantity,
            valid=alignments_2d_com.Valid
        )

@dataclass
class Align2DPoint(BaseClass):
    """2D alignment point."""
    scanner_x: float
    scanner_y: float
    video_x: float
    video_y: float

    @classmethod
    def from_com(cls, align_2d_point_com):
        return cls(
            scanner_x=align_2d_point_com.ScannerX,
            scanner_y=align_2d_point_com.ScannerY,
            video_x=align_2d_point_com.VideoX,
            video_y=align_2d_point_com.VideoY
        )

@dataclass
class Alignment3D(BaseClass):
    """3D alignment information."""
    align_3d_points: List[Align3DPoint]
    coord_definition_mode: Any
    current_quality: float
    scan_head_type: str
    target_quality: float
    valid: bool

    @classmethod
    def from_com(cls, alignment_3d_com):
        return cls(
            align_3d_points=unpack_collection(alignment_3d_com.Align3DPoints, Align3DPoint),
            coord_definition_mode=alignment_3d_com.CoordDefinitionMode,
            current_quality=alignment_3d_com.CurrentQuality,
            scan_head_type=alignment_3d_com.ScanHeadType,
            target_quality=alignment_3d_com.TargetQuality,
            valid=alignment_3d_com.Valid
        )

@dataclass
class Align3DPoint(BaseClass):
    """3D alignment point."""
    caps: Any
    distance: float
    label: str
    point_type: Any
    quality: float
    scanner_x: float
    scanner_y: float
    video_x: float
    video_y: float
    x: float
    y: float
    z: float

    @classmethod
    def from_com(cls, align_3d_point_com):
        return cls(
            caps=align_3d_point_com.caps,
            distance=align_3d_point_com.Distance,
            label=align_3d_point_com.Label,
            point_type=align_3d_point_com.PointType,
            quality=align_3d_point_com.Quality,
            scanner_x=align_3d_point_com.ScannerX,
            scanner_y=align_3d_point_com.ScannerY,
            video_x=align_3d_point_com.VideoX,
            video_y=align_3d_point_com.VideoY,
            x=align_3d_point_com.X,
            y=align_3d_point_com.Y,
            z=align_3d_point_com.Z
        )

@dataclass
class VideoBitmap(BaseClass):
    """Video bitmap information from file.Infos.VideoBitmap"""
    image: Any
    image_rect: Tuple[int, int, int, int]  # Rectangle coordinates
    texture_columns: int
    texture_coordinates: Any
    texture_count: int
    texture_rows: int

    @classmethod
    def from_com(cls, video_bitmap_com):
        return cls(
            image=video_bitmap_com.Image,
            image_rect=video_bitmap_com.GetImageRect(),
            texture_columns=video_bitmap_com.TextureColumns,
            texture_coordinates=video_bitmap_com.TextureCoordinates,
            texture_count=video_bitmap_com.TextureCount,
            texture_rows=video_bitmap_com.TextureRows
        )

@dataclass
class VideoMappingInfo(BaseClass):
    """Video mapping information from file.Infos.VideoMappingInfo"""
    active_calibration: Any
    calibrations: List[LensCalibration]
    camera_size: CameraSize
    _video_mapping_info_com: Any = field(repr=False)  # Keep COM object for CoordXYFromVideo & method

    @classmethod
    def from_com(cls, video_mapping_info_com):
        return cls(
            active_calibration=video_mapping_info_com.ActiveCalibration,
            calibrations=unpack_collection(video_mapping_info_com.Calibrations, LensCalibration),
            camera_size=CameraSize.from_com(video_mapping_info_com.CameraSize),
            _video_mapping_info_com=video_mapping_info_com
        )
    
    def coord_xy_from_video(self, video_x: float, video_y: float) -> Tuple[float, float]:
        """
        IN: video coordinates.
        OUT: world coordinates in meter.
        """
        coord_x,coord_y=self._video_mapping_info_com.CoordXYFromVideo(video_x, video_y)
        return coord_x,coord_y 
    
    def video_xy_from_coord(self, world_x: float, world_y: float) -> Tuple[float, float]:
        """
        IN: world coordinates in meter.
        OUT: video coordinates.
        """
        video_x,video_y=self._video_mapping_info_com.VideoXYFromCoord(world_x, world_y)
        return video_x,video_y

@dataclass
class LensCalibration(BaseClass):
    """Lens calibration information from file.infos.VideoMappingInfo.Calibrations
    
    Returns or specifies the capabilities of this objective. This can be a combination
    of the PTCLensCalibrationCapsType constants.
    """
    caps: Optional[str]  # Capabilities of this objective
    distance_x: float  # horizontal calibration distance in meter
    distance_y: float  # vertical calibration distance in meter
    gouy_correction_factor: Optional[float]  # Gouy correction factor
    magnification_factor_x: float  # horizontal objective calibration factor
    magnification_factor_y: float  # vertical objective calibration factor
    name: str  # name of the objective calibration
    optical_path_length: Optional[float]  # optical path for this objective
    pixels_x: float  # horizontal calibration distance in pixels
    pixels_y: float  # vertical calibration distance in pixels

    @classmethod
    def from_com(cls, lens_calibration_com):
        return cls(
            caps=get_key_from_value(lens_calibration_com.Caps, PTCLensCalibrationCapsType),
            distance_x=lens_calibration_com.DistanceX,
            distance_y=lens_calibration_com.DistanceY,
            gouy_correction_factor=getattr(lens_calibration_com, 'GouyCorrectionFactor', None),
            magnification_factor_x=lens_calibration_com.MagnificationFactorX,
            magnification_factor_y=lens_calibration_com.MagnificationFactorY,
            name=lens_calibration_com.Name,
            optical_path_length=getattr(lens_calibration_com, 'OpticalPathLength', None),
            pixels_x=lens_calibration_com.PixelsX,
            pixels_y=lens_calibration_com.PixelsY
        )

@dataclass
class CameraSize(BaseClass):
    """Camera size information from file.infos.VideoMappingInfo.CameraSize"""
    chip_x: float  # horizontal size of the camera chip in meter
    chip_y: float  # vertical size of the camera chip in meter
    image_x: int  # horizontal number of pixels of the camera chip
    image_y: int  # vertical number of pixels of the camera chip

    @classmethod
    def from_com(cls, camera_size_com):
        return cls(
            chip_x=camera_size_com.ChipX,
            chip_y=camera_size_com.ChipY,
            image_x=camera_size_com.ImageX,
            image_y=camera_size_com.ImageY
        )

@dataclass
class SummaryInfo(BaseClass):
    """Summary information about the Polytec file."""
    author: str
    comments: str
    subject: str
    title: str

    @classmethod
    def from_com(cls, summary_info_com):
        return cls(
            author=summary_info_com.Author,
            comments=summary_info_com.Comments,
            subject=summary_info_com.Subject,
            title=summary_info_com.Title
        )

@dataclass
class Version(BaseClass):
    """Version information about the Polytec file and program."""
    file_id: int
    file_id_string: Optional[str]
    file_time: Any
    file_version: str
    program_id: int
    program_id_string: Optional[str]
    program_version: str
    program_version_build: int
    program_version_major: int
    program_version_minor: int
    program_version_revision: int
    program_version_string: str

    @classmethod
    def from_com(cls, version_com):
        file_id_str = get_key_from_value(version_com.FileID, PTCFileID)
        program_id_str = get_key_from_value(version_com.ProgramID, PTCProgramID)
        program_id_clean = program_id_str.removeprefix("ptcProgramID") if program_id_str else None
        
        return cls(
            file_id=version_com.FileID,
            file_id_string=file_id_str,
            file_time=version_com.FileTime,
            file_version=version_com.FileVersion,
            program_id=version_com.ProgramID,
            program_id_string=program_id_clean,
            program_version=version_com.ProgramVersion,
            program_version_build=version_com.ProgramVersionBuild,
            program_version_major=version_com.ProgramVersionMajor,
            program_version_minor=version_com.ProgramVersionMinor,
            program_version_revision=version_com.ProgramVersionRevision,
            program_version_string=version_com.ProgramVersionString
        )

@dataclass
class FrontEndProperties(BaseClass):
    """Properties of the front end amplifier."""
    generator_amplifier_enabled: bool
    generator_amplifier_factor: float

    @classmethod
    def from_com(cls, front_end_properties_com):
        return cls(
            generator_amplifier_enabled=front_end_properties_com.GeneratorAmplifierEnabled,
            generator_amplifier_factor=front_end_properties_com.GeneratorAmplifierFactor
        )

@dataclass
class SignalEnhancementProperties(BaseClass):
    """Signal enhancement properties."""
    mode: Optional[str]
    speckle_tracking: bool

    @classmethod
    def from_com(cls, signal_enhancement_properties_com):
        return cls(
            mode=get_key_from_value(signal_enhancement_properties_com.Mode, PTCSignalEnhancementMode),
            speckle_tracking=signal_enhancement_properties_com.SpeckleTracking
        )

@dataclass
class AverageProperties(BaseClass):
    """Averaging properties for acquisitions."""
    type: Optional[str]
    count: int

    @classmethod
    def from_com(cls, average_properties_com):
        return cls(
            type=get_key_from_value(average_properties_com.Type, PTCAverageType),
            count=average_properties_com.Count
        )

@dataclass
class TimeAcqProperties(BaseClass):
    """Time domain acquisition properties."""
    sample_frequency: float
    sample_resolution: float
    samples: int  # the number of samples
    sample_time: float

    @classmethod
    def from_com(cls, time_acq_properties_com):
        return cls(
            sample_frequency=time_acq_properties_com.SampleFrequency,
            sample_resolution=time_acq_properties_com.SampleResolution,
            samples=time_acq_properties_com.Samples,
            sample_time=time_acq_properties_com.SampleTime
        )

@dataclass
class FftAcqProperties(BaseClass):
    """FFT acquisition properties."""
    bandwidth: float  # in Hz
    end_frequency: float  # The end frequency of the FFT in Hz
    lines: int  # The number of FFT lines
    overlap: float  # The overlap of the FFT in percent
    sample_frequency: float  # in Hz
    sample_resolution: float  # in Hz
    samples: int  # the number of samples
    sample_time: float
    start_frequency: float

    @classmethod
    def from_com(cls, fft_acq_properties_com):
        return cls(
            bandwidth=fft_acq_properties_com.Bandwidth,
            end_frequency=fft_acq_properties_com.EndFrequency,
            lines=fft_acq_properties_com.Lines,
            overlap=fft_acq_properties_com.Overlap,
            sample_frequency=fft_acq_properties_com.SampleFrequency,
            sample_resolution=fft_acq_properties_com.SampleResolution,
            samples=fft_acq_properties_com.Samples,
            sample_time=fft_acq_properties_com.SampleTime,
            start_frequency=fft_acq_properties_com.StartFrequency
        )

@dataclass
class GeneralAcqProperties(BaseClass):
    """General acquisition properties."""
    is_auto_remeasure_possible: bool
    auto_remeasure: Optional[bool]
    auto_remeasure_auto_range: Optional[bool]
    bandwidth_extension: Any
    principal_component_analysis: bool
    real_sample_frequency: float

    @classmethod
    def from_com(cls, general_acq_properties_com):
        is_auto_remeasure_possible = general_acq_properties_com.IsAutoRemeasurePossible
        return cls(
            is_auto_remeasure_possible=is_auto_remeasure_possible,
            auto_remeasure=general_acq_properties_com.AutoRemeasure if is_auto_remeasure_possible else None,
            auto_remeasure_auto_range=general_acq_properties_com.AutoRemeasureAutoRange if is_auto_remeasure_possible else None,
            bandwidth_extension=general_acq_properties_com.BandwidthExtension,
            principal_component_analysis=general_acq_properties_com.PrincipalComponentAnalysis,
            real_sample_frequency=general_acq_properties_com.RealSampleFrequency
        )

@dataclass
class TriggerProperties(BaseClass):
    """Trigger properties for acquisitions."""
    analog_source_channel: Any
    edge: Any
    level: Any
    phase_from_ref: Any
    pre_trigger_possible: bool
    pre_trigger: Optional[Any]
    source: Any

    @classmethod
    def from_com(cls, trigger_properties_com):
        pre_trigger_possible = trigger_properties_com.PreTriggerPossible
        return cls(
            analog_source_channel=trigger_properties_com.AnalogSourceChannel,
            edge=trigger_properties_com.Edge,
            level=trigger_properties_com.Level,
            phase_from_ref=trigger_properties_com.PhaseFromRef,
            pre_trigger_possible=pre_trigger_possible,
            pre_trigger=trigger_properties_com.PreTrigger if pre_trigger_possible else None,
            source=trigger_properties_com.Source
        )

@dataclass
class ChannelAcqProperties(BaseClass):
    """Acquisition properties for a channel."""
    active: bool
    calibration: float
    differential_input: bool
    digital_filter: Any
    direction_of_vibration: Optional[str]
    icp_input: bool
    input_coupling: Optional[str]
    input_impedance: float
    input_range: float
    int_diff_physical_quantity: Optional[str]
    name: str
    quantity: Optional[str]
    reference: bool
    reference_point_index: int
    se_active: bool
    short_name: str
    signal_delay: float
    type: Optional[str]
    unit: str
    vibrometer_index: int
    window_function: Optional[str]
    window_function_params: Any
    window_function_rms_correction: float

    @classmethod
    def from_com(cls, channel_data_com):
        return cls(
            active=channel_data_com.Active,
            calibration=channel_data_com.Calibration,
            differential_input=channel_data_com.DifferentialInput,
            digital_filter=channel_data_com.DigitalFilter,
            direction_of_vibration=get_key_from_value(channel_data_com.DirectionOfVibration, PTCVibrationDirection),
            icp_input=channel_data_com.ICPInput,
            input_coupling=get_key_from_value(channel_data_com.InputCoupling, PTCInputCoupling),
            input_impedance=channel_data_com.InputImpedance,
            input_range=channel_data_com.InputRange,
            int_diff_physical_quantity=get_key_from_value(channel_data_com.IntDiffPhysicalQuantity, PTCPhysicalQuantity),
            name=channel_data_com.Name,
            quantity=get_key_from_value(channel_data_com.Quantity, PTCPhysicalQuantity),
            reference=channel_data_com.Reference,
            reference_point_index=channel_data_com.ReferencePointIndex,
            se_active=channel_data_com.SEActive,
            short_name=channel_data_com.ShortName,
            signal_delay=channel_data_com.SignalDelay,
            type=get_key_from_value(channel_data_com.Type, PTCChannelType),
            unit=channel_data_com.Unit,
            vibrometer_index=channel_data_com.VibrometerIndex,
            window_function=get_key_from_value(channel_data_com.WindowFunction, PTCWindowFunction),
            window_function_params=channel_data_com.WindowFunctionParams,
            window_function_rms_correction=channel_data_com.WindowFunctionRMSCorrection
        )

@dataclass
class FilterSettings(BaseClass):
    """Filter settings for vibrometer."""
    key: Any
    name: str
    range: Any

    @classmethod
    def from_com(cls, filter_settings_com):
        return cls(
            key=filter_settings_com.Key,
            name=filter_settings_com.Name,
            range=filter_settings_com.Range
        )

@dataclass
class DecoderInfo(BaseClass):
    """Decoder information."""
    name: str
    firmware_version: str

    @classmethod
    def from_com(cls, decoder_info_com):
        return cls(
            name=decoder_info_com.Name,
            firmware_version=decoder_info_com.FirmwareVersion
        )

@dataclass
class QuantitySettings(BaseClass):
    """Quantity settings for vibrometer."""
    bandwidth_range: Any
    decoder_info: Optional[DecoderInfo]
    key: Any
    max_velocity_range: Any
    name: str
    output_active: bool
    overrun: Any
    range: Any

    @classmethod
    def from_com(cls, quantity_settings_com):
        return cls(
            bandwidth_range=quantity_settings_com.BandwidthRange,
            decoder_info=DecoderInfo.from_com(quantity_settings_com.DecoderInfo) if quantity_settings_com.DecoderInfo is not None else None,
            key=quantity_settings_com.Key,
            max_velocity_range=quantity_settings_com.MaxVelocityRange,
            name=quantity_settings_com.Name,
            output_active=quantity_settings_com.OutputActive,
            overrun=quantity_settings_com.Overrun,
            range=quantity_settings_com.Range
        )

@dataclass
class SensorHeadInfo(BaseClass):
    """Information about the sensor head."""
    beam_gap: float
    bragg_cell_frequency: float
    caps: Any
    caps2: Any
    coherence_optimizer_firmware_version: str
    connected: bool
    dimmer_max: int
    dimmer_min: int
    firmware_version: str
    focus_far_limit: float
    focus_near_limit: float
    illumination_intensity_max: int
    illumination_intensity_min: int
    laser_delay: float
    laser_firmware_version: str
    laser_power: float
    laser_wavelength: float
    lower_objective_threshold_temperature: float
    max_rgb_led_color_index: int
    max_signal_balance: float
    min_rgb_led_color_index: int
    min_signal_balance: float
    name: str
    sonde_name: str
    standoff_dist: float
    system_laser_power: float
    unique_id: str
    upper_objective_threshold_temperature: float

    @classmethod
    def from_com(cls, sensor_head_info_com):
        return cls(
            beam_gap=sensor_head_info_com.BeamGap,
            bragg_cell_frequency=sensor_head_info_com.BraggCellFrequency,
            caps=sensor_head_info_com.Caps,
            caps2=sensor_head_info_com.Caps2,
            coherence_optimizer_firmware_version=sensor_head_info_com.CoherenceOptimizerFirmwareVersion,
            connected=sensor_head_info_com.Connected,
            dimmer_max=sensor_head_info_com.DimmerMax,
            dimmer_min=sensor_head_info_com.DimmerMin,
            firmware_version=sensor_head_info_com.FirmwareVersion,
            focus_far_limit=sensor_head_info_com.FocusFarLimit,
            focus_near_limit=sensor_head_info_com.FocusNearLimit,
            illumination_intensity_max=sensor_head_info_com.IlluminationIntensityMax,
            illumination_intensity_min=sensor_head_info_com.IlluminationIntensityMin,
            laser_delay=sensor_head_info_com.LaserDelay,
            laser_firmware_version=sensor_head_info_com.LaserFirmwareVersion,
            laser_power=sensor_head_info_com.LaserPower,
            laser_wavelength=sensor_head_info_com.LaserWavelength,
            lower_objective_threshold_temperature=sensor_head_info_com.LowerObjectiveThresholdTemperature,
            max_rgb_led_color_index=sensor_head_info_com.MaxRGBLedColorIndex,
            max_signal_balance=sensor_head_info_com.MaxSignalBalance,
            min_rgb_led_color_index=sensor_head_info_com.MinRGBLedColorIndex,
            min_signal_balance=sensor_head_info_com.MinSignalBalance,
            name=sensor_head_info_com.Name,
            sonde_name=sensor_head_info_com.SondeName,
            standoff_dist=sensor_head_info_com.StandoffDist,
            system_laser_power=sensor_head_info_com.SystemLaserPower,
            unique_id=sensor_head_info_com.UniqueID,
            upper_objective_threshold_temperature=sensor_head_info_com.UpperObjectiveThresholdTemperature
        )

@dataclass
class VideoCameraSettings(BaseClass):
    """Video camera settings."""
    gamma: float
    automatic_gain_control: bool
    manual_gain: float
    mirror: bool
    shutter_speed: float

    @classmethod
    def from_com(cls, video_camera_settings_com):
        return cls(
            gamma=video_camera_settings_com.Gamma,
            automatic_gain_control=video_camera_settings_com.AutomaticGainControl,
            manual_gain=video_camera_settings_com.ManualGain,
            mirror=video_camera_settings_com.Mirror,
            shutter_speed=video_camera_settings_com.ShutterSpeed
        )

@dataclass
class SensorHeadSettings(BaseClass):
    """Settings for a sensor head."""
    dimmer: int
    illumination_intensity: float
    illumination_state: Any
    key: Any
    name: str
    sensor_head_info: SensorHeadInfo
    video_camera_settings: Optional[VideoCameraSettings]

    @classmethod
    def from_com(cls, sensor_head_settings_com):
        return cls(
            dimmer=sensor_head_settings_com.Dimmer,
            illumination_intensity=sensor_head_settings_com.IlluminationIntensity,
            illumination_state=sensor_head_settings_com.IlluminationState,
            key=sensor_head_settings_com.Key,
            name=sensor_head_settings_com.Name,
            sensor_head_info=SensorHeadInfo.from_com(sensor_head_settings_com.SensorHeadInfo),
            video_camera_settings=VideoCameraSettings.from_com(sensor_head_settings_com.VideoCameraSettings) if sensor_head_settings_com.VideoCameraSettings is not None else None
        )

@dataclass
class ControllerInfo(BaseClass):
    """Information about the vibrometer controller."""
    caps: Any
    device_name: str
    firmware_version: str
    name: str
    serial_number: str
    unique_id: str

    @classmethod
    def from_com(cls, controller_info_com):
        return cls(
            caps=controller_info_com.Caps,
            device_name=controller_info_com.DeviceName,
            firmware_version=controller_info_com.FirmwareVersion,
            name=controller_info_com.Name,
            serial_number=controller_info_com.SerialNumber,
            unique_id=controller_info_com.UniqueId
        )

@dataclass
class VibControllerSettings(BaseClass):
    """Settings for the vibrometer controller."""
    controller_info: ControllerInfo
    decoder_clear_mode: Any
    filter_settings_collection: List[FilterSettings]
    quantity_settings_collection: List[QuantitySettings]
    sensor_head_settings_collection: List[SensorHeadSettings]

    @classmethod
    def from_com(cls, vib_controller_settings_com):
        return cls(
            controller_info=ControllerInfo.from_com(vib_controller_settings_com.ControllerInfo),
            decoder_clear_mode=vib_controller_settings_com.DecoderClearMode,
            filter_settings_collection=unpack_zero_based_collection(vib_controller_settings_com.FilterSettingsCollection, FilterSettings),
            quantity_settings_collection=unpack_zero_based_collection(vib_controller_settings_com.QuantitySettingsCollection, QuantitySettings),
            sensor_head_settings_collection=unpack_zero_based_collection(vib_controller_settings_com.SensorHeadSettingsCollection, SensorHeadSettings)
        )

@dataclass
class VibrometerAcqProperties(BaseClass):
    """Acquisition properties for vibrometers."""
    vib_controller_settings: VibControllerSettings

    @classmethod
    def from_com(cls, vibrometer_acq_properties_com):
        return cls(
            vib_controller_settings=VibControllerSettings.from_com(vibrometer_acq_properties_com.VibControllerSettings)
        )

@dataclass
class MeasPoint(BaseClass):
    """Measurement point information."""
    coord_xyz: Tuple[float, float, float]
    #texture_xy_index: Tuple[int, int]
    video_xy: Tuple[float, float]
    average_count: int
    component: Any
    contained_file_active_id: Any
    contained_file_active_mp_index: Any
    contained_file_index: Any
    contained_file_meas_point_index: Any
    focus_status: Any
    focus_values: Any
    geometry_status: Any
    index: int
    label: str
    min_sigma: float
    sigma: float
    vibrometer_range: Any
    #path_length_status: Any
    #path_length_value: float
    scan_distances: Optional[Any] = None
    scan_quantities_x: Optional[Any] = None
    scan_quantities_y: Optional[Any] = None
    scan_quantity: Optional[Any] = None
    scan_status: Optional[int] = None
    scan_status_disabled: Optional[bool] = None

    @classmethod
    def from_com(cls, meas_point_com):
        return cls(
            coord_xyz=meas_point_com.CoordXYZ(),
            #texture_xy_index=meas_point_com.TextureXYIndex(),
            video_xy=meas_point_com.VideoXY(),
            average_count=meas_point_com.AverageCount,
            component=meas_point_com.Component,
            contained_file_active_id=meas_point_com.ContainedFileActiveID,
            contained_file_active_mp_index=meas_point_com.ContainedFileActiveMPIndex,
            contained_file_index=meas_point_com.ContainedFileIndex,
            contained_file_meas_point_index=meas_point_com.ContainedFileMeasPointIndex,
            focus_status=meas_point_com.FocusStatus,
            focus_values=meas_point_com.FocusValues,
            geometry_status=meas_point_com.GeometryStatus,
            index=meas_point_com.Index,
            label=meas_point_com.Label,
            min_sigma=meas_point_com.MinSigma,
            #path_length_status=meas_point_com.PathLengthStatus, #The optical path length is only relevant on MSA-650 systems
            #path_length_value=meas_point_com.PathLengthValue,
            scan_distances=getattr(meas_point_com, 'ScanDistances', None),
            scan_quantities_x=getattr(meas_point_com, 'ScanQuantitiesX', None),
            scan_quantities_y=getattr(meas_point_com, 'ScanQuantitiesY', None),
            scan_quantity=getattr(meas_point_com, 'ScanQuantity', None),
            scan_status=getattr(meas_point_com, 'ScanStatus', None),
            scan_status_disabled=getattr(meas_point_com, 'ScanStatusDisabled', None),
            sigma=meas_point_com.Sigma,
            vibrometer_range=meas_point_com.VibrometerRange
        )

    def is_valid(self) -> bool:
        """Check if the measurement point is valid."""
        return self.scan_status == PTCScanStatus["ptcScanStatusValid"]

    def is_disabled(self) -> bool:
        """Check if the measurement point is disabled."""
        return self.scan_status == PTCScanStatus["ptcScanStatusDisabled"]

@dataclass
class MeasPoints(BaseClass):
    """Collection of measurement points."""
    has_3d_coordinates: bool
    has_non_optimal_points_to_invalidate: bool
    has_sigma: bool
    is_3d_calculation_possible: bool
    average_sigma: Optional[float]
    component: Any
    coordinates_to_array: Any
    count: int
    video_coordinates_to_array: Any
    video_rect: Tuple[int, int, int, int]  # left, top, right, bottom
    meas_points_list: List[MeasPoint]

    @classmethod
    def from_com(cls, meas_points_com):
        return cls(
            has_3d_coordinates=meas_points_com.Has3DCoordinates,
            has_non_optimal_points_to_invalidate=meas_points_com.HasNonOptimalPointsToInvalidate,
            has_sigma=meas_points_com.HasSigma,
            is_3d_calculation_possible=meas_points_com.Is3DCalculationPossible,
            average_sigma=meas_points_com.AverageSigma if meas_points_com.HasSigma else None,
            component=meas_points_com.Component,
            coordinates_to_array=meas_points_com.CoordinatesToArray,
            count=meas_points_com.Count,
            video_coordinates_to_array=meas_points_com.VideoCoordinatesToArray,
            video_rect=meas_points_com.GetVideoRect(),
            meas_points_list=unpack_collection(meas_points_com, MeasPoint)
        )

    def print_points(self):
        """Print all measurement points."""
        for point in self.meas_points_list:
            print(point)

@dataclass
class Hardware(BaseClass):
    """Hardware information."""
    acquisition_board: str
    acquisition_board_channel_count: Optional[int]
    acquisition_board_firmware_version: Optional[str]
    generator_channel_count: Optional[int]
    generator_firmware_version: Optional[str]
    generator_type: Optional[str]
    front_end: str
    front_end_firmware_version: Optional[str]
    scan_head_type: Optional[str]
    sensor_head_names: Optional[str]
    sensor_head_versions: Optional[str]
    coherence_optimizer_versions: Optional[str]
    teds_sensors: Optional[Any]

    @classmethod
    def from_com(cls, hardware_com):
        return cls(
            acquisition_board=hardware_com.AcqBoard,
            acquisition_board_channel_count=getattr(hardware_com, 'AcqBoardChannelCount', None),
            acquisition_board_firmware_version=getattr(hardware_com, 'AcqBoardFirmwareVersion', None),
            generator_channel_count=getattr(hardware_com, 'GeneratorChannelCount', None),
            generator_firmware_version=getattr(hardware_com, 'GeneratorFirmwareVersion', None),
            generator_type=getattr(hardware_com, 'GeneratorType', None),
            front_end=hardware_com.FrontEnd,
            front_end_firmware_version=getattr(hardware_com, 'FrontEndFirmwareVersion', None),
            scan_head_type=getattr(hardware_com, 'ScanHeadType', None),
            sensor_head_names=', '.join(getattr(hardware_com, 'SensorHeadNames', [])),
            sensor_head_versions=', '.join(getattr(hardware_com, 'SensorHeadVersions', [])),
            coherence_optimizer_versions=', '.join(getattr(hardware_com, 'CoherenceOptimizerVersions', [])),
            teds_sensors=getattr(hardware_com, 'TedsSensors', None)
        )

@dataclass
class ActiveProperties(BaseClass):
    """Active properties containing all acquisition settings."""
    has_average_properties: bool
    has_channels_properties: bool
    has_fast_scans_properties: bool
    has_fft_properties: bool
    has_front_end_properties: bool
    has_general_properties: bool
    has_generators_properties: bool
    has_multi_frame_properties: bool
    has_signal_enhancement_properties: bool
    has_time_properties: bool
    has_trigger_properties: bool
    has_vibrometers_properties: bool
    has_zoom_fft_properties: bool
    
    average_properties: Optional[AverageProperties]
    channels_properties: Optional[List[ChannelAcqProperties]]
    fast_scans_properties: Optional[Any]
    fft_properties: Optional[FftAcqProperties]
    front_end_properties: Optional[FrontEndProperties]
    general_properties: Optional[GeneralAcqProperties]
    generators_properties: Optional[Any]
    multi_frame_properties: Optional[Any]
    signal_enhancement_properties: Optional[SignalEnhancementProperties]
    time_properties: Optional[TimeAcqProperties]
    trigger_properties: Optional[TriggerProperties]
    vibrometers_properties: Optional[List[VibrometerAcqProperties]]
    zoom_fft_properties: Optional[Any]

    @classmethod
    def from_com(cls, active_properties_com):
        return cls(
            has_average_properties=active_properties_com.HasAverageProperties,
            has_channels_properties=active_properties_com.HasChannelsProperties,
            has_fast_scans_properties=active_properties_com.HasFastScansProperties,
            has_fft_properties=active_properties_com.HasFftProperties,
            has_front_end_properties=active_properties_com.HasFrontEndProperties,
            has_general_properties=active_properties_com.HasGeneralProperties,
            has_generators_properties=active_properties_com.HasGeneratorsProperties,
            has_multi_frame_properties=active_properties_com.HasMultiFrameProperties,
            has_signal_enhancement_properties=active_properties_com.HasSignalEnhancementProperties,
            has_time_properties=active_properties_com.HasTimeProperties,
            has_trigger_properties=active_properties_com.HasTriggerProperties,
            has_vibrometers_properties=active_properties_com.HasVibrometersProperties,
            has_zoom_fft_properties=active_properties_com.HasZoomFftProperties,
            
            average_properties=AverageProperties.from_com(active_properties_com.AverageProperties) if active_properties_com.HasAverageProperties else None,
            channels_properties=unpack_collection(active_properties_com.ChannelsProperties, ChannelAcqProperties) if active_properties_com.HasChannelsProperties else None,
            fast_scans_properties=active_properties_com.FastScansProperties if active_properties_com.HasFastScansProperties else None,
            fft_properties=FftAcqProperties.from_com(active_properties_com.FftProperties) if active_properties_com.HasFftProperties else None,
            front_end_properties=FrontEndProperties.from_com(active_properties_com.FrontEndProperties) if active_properties_com.HasFrontEndProperties else None,
            general_properties=GeneralAcqProperties.from_com(active_properties_com.GeneralProperties) if active_properties_com.HasGeneralProperties else None,
            generators_properties=active_properties_com.GeneratorsProperties if active_properties_com.HasGeneratorsProperties else None,
            multi_frame_properties=active_properties_com.MultiFrameProperties if active_properties_com.HasMultiFrameProperties else None,
            signal_enhancement_properties=SignalEnhancementProperties.from_com(active_properties_com.SignalEnhancementProperties) if active_properties_com.HasSignalEnhancementProperties else None,
            time_properties=TimeAcqProperties.from_com(active_properties_com.TimeProperties) if active_properties_com.HasTimeProperties else None,
            trigger_properties=TriggerProperties.from_com(active_properties_com.TriggerProperties) if active_properties_com.HasTriggerProperties else None,
            vibrometers_properties=unpack_collection(active_properties_com.VibrometersProperties, VibrometerAcqProperties) if active_properties_com.HasVibrometersProperties else None,
            zoom_fft_properties=active_properties_com.ZoomFftProperties if active_properties_com.HasZoomFftProperties else None
        )

@dataclass
class AcquisitionInfoModes(BaseClass):
    """Acquisition information modes."""
    active_mode: Any
    active_properties: ActiveProperties

    @classmethod
    def from_com(cls, acquisition_info_modes_com):
        return cls(
            active_mode=acquisition_info_modes_com.ActiveMode,
            active_properties=ActiveProperties.from_com(acquisition_info_modes_com.ActiveProperties)
        )

@dataclass
class Domain(BaseClass):
    """Domain information."""
    name: str
    type: Any

    @classmethod
    def from_com(cls, domain_com):
        return cls(
            name=domain_com.Name,
            type=domain_com.Type
        )

@dataclass
class Display(BaseClass):
    """Display information."""
    name: str
    type: Any

    @classmethod
    def from_com(cls, display_com):
        return cls(
            name=display_com.Name,
            type=display_com.Type
        )

@dataclass
class Signal(BaseClass):
    """Signal information."""
    description: SignalDesc  # Forward reference
    name: str
    displays: List[Display]

    @classmethod
    def from_com(cls, signal_com):
        return cls(
            description=SignalDesc.from_com(signal_com.Description),
            name=signal_com.Name,
            displays=unpack_collection(signal_com.Displays, Display)
        )
    
    def check_display_exist(self, display_name: str) -> bool:
        """Check if a display with the given name exists."""
        return any(display.name == display_name for display in self.displays)
    
    def assert_display_exist(self, display_name: str) -> None:
        """Assert that a display with the given name exists."""
        if not self.check_display_exist(display_name):
            available_names = [display.name for display in self.displays]
            raise AssertionError(f"The display name {display_name} does not exist. Available: {available_names}")
    
    def get_display(self, display_name: str) -> Display:
        """Get a display by name."""
        self.assert_display_exist(display_name)
        return next(display for display in self.displays if display.name == display_name)

@dataclass
class Channel(BaseClass):
    """Channel information."""
    caps: Any
    domain: Domain
    name: str
    signals: List[Signal]

    @classmethod
    def from_com(cls, channel_com):
        return cls(
            caps=channel_com.Caps,
            domain=Domain.from_com(channel_com.Domain),
            name=channel_com.Name,
            signals=unpack_collection(channel_com.Signals, Signal)
        )
    
    def check_signal_exist(self, signal_name: str) -> bool:
        """Check if a signal with the given name exists."""
        return any(signal.name == signal_name for signal in self.signals)
    
    def assert_signal_exist(self, signal_name: str) -> None:
        """Assert that a signal with the given name exists."""
        if not self.check_signal_exist(signal_name):
            available_names = [signal.name for signal in self.signals]
            raise AssertionError(f"The signal name {signal_name} does not exist. Available: {available_names}")
    
    def get_signal(self, signal_name: str) -> Signal:
        """Get a signal by name."""
        self.assert_signal_exist(signal_name)
        return next(signal for signal in self.signals if signal.name == signal_name)

@dataclass
class DataPoint(BaseClass):
    """Data point information."""
    meas_point: Optional[MeasPoint]
    average_count: int
    _datapoint_com: Any = field(repr=False)  # Keep COM object for getData method

    @classmethod
    def from_com(cls, datapoint_com):
        try:
            meas_point = MeasPoint.from_com(datapoint_com.MeasPoint)
        except pywintypes.com_error:
            print("This file type (PVD?) does not have measpoint attribute")
            meas_point = None
        
        return cls(
            meas_point=meas_point,
            average_count=datapoint_com.AverageCount,
            _datapoint_com=datapoint_com
        )
    
    def get_data(self, display, frame):
        """Get data using the original COM object."""
        return self._datapoint_com.GetData(display, frame)
    
    def is_disabled(self) -> bool:
        """Check if the data point is disabled."""
        return self.meas_point.is_disabled() if self.meas_point else False

@dataclass
class PointDomain(BaseClass):
    """Point domain containing channels and data points."""
    channels: List[Channel]
    rms_correction_factor: float
    type: Optional[str]
    data_points: List[DataPoint]

    @classmethod
    def from_com(cls, point_domain_com):
        return cls(
            channels=unpack_collection(point_domain_com.Channels, Channel),
            rms_correction_factor=point_domain_com.RmsCorrectionFactor,
            type=get_key_from_value(point_domain_com.Type, PTCDomainType),
            data_points=unpack_collection(point_domain_com.DataPoints, DataPoint)
        )
    
    def check_channel_exist(self, channel_name: str) -> bool:
        """Check if a channel with the given name exists."""
        return any(channel.name == channel_name for channel in self.channels)
    
    def assert_channel_exist(self, channel_name: str) -> None:
        """Assert that a channel with the given name exists."""
        if not self.check_channel_exist(channel_name):
            available_names = [channel.name for channel in self.channels]
            raise AssertionError(f"The channel name {channel_name} does not exist. Available: {available_names}")
    
    def get_channel(self, channel_name: str) -> Channel:
        """Get a channel by name."""
        self.assert_channel_exist(channel_name)
        return next(channel for channel in self.channels if channel.name == channel_name)

@dataclass
class Xaxis(BaseClass):
    """X-axis information for signal description."""
    name: str
    unit: str
    min: float
    max: float
    count: int
    xaxis_com_object: Any = field(repr=False)

    @classmethod
    def from_com(cls, xaxis_com):
        return cls(
            name=xaxis_com.Name,
            unit=xaxis_com.Unit,
            min=xaxis_com.Min,
            max=xaxis_com.Max,
            count=xaxis_com.MaxCount,
            xaxis_com_object=xaxis_com
        )

    def get_mid_x(self, num: int) -> float:
        """Returns the x-axis position of the middle of the bin that should display the data."""
        return self.xaxis_com_object.GetMidX(num)

@dataclass
class Yaxis(BaseClass):
    """Y-axis information for signal description."""
    name: str
    unit: str
    min: float
    max: float

    @classmethod
    def from_com(cls, yaxis_com):
        return cls(
            name=yaxis_com.Name,
            unit=yaxis_com.Unit,
            min=yaxis_com.Min,
            max=yaxis_com.Max
        )

@dataclass
class SignalDesc(BaseClass):
    """Signal description with axis information."""
    complex: bool
    data_type: Any
    db_reference: float
    domain_type: Optional[str]
    function_type: Any
    name: str
    power_signal: bool
    rms_correction_factor: float
    xaxis: Xaxis
    yaxis: Yaxis
    attributes: Any

    @classmethod
    def from_com(cls, signal_desc_com):
        return cls(
            complex=signal_desc_com.Complex,
            data_type=signal_desc_com.DataType,
            db_reference=signal_desc_com.DbReference,
            domain_type=get_key_from_value(signal_desc_com.DomainType, PTCDomainType),
            function_type=signal_desc_com.FunctionType,
            name=signal_desc_com.Name,
            power_signal=signal_desc_com.PowerSignal,
            rms_correction_factor=signal_desc_com.RMSCorrectionFactor,
            xaxis=Xaxis.from_com(signal_desc_com.XAxis),
            yaxis=Yaxis.from_com(signal_desc_com.YAxis),
            attributes=signal_desc_com.Attributes
        )
