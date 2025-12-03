"""
Author: Maxime Leurquin
Date: Nov-2023
Description: A collection enumerations defined by polytec.
"""
def get_key_from_value(value,dictionnary:dict):
    """
    Returns the key associated with a given value in a dictionary.
    :param value: any, value to search for in the dictionary.
    :param dictionary: dict, input dictionary.
    :return: any, key associated with the given value in the dictionary. Returns None if the value is not found.
    """
    for key, val in dictionnary.items():
        if val == value:
            return key
    return None

#-----------------Polytec PolyFile Type library----------------------------------
PTCDomainType={
    "ptcDomain3rdOctave": 4,  # Domain 3rdOctave.
    "ptcDomainNotAvail": 0,  # Domain is not available.
    "ptcDomainRMS": 6,  # Domain RMS.
    "ptcDomainSpectrum": 3,  # Domain Spectrum.
    "ptcDomainTime": 1,  # Domain Time.
}

PTCDisplayType = {
    'ptcDisplayImag': 8,                   # The imaginary part.
    'ptcDisplayInstVal': 11,               # The instantaneous value for a given phase.
    'ptcDisplayIQFitted': 13,              # The IQ display (digital demodulation, VDD only).
    'ptcDisplayMag': 1,                    # The magnitude.
    'ptcDisplayMagDb': 3,                  # The magnitude in dB.
    'ptcDisplayMagDbA': 5,                 # The magnitude in dBA.
    'ptcDisplayMagDbPhase': 4,             # The magnitude in dB and the phase in degree.
    'ptcDisplayMagDbPhaseRad': 15,         # The magnitude in dB and the phase in radian.
    'ptcDisplayMagPhase': 2,                # The magnitude and the phase in degree.
    'ptcDisplayMagPhaseRad': 14,           # The magnitude and the phase in radian.
    'ptcDisplayNotAvail': 0,               # The display is not available.
    'ptcDisplayNyquist': 10,               # Nyquist.
    'ptcDisplayPhase': 6,                  # The phase.
    'ptcDisplayReal': 7,                   # The real part.
    'ptcDisplayRealImag': 9,               # The real and the imaginary part.
    'ptcDisplaySamples': 12                # The samples (for the time domain only).
}

PTCInfoType  = { #useful when creating an svd file
    'ptcInfoAcquisition': 0,
    'ptcInfoAlignments': 13,
    'ptcInfoAPS': 4,
    'ptcInfoCameraSettings': 7,
    'ptcInfoDbReferences': 11,
    'ptcInfoElements': 2,
    'ptcInfoHardware': 1,
    'ptcInfoMeasPoints': 5, 
    'ptcInfoMeasurementLocations': 18,
    'ptcInfoPositionDevice': 17,
    'ptcInfoPositioningStage': 15,
    'ptcInfoProfiles': 6,
    'ptcInfoScanHead': 8,
    'ptcInfoScanHeadDevices': 12,
    'ptcInfoTextures': 19,
    'ptcInfoVibrometers': 16,
    'ptcInfoVideo': 3,
    'ptcInfoVideoBitmap': 9,
    'ptcInfoVideoMapping': 14
}


PTCFileID = {#useful when creating an svd file
    'ptcFileIDCombinedFile': 7, #The file is a PSV combined measurement file. 
    'ptcFileIDMPVFile': 9, #he file is a MPV measurement file. 
    'ptcFileIDMPVSettings': 8, #The file is a MPV settings file. 
    'ptcFileIDPMAFile': 5,
    'ptcFileIDPSVFile': 4, #The file is a PSV measurement file. 
    'ptcFileIDPSVSettings': 3, #The file is a PSV settings file.
    'ptcFileIDSigProFile': 6, #The file is a Signal Processor file. 
    'ptcFileIDUnknown': 0, #The file has an unknown file type. 
    'ptcFileIDVibSoftFile': 2,#The file is a VibSoft measurement file. 
    'ptcFileIDVibSoftSettings': 1, #The file is a VibSoft settings file. 
}


PTCCreateFlags = {
    "ptcCreateAlways": 1, #The file will be always created. An existing file will be overwritten.
    "ptcCreateNone": 0, #No create flags.
}

PTCFocusStatus = {
    "ptcFocusStatusAuto": 128,  # Laser focus was automatically assigned.
    "ptcFocusStatusBest": 8,    # Laser focus was assigned best.
    "ptcFocusStatusCalculated": 64,  # Laser focus was calculated.
    "ptcFocusStatusFailed": 32,  # Laser focus measurement failed.
    "ptcFocusStatusFast": 4,   # Laser focus was assigned fast.
    "ptcFocusStatusInterpolated": 16,  # Laser focus was interpolated by valid neighbors.
    "ptcFocusStatusManual": 2,  # Laser focus was assigned manually.
    "ptcFocusStatusNone": 0,  # Point has no laser focus.
    "ptcFocusStatusValid": 1,  # Point has a valid laser focus.
}

PTCGeometryStatus = {
    "ptcGeoStatusCalculated2dFrom3d": 4096,  # 2D coordinate (video coordinate) was calculated from the 3D coordinate using the 2D alignment.
    "ptcGeoStatusCalculated3dFrom2d": 2048,  # 3D coordinate was calculated from the 2D coordinate (video coordinate), e.g., when using PSV-UHF.
    "ptcGeoStatusCalculated3dFrom2dAndDistance": 16384,  # 3D coordinate was calculated from the 2D coordinate (video coordinate), the corresponding scanner angles, and the distance to the measurement object.
    "ptcGeoStatusFailed": 512,  # Measurement of 3D coordinate has failed.
    "ptcGeoStatusImported": 8,  # 3D coordinate was imported (UFF or MEScope).
    "ptcGeoStatusInterpolated": 32,  # 3D coordinate was interpolated by valid neighbors.
    "ptcGeoStatusMeasured": 4,  # 3D coordinate was measured with the geometry scan unit (PSV-A-420).
    "ptcGeoStatusModified": 64,  # 3D coordinate was modified by the user.
    "ptcGeoStatusNone": 0,  # Point has no 3D coordinate.
    "ptcGeoStatusOptimalMeasured": 1024,  # 3D coordinate was measured with the geometry scan unit (PSV-A-420) with optimal signal level.
    "ptcGeoStatusTooBright": 128,  # No valid 3D coordinate. Too much light (PSV-A-420).
    "ptcGeoStatusTooDark": 256,  # No valid 3D coordinate. Too little light (PSV-A-420).
    "ptcGeoStatusTriangulation": 16,  # 3D coordinate was calculated by PSV-3D triangulation (PSV-3D only).
    "ptcGeoStatusValid": 1,  # Point has a valid 3D coordinate.
    "ptcGeoStatusVideoTriangulated": 8192,  # 3D coordinate was calculated by video-triangulation.
}


PTCGraphicFormatType = {
    "ptcGraphicFormatBitmap": 0,
    "ptcGraphicFormatGIF": 12,
    "ptcGraphicFormatJPEG": 7,
    "ptcGraphicFormatPNG": 2,
    "ptcGraphicFormatTIFF": 1,
}

PTCPathLengthStatus = {
    "ptcPathLengthStatusAuto": 8,  # The optical path length has been assigned automatically.
    "ptcPathLengthStatusFailed": 2,  # The optical path length adjustment has failed.
    "ptcPathLengthStatusInterpolated": 16,  # The optical path length has been interpolated.
    "ptcPathLengthStatusManual": 4,  # The optical path length has been assigned manually.
    "ptcPathLengthStatusNone": 0,  # No optical path length value has been set.
    "ptcPathLengthStatusValid": 1,  # The optical path length value is valid.
}


PTCSaveAs = {
    "ptcSaveAsAll": 0,
    "ptcSaveAsAverageSpectrum": 3,
    "ptcSaveAsBands": 1,
    "ptcSaveAsSinglePoint": 2,
    "ptcSaveAsTypeAll": 508,
    "ptcSaveAsTypeAllNoOriginal": 420,
    "ptcSaveAsTypeAllNoSinglePoint": 460,
    "ptcSaveAsTypeAllNoUser": 92,
    "ptcSaveAsTypeAverageSpectrum": 64,
    "ptcSaveAsTypeAverageSpectrumUser": 128,
    "ptcSaveAsTypeBands": 8,
    "ptcSaveAsTypeBandsUser": 256,
    "ptcSaveAsTypeInfos": 4,
    "ptcSaveAsTypeSinglePoint": 16,
    "ptcSaveAsTypeSinglePointUser": 32
}

#See software manual PSV chapter 8 for more
"""
-Not Measured: Before a scan, all scan points have the Not Measured status.

-Valid: When scanning, the software gives every scan point the Valid, Optimal
or Overrange status.

-Optimal: The Optimal status can only occur if Signal Enhancement is active
for at least one channel. See SECTION 7.2.7 on this as well. At these scan
points, the signal-to-noise ratio is relatively high.

-Overrange: At scan points with Overrange status, the input range of the data
acquisition board was exceeded on at least one channel. However, the PSV
works with an amplitude level reserve of 25% (M-/V systems) or of 100% (A-/
B-/H systems) respectively. The measurement signals can exceed the
nominal input range by the specified amounts (is detected as Overrange),
without them being limited.

-Invalidated: After the scan, you can manually assign the scan points the
Invalidated status. You will find details on this in presentation mode.

-Disabled: If you do not want to scan single scan points, you can assign them
the Disabled status. See SECTION 6.6.3 on this.

-Not Reachable: If you define the scan points, then scan points which are
outside the scan area of at least one scanning head are given the Not
Reachable status.

-Hidden: If you are working with 3D geometries and parts of the object hide
scan points so that they cannot be reached by at least one laser beam, then
these scan points are identified as Hidden. Prerequisite of this is that in the
Preferences dialog on the Geometry page, you have activated Hidden Points
Calculation. See SECTION 3.4 on this as well.

-VT Failed: If VideoTriangulation fails, the geometry status remains
unchanged, no vibration measurement data is recorded, and the scan points
are given the VT Failed scan status.

IMPORTANT NOTE: The status can be a combination of several bit constants defined in the enumeration.
To check what the status value mean take the status value, perform a logical and with eg: ptcScanStatusValid and check whether the result is equal to ptcScanStatusValid.
"""
PTCScanStatus = {
    "ptcScanStatusAssignedChannel1D": 2048,  # Point mapped to a 1D channel, usable only with MPV files.
    "ptcScanStatusAssignedChannel3D": 4096,  # Point mapped to a 3D channel, usable only with MPV files.
    "ptcScanStatusAssignedSensor": 8192,  # Point mapped to a sensor, usable only with MPV files.
    "ptcScanStatusDisabled": 16,  # Point is disabled for scan.
    "ptcScanStatusHidden": 128,  # Point is hidden by the 3D geometry.
    "ptcScanStatusInterpolate": 512,  # The data at the point shall be interpolated from the data of its neighbors.
    "ptcScanStatusInterpolationFailed": 1024,  # The interpolation of the data of this point failed.
    "ptcScanStatusInvalidated": 8,  # Point is invalidated.
    "ptcScanStatusInvalidFrames": 32,  # Point has invalid frames (MultiFrame only).
    "ptcScanStatusNone": 0,  # Point is not measured.
    "ptcScanStatusNotReachable": 64,  # Point was not reachable by the scanner.
    "ptcScanStatusOptimal": 2,  # Point is optimal (Signal Enhancement only).
    "ptcScanStatusOverrange": 4,  # Point has overrange status.
    "ptcScanStatusValid": 1,  # Point is valid.
    "ptcScanStatusVideoTriangulationFailed": 256,  # The video-triangulation failed for this point.
}

PTCAverageType={
    "ptcAverageComplex":2, #complex averaging,
    "ptcAverageMagnitude":1, #Magnitude averaging,
    "ptcAverageOff":0, # averaging is off,
    "ptcAveragePeakhold":4, #Peak hold averaging,
    "ptcAverageTime":3 #Time mode averaging,
}

PTCSettings = {
    "ptcSettings2DAlignment": 512, # 2D alignment settings.    
    "ptcSettings3DAlignment": 1024,# 3D alignment settings.        
    "ptcSettingsAcquisition": 2,# Acquisition settings.    
    "ptcSettingsAlignment": 4,# Alignment settings.   
    "ptcSettingsAll": 65535,# All settings.   
    "ptcSettingsAPS": 312,# Scan point definition settings.
    "ptcSettingsCamera": 64,# Camera settings.  
    "ptcSettingsCameraAndAlignment": 68,# Camera and alignment settings. 
    "ptcSettingsNone": 0,# No settings.    
    "ptcSettingsWindows": 128# Window layout settings.
}


PTCSignalEnhancementMode={
"ptcSignalEnhancementModeBest":4,
"ptcSignalEnhancementModeFast":0,
"ptcSignalEnhancementModeFastStandard":1,
"ptcSignalEnhancementModeStandard":2,
"ptcSignalEnhancementModeStandardBest":3,
}

PTCVibrationDirection = {
    "ptc3DVector": 6,  # The direction is a vector. This means the channel is a 3D channel.
    "ptcXNeg": 3,      # The direction is negative x.
    "ptcXPos": 2,      # The direction is positive x.
    "ptcYNeg": 5,      # The direction is negative y.
    "ptcYPos": 4,      # The direction is positive y.
    "ptcZNeg": 1,      # The direction is negative z.
    "ptcZPos": 0       # The direction is positive z.
}


PTCInputCoupling = {
    "ptcInputCouplingAC": 0,       # The input coupling is AC.
    "ptcInputCouplingDC": 1,       # The input coupling is DC.
    "ptcInputCouplingUnknown": 2,  # The input coupling is unknown.
}


PTCPhysicalQuantity = {
    "ptcPhysicalQuantityAcceleration": 0,                   # Quantity acceleration.
    "ptcPhysicalQuantityAngle": 1,                          # Quantity angle.
    "ptcPhysicalQuantityAngleDegree": 24,                   # Quantity angle in degree.
    "ptcPhysicalQuantityAngularAcceleration": 2,            # Quantity angular acceleration.
    "ptcPhysicalQuantityAngularAccelerationDegree": 25,     # Quantity angular acceleration in degree.
    "ptcPhysicalQuantityAngularVelocity": 3,                # Quantity angular velocity.
    "ptcPhysicalQuantityAngularVelocityDegree": 26,         # Quantity angular velocity in degree.
    "ptcPhysicalQuantityCounts": 15,                        # Quantity counts.
    "ptcPhysicalQuantityDisplacement": 4,                   # Quantity displacement.
    "ptcPhysicalQuantityElectricalCurrent": 5,              # Quantity electrical current.
    "ptcPhysicalQuantityForce": 6,                          # Quantity force.
    "ptcPhysicalQuantityFrequency": 23,                     # Quantity frequency.
    "ptcPhysicalQuantityGravitationalAcceleration": 20,      # Quantity gravitational acceleration.
    "ptcPhysicalQuantityPower": 7,                          # Quantity power.
    "ptcPhysicalQuantityPressure": 8,                       # Quantity pressure.
    "ptcPhysicalQuantityRevolution": 9,                     # Quantity revolution.
    "ptcPhysicalQuantityRPM": 10,                           # Quantity RPM.
    "ptcPhysicalQuantitySoundPressure": 11,                 # Quantity sound pressure.
    "ptcPhysicalQuantityStrain": 19,                        # Quantity strain.
    "ptcPhysicalQuantityStress": 21,                        # Quantity stress.
    "ptcPhysicalQuantityTime": 22,                          # Quantity time.
    "ptcPhysicalQuantityTorque": 12,                        # Quantity torque.
    "ptcPhysicalQuantityVelocity": 13,                      # Quantity velocity.
    "ptcPhysicalQuantityVoltage": 14,                       # Quantity voltage.
    "ptcPhysicalQuantityVolume": 16,                        # Quantity volume.
    "ptcPhysicalQuantityVolumeAcceleration": 18,            # Quantity volume acceleration.
    "ptcPhysicalQuantityVolumeVelocity": 17                 # Quantity volume velocity.
}

PTCWindowFunction  = {
    "ptcWindowFctBartlett": 1,            # The window function is Bartlett.
    "ptcWindowFctBlackmanHarris": 2,      # The window function is Blackman Harris.
    "ptcWindowFctExponential": 3,         # The window function is exponential.
    "ptcWindowFctFlatTop": 4,             # The window function is flat top.
    "ptcWindowFctForce": 5,               # The window function is force.
    "ptcWindowFctHamming": 6,             # The window function is Hamming.
    "ptcWindowFctHanning": 7,             # The window function is Hanning.
    "ptcWindowFctRectangle": 0,           # The window function is rectangle.
    "ptcWindowFctTaperedHanning": 8       # The window function is tapered Hanning.
}

PTCChannelType  = {
    "ptcChannelTypeAnalog": 0,             # The signal type is analog.
    "ptcChannelTypeAnalogFixed": 1,        # The signal type is analog, settings cannot be changed.
    "ptcChannelTypeDigitalVib": 5,         # The channel is connected digitally to the vibrometer.
    "ptcChannelTypeFringeCounter": 2,      # The signal type is fringe counter.
    "ptcChannelTypeLogical": 3,            # The signal type is logical.
    "ptcChannelTypeModulated": 4,          # The signal type is modulated, i.e. the software will perform a digital demodulation on this channel.
    "ptcChannelTypeMultiPointVib": 6       # The channel is connected to a multipoint vibrometer.
}


PTCProgramID={
    "ptcProgramIDMPV":7,
    "ptcProgramIDMSVInplane":4,
    "ptcProgramIDPMA":4,
    "ptcProgramIDPolyFile":6,
    "ptcProgramIDPSV":2,
    "ptcProgramIDScannerControl":3,
    "ptcProgramIDTMS":5,
    "ptcProgramIDUnknown":0,
    "ptcProgramIDVibSoft":1,
    "ptcProgramIDVideoPanel":8


}
#--------------------------polytec PSV type enumerations---------------------------------
PTCAcqStartMode={
"ptcAcqStartContinuous":1, #Continuous measurement. 
"ptcAcqStartSingle":0, #Single measurement. 
}

PTCAcqState={
"ptcAcqStateAutoFocus":6, #A focus measurement is in progress.
"ptcAcqStateContinuous":2, #Continuous measurement is in progress.
"ptcAcqStatePathLength":7, #A measurement of the optical path length is in progress.
"ptcAcqStateScanAll":3, #A scanning measurement of all points is in progress.
"ptcAcqStateScanContinue":4, #A scanning measurement is being continued.
"ptcAcqStateScanRemeasure":5, #A scanning re-measurement is in progress.
"ptcAcqStateSingle":1, #Single shot measurement is in progress.
"ptcAcqStateStopped":0, #Measurement is stopped.
}

PTCApplicationMode={
    "ptcApplicationModeAcquisition":1, #Acquisition mode (only PSV). 
    "ptcApplicationModeNormal":0, #Normal mode (only VibSoft). 
    "ptcApplicationModePresentation":2 #Presentation mode (only PSV). 
}

PTCArrangeStyle={
"ptcArrangeCascaded":3, #Cascade the windows.
"ptcArrangeIcons":0, #Arrange all windows as minimized icons.
"ptcArrangeTiledHorizontal":1, #Tile the windows horizontally.
"ptcArrangeTiledVertical":2, #Tile the windows vertically.
}

PTCAssignFocusAutomaticallyCaps={
"ptcAssignFocusAutomaticallyCapsNone":0, #The focus values are determined from and assigned to all scan points.
"ptcAssignFocusAutomaticallyCapsSelected":1, #The focus values are determined from and assigned to the selected scan points, only.
}

PTCDigitalPort={
"ptcDigitalPortGate":0, #The port representing the Gate connector.
"ptcDigitalPortIn1":2, #The port representing the Aux In connector.
"ptcDigitalPortOut1":1, #The port representing the Aux Out connector.
}

PTCDigitalPortType={
"ptcDigitalPortTypeRead":0, #Read only port.
"ptcDigitalPortTypeReadWrite":2, #Read and write port.
"ptcDigitalPortTypeWrite":1, #Write only port.
}

PTCDocumentType={
"ptcDocumentTypeSigPro":1, #The document is a SigProDocument.
"ptcDocumentTypeVib":0, #The document is a VibDocument.
}

PTCExportType={
"ptcExportTypeAsamOdsAtfx":2, #ASAM ODS ATFX export file type
"ptcExportTypeMEScope":1, #ME'scope export file type
"ptcExportTypeUniversalFile":0, #Universal File (UFF) export file type
}

PTCFileFormat={
"ptcFileFormatAnalyzer":0, #Analyzer file format (.pvd).
"ptcFileFormatGraphic":2, #Graphic format.
"ptcFileFormatText":1, #ASCII format.
}

PTCGeometryScanCaps={
"ptcGeometryScanCapsNone":0, #The geometry scan is performed for all scan points and without refining the measured distances using the focusing unit.
"ptcGeometryScanCapsRefineWithAutoFocus":2, #Uses the focusing unit of the scanning head after measuring the distances to refine the measured distances.
"ptcGeometryScanCapsSelected":1, #Performs the geometry scan for the selected scan points, only.
}

PTCMessageAnswer={
"ptcMessageAnswerAbort":3, #Answer as if Abort was clicked.
"ptcMessageAnswerCancel":2, #Answer as if Cancel was clicked.
"ptcMessageAnswerIgnore":5, #Answer as if Ignore was clicked.
"ptcMessageAnswerNo":7, #Answer as if No was clicked.
"ptcMessageAnswerOk":1, #Answer as if Ok was clicked.
"ptcMessageAnswerRetry":4, #Answer as if Retry was clicked.
"ptcMessageAnswerYes":6, #Answer as if Yes was clicked.
}

PTCMessageStyle={
"ptcMessageStyleABORTRETRYIGNORE":2, #Abort, Retry and Ignore buttons.
"ptcMessageStyleICONASTERISK":64, #The asterix icon.
"ptcMessageStyleICONEXCLAMATION":48, #The exclamation mark icon.
"ptcMessageStyleICONHAND":16, #The hand icon.
"ptcMessageStyleICONQUESTION":32, #The question mark icon.
"ptcMessageStyleOK":0, #OK button.
"ptcMessageStyleOKCANCEL":1, #OK and Cancel button.
"ptcMessageStyleRETRYCANCEL":5, #Retry and Cancel buttons.
"ptcMessageStyleYESNO":4, #Yes and No buttons.
"ptcMessageStyleYESNOCANCEL":3, #Yes, No and Cancel buttons.
}

PTCPrintOptions={
"ptcPrintLower":4, #Print the lower pane of the window.
"ptcPrintNoUserPrompt":1, #Do not prompt the user.
"ptcPrintUpper":2, #Print the upper pane of the window.
}

PTCScanCaps={
"ptcScanCapsAutoFocusDuringScan":8, #The autofocusing will be performed during the scan.
"ptcScanCapsNone":0, #No capabilities.
"ptcScanCapsUseAssignedFocusValuesDuringScan":1, #The assigned focus values will be used during the scan.
"ptcScanCapsVideoTriangulation":2, #The video triangulation will be performed during the scan.
"ptcScanCapsVideoTriangulationFast":4, #The fast video triangulation will be performed during the scan.
}

PTCScanMode={
"ptcScanAll": 0, #Starts a complete scan of all measurement points. 
"ptcScanContinue": 1, #Continues a previously started scan. 
"ptcScanRemeasure": 2, #Starts a re-measurement of a previously started scan. 
"ptcScanRemeasureAutoRange": 3, #Starts a re-measurement of a previously started scan with automatic adjustment of the vibrometer range for scan points with scan status overrange. 
"ptcScanRemeasureFile": 4, #Starts a re-measurement of the scan file specified in the ScanFileName Property. 
}

PTCScanState={
"ptcScanStateBeginScan":0, #The scan has begun.
"ptcScanStateBeginScanPoint":2, #The mirrors have been positioned and the measurement at the scan point is about to begin.
"ptcScanStateEndScan":1, #The scan has finished.
"ptcScanStateEndScanPoint":3, #The measurement at the scan point has finished.
}

PTCSnapshotType={
"ptcSnapshotTypeAuxCamera":1, #A snapshot of the auxiliary camera is generated.
"ptcSnapshotTypeMainCamera":0, #A snapshot of the main camera is generated.
"ptcSnapshotTypeOverviewImage":2, #A snapshot of the overview image is generated.
}

PTCViewSelection={
"ptcViewSelectionBoth":3, #Both the upper and the lower view are selected.
"ptcViewSelectionLower":2, #The lower view is selected.
"ptcViewSelectionUpper":1, #The upper view is selected.
}

PTCWarningLevel={
"ptcWarningLevelAll":0, #Show all messages.
"ptcWarningLevelErrors":2, #Show error messages only.
"ptcWarningLevelNone":3, #Do not show any messages.
"ptcWarningLevelWarnings":1, #Show warning and error messages only.
}

PTCWindowType={
"ptcWindowTypeAnalyzer":0, #The window is an analyzer window.
"ptcWindowTypeArea":1, #The window is an area window.
"ptcWindowTypeSigPro":2, #The window is a signal processor window.
}

PTCAcqMode = {
    "ptcAcqModeFastScan": 4,  # Acquisition mode FastScan.
    "ptcAcqModeFft": 1,  # Acquisition mode FFT.
    "ptcAcqModeInplane": 7,  # Acquisition mode for the PMA software.
    "ptcAcqModeIQ": 6,  # Acquisition mode IQ for digital demodulation (VDD)
    "ptcAcqModeMultiFrame": 3,  # Acquisition mode MultiFrame.
    "ptcAcqModeOrderTracking": 5,  # Acquisition mode OrderTracking.
    "ptcAcqModeTime": 0,  # Acquisition mode time.
    "ptcAcqModeZoomFft": 2,  # Acquisition mode Zoom-FFT.
}

PTCTriggerSource = {
    "ptcTriggerSourceAnalog": 2,  # The trigger source is analog.
    "ptcTriggerSourceExternal": 1,  # The trigger source is external.
    "ptcTriggerSourceInternal": 3,  # The trigger source is internal.
    "ptcTriggerSourceOff": 0,  # The trigger source is off.
}

PTCZStageType ={
    "ptcZStageE7XX":2, #E7XX z-stage
    "ptcZStageGCS2":3, #GCS2 z-stage 
    "ptcZStageGCS2_E709":4, #GCS2 E709 z-stage 
    "ptcZStageNone":0, #No z-stage 
    "ptcZStageSimulation":1 #Simulated z-stage 
}

PTCLensCalibrationCapsType = {
    "ptcLensCalibrationCapsInterference": 1,  # Interference objective for topography measurements.  
}