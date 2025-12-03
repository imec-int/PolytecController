"""
Author: Maxime Leurquin
Date: Feb-2024
"""
import os,sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from PolytecController.polysoft import Psv,Vibsoft,Pylon
from PolytecController.polyfile import Svd,Pvd,PolySettings
from PolytecController.classes import Infos
from PolytecController import polytec_common
from PolytecController.enumerations import *

"""
from enumerations import *
from polysoft import Vibsoft,Psv
from polyfile import Pvd,Svd,PolySettings
from polytec_common import translate_status
import utils as ut
"""

class PSV_tests():
    def test_offset_scanpoint(settings_filepath):
        psv=Psv()
        psv.open()
        psv.switch_to_acquisition_mode()
        modified_settings_path=os.path.join(psv.temp_folder,"testModdedSettings.set")
        psv.load_settings_from_file(settings_filepath)
        psv.offset_scanpoints(world_offset=np.array([200,400]))
        psv.save_current_settings(modified_settings_path)

        #display the modified settings scanpoints along the original ones
        original_settings=Svd(settings_filepath)
        original_scanpoints=original_settings.get_scanpoints(category="All",ref_frame="pixel")
        original_settings.close_file()

        modded_settings=Svd(modified_settings_path)
        modded_scanpoints=modded_settings.get_scanpoints(category="All",ref_frame="pixel")
        modded_settings.close_file()

        fig,ax=plt.subplots(figsize=(10,10))
        ax.scatter(original_scanpoints[:,0],original_scanpoints[:,1],label="original")
        ax.scatter(modded_scanpoints[:,0],modded_scanpoints[:,1],label="modded")
        ax.legend()
        ax.grid()
        plt.show()
        psv.clear_temp_directory()
        psv.close()
        return

    def test_scan(psv,settings_filepath,scan_savepath):
        psv.load_settings_from_file(settings_filepath)
        psv.trigger_scan(scan_savepath)
        print("loading settings & saving scan success")
        return

    def test_snapshot(psv,savepath):
        psv.export_snapshot(savepath)
        print("export_snapshot success")
        return

    def test_get_scanpoints(psv):
        print(psv.get_scanpoints(category="All",ref_frame="pixel"))
        print("get_scanpoints success")
        return

    def test_with_statement():
        with Psv() as psv:
            psv.switch_to_acquisition_mode()
        return
    
    def test_switch_to_acqusition():
        psv=Psv()
        psv.open()
        psv.switch_to_acquisition_mode()
        print("switch_to_acquisition success")
        return

class Vibsoft_tests():
    def test_scan(savepath):
        with Vibsoft() as vibsoft:
            savepath=r'D:\MaximeLeurquin\testVibsoft.pvd'
            vibsoft.trigger_scan(savepath)
        print("test_scan success")
        return
    
    
class Pylon_tests():
    def test_export_snapshot(savepath):
        with Pylon() as camera:
            camera.export_snapshot(savepath=savepath)
        print("export_snapshot success")
        return

    def test_load_camera_settings(settings):
        with Pylon() as camera:
            camera.load_settings(settings)
        print("load camera settings success")
        return
    
    def test_live_view(savepath):
        with Pylon() as camera:
            camera.start_live_view()
            camera.export_snapshot_in_thread(savepath=savepath)
        print("live_view & screenshot success")
        return
    

class PolySettings_tests():
    def test_read_settings(settings):
        with PolySettings(settings) as set:
            print(set.infos)
        return  

if __name__=="__main__":
    polytec_settings=r"my/path/to/SettingsL8.set"
    camera_settings=r"my/path/to/vibroflex_camera_settings.pfs"

    psv_scan_savepath=r'D:\MaximeLeurquin\testScan.svd'
    pvd_scan_savepath=r'D:\MaximeLeurquin\testScan.pvd'
    snapshot_savepath=r'D:\MaximeLeurquin\testSnapshot.png'

    ##############################################
    """
    Pylon_tests.test_live_view(snapshot_savepath)
    Pylon_tests.test_export_snapshot(snapshot_savepath)
    Pylon_tests.test_load_camera_settings(camera_settings)

    
    Vibsoft_tests.test_scan(pvd_scan_savepath)
    """
    ############################################
    psv=Psv()
    psv.open()

    #PSV_tests.test_scan(psv,psv_scan_savepath)
    #PSV_tests.test_snapshot(psv,snapshot_savepath)
    #PSV_tests.test_get_scanpoints(psv)
    PSV_tests.test_switch_to_acqusition()
    PSV_tests.test_with_statement()
    #PSV_tests.test_offset_scanpoint(polytec_settings)

    ###############################################
    PolySettings_tests.test_read_settings(polytec_settings)

