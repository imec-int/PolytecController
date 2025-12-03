"""
Author: Maxime Leurquin
Date: Mar-2024
"""
import os,sys
import numpy as np
import matplotlib.pyplot as plt

# Add the src directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from PolytecController.polyfile import Pvd, Svd, PolySettings
import PolytecController.utils as ut
from PolytecController.polytec_common import translate_status


#TEST_SVD=os.path.join("tests","random_polyfiles","msa600_measurement_results.svd") #add your own.
TEST_SVD=os.path.join("tests","random_polyfiles","my_svd_file.svd") #add your own.
TEST_SET=os.path.join("tests","random_polyfiles","my_settings.set") #add your own.

def test_plot_image():
    with Svd(TEST_SVD) as svd:
        fig,ax=plt.subplots(figsize=(10,10))
        svd.plot_image(ax=ax,show_scanpoints=True,category="all",label_enabled_scanpoints=False)
    plt.show()

def test_scanpoint_conversion_and_plotting():
    #get the scanpoints of the svd file in their 3 different reference frames and plot them side by side, with the image
    fig,axs=plt.subplots(nrows=1,ncols=4,figsize=(12,4))
    with Svd(TEST_SVD) as svd:
        svd.plot_scanpoints(ax=axs[0],category="All",ref_frame="pixel",title="Pixel sp")
        svd.plot_scanpoints(ax=axs[1],category="All",ref_frame="raw",title="raw sp")
        svd.plot_scanpoints(ax=axs[2],category="All",ref_frame="video",title="video sp")
        svd.plot_image(ax=axs[3],show_scanpoints=True)
    for ax in axs:
        ax.grid()
        ax.legend()
    fig.tight_layout()
    plt.show()

def test_pvd():
    with Pvd(TEST_PVD) as pvd:
        print(pvd)
        pvd.print_available_data()
        freq,displacements,usd=pvd.get_point_data(domain_name="FFT",channel_name="Vib", signal_name="Displacement", display_name="Magnitude")

    fig,ax=plt.subplots(figsize=(10,5))
    ax.plot(freq,displacements[0])
    ax.set_title(TEST_PVD)
    ax.grid()
    plt.show()
    return

def test_modify_scanpoints():
    new_file=ut.postpend_filename(TEST_SET,"mod")
    with PolySettings(TEST_SET) as settings:
        sp=settings.get_scanpoints(category="All",ref_frame="video")
        mod_scanpoints=ut.scale_point_grid(sp,factor=2)#sp+np.array([2,4])
        settings.modify_scanpoints(new_file,mod_scanpoints)
        
    with PolySettings(new_file) as new_sett:
        print(f"{new_sett.filename=}")
        new_sp=new_sett.get_scanpoints(category="All",ref_frame="video")

    fig,ax=plt.subplots(figsize=(10,10))
    ax.scatter(sp[:,0],sp[:,1],color="orange",label="ori",s=20,zorder=1)
    ax.scatter(mod_scanpoints[:,0],mod_scanpoints[:,1],color="purple",label="should be",zorder=1,s=20,alpha=0.5)
    ax.scatter(new_sp[:,0],new_sp[:,1],color="b",label="new",zorder=5,s=5)
    
    ax.grid()
    ax.legend()
    plt.show()
    return


def test_get_scanpoint_statuses():
    with Svd(TEST_SVD) as svd:
        statuses=svd.get_scanpoints_statuses()
        translated_statuses = [translate_status(status) for status in statuses]
        all=svd.get_scanpoints(category="All",ref_frame="pixel")
        enabled=svd.get_scanpoints(category="Enabled",ref_frame="pixel")
        over=svd.get_scanpoints(category="Overrange",ref_frame="pixel")
        print(f"{all=}")
        print(f"{len(all)=}")
        print(f"{len(enabled)=}")
        print(f"{over=}")

    print(f"{statuses=}")
    print(f"{translated_statuses=}")
    return
    
def check_versions():
    old_file=TEST_SVD
    new_file=TEST_SVD4
    with Svd(old_file) as svd:
        print(f"old file: {svd.version}")

    with Svd(new_file) as svd:
        print(f"new file: {svd.version}")
    return
   
def test_version_object():
    with Svd(TEST_SVD) as svd:
        version=svd.version
        print(version)

        print(version.file_id_string)
    
        time_t=version.file_time
        print(time_t.strftime('%d-%b-%Y %H:%M:%S %Z'))
        print(time_t.timestamp())

        
def test_get_laser_intensity():
    with Svd(TEST_SVD4) as svd:
        laser_intensity=svd.get_laser_intensity()
        print(f"{laser_intensity=}")
    return

def test_get_laserPower():
    with Svd(TEST_SVD) as svd:
        print(svd.infos)
        vibProp=svd.infos.acquisition_info_modes.active_properties.vibrometers_properties[0]
        #print(vibProp)
        #print(f'{vibProp.vibControllerSettings.sensorHeadSettingsCollection[0].dimmer=}')
    return

if __name__=="__main__":
    test_get_laserPower()
    test_version_object()
    #test_get_laser_intensity()
    check_versions()

    test_get_scanpoint_statuses()
    #test_modify_scanpoints()
    test_plot_image()
    #assert False
    test_scanpoint_conversion_and_plotting()
    test_pvd()