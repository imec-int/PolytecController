
# Dependencies
 - Make sure polytec file access is installed on your computer. You can get it through [polytec update](https://www.polytec.com/int/vibrometry/products/software/polytec-update). This will allow you to use the functions defined in the polyfile.py file of this repository, which are useful to extract data from .svd or .pvb files.
 - If you want to actually control a Laser Doppler Vibrometer you will need to have the proprietary "PSV Acquisition" or "Vibsoft" software installed additionnaly - those require specific licenses or hardware dongles. For just reading .svd or .pvd files this is not required.
 - If you want to control the camera of a Polytec Vibroflex you will need to have [Pylon](https://www.baslerweb.com/en/downloads/software/1378313866/) installed.
 
 # Installation instructions
 This assumes you are using uv to manage your python environments.
 1. Your ```pyproject.toml``` should look like this:
  
```python
#Contents of a simple pyproject.toml file
[project]
 #...
dependencies = [
    #...
    "PolytecController>=0.1.0",
]

[tool.uv.sources]
PolytecController = { git = "https://github.imec.be/MMICRO/PolytecController.git", branch = "main" }

```
2. Run ```uv sync``` to update all the dependencies.



### Accessing data stored in .svd files
```python
from PolytecController.polyfile import Svd
with Svd("mypath\my_file.svd") as svd:
  svd.print_available_data() #show contents available in the svd file
  x,y,usd=svd.get_point_data(domain_name="FFT",channel_name="Vib",signal_name="Velocity",display_name="Magnitude",point_index=-1, frame= 0)#access data
  svd.plot_image(savepath=savepath,show_scanpoints=True,
                           label_scanpoints=True,
                           label_scanstatus=True,
                           category='All') #plot image and overlay scanpoints on it.
  image=svd.get_image() #May also be of interest: access image stored in svd file
```

### Accessing data stored in .pvd files
```python
from PolytecController.polyfile import Pvd
with Pvd("mypath\my_file.pvd") as pvd:
  pvd.print_available_data()
  x,y,usd=pvd.get_point_data(domain_name="FFT",channel_name="Vib",signal_name="Velocity",display_name="Magnitude",point_index=-1, frame= 0)
```
### Reading settings files
```python
from PolytecController.polyfile import PolySettings
with PolySettings("mypath\my_file.set") as my_set:
  scanpoints=my_set.get_scanpoints(category="All",ref_frame="video")
  print(my_set.infos)
```
### Launching a measurement with psv.
```python
from PolytecController.polysoft import Psv
with Psv() as psv:
  psv.load_settings_from_file(settings_filepath="my_settings.set") #load saved settings
  psv.trigger_scan(savepath="my_results.svd") #launch a scan
  psv.wait_for_scan_end()

  #May also be of interest:
  psv.export_snapshot(savepath="my_image.png") #save the image displayed in the polytec software. Note that this is accessible directly from the svd file if a scan is made.
  psv.save_current_settings(savepath="my_settings.set") #save scan settings
```
### Launching a measurement with Vibsoft.
```python
from PolytecController.polysoft import Vibsoft
with Vibsoft() as vibsoft:
  vibsoft.trigger_scan(savepath="mysavepath.pvd")
```
### Getting a picture from the vibroflex camera.
```python
from PolytecController.polysoft import Pylon
Pylon.export_snapshot(savepath="mypic.png")
```

# Troubleshooting
## Unable to launch any command
Try running the most basic thing you can run:
```python
#This should launch PSV
from win32com.client import Dispatch
app=Dispatch("PSV.Application")
app.Activate()
```
If you are unable to run this it likely indicates that you have a missing registry key. To fix it:
1. Quit the PSV software
2. Start a command prompt with administrative rights
3. Navigate to C:\Program Files\Polytec\PSV 10.1 (or 10.2 etc) using the ```cd``` command
4. run ```psv.exe /regserver```
5. Restart the computer & try again.
