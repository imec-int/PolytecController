import os,cv2
from tqdm import tqdm

from .polyfile import Svd
from . import utils as ut

def extract_pictures(svd_folder:str,savefolder:str,**kwargs):
    """
    Get all the pngs out of the svd files and save them to a folder.
    
    Parameters:
    -----------
    svd_folder : str
        Folder containing svd files
    savefolder : str  
        Folder to save the png files
    **kwargs : dict
        Additional options:
        - clean_image : bool, default False
            If True, extracts clean images (same size as on psv, nothing displayed on it).
            Useful for testing computer vision models.
        - show_scanpoints : bool, default True
            Whether to show scan points on the image (ignored if clean_image=True)
        - category : str, default "All"
            Category filter for scan points (ignored if clean_image=True)
        - label_scanpoints : bool, default True
            Whether to label scan points (ignored if clean_image=True)
        - label_scanstatus : bool, default True
            Whether to label scan status (ignored if clean_image=True)
    """
    options = {"clean_image": False, "show_scanpoints": True,"category":"All","label_scanpoints": True,"label_scanstatus":True}
    #Merge defaults and kwargs. the value associated with the key in kwargs takes precedence over the value associated with the same key in defaults
    options.update(kwargs)
    files=[file for file in os.listdir(svd_folder) if file.endswith(".svd")]
    for filename in tqdm(files,unit="file"):
        name, extension = os.path.splitext(filename)
        filepath=os.path.join(svd_folder,filename)
        savepath=os.path.join(savefolder,f"{name}.png")
        
        with Svd(filepath) as svd:
            if options["clean_image"]:# Extract clean image without any overlays, useful for computer vision
                image=svd.get_image(cropbox=None)  
                ut.make_dir(savepath)
                cv2.imwrite(savepath, image)
            else:
                svd.plot_image(savepath=savepath,show_scanpoints=options["show_scanpoints"],
                               label_scanpoints=options["label_scanpoints"],
                               label_scanstatus=options["label_scanstatus"],
                               category=options["category"]
                               )                
    return