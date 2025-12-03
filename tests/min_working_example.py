
#OPEN PSV:
from win32com.client import Dispatch
app=Dispatch("PSV.Application")

import sys,os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
#OPEN SVD FILE
from PolytecController.polyfile import Svd
SVD_FILE=r"Z:\SCAN_PSV103.svd"
if __name__ == "__main__":
    with Svd(SVD_FILE) as svd:
        print(svd.version)
        