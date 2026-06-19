# CAD MODELS

These folders contain the base files of the CAD models designed for each stage of the **TRAXXAS** robot. Each version changed based on the required hardware, physical limitations and other basic needs that were covered over time. 

For each version the following subfolders will be found (unless they were not required):
- **MODEL**: CAD models for each used part in ```.SLDPRT``` format and assemblys that show the complete model in ```.SLDASM``` format.
- **DRAW**: Technical drawings in both ```.SLDDRW``` and ```.PDF``` format for each part of the *MODEL* folder, that describe their meassurements in milimeters (mm). 
- **STL**: This folder contains 3D printable files that were exported from its corresponding ```.SLDPRT``` file, in ```.STL``` format. Depending on the TRAXXAS version, printable files from deprecated or test parts might be found. *If this is the case, do not consider the file if its correspondig CAD model is not found in the **MODEL** folder.* 

>IMPORTANT: All the CAD models, assemblys and drawings were made in the SolidWorks 2025 | Student Edition CAD software. 

--- 
## Display Base
This model was created in order to have a platform to display the robot, perform mechanic or electronic adjustments, and mainly to perform motor tests without harming the motor platform or other hardware. 