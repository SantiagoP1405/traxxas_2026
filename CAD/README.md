# CAD MODELS
>Main contributors: Santiago Patricio Gómez Ochoa | 
These folders contain the base files of the CAD models designed for each stage of the **TRAXXAS** robot. Each version changed based on the required hardware, physical limitations and other basic needs that were covered over time. 

For each version the following subfolders will be found (unless they were not required):
- **MODEL**: CAD models for each used part in ```.SLDPRT``` format and assemblys that show the complete model in ```.SLDASM``` format.
- **DRAW**: Technical drawings in both ```.SLDDRW``` and ```.PDF``` format for each part of the *MODEL* folder, that describe their meassurements in milimeters (mm). 
- **STL**: This folder contains 3D printable files that were exported from its corresponding ```.SLDPRT``` file, in ```.STL``` format. Depending on the TRAXXAS version, printable files from deprecated or test parts might be found. *If this is the case, do not consider the file if its correspondig CAD model is not found in the **MODEL** folder.* 

>IMPORTANT: All the CAD models, assemblys and drawings were made in the SolidWorks 2025 | Student Edition CAD software. Keep in mind that all the files were named in the *spanish* language in order to keep simplicity. 

--- 
## [Display Base](./DISPLAY_BASE)
- [***Technical drawings***](./DISPLAY_BASE/DRAW)
- [***CAD***](./DISPLAY_BASE/MODEL)
- [***Printable files***](./DISPLAY_BASE/STL)
This model was created in order to have a platform to display the robot, perform mechanic or electronic adjustments, and mainly to perform motor tests without harming the motor platform or other hardware. 
<p align="center">
  <img width="556" height="565" alt="Display Base"
       src="https://github.com/user-attachments/assets/67d0e958-be2e-48f4-aa65-c753a7114866" />
</p>


This model is shaped by the following parts:
   - ```base_inferior```: Serving as foundation support for the whole structure. Holds the part ```pilar```, and counts with a slot for the same part whenever the whole assembly needs to be storaged. 
     <p align="center">
     <img width="450" height="300" alt="image" src="https://github.com/user-attachments/assets/394f3e89-f902-439a-8dd9-36542515143e" />
     </p>
   - ```soporte```: Serves as a block beneath ```base_inferior``` to give extra support for ```pilar```, and avoid any breaking.
     <p align="center">
      <img width="450" height="300" alt="image" src="https://github.com/user-attachments/assets/520fd9c4-6739-47c5-a3db-c27741ef7754" />
      <img width="450" height="300" alt="image" src="https://github.com/user-attachments/assets/bebc5260-50cc-422a-bfdf-8d69c467ec88" />
     </p>
   - ```pilar```: Allows the display to have a desired hight. Two versions of this piece were made, one 128.7 mm and the other 117 mm long.
     <p align= "center">
     <img width="450" height="300" alt="image" src="https://github.com/user-attachments/assets/072dae7a-9877-401a-8e1f-010022b78e1f" />
     <img width="450" height="300" alt="image" src="https://github.com/user-attachments/assets/8ef33245-752f-4a7c-93bb-d79b97c428b2" />
     </p>
   -```base_superior```: Serves as the main platform that ultimately elevates the robot.
   <p align="center">
     <img width="450" height="300" alt="image" src="https://github.com/user-attachments/assets/77ed78c1-7dea-4f3f-9251-fd4beed00e7c" />
   </p>
   <p align="center">
     <img width="450" height="300" alt="image" src="https://github.com/user-attachments/assets/007ba711-bd63-4e1e-924d-808d8d028b44" />
     <img width="450" height="300" alt="image" src="https://github.com/user-attachments/assets/502a7add-a415-4637-ad54-47b41418e33f" />
   </p>
