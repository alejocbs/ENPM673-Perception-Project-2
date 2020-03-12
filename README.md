# Project 2 - Lane Detection and Dark Video Correction
- Zack Taylor
- Christian Bueno
- John Dinofrio

### Files
Three python files have been submitted for the code part of this project.

1. problem1.py -> Brightens image and improves visibility of lane markings, other vehicles, and road signs.

2. problem2_lines.py -> Find lanes and impose them back into the image. Determine heading of the vehicle.

3. helpers.py -> utility functions specific to supporting the first two files.

### Dependencies
There are dependencies on external libraries openCV2 and numpy. Both can be installed using pip3 for python3:

pip3 install numpy
pip3 install opencv-python

The helpers.py is a library of helper functions used by both the other files.

### Directory Configuration
All the files need to be in the same directory. you also need the video files to be in the same
folder structure

-[Code]
  --problem1.py
  --problem2_lines.py
  --helpers.py
  --kalman.py
  --Videos
    ---NightDrive-2689.mp4
    ---data_1
        ----Camera parameters and individual frames using 10 digits and .png extension. (e.g. 0000000024.png
    ---data_2
        ----Camera parameters
        ----challenge_video.mp4

The video files will not be included in our zip file due to size constraints, but must be set up in this structure in order for the code to work.
We did not get our solution fully implemented for the challenge video as we struggled to consistently identify the right lane marker.

### Execution
These scripts can be executed using python 3 in the normal way:

python3 [filename].py

During execution some of the output windows may be stacked. Move them around to see the various stages of processing that we
output along the way all at once.