# Analysis workflow

The analysis assumes that there is a repository of raw data in some directory `repo` and outputs all interim files to the repository during cleaning.  

## Trajectory preparation

First we use `python robotracker.py repo/run/run` to extract the trajectory for a run using the optical flow method in OpenCV.  Note that the path to the run video excluded the file extension `.mp4`.  

