Supporting code for the case study in Chapter 5.   

## Trajectory preparation

First we run `python robotracker.py repo/run/run` to extract the trajectory for a run using the optical flow method in OpenCV.  Note that the path to the run video excludes the file extension `.mp4`.  As the optical flow method can be sensitive to light conditions, the process for extracting and cleaning trajectories for analysis is semi-automatic.   Visual inspection is necessary for precision.  

Running the `robotracker.py` script outputs three files into the directory `repo/run`: 

- `run_coord.txt`
- `run_trace.png`
- `run_1stframe.png`

Once these files have been generated, it is then possible to load the trajectory using `clean_traj.ipynb`, manually inspect and clean it then write the final trajectory to `repo/run/run_coord.csv`.  Cleaning involves removing additional points that may have been picked up by the optical flow method in OpenCV and truncating so all trajectories start and end at the same entrance to the cul-de-sace.  We also specify the number of collisions observed and add this to run metadata in `repo/run/run.json`. This is repeated for each run. 

## Extracting and cleaning log data

Once the trajectories have been extracted, we then use `clean_logs.ipynb` to extract relevant data and add it to `repo/run/run.json`.  Specifically, this extracts plan data, model checking memory usage, and process memory useage. Note that this is fully automated.  

## Building the dataset

Once the above steps have been done, we then run `python build_dataset.py` to iterate through all runs in `repo` and build a dataset, output as a `.csv` file. We have included `DATASET.csv` which is the dataset we used for analysis.  

## Running analysis

It is then possible to run `analysis.ipynb` to generate results.  
