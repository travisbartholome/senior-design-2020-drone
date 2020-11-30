# AirSim stuff

## Running

To run demos, download the demo (zip file) from the airsim releases page on github.
They're labeled as "Environments".
Extract the file contents, then run the main executable from the zip.
Then run one of the demos from the airsim repo.

For example: `python AirSim/PythonClient/multirotor/hello_drone.py`

Note that the engine/simulator/executable has to be running before you run the python script.
The script seems to just connect to the simulator.

## Generated figures

The code I've been writing sometimes generates plots (e.g., position plots).
These images will be created in the `local-figures` directory.
This directory isn't tracked by Git.
If you want to save a generated figure, move it to the `saved-figures` directory.
