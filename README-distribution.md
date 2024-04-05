# Python
This folder contains libraries, tools, and tests to interact with the ams OSRAM TMF882X devices over I2C and GPIO.

## Setup Python Environment
This framework requires to use python 3.6 or newer for Window 10.
To install the required packages, run `python -m pip install -r requirements.txt`.

## Adding your own sub-directory
When you add a new sub-directory and want to execute python files there you need to 
`import __init__` before any other local import. Also you need to copy the file `__init__.py` into this subdirectory
and make sure the python root path points to the directory where this readme.md file is located.
`TOF_PYTHON_ROOT_DIR = os.path.normpath(os.path.dirname(__file__) + "/..")`

This is a summary of the files and sub-folders:
	
## ./tmf882x
All python classes, files and functions, specific to the TMF8820, TMF8821 and TMF8828.
There is a python class to control the device hardware and the bootloader that also allows to download intel hex files to the device.
There is a second class that implements measurement application specific commands and controls and data readout, as well as histogram configuration and readout.

## ./tmf882x/dll
This directory contains the DLL for the TMF8xxx descattering filter.

### ./tmf882x/examples
Several examples that show the usage:
- how to perform measurements and read them out
- how to perform i2c slave address change 
- how to configure for histogram dumping and how to read them out

### ./tmf882x/hex_files
Hex files that are used by the above examples to run the device correctly.
 
### ./tmf882x/tests
Python tests to verify functonality of device and/or scripts.
