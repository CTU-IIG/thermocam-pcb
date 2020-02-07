# thermocam-pcb
Tool for measuring temperature of PCB board with WorksWell thermo camera

## Requirements

### Operating System

32/64b x86  Linux, preferably Ubuntu, for maximum compatibility with the eBUS SDK. 

### WIC SDK

To download the WIC SDK, go to the [workswell SDK website](workswell.cz/WIC_SDK/). For installation follow their [documentation steps](https://workswell.cz/WIC_SDK_new/Linux/doc/).

The installer installs the WIC SDK, and the eBUS SDK (required for the WIC SDK to work). 

For me, the script setting the WIC and eBUS environment variables(`/opt/workswell/wic_sdk/set_env_variables`) did not set all environment variables correctly. If this is the case for you, you may try running the following commands to set up your environment (make sure to enter the folder name with your linux distribution in the second command):

```
LD_LIBRARY_PATH=/opt/workswell/wic_sdk/lib:${LD_LIBRARY_PATH}

export PUREGEV_ROOT=/opt/pleora/ebus_sdk/YOUR_LINUX_DISTRIBUTION
export GENICAM_ROOT=$PUREGEV_ROOT/lib/genicam
export GENICAM_ROOT_V2_4=$GENICAM_ROOT
export GENICAM_LOG_CONFIG=$GENICAM_ROOT/log/config/DefaultLogging.properties
export GENICAM_LOG_CONFIG_V2_4=$GENICAM_LOG_CONFIG
if [ "$HOME" = "/" ]; then
export GENICAM_CACHE_V2_4=/.config/Pleora/genicam_cache_v2_4
else
export GENICAM_CACHE_V2_4=$HOME/.config/Pleora/genicam_cache_v2_4
fi
export GENICAM_CACHE=$GENICAM_CACHE_V2_4
export GENICAM_LIB_DIR=$GENICAM_ROOT/bin/Linux64_x64
mkdir -p $GENICAM_CACHE
export GENICAM_ROOT_V3_0=$GENICAM_ROOT
```

Whichever solution works for you, you may want to insert it into your .bashrc to be loaded each time the console starts.

If you selected manual running of the eBUS daemon during installation, don't forget to run `service eBUSd start` before running the program.

### OpenCV 4

Install OpenCV 4 for C++. There are many guides out there, I used [this](https://www.learnopencv.com/install-opencv-4-on-ubuntu-18-04/) one.

## Compilation

Before running make, be sure to change the variables `WIC_HOME` and `EBUS_HOME` in the Makefile to the paths where you installed the WIC and eBUS SDK.

After this, the program can be simply compiled with running `make` in the project folder.

## Usage

The program requires the path to the directory where the WIC license file is stored to run. To simply display the thermocamera image, place the license file to the project directory and run:

`./thermocam-pcb -l .`

You can use the additional arguments of the program to import and/or enter coordinates of points to be measured. 

During entering points, the points you have imported will be visible and you can modify them. 

After the entering phase, the temperature of the points will be written to stdout. 

### Entering points

To enter points manually, run the command:

`./thermocam-pcb -l . -e`

Click on the image to enter a point, press Esc to delete the last entered point, press enter to finish selection. After finishing, you will be given the option to export them in a json format, that can be imported in subsequent runs.

### Importing points

You can import previously saved points by running:

`./thermocam-pcb -l . -p my_exported_points.json`
