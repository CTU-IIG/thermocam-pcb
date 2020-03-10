# thermocam-pcb
Tool for measuring temperature of PCB board with WorksWell thermo camera

## Requirements

### Operating System

32/64b x86  Linux, preferably Ubuntu 16.04, for maximum compatibility with the eBUS SDK. 

### WIC SDK

You can find the official documentation of the WIC SDK at `https://software.workswell.eu/wic_sdk/Linux`.

#### Download & prerequisites
To download the WIC SDK installer, go to `software.workswell.eu/wic_sdk/Linux`. You need to enter your email to get the download link. Then extract and run the executable for your chosen distribution (`WIC_SDK-Linux_Ubuntu_16.04_64b-1.1.0.run` for 64bit Ubuntu 16.04). 

The packages `build-essential` and `libjpeg-dev` are requied to install the WIC SDK. The installer installs the WIC SDK and the eBUS SDK (required for the WIC SDK to work). 

#### Installation

All files are installed in /opt folder.

* The WIC SDK is found in folder: /opt/workswell/wic_sdk
* The eBUS SDK is found in folder: /opt/pleora/ebus_sdk/YOUR_LINUX_DISTRIBUTION

During the installation of the eBUS SDK:

* Select to add eBUS libraries to the path
* Don't install eBUS for Ethernet
* Select to install the eBUS daemon

 You can select for manual or automatic startup of the daemon - if you select manual, don't forget to run `service eBUSd start` before running the program.

If you selected `auto` running during installation, the daemon may still not start properly on startup. You may check its status with `systemctl status eBUSd`. If status shows "inactive (dead)", add runlevels at the following line to the daemon script located in `/etc/init.d/eBUSd`: 

`Default-Start:     2 3 4 5`

Adding runlevels to the script will enable `update-rc.d` to work with it. Make sure to run `sudo update-rc.d eBUSd defaults` and `sudo update-rc.d eBUSd enable` afterwards so the daemon runs at startup.

#### Environment setup

You need to set up your environment variables to run the program and use the WIC and eBUS SDKs. Run `/opt/workswell/wic_sdk/set_env_variables` to do so.

For me the script did not set all environment variables correctly. If this is the case for you, you may try running the following commands to set up your environment (make sure to enter the folder name with your linux distribution in the second command):

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

### OpenCV

For development, download and compile OpenCV source version 2.4 (the latest stable build on Ubuntu 16.04). For simply running the executable on an Ubuntu 16.04 machine it is enough to install the libopencv-dev library.

## Compilation

Before running make, be sure to change the variables `WIC_HOME`, `EBUS_HOME` and `OPENCV_HOME` in the Makefile to the paths where you installed the WIC and eBUS SDK and OpenCV respectively.

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

Click on the image to enter a point, press Esc to delete the last entered point, press enter to finish selection.

To export the points to a json format that can be imported in subsequent runs, enter the path to the export file after the option `-e/--enter_poi` as follows:

`./thermocam-pcb -l . -epath/to/myfile.json`
or
`./thermocam-pcb -l . --enter_poi=path/to/myfile.json`

### Importing points

You can import previously saved points by running:

`./thermocam-pcb -l . -p my_exported_points.json`

### Recording video

You can record lossless video by running:

`./thermocam-pcb -l . -r video_filename.avi`

Recording starts after pressing Enter, and ends with pressing Esc.

### Setting video as input

You can set video as input for the program by running:

`./thermocam-pcb -v video_filename.avi`

If the input is video, there is no need to specify the WIC license path, as the camera is not used. All functions (entering points, etc.) work exactly the same way as when the input is from the camera.
