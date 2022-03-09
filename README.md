# thermocam-pcb
Tool for measuring temperature of PCB board with WorksWell thermal camera and for detection of heat sources.

The web interface of this program in action is shown below. You can see the workload migrating between two CPUs every 1 second.
![thermocam-pcb](https://user-images.githubusercontent.com/140542/128368657-89a24322-7181-4b47-893c-71b7a0477037.gif)

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Requirements](#requirements)
    - [Operating System](#operating-system)
    - [WIC SDK](#wic-sdk)
        - [Download & prerequisites](#download--prerequisites)
        - [Installation](#installation)
        - [Environment variables](#environment-variables)
    - [OpenCV](#opencv)
    - [Webserver](#webserver)
- [Compilation](#compilation)
- [Building and deploying with Nix](#building-and-deploying-with-nix)
- [Usage](#usage)
    - [Basic functionality](#basic-functionality)
    - [Additional functionality](#additional-functionality)
    - [Setting video as input instead of camera](#setting-video-as-input-instead-of-camera)
    - [Changing between views](#changing-between-views)
    - [Point tracking](#point-tracking)
    - [Heat source detection in a defined area](#heat-source-detection-in-a-defined-area)
    - [Built-in webserver](#built-in-webserver)
- [Precision of temperature measurement](#precision-of-temperature-measurement)
- [Command line reference](#command-line-reference)

<!-- markdown-toc end -->


## Requirements

### Operating System

WIC SDK for x86 seems to be supported only on *Ubuntu 16.04* due to
requirements of the eBUS SDK. Since Ubuntu 16.04 is no longer
officially supported, we recommend to use the [Nix][] package manager,
which can be used on any Linux distribution and solves all dependency
problems. If you use Nix, skip to [Building and deploying with
Nix](#building-and-deploying-with-nix).

[Nix]: https://nixos.org/

### WIC SDK

You can find the official documentation of the WIC SDK at `https://software.workswell.eu/wic_sdk/Linux`.

#### Download & prerequisites

Download the [WIC SDK](https://software.workswell.eu/wic_sdk/Linux).
You need to enter your email to get the download link.

The packages `build-essential` and `libjpeg-dev` are required to
install the WIC SDK. The installer installs the WIC SDK and the eBUS
SDK (required for the WIC SDK to work).

#### Installation

Extract the downloaded `WIC_SDK_Linux.zip` archive and install the
eBUS SDK by running:

    sudo dpkg -i eBUS_SDK_Ubuntu-x86_64-5.1.10-4642.deb


Install other needed packages:

    apt install build-essential libjpeg-dev libboost-dev libtbb-dev

#### Environment variables

You need to set up your environment variables to run the programs
using the WIC and eBUS SDKs. Correct values of the needed variables
will be found in the [`./run`](./run.in) script. After compiling
thermocam-pcb, it will be located at `build/run`.

### OpenCV

Download and compile OpenCV source version 4.5.1 (or later). This
might be tricky to do on Ubuntu 16.04. Therefore we recommend using
the [Nix package manager](#building-and-deploying-with-nix).

Recorded video is stored in a lossless HuffYUV(HFYU) format which
OpenCV does not have built in, so its codec is needed to be installed
externally, e.g. by part of `libavcodec`.

### Webserver

We use the crow C++ webserver, which requires the `boost` and `pthread` libraries.

## Compilation

To compile the program, run:

    meson setup build [options]
	ninja -C build

Useful `options` are:
- `-Dpkg_config_path=$HOME/opt/opencv-4.5.4/lib/pkgconfig` to specify
  path to specific OpenCV installation.
- `-Debus_home=` or `-Dwic_home` to specify paths to the eBUS and WIC SDKs
- `-Dcpp_link_args=-static-libstdc++` if you compile on a system with
  newer libstdc++ and running the resulting binary on the target
  systems fails with: /usr/lib/x86_64-linux-gnu/libstdc++.so.6:
  version `GLIBCXX_3.4.26' not found
- other options reported by `meson configure` or `meson setup -h`.

## Building and deploying with Nix

Compiling thermocam-pcb with the Nix package manager is simpler than
the above procedure, as Nix automatically handles all the
dependencies.

1. Install Nix, e.g. `sh <(curl -L https://nixos.org/nix/install) --daemon`
2. (optional) Enable THERMAC binary cache to get prebuilt OpenCV
   - `nix-env -iA cachix -f https://cachix.org/api/v1/install`
   - `cachix use thermac`
3. Add WIC SDK to the nix store:
   - `nix-store --add-fixed WIC_SDK_Linux.zip`
4. Run `nix-build`
5. Run the program with `./result/bin/run` or `./result/bin/thermocam-pcb`.

To deploy the compiled program to the turbot board (where our camera
is connected) run:

1. Copy the program and all dependencies to turbot:
   ```sh
   nix copy --to ssh://root@turbot $(readlink result)
   ```
2. Test whether the program runs there:
   ```shell
   ssh -X ubuntu@turbot $(readlink result)/bin/run -l license_163C1908.wlic
   ```
3. If everything works well, install it permanently:
   ```sh
   ssh ubuntu@turbot "
       nix-env -i $(readlink result) &&
       systemctl --user daemon-reload &&
       systemctl --user restart thermocam-pcb
   "
   ```

## Usage

### Basic functionality

To simply display the thermocamera image, run the program and specify
the WIC camera license file:

    ./build/thermocam-pcb -l license_XXXXXXXX.wlic

or

    ./build/run -l license_XXXXXXXX.wlic

### Additional functionality

You can use multiple functions, many of them simultaneously:

* Enter points on the image and print their temperature
* Export or import these points to a json format
* Record video
* Use the recorded video as input instead of grabbing images from the camera
* Set delay between prints/display
* Display exported points and their corresponding camera image
* Enable web server for live presentation of grabbed and processed data
* Point tracking (if the board or camera moves, the tracked points
  stay at the same location on the board)

### Setting video as input instead of camera

Add the path to your video to the arguments as `-v myvideo.avi`. 
Running the previous example(enter,import,export,record) with video input instead of camera can thus be done with:

`./build/thermocam-pcb -v myvideo.avi -p import.json --enter-poi=export.json -r recording.avi`

### Changing between views

There are 3 views available to display points and their temperature:

* Full view - the name of the point and its temperature is displayed next to the point
* Temperature only view - only the temperature is displayed next to the point
* Legend view - the point names and temperatures are listed next to the image and only their index is displayed next to the point

You can change between these views by pressing Tab.

### Point tracking

Point tracking allows to track positions of points of interest (POI)
even when the board or camera moves. The POIs to track are specified
via a reference image. You can create the reference image with POIs
by:

    ./build/thermocam-pcb --enter-poi=points.json

The result can be later viewed by:

    ./build/thermocam-pcb -s points.json

The POIs can be edited by:

	./build/thermocam-pcb -p points.json --enter-poi=points.json

To change the names of the points, edit the resulting JSON file by hand.

To enable point tracking use `-p` together with the `-t` switch.
Several tracking modes can be specified via an optional argument:
- `-t`: Synchronous tracking of every frame; most likely decreases
  frame rate.
- `-tonce`: Tracking is applied only to the first grabbed frame;
  for later frames POI location remains constant.
- `-tbg`: Tracking is computed in background. This results in full
  frame rate, but when the board/camera moves, POI-related data may be
  incorrect for a few frames.

### Heat source detection in a defined area

Four of the POIs specified via `-p` can be used as a border of area
for heat source detection. The names of the points (as stored in the
`.json` file) need to be specified as a comma-separated list to
`--heat-sources=` argument. For example:

    ./build/thermocam-pcb -p points.json --heat-sources=tl,tr,br,bl

The heat source locations are calculated by applying a negative
Laplacian kernel on the smoothed polygon area. This is based on the
heat diffusion equation, by ignoring the temporal term and finding the
local maxima of the negative Laplacian:

![heat_diffusion_equation](heat_diffusion_equation.png "Heat diffusion equation")

### Built-in webserver

The parameter `-w` starts a webserver on port `8080`.

The following URLs are available:

* `/` (e.g. http://localhost:8080/) Visit this with a web browser for
  live visualisation of all data
* `/ws` web socket where information about new frames is pushed. To
  access the data from the command line, use e.g. the
  [websocat](https://github.com/vi/websocat) tool. Thermocam-PCB sends
  there JSON-formatted data. To access location of heat sources, use
  e.g.:

        websocat ws://turbot:8080/ws | jq -c .heat_sources

* `/XXX.jpg` and `XXX.tiff`, where XXX is e.g. `laplacian-current`:
  Images with preprocessed data from thermocamera. The `.jpg` is
  color-full image for showing on the `/` webpage, the `.tiff` version
  contains raw data (64bit float pixels).
* `/temperatures.txt` returns the current POI Celsius temperatures in
  `name=temp` format
* `/heat-sources.txt` returns the heat source locations in the format
  `heat_sources=x₀,y₀,–∇²₀;x₁,y₁,–∇²₁; ...`
* `/points.txt` returns both POI temperatures and heat source
  locations in a single response
* `/position-std.txt` returns the current rolling standard deviations
  of POI positions in `name=position` format, which is 0 if tracking
  is not enabled.
* `/uptime.txt` contains server uptime in seconds.
* `/frame.txt` contains the number of processed frames.
* `/users.txt` the number of active websocket connections.
* `/metrics` – POI temperatures and other information as [Prometheus](https://prometheus.io/) metrics.

The `.tiff` images downloaded from the web server can be processed by
[ThermocamPCB Julia package](./julia).

## Precision of temperature measurement

The [WIC specifications](https://workswell-thermal-camera.com/workswell-infrared-camera-wic) state a measurement accuracy of ±2°C. If the measurement accuracy is lower than this, check that that the thermal emissivity of the measured object is equal to the value set in the WIC SDK - 0.95 by default. Masking the surface with black electrical insulating tape achieves an emissivity of 0.95-0.97.

## Command line reference

<!-- help start -->
```
Usage: thermocam-pcb [OPTION...] [--] COMMAND...
Displays thermocamera image and entered points of interest and their
temperature. Writes the temperatures of entered POIs to stdout.

  -c, --csv-log=FILE         Log temperature of POIs to a csv file instead of
                             printing them to stdout.
      --compenzation-img=FILE   Compenzation image (to subtract from grabbed
                             image)
  -d, --delay=NUM            Set delay between each measurement/display in
                             seconds.
  -e, --enter-poi[=FILE]     Enter Points of interest by hand, optionally save
                             them to json file at supplied path.
      --fourcc=CODE          4-letter code for video codec used by -r (e.g.
                             MJPG, h264), default: HFYU
  -h, --heat-sources=PT_LIST Enables heat sources detection. PT_LIST is a comma
                             separated list of names of 4 points (specified
                             with -p) that define detection area. In most
                             cases, you'll want to enable -t too.
  -l, --license-file=FILE    Path of WIC license file.
  -p, --poi-path=FILE        Path to config file containing saved POIs.
  -r, --record-video=FILE    Record video and store it with entered filename
  -s, --show-poi=FILE        Show camera image taken at saving POIs.
      --save-img-dir=DIR     Target directory for saving an image with POIs
                             every "save-img-period" seconds.
                             "." by default.
      --save-img-period=SECS Period for saving an image with POIs to
                             "save-img-dir".
                             1s by default.
  -t, --track-points[=once]  Turn on tracking of points. If "once" is
                             specified, tacking happens only for the first
                             image. This allows faster processing if the board
                             doesn't move. If "bg" is specified, calculations
                             run in a background thread.
  -v, --load-video=FILE      Load and process video instead of camera feed
  -w, --webserver            Start webserver to display image and
                             temperatures.
  -?, --help                 Give this help list
      --usage                Give a short usage message
  -V, --version              Print program version

Mandatory or optional arguments to long options are also mandatory or optional
for any corresponding short options.

Requires path to directory containing WIC license file to run with camera.

Controls:
Tab                - Change view  (Full | Temperature only | Legend)
Mouse click (left) - Enter point  (only with --enter-poi)
Backspace          - Remove point (only with --enter-poi)
Esc                - Exit program

Report bugs to https://github.com/CTU-IIG/thermocam-pcb/issues.
```
<!-- help end -->

<!-- Local Variables: -->
<!-- markdown-toc-user-toc-structure-manipulation-fn: markdown-toc-structure-demote -->
<!-- End: -->
