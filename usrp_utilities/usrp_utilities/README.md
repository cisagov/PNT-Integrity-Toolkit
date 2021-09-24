# USRP Utilities


## Dependencies

### Baseline build tools
The following tools are required to build the usrp_utilities library. This do not have to be installed if there is an existing ROS installation (melodic or newer)

* Python3
* build-essential
* cmake

### VCSTool

On OSX run:

    pip install -U vcstool

On Ubuntu run:

	sudo apt-get install python3-vcstool
	sudo apt install python-vcstools

### Colcon

[Install Colcon](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon) (if ROS2 is not installed).  

On MacOS run:

    pip install -U colcon-common-extensions

On Ubuntu run:

    sudo apt install python3-colcon-common-extensions

### uhd
The uhd library from Ettus is needed to communicate with the USRP hardware. 
Detailed information on installing UHD can be found [here](https://files.ettus.com/manual/page_install.html)

On Ubuntu / Debian:

```bash
sudo add-apt-repository ppa:ettusresearch/uhd
sudo apt-get update
sudo apt-get install libuhd-dev libuhd003 uhd-host
```
On OSX:

It is recommended to [build UHD from source](https://files.ettus.com/manual/page_build_guide.html) with the correct version of boost (above) preinstalled

### Fast-RTPS 
Download and install Fast-RTPS.  Note if code generation tools are not needed or Java is not available, run cmake without the '-DBUILD_JAVA=ON' option.  

    git clone https://github.com/eProsima/Fast-RTPS
    cd Fast-RTPS
    git checkout v1.8.2
    mkdir build
    cd build
    cmake ../ -DTHIRDPARTY=ON
    make
    make install #sudo if necessary

## Installation

Create a workspace directory:

	mkdir -p usrp_utilities_ws/src

Download the usrp_utilities.repos file, which contains the list of repositories to clone, into the *usrp\_utilities\_ws* directory. Clone the repositories into the source folder:

	vcs import src < usrp_utilities.repos

Build the workspace using colcon:

    colcon build --merge-install --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'


## Usage

  *Comming in a future release*

