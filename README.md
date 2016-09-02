# OSVR-Vive-Libre

[![Join the chat at https://gitter.im/OSVR-Vive-Libre/Lobby](https://badges.gitter.im/OSVR-Vive-Libre/Lobby.svg)](https://gitter.im/OSVR-Vive-Libre/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Upstream <https://github.com/lubosz/OSVR-Vive-Libre>

A Free Software driver for OSVR that provides support for the HTC Vive headset
without the need of proprietary software dependencies. 
Vive Libre is based on [OpenHMD](https://github.com/OpenHMD/OpenHMD)'s Vive driver, [LighthouseRedox](https://github.com/nairol/LighthouseRedox/) and [ouvrt](https://github.com/pH5/ouvrt)
and was developed at [Collabora](https://www.collabora.com/) R&D.

## Features

Currently Vive Libre provides basic IMU tracking support via the OSVR API.
Lighthouse external tracking is WIP. 
Our efforts at analysing the Lighthouse signal can be found at
[vive-libre-analysis-and-data](https://git.collabora.com/cgit/user/lubosz/vive-libre-analysis-and-data.git/)

## Distribution

OSVR Vive Libre is available on the Arch User Repository. (AUR: [osvr-vive-libre-git](https://aur.archlinux.org/packages/osvr-vive-libre-git)).

## Build

We use CMake

    $ cmake .
    $ make

## Usage

### vivectl

A tool to dump Vive raw sensor and config data, and send commands to the device.
	
	$ vivectl -h

### OSVR Server

You can run the OSVR Server with the provided config files or you can copy the file from `/usr/share/osvrcore/sample-configs/` and edit it for your needs, like monitor settings.

Starting the OSVR server

	$ osvr_server osvr_server_config.vive_libre.sample.json

Now you can see if the tacking works with the OSVR-Tracker-Viewer. (AUR: osvr-tracker-viewer-git)

	$ OSVRTrackerView

You can also check out the OSVR-RenderManager demos. (AUR: osvr-rendermanager-git)

	$ RenderManagerOpenGLExample

## License

Vive Libre is licensed under the LGPLv3+.


