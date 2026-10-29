# Official ROS driver for [Ensenso](http://www.ensenso.com) stereo cameras [![Build&Test](https://github.com/ensenso/ros_driver/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/ensenso/ros_driver/actions/workflows/build-and-test.yml)

## Documentation

This package is ready to be used with ROS2. Take a look at the [getting started guide](docs/getting-started.md) for an introduction to the package.

If you want to use the package with ROS1, see our [ROS1 documentation](docs/ros1.md) on how to build for ROS1 and for further information.

## Remarks

- For using this package, you need to have the [Ensenso SDK](https://www.ensenso.com/support/sdk-download) installed.
- Ensenso B-Series cameras require at least version 2.1.3
- Ensenso C-Series cameras require at least version 2.1.1
- Ensenso S- and XR-Series cameras require at least version 1.7.0.
- Version 1.7.0 and newer requires at least Ensenso SDK 3.0.
- Version 1.6.3 and older requires at least Ensenso SDK 2.0. Older versions are not supported.
- All inputs and outputs of this package are in meters and seconds, as the convention for ROS requires. This is different from the NxLib, which uses millimeters and milliseconds.

## Report a Bug

For general problems with your camera or the Ensenso SDK, please visit the [Ensenso community forum](https://community.ensenso.com) or contact the [IDS support](https://en.ids-imaging.com/support.html).

Bugs of the ROS driver can be reported directly in the [issue tracker](https://github.com/ensenso/ros_driver/issues).

## Acknowledgements

<img src="https://raw.githubusercontent.com/ensenso/ros_driver/master/media/rosin.png" alt="ROSIN" title="ROSIN" height="70">&nbsp;<img src="https://raw.githubusercontent.com/ensenso/ros_driver/master/media/eu.png" alt="EU" title="EU" height="70">

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: http://rosin-project.eu/

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement no. 732287.
