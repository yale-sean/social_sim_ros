# Social Sim Teleop

For teleoperation of a robot in the simulator.

Tested with a Sony PS3 controller.

# Setup

Plug it into your machine using the USB cable.

Symlink the device.

Note: this is already mapped to the Docker container, so this step is not necessary if you are using Docker

    sudo ln -s /dev/input/js0 /dev/ps3joy

# Usage

Launch the nodes:

    roslaunch --wait social_sim_teleop ps3_teleop.launch

Press and hold the left trigger (1) to start sending commands.


## Notes

`velocity_smoother` is not used
