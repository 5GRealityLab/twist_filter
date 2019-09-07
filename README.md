# Dynamic Twist Filter

This package is designed to work with the ROS framework to filter twist messages via ROS topics. There are two layers of filtering: individual signal filtering and over atwist filtering. The user can dynamically adjust the overall twist filter parameters like maximum acceleration and maximum velocity for both the linear and angular components of the twist. There are three kinds of filters available: a moving average filter, a more generic FIR filter, and an IIR filter.

## Build Instructions

1. Clone the package to your catkin workspace and run `catkin_make`.
2. Source your workspace.

## Run Instructions

Run the filters with their respective launch files:

``` bash
$ roslaunch twist_filter avg_filter.launch      // Moving average filter

$ roslaunch twist_filter fir_filter.launch      // FIR filter

$ roslaunch twist_filter iir_filter.launch      // IIR filter
```

You can remap the input and output filter topics as well:

``` bash
$ roslaunch twist_filter avg_filter.launch input:=cmd_in output:=cmd_out
```

You can also specify which motion configuration profile you would like to use:

``` bash
$ roslaunch twist_filter fir_filter.launch config:=fir_config.yaml
```

*NOTE: Motion config profiles must be `.yaml` files that are saved in the `/config` directory of the package.*

### Reconfiguration

You can update the motion profile values in real time by publishing to the `/filter_config` topic:

``` bash
$ rostopic pub /filter_config twist_filter/FilterConfig "linear_vel_max: 0.7
> linear_acc_max: 1.0
> angular_vel_max: 0.7
> angular_acc_max: 1.0"
```