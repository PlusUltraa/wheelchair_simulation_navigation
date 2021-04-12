# wheelchair simulation navigation
* ROS version: Melodic
* To be used in dissertation
* Some maps are included

## Dependencies

* [Intellwheels Navigation](https://github.com/siferati/intellwheels_nav)
* [LaMa ROS - Alternative Localization and Mapping for ROS](https://github.com/iris-ua/iris_lama_ros/)
* [OrientationFilter](https://github.com/PlusUltraa/orientationFilter)

## Build from source

* To run the simulation, on your catkin workspace (catkin_ws) do :
    ```bash
    $ catkin build
    $ source ~/catkin_ws/devel/setup.bash
    ```

* Run simulator :
    ```bash
    $ roslaunch wheelchair_simulation_navegation gazebo_simulator.launch world:=LVL_Two.world
    ```

* Run navigator :
    ```bash
    $ roslaunch wheelchair_simulation_navegation navegation.launch planner:=teb world:=LVL_Two.yaml
    ```

* Run a series of know problems (1 - Corners, 2 - Doorways, 3 - Tight corridor) :
    ```bash
    $ roslaunch wheelchair_simulation_navegation issues_simulator.launch problem:=1
    ```

You can use other worlds that all ready exists in the [worlds](https://github.com/PlusUltraa/wheelchair_simulation_navigation/tree/master/worlds) folder. But you must be careful to also change the map that is being used in the navigation. The maps are located in the [maps](https://github.com/PlusUltraa/wheelchair_simulation_navigation/tree/master/maps) folder

## Test on the problems (TEB)

|Problem|No alterations|Solution 1|
|:--:|:--:|:--:|
| Corners         |Don't Solve| |
| Doorways        |Don't Solve| |
| Tight corridors |Don't Solve|Solve|

### Solution 1
It's the implementation of the orientationFilter in the global planner.
It consists of calculate the surrounding area of the wheelchair and, depending on the value of the area, chooses witch orientation filter to use (between orientation forward and interpolation).

To use this solution the parameter "global_plan_overwrite_orientation" of the local planner must be false. This allows the global planner to dictate the direction the wheelchair must go instead of the local planner.
