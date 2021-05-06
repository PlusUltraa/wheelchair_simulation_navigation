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
