# wheelchair simulation navigation
* ROS version: Melodic
* To used in dissertation
* Some maps are included

## Dependencies

* [Intellwheels Navigation](https://github.com/siferati/intellwheels_nav)
* [LaMa ROS - Alternative Localization and Mapping for ROS](https://github.com/iris-ua/iris_lama_ros/)

## Build from source

* To run the simulation, on your catkin workspace (catkin_ws) do:
    ```bash
    $ catkin build
    $ source ~/catkin_ws/devel/setup.bash
    ```

* Run simulator:
    ```bash
    $ roslaunch wheelchair_simulation_navegation gazebo_simulator.launch world:=LVL_Two.world
    ```

* Run navigator:
    ```bash
    $ roslaunch wheelchair_simulation_navegation navegation.launch planner:=teb world:=LVL_Two.yaml
    ```
You can use other worlds that all ready exists in the [worlds] folder. But you must be careful to also change the map that is being used in the navigation. The maps are located in the [maps] folder
