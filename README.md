# oculus_sonar

ROS package to communicate with the oculus sonar, and to convert the sonar data to a ros message of type sensor_msgs/Image

### Dependencies
- std_msgs
- sensor_msgs
- cv_bridge
- dynamic_reconfigure

### Building 
- Create catkin workspace
```bash
#create new workspace
mkdir -p ~/oculus_sonar_ws/src

# change to the src directory in your Catkin workspace
cd ~/oculus_sonar_ws/src
```

- Clone oculus_sonar package
```bash
git clone https://github.com/mahmoud-a-ali/oculus_m750d.git
```
- Change to the main Catkin workspace
```bash
 cd ..
```
- Build the workspace (using catkin_tools)
```bash
 catkin build
```
- Source workspace
```bash
source devel/setup.bash
```

### Running
- Run the ros wrapper node using the launch file
```bash
roslaunch oculus_ros oculus_node.launch [bag_name] [record_bag]
```

### Parameters
 - #### Sonar parameters: check the [oculus_m750d][] datasheet 
    -  masterMode : frequency mode (low or high)
    -  gamma : Gamma correction
    -  range_m : sonar range in meter
    -  gain : Gain in percentage
    -  vOfSound : velocity of sound in water
    -  salinity : salinity of water 
    -  pingRate : sonar ping rate 

- #### OpenCV parameters
    - clh_tile_size : createCLAHE's tile grid size
    - clh_clp_lmt : createCLAHE's clip limit
    - cny_min_thrshld : Canny's min threshold
    - cny_max_thrshld : Canny's max threshold
    - cny_l2g : Canny's L2gradient
    - mrphlgy_krnl : morphologyEx's kernel
    - Mrphlgcl_sel : morphologyEx's type

- record_bag : true/false option to automatically record ros bage 
- bag_name : only valuable if record_bag value is true 

### sample output
![](./sonar_image.png)
![](./processed_image.png)

### Limitation
the final output is not a real 3D point coud as the sonar gives only a 2D information about the environment: azmith angle and distance to sensor, for more information check the  [oculus_m750d][] datasheet. The final output mimic the output of a 2d laser scanner

[oculus_m750d]: https://www.blueprintsubsea.com/pages/product.php?PN=BP01032

