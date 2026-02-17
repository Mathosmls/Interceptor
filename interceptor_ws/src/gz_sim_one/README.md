# GAZEBO SIMULATION FOR THE MONODROME1800

All the sensors of the real robot are implemented, except the thermal camera and the radar. However, there is still some problems with the lidar (it detects the waters).

There is 2 boat on the simulation, one target and one interceptor. I made one sdf file for each boat, it's not optimal at all but gazebo is a pain if you start to do nested models. Another solution is just to write python (or xacro) script to create these 2 different files.

Each boat can be controlled in thanks to its 2 motors. You can have a look at the ctrl_pid pkg if you want an example.

## Launch

First you have to tell to gazebo where to find your models. You can run (or add to your bashrc) the setup_gz.zsh (I use zsh, just change the extension name). You can find this file at the ws root. **Make sure to change the absolute path included in this file**

To run the simulation, you still have to launch the next cmd in the **worlds** folder of the **gz_sim_one** pkg :  
```bash
gz sim interceptor_sim_one.sdf
```

