# for future use - to support multiple Gazebo versions
export GZ_VERSION=harmonic

# ensure the model and world files are found
export GZ_SIM_RESOURCE_PATH=\
$GZ_SIM_RESOURCE_PATH:\
$HOME/Documents/M2/Guerledan/interceptor_ws/src/asv_wave_sim/gz-waves-models/models:\
$HOME/Documents/M2/Guerledan/interceptor_ws/src/asv_wave_sim/gz-waves-models/world_models:\
$HOME/Documents/M2/Guerledan/interceptor_ws/src/asv_wave_sim/gz-waves-models/worlds:\
HOME/Documents/M2/Guerledan/interceptor_ws/src/gz_sim_one/worlds:\
$HOME/Documents/M2/Guerledan/interceptor_ws/src/gz_sim_one/models:\
$HOME/Documents/M2/Guerledan/interceptor_ws/src/gz_sim_one


# ensure the system plugins are found
export GZ_SIM_SYSTEM_PLUGIN_PATH=\
$GZ_SIM_SYSTEM_PLUGIN_PATH:\
$HOME/Documents/M2/Guerledan/interceptor_ws/install/lib

# ensure the gui plugin is found
export GZ_GUI_PLUGIN_PATH=\
$GZ_GUI_PLUGIN_PATH:\
$HOME/Documents/M2/Guerledan/interceptor_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control/build

export LD_LIBRARY_PATH=$HOME/Documents/M2/Guerledan/interceptor_ws/install/lib:$LD_LIBRARY_PATH
