for ct in 1. 0.9 0.8 0.7 0.6 0.5 0.4 0.3
do
    roslaunch ant_model_config ant_model_gazebo.launch cycle_time:=$ct
done
