-###################### historical pipeline notes ######################-
roslaunch lste_core lab_with_pro3.launch \
  world_name:=/worlds/topo2.0_explore_mode/place2.world
  
rosrun lste_core lste_task_node.py _json_path:=$LSTE_WS/model/Data_exchange/vlm_prompt/lab/yellow_cup.json _task_id:=yellow_cup

conda activate minicpm
cd ~/lste_ws/model/MiniCPM/test/
bash start.sh 

conda activate minicpm
rosparam set /lste_prompt_node/vllm_stop_command "pkill -f 'vllm serve'"
rosrun lste_core lste_prompt_node.py \
  _vllm_base_url:=http://localhost:8000/v1 \
  _vllm_model_name:=$LSTE_WS/model/MiniCPM/OpenBMB/MiniCPM4-0___5B

conda activate dino
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7
rosrun lste_core lste_det_node.py

rosrun lste_core lste_score_node.py \
    _w_target:=0.2 _w_env:=0.3 _w_ctx:=0.5 \
    _lambda_neg:=0.7 _pos_midpoint:=0.25 _pos_steepness:=6 \
    _neg_midpoint:=0.15 _neg_steepness:=12

rosrun lste_core lste_state_node.py


roslaunch lste_core lste_det_vis.launch








-######################- tools -################################-
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel  #操控车来移动（键盘操控）
rostopic echo /lste/task  #能看到发布的task
rostopic echo /lste/prompts  #能看到发布的prompts

conda activate vsgp
rosrun vanish_point_detection vanish_point_detection.py






-####################- access_topo -############################-
roslaunch lste_core lab_with_pro3.launch \
  world_name:=worlds/room.world


conda activate vsgp
roslaunch lste_topo_access gp_frontier.launch

roslaunch lste_oc_srfc oc_srfc_proj.launch

rviz -d $LSTE_WS/src/lste_topo_access/launch/gp_frontier.rviz
