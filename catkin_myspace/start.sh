#启动roscore
#gnome-terminal -t "roscore" -x bash -c "roscore;exec bash;"
#sleep 2s
#数据预处理节点启动
gnome-terminal -t "node1" -x bash -c "source devel/setup.bash;rosrun multisensor_localization data_pretreat_node;exec bash;"
#前端里程计节点启动
gnome-terminal -t "node2" -x bash -c "source devel/setup.bash;rosrun multisensor_localization front_end_node;exec bash;"
