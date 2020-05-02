#本脚本启动的是zhenchaji

#zsh
gnome-terminal --window  -e 'zsh -c "cd ~/catkin_ws; catkin_make; exec zsh"' \
--tab -e 'zsh -c "sleep 3; cd /home/ls/src/Firmwarelee1.9.2; make px4_sitl gazebo_plane; exec zsh"' \
--tab -e 'zsh -c "sleep 5; source ~/.zshrc&&roslaunch mavros px4.launch  fcu_url:="udp://:14540@127.0.0.1:14557"; exec zsh"' \
--tab -e 'zsh -c "sleep 8; rosrun fixwing_path_planning zhenchaji_fw; exec zsh"' \

