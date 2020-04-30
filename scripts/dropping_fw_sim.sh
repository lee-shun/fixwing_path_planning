#本脚本启动的是toudanji

#zsh
gnome-terminal --window  -e 'zsh -c "cd ~/catkin_ws; catkin_make; exec zsh"' \
--tab -e 'zsh -c "sleep 3; source ~/.zshrc&&roslaunch px4 mavros_posix_sitl.launch;  exec zsh"' \
--tab -e 'zsh -c "sleep 4; rosrun fixwing_path_planning dropping_fw; exec zsh"' \

