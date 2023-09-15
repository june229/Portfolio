# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

#alias foxy='source /opt/ros/foxy/setup.bash; source ~/robot_ws/install/local_setup.bash; source ~/microros_ws/install/local_setup.bash; source ~/drone_ws/install/local_setup.bash; source ~/delivery_ws/install/local_setup.bash; source ~/second_ros2_ws/install/local_setup.bash; echo \"foxy is Activated!\"'
#alias noetic='source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/local_setup.bash; echo \"Noetic is Activated!\"'

#source /opt/ros/foxy/setup.bash
#source ~/robot_ws/install/local_setup.bash

#source ~/turtlebot3_ws/install/local_setup.bash
#source ~/ros2_ws/install/local_setup.bash
#source ~/ws_moveit2/install/setup.bash

#source ~/microros_ws/install/local_setup.bash
#source ~/drone_ws/install/local_setup.bash
#source ~/delivery_ws/install/local_setup.bash
#source ~/turtlebot3_ws/install/local_setup.bash

#source ~/yolo/ROS-Tutorials/ROS2_Yolov5_Docker/Out_Docker/install/setup.bash

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/vcstool-completion/vcs.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/robot_ws

#export ROS_DOMAIN_ID=5
#export ROS_NAMESPACE=robot1

#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_connext_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_gurumdds_cpp

# export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})'
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1

alias cw='cd ~/robot_ws'
alias cs='cd ~/robot_ws/src'
alias ccd='colcon_cd'

alias cb='cd ~/robot_ws && colcon build --symlink-install'
alias cbs='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias cbu='colcon build --symlink-install --packages-up-to'
alias ct='colcon test'
alias ctp='colcon test --packages-select'
alias ctr='colcon test-result'
alias sb='source ~/.bashrc'
alias rd='rosdep install -i --from-path src --rosdistro foxy -y'
alias gb='gedit ~/.bashrc'

alias rt='ros2 topic list'
alias re='ros2 topic echo'
alias rn='ros2 node list'

alias killgazebo='killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient'

alias af='ament_flake8'
alias ac='ament_cpplint'

alias testpub='ros2 run demo_nodes_cpp talker'
alias testsub='ros2 run demo_nodes_cpp listener'
alias testpubimg='ros2 run image_tools cam2image'
alias testsubimg='ros2 run image_tools showimage'

alias 2='source /opt/ros/foxy/setup.bash && source ~/robot_ws/install/local_setup.bash && source ~/microros_ws/install/local_setup.bash && source ~/drone_ws/install/local_setup.bash && source ~/delivery_ws/install/local_setup.bash && source ~/turtlebot3_ws/install/local_setup.bash && source ~/yolo/ROS-Tutorials/ROS2_Yolov5_Docker/Out_Docker/install/setup.bash && source ~/pointcloud_lecture/install/setup.bash'
alias 1='source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash' #&& source ~/avoidance_ws/devel/setup.bash'

#export TURTLEBOT3_MODEL=burger
#export TURTLEBOT3_MODEL=waffle
#export TURTLEBOT3_MODEL=waffle_pi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export ROS_MASTER_URI=http://localhost:11311
#export ROS_IP=192.168.0.3
#export ROS_NAMESPACE=/my/namespace
export ROS_DOMAIN_ID=2
export ROS_MASTER_URI=http://192.168.0.13:11311
export ROS_HOSTNAME=192.168.0.13
#export PATH=$PATH:/usr/bin/python3
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/jun/robot_ws/src/basic_mobile_robot/models/

export LDS_MODEL=LDS-02

#. ~/Firmware/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/avoidance/avoidance/sim/models:~/catkin_ws/src/avoidance/avoidance/sim/worlds
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Firmware
