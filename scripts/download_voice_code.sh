#!/bin/bash

##############################################################
# Author: www.corvin.cn
##############################################################
# Description: 该脚本只有在需要分发本代码时发送给其他人执行
#  即可，其他人执行该脚本即可自动的下载语音相关所有源码。
#  而且该脚本还可以在下载完成源码后，然后自动切换分支，
#  更新当前分支代码与服务器代码同步,然后调用setup_config.sh 
#  脚本来自动配置编译环境，安装缺失软件包，编译整个源码。
#
##############################################################
# History:
#    20171205 - init this bash file
#    20171212 - 当git clone前首先来安装git，防止没有下载
#      软件就开始git clone下载代码;
#    20180102-增加可以下载树莓派分支代码功能，增加提示信息，让
#      用户根据自己设备要求来选择相应分支代码;
#
##############################################################

green="\e[32;1m"
red="\e[31m"
blue="\e[34m"
normal="\e[0m"
FILE_PATH=$(dirname $(readlink -f "$0"))
SELECT_OK="false"

echo -e "${green}***************************************************************************** ${normal}\n"
echo -e "${green}********** Welcome Download ROS Chinese Voice System Source Code ! ********** ${normal}\n"
echo -e "${green}**********                      www.corvin.cn                      ********** ${normal}\n"
echo -e "${green}***************************************************************************** ${normal}\n"
echo -e "${green}In Server All Git Branches List Blow:${normal}"
echo -e "${green}1: ubuntu14.04_x64_indigo${normal}"
echo -e "${green}2: ubuntu16.04_x64_kinetic${normal}"
echo -e "${green}3: raspberry_jessie_indigo${normal}"
echo -e "${green}4: raspberry_ubuntuMate16.04_kinetic${normal}"

while [ $SELECT_OK == "false" ]
do
read -p "Please select download branch code based on your device and ros version: " index
case $index in
    1)GIT_BRANCH="ubuntu14.04_x64_indigo"
      SELECT_OK="true";;
    2)GIT_BRANCH="ubuntu16.04_x64_kinetic"
      SELECT_OK="true";;
    3)GIT_BRANCH="raspberry_jessie_indigo"
      SELECT_OK="true";;
    4)GIT_BRANCH="raspberry_ubuntuMate16.04_kinetic"
      SELECT_OK="true";;
    *) echo -e "${red}Selected index error! ${normal}";;
esac
done

echo -e "${green} Now will try git clone ${GIT_BRANCH} branch code...${normal}"

#check if exists setup_config.sh or ros_voice_system folder
echo -e "${green} 1: Check if need download voice source code ${normal}"
if [ ! -f "setup_config.sh" ] && [ ! -d "ros_voice_system" ]
then
  sudo apt-get install -y git
  if [ $? -eq 0 ]
  then
      git clone -b ${GIT_BRANCH} http://corvin.cn:8081/gerrit/ros_voice_system
      if [ $? -ne 0 ]
      then
          echo -e "${red}ERROR! Git clone source code unknown error, please retry later...${normal}"
          exit 1
      fi
  else
      echo -e "${red}Can't install git pkg, please retry install later...${normal}" 
      exit 2
  fi
else
  echo -e "${red}Already download ros_voice_system source code, now will exit...${normal}"
  exit 3
fi

#git clone code over,now will auto invoke bash to compile souce code
echo -e "${green} 2: Will auto config and compile ros_voice_system code ${normal}"
cd ${FILE_PATH}/ros_voice_system/scripts/
chmod +x setup_config.sh
. ./setup_config.sh

exit 0

