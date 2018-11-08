#!/bin/bash

#############################################################################
# Author: www.corvin.cn
#############################################################################
# Description:该脚本只需要在下载代码后执行一次即可，注意该脚本位置需要在代码
#   的根目录下，不要随便移动位置，否则导致无法正常配置和编译。该脚本注意功能
#   就是安装缺少运行语音程序的软件包，编译整个代码空间，配置环境变量，可以说
#   是一键执行脚本，减少大家烦琐的配置编译步骤。
#
#############################################################################
# History:
#    20171205-init bash script
#    20171213-在安装pocketSphinx时需要注意先安装base包再安装其他的,更新
#       wakeup的提示音从wav格式到mp3,最后增加source devel/setup.bash,直接
#       source ~/.bashrc没有反应。
#    20180102-更新注释信息,先更新系统软件源列表为阿里源后再update，方便安装
#       软件更快;
#
#############################################################################

CURRENT_PATH=$(dirname $(readlink -f "$0"))
WORKSPACE_PATH=${CURRENT_PATH%scripts}
green="\e[32;1m"
normal="\e[0m"

function updateSourceList()
{
    SOURCE_FILE=/etc/apt/sources.list
    sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
    sudo sh -c "echo #deb cdrom:[Ubuntu 16.04 LTS _Xenial Xerus_ - Release amd64 (20160420.1)]/ xenial main restricted > ${SOURCE_FILE}"
    sudo sh -c "echo deb-src http://archive.ubuntu.com/ubuntu xenial main restricted #Added by software-properties >> ${SOURCE_FILE}"
    sudo sh -c "echo deb http://mirrors.aliyun.com/ubuntu/ xenial main restricted >> ${SOURCE_FILE}"
    sudo sh -c "echo deb-src http://mirrors.aliyun.com/ubuntu/ xenial main restricted multiverse universe #Added by software-properties >> ${SOURCE_FILE}"
    sudo sh -c "echo deb http://mirrors.aliyun.com/ubuntu/ xenial-updates main restricted >> ${SOURCE_FILE}"
    sudo sh -c "echo deb-src http://mirrors.aliyun.com/ubuntu/ xenial-updates main restricted multiverse universe #Added by software-properties >> ${SOURCE_FILE}"
    sudo sh -c "echo deb http://mirrors.aliyun.com/ubuntu/ xenial universe >> ${SOURCE_FILE}"
    sudo sh -c "echo deb http://mirrors.aliyun.com/ubuntu/ xenial-updates universe >> ${SOURCE_FILE}"
    sudo sh -c "echo deb http://mirrors.aliyun.com/ubuntu/ xenial multiverse >> ${SOURCE_FILE}"
    sudo sh -c "echo deb http://mirrors.aliyun.com/ubuntu/ xenial-updates multiverse >> ${SOURCE_FILE}"
    sudo sh -c "echo deb http://mirrors.aliyun.com/ubuntu/ xenial-backports main restricted universe multiverse >> ${SOURCE_FILE}"
    sudo sh -c "echo deb-src http://mirrors.aliyun.com/ubuntu/ xenial-backports main restricted universe multiverse #Added by software-properties >>${SOURCE_FILE}"
    sudo sh -c "echo deb http://archive.canonical.com/ubuntu xenial partner >> ${SOURCE_FILE}"
    sudo sh -c "echo deb-src http://archive.canonical.com/ubuntu xenial partner >> ${SOURCE_FILE}"
    sudo sh -c "echo deb http://mirrors.aliyun.com/ubuntu/ xenial-security main restricted >> ${SOURCE_FILE}"
    sudo sh -c "echo deb-src http://mirrors.aliyun.com/ubuntu/ xenial-security main restricted multiverse universe #Added by software-properties >> ${SOURCE_FILE}"
    sudo sh -c "echo deb http://mirrors.aliyun.com/ubuntu/ xenial-security universe >>${SOURCE_FILE}"
    sudo sh -c "echo deb http://mirrors.aliyun.com/ubuntu/ xenial-security multiverse >> ${SOURCE_FILE}"
}

echo -e "${green}>>> WORKSPACE_PATH=${WORKSPACE_PATH} <<< ${normal}"

echo -e "\n${green} 0x00: Update packages source list ${normal}"
updateSourceList
sudo apt-get update


echo -e "\n${green} 0x01: Install sox pkg and lib for iflytek_tts ${normal}"
sudo apt-get install -y sox libsox-fmt-all 
cd ${WORKSPACE_PATH}
sudo cp src/iflytek_tts/lib/libmsc_tts.so /usr/lib/


echo -e "\n${green} 0x02: Install pkg for iflytek_asr ${normal}"
cd ${WORKSPACE_PATH}
sudo cp src/iflytek_asr/lib/libmsc.so /usr/lib/
sudo apt-get install -y libasound2 libasound2-dev


echo -e "\n${green} 0x03: Install pkgs for tuling_nlu ${normal}"
sudo apt-get install -y libcurl3 libcurl4-openssl-dev libjsoncpp1 libjsoncpp-dev


echo -e "\n${green} 0x04: Instal requests and vlc pkg for baidu_speech ${normal}"
#first,config pip install source
cd ~
sudo rm -rf .pip
mkdir .pip
touch ~/.pip/pip.conf
echo "[global]">~/.pip/pip.conf
echo "timeout = 600">>~/.pip/pip.conf
echo "index-url = https://pypi.tuna.tsinghua.edu.cn/simple">>~/.pip/pip.conf
echo "[list]">>~/.pip/pip.conf
echo "format = columns">>~/.pip/pip.conf
#seconde,install pkg
sudo apt-get install -y python-pip python-dev python-pyaudio build-essential
sudo -H pip install --upgrade pip
cd ${WORKSPACE_PATH}src/baidu_speech/requests/
sudo -H pip install .
cd ${WORKSPACE_PATH}src/baidu_speech/vlc-python/
sudo cp generated/vlc.py /usr/lib/python2.7/  
#third,for baidu asr
sudo apt-get install -y gstreamer0.10-*
sudo apt-get install -y multimedia-jack
jack_control start


echo -e "\n${green} 0x05: Config respeaker mic array pkg ${normal}"
#first,install swig by source code
sudo apt-get install -y byacc
cd ${WORKSPACE_PATH}src/respeaker/swig-3.0.12/
./configure
make
sudo make install
make clean
#seconde,install necessary python pkg
sudo apt-get install -y libpulse-dev
sudo -H pip install pocketsphinx webrtcvad
sudo -H pip install respeaker --upgrade
sudo -H pip install pyusb


echo -e "\n${green} 0x06: Setup pocketsphinx for offline wakeup${normal}"
sudo apt-get install -y ros-kinetic-audio-common
#copy wakeup notice wav to Music folder
cd ${WORKSPACE_PATH}
cp ${WORKSPACE_PATH}src/sphinx_wakeup/wakeup_alert_sound/wakeup_alert.mp3 ~/Music/
#copy language model to system env path
LM_PATH=/usr/share/pocketsphinx/model/lm/zh_CN/
sudo mkdir -p ${LM_PATH}
sudo cp ${WORKSPACE_PATH}src/pocketsphinx/model/*.DMP ${LM_PATH}
sudo cp ${WORKSPACE_PATH}src/pocketsphinx/model/*.dic ${LM_PATH}
HMM_PATH=/usr/share/pocketsphinx/model/hmm/zh/
sudo mkdir -p ${HMM_PATH}
sudo cp -rf ${WORKSPACE_PATH}src/pocketsphinx/model/zh_broadcastnews_ptm256_8000 ${HMM_PATH}
#install neceesary apt-get pkg and debian pkg
cd ${WORKSPACE_PATH}src/pocketsphinx/debian/
sudo apt-get install -y libgstreamer-plugins-base0.10-0
sudo dpkg -i gstreamer0.10-pocketsphinx_0.8-5_amd64.deb
sudo dpkg -i libsphinxbase1_0.8-6_amd64.deb
sudo dpkg -i libpocketsphinx1_0.8-5_amd64.deb
sudo apt-get install -y python-gst0.10


echo -e "\n${green} 0x07: Autoremove useless pkgs for cleaning disk ${normal}"
sudo apt-get autoremove -y


echo -e "\n${green} 0x08: Compile ros_voice_system workspace ${normal}"
cd ${WORKSPACE_PATH}
catkin_make

echo -e "\n${green} 0x09: Setup ros_voice_system env to .bashrc ${normal}"
cd ${WORKSPACE_PATH}
source devel/setup.bash
echo "#config ros_voice_system env">>~/.bashrc
echo "source ${WORKSPACE_PATH}devel/setup.bash">>~/.bashrc

echo -e "${green}-------------------------------------------------${normal}"
echo -e "${green} Congratulation! All be completed successfully! ${normal}"
echo -e "${green}   Now will auto startup ros_voice_system ${normal}"
echo -e "\n"
echo -e "${green} roslaunch voice_bringup voice_bringup.launch ${normal}"
echo -e "${green}-------------------------------------------------${normal}"
roslaunch voice_bringup voice_bringup.launch


