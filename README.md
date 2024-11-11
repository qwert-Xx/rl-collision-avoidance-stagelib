#  Towards Optimally Decentralized Multi-Robot Collision Avoidance via Deep Reinforcement Learning 的部分实现

只实现了24辆小车随机初始点和随机目标点之间的无碰撞时间最优运动

环境需求:
* pytorch 
* cuda 
* pybind11
* stage

# 已测试环境:
* pytorch 2.5.1+cu124
* cuda 12.4
* wsl Ubuntu 22.04.5 LTS 
* 5.15.153.1-microsoft-standard-WSL2
* stage 4.3.0
* Python 3.10.12
* CPU:12400F
* GPU:3070 8G

# 快速部署Stage:
```bash
    #假设你已经安装好了pytorch,cuda,和pybind11
    #首先需要安装stage
    sudo apt update
    sudo apt install build-essential
    sudo apt install git cmake g++ libjpeg8-dev libpng-dev libglu1-mesa-dev libltdl-dev libfltk1.1-dev
    git clone https://github.com/rtv/Stage.git
    cd Stage
    mkdir build
    cd build
    cmake ../
    make
    sudo make install
    #还需要重新配置一下ld
    sudo ldconfig
    #最后检查一下stage是否正常安装上了
    stage
    #如果最后输出
    # Stage 4.3.0
    #
    # [Stage: done]
    #则安装成功
    #检查stagelib是否安装成功
    find /usr/local/ -name "stage.hh"
    #若输出
    #/usr/local/include/Stage-4.3/stage.hh
    #则安装成功
```
# 部署本项目
```bash
    git clone 项目地址
    cd 项目目录
    cd StageLibCpp/
    ./build.sh
    cd ..
    cd RLPy
    #开始测试
    python3 startTest.py

