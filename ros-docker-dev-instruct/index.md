# 在松灵 LIMO 开发套件上使用 Docker 进行 ROS2 远程开发




截至目前，Jetson Nano 平台官方仍不提供 Ubuntu 20.04 固件，所以使用 Jetson Nano 平台开发 ROS2 存在巨大的困难，但是好在 Docker 提供的容器技术，**让我们能非常方便的虚拟出一个 Ubuntu 20.04 环境进行 ROS2 开发并通过云端快速分享自己的开发环境**，本文我将以松灵 LIMO 机器人套件为例向大家演示如何使用 Docker 进行 ROS2 远程开发以及这种开发模式的优越性。

此外，在 ROS 开发过程中（尤其是在多人团队协同中）你一定遇到过这样的问题：需要快速（在不同架构机器上）部署相同开发环境、别人电脑上可以编译但到了自己这里就会报错、无法复现别人的 Bug、经常会出现缺少支持库或者支持库冲突的情况，**尤其是一台电脑上需要做多种任务的情况**（比如 深度学习、QT 开发、ROS1、ROS2）则会更糟糕…… 这个时候我们就有必要使用 Docker 进行开发

<!--more-->

本文分为两部分，`1-6`节讲解 使用 Docker+VSCode 开发方式，`7` 节将带领大家在 LIMO 平台进行实战

无论您之前使用什么样的方式进行开发，笔者在这里都推荐您尝试 Docker + VSCode，虽然这可能需要两三个下午的时间进行学习，但请相信这一切都是值得的！

> 如果您不会使用 docker 或者没有尝试过类似的操作，请您先阅读 《Docker 配合 VSC 开发最佳实践》(https://anthonysun256.github.io/docker-with-vsc_best-practice/)

## 1. 镜像选取

Docker 官方镜像服务器上有很多的 ROS 开发镜像，这里我们可以有如下选择（这几个镜像也是笔者常用的）：

- ROS 官方镜像：https://hub.docker.com/_/ros

![image-20220506111110220](images/1.png)

![image-20220506112024537](images/2.png)

内含所有 ROS 发行版的基础开发镜像，而且支持多种架构，非常适合在多种架构的机器上进行部署。

- althack 的开发镜像 https://hub.docker.com/u/althack

![image-20220506114202505](images/3.png)

![4](images/4.png)98

> althack 提供了 VSCode 的开发工作环境模板：https://github.com/athackst/vscode_ros2_workspace
>
> 虽然模板是 ROS2 但是只要简单修改 Dockerfile 中引用的镜像名称即可改成 ROS1 环境
>
> Dockerfile 源码：https://github.com/athackst/dockerfiles

这个镜像优点是提供了 ROS-desktop-full 的开发环境，省去了自行安装的麻烦，非常适合学习使用。缺点是只支持 PC，如果其他环境需要自己使用 Dockerfile 编译。

## 2. 开发环境搭建

我们新建的工作空间目录格式如下：

```
.
├── .devcontainer # docker 配置文件
│   ├── devcontainer.json
│   └── Dockerfile
└── src # ROS 源码目录
```

VSCode 会自动读取 `.devcontainer` 中的配置文件。对于我个人学习而言我一直使用的是 https://hub.docker.com/u/althack 的镜像和配置，下面的配置文件也是改变于此，各位读者可以根据自己的需要进行修改：

**Dockerfile**：

```dockerfile
# 改编自 https://github.com/athackst/vscode_ros2_workspace
# 我使用的 ROS 学习镜像，根据您的喜好可以自行更改
FROM althack/ros:melodic-gazebo as gazebo 

# ** [可选] 取消注释安装其他组件 **
#
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# 为 ros 用户配置自动 source
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
```

如果需要 GPU 支持（需要安装 nvidia-docker [  https://anthonysun256.github.io/docker_with_nvidia/  ] ）（Jetson 平台不要用这个指导！！请使用 Jetpack 套件固件自带的 nvidia-docker2 或者直接进行安装 `sudo install nvidia-docker2` （使用 l4t2 的源）则在上述脚本之后追加如下指令：

```dockerfile
RUN apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Nvidia 运行时变量.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

ENV QT_X11_NO_MITSHM=1
```
**devcontainer.json**

```json
// 改编自 https://github.com/athackst/vscode_ros2_workspace
// See https://aka.ms/vscode-remote/devcontainer.json for format details.
{
    "dockerFile": "Dockerfile",
    "build": {
        "args": {
            "WORKSPACE": "${containerWorkspaceFolder}"
        }
    },
    "remoteUser": "ros",
    "runA虽然模板是 ROS2 但是只要简单修改 Dockerfile 中引用的镜像名称即可改成 ROS1 环境rgs": [
        "--network=host",
        "--cap-add=SYS_PTRACE",
        "--cap-add=SYS_RAWIO",
        "--privileged=true",
        "--security-opt=seccomp:unconfined",
        "--security-opt=apparmor:unconfined",
        "--volume=/tmp/.X11-unix:/tmp/.X11-unix",
        // "--gpus" "all", // 取消该注释使用 GPU 功能
        "--device=/dev/bus/usb:/dev/bus/usb", // 挂载外部硬件，比如摄像头，根据需要修改路径
    ],
    "containerEnv": {
        "DISPLAY": "${localEnv:DISPLAY}"
    },
    // Set *default* container specific settings.json values on container create.
    "settings": {
        "terminal.integrated.profiles.linux": {
            "bash": {
                "path": "bash"
            },
        },
        "terminal.integrated.defaultProfile.linux": "bash"
    },
    // 推荐安装的插件
    "extensions": [
        "dotjoshjohnson.xml",
        "zachflower.uncrustify",
        "ms-azuretools.vscode-docker",
        "ms-iot.vscode-ros",
        "ms-python.python",
        "ms-vscode.cpptools",
        "redhat.vscode-yaml",
        "smilerobotics.urdf",
        "streetsidesoftware.code-spell-checker",
        "twxs.cmake",
        "yzhang.markdown-all-in-one"
    ]
}
```

## 3. 启动开发环境

我们使用 VSCode 打开文件夹，并且 `Reopen in docker` 即可（不要自己只使用 Dockerfile 进行构建，因为在 devcontainer.json 中我们还挂载了一些目录，添加了一些参数）

![5](images/5.png)

![6](images/6.png)

> 根据网络状况不同初次启动这里可能需要等待 VSCode 安装一些组件，请耐心等待，下次启动即可极速启动
>
> 部分挂载的设备可能需要 root 权限才能访问，这一点需要注意

## 4. 查看图形界面

在我们的主机中，打开一个终端，运行 `xhost +` 

之后在我们 VSCode 中的命令行中随便打开一个图形界面，比如 rqt，可以看到 rqt 直接显示在了我们的屏幕上

![7](images/7.png)

> 对于 Rviz 和 Gazebo，如果您的电脑上没有显卡、未正确安装闭源显卡驱动或者没有正确开启 nvidia-docker 显卡功能那界面会很卡，这和用不用 Docker 没有关系

## 5.插件简介

笔者的 `devcontainer.json` 中会为大家安装一些实用插件，这部分我们简略了解一下

- ROS——这个插件可以帮助我们快速更新 VSCode 中有关 ROS include 的路径，而且提供了图形界面查看 ROS 话题、服务等

  ![8](images/8.png)

  按下 `shift+ctrl+p` 呼出快捷命令栏，输入 ros，可以看到自动更新选项

  ![10](images/10.png)

  - yaml xml xacro 语法助手

  这个可说的不多，能够帮助我们快速编写相应的文件，为其提供智能补全

  ![9](images/9.png)

## 6.修复本地包 include / import 错误

默认情况下 ROS 插件可能不会包含我们本地路径的包，这并不会导致编译错误，但是会导致我们无法使用智能提示。

> msg 类型的包需先编译生成对应的 c++或者python头文件（在 devel 或者 install 中寻找对应目录）

这时候我们需要手动添加路径到 `c_cpp_properties.json`（注意每条路径后需要有个逗号：

![11](images/11.png)

> 提示：我们**可以直接**在左侧资源管理器中右键文件夹在弹出的菜单中**复制文件夹的绝对路径**

python 同理：

![12](images/12.png)

至此我们就能愉快的进行开发了，而且您还能保存自己的开发环境、推送到云端或者整理成压缩包进行分享，让团队中的所有人都能在几分钟之内拥有一致的开发环境，最重要的是还不会搞坏电脑本身的环境！

## 7. 远程开发实战

本节我们将运用上面所讲的知识，在松灵机器人的 limo 套件上使用 docker 进行远程开发！

首先运行docker 环境（docker 在默认系统中已经安装，这里我们不使用图形界面，不需要开启显卡支持）

```shell
$ mkdir limo_foxy_dev && cd limo_foxy_dev
$ docker run --network=host  --cap-add=SYS_PTRACE --cap-add=SYS_RAWIO --security-opt=seccomp:unconfined --security-opt=apparmor:unconfined --volume=/tmp/.X11-unix:/tmp/.X11-unix --runtime=nvidia --device /dev/ttyTHS1:/dev/ttyTHS1 --device /dev/ttyUSB0:/dev/ydlidar --device /dev/bus/usb/:/dev/bus/usb -v $(pwd)/：workspace --name limo-foxy-dev  -w /workspace -id ros:foxy
$ docker exec -it limo-foxy-dev /bin/bash
```

这里我们挂载了底盘通信接口、激光雷达、USB 设备到我们的容器之中，以及将本地的 `limo_foxy_dev` 文件夹挂载到了容器中的 `/workspace` 之中并开启了一些相应的权限。

之后等待 docker 启动（第一次需要拉取远程镜像，速度稍慢，再次启动无需等待）进入之后可以看到如下界面：

![13](images/13.png)

接下来我们换源，提升下载速度：

```shell
# 更换系统源
$ sed -i "s/ports.ubuntu.com/mirrors.ustc.edu.cn/g" /etc/apt/sources.list
# 更换 ros2 源
$ sed -i "s/packages.ros.org/repo.huaweicloud.com/g" /etc/apt/sources.list.d/ros2-latest.list
$ apt update
```

设置一下自动 source

```shell
$ echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc
```

安装 `rosdepc` 提升 `rosdep` 下载速度（注意这俩不是一个东西，多了个 `c`）：

```shell
$ apt install -y python3-pip
$ pip install rosdepc
$ rosdepc init && rosdepc update
```

安装远程开发必备组件：

```shell
$ apt install openssh-server systemctl udev swig # 分别为 远程连接组件 服务控制组件 驱动控制组件 雷达驱动组件
$ service  ssh start
$ echo -e "Port 10022\nPermitRootLogin yes\nPermitEmptyPasswords yes" >> /etc/ssh/sshd_config.d/dev.conf
$ service ssh restart
```

> 重要！：容器重启后 ssh 会无法连接，请重新在容器中运行 ``service ssh start``

设定密码：

```shell
$ passwd # 之后输入新的密码（在这个过程中不会显示文字，输入完成按回车即可）
```

之后我们离开镜像，保证我们的小车和电脑在同一个局域网中，在本地电脑中打开 VSCode 使用 Remote ssh 插件连接镜像：

![14](images/14.png)

![15](images/15.png)

任选一个文件进行编辑：

![16](images/16.png)

`Host` 名称自定义，`HostName` 是 limo 在局域网中的 ip，`Port` 是 ssh-server 连接的端口，用户是 root

保存退出。

![17](images/17.png)

> 如果大家不想使用 root 登陆请注意自定义用户组的权限，不然可能无法访问硬件（如雷达等）

重新打开 `Connect to Host` 选项，选择我们刚刚保存的条目进行连接：

![18](images/18.png)

之后会弹出一个新的窗口，在那里输入我们之前设定的密码:

![19](images/19.png)

之后打开我们挂载的 `workspace` 文件夹，这里会再次要求我们输入密码：

![20](images/20.png)

选择信任：

![21](images/21.png)

这里我在远程安装了如下插件方便我的开发：

![22](images/22.png)

![23](images/23.png)





之后打开命令行（快捷键 `Shift+Ctrl+~`)下载 ydlidar 的驱动并编译安装（当前目录 `/workspace`）

```shell
$ git clone https://ghproxy.com/https://github.com/YDLIDAR/YDLidar-SDK.git # 使用了ghproxy.com 加速
$ mkdir -p YDLidar-SDK/build
$ cd YDLidar-SDK/build
$ cmake ..
$ make
$ make install # 安装 C/C++ 支持
$ cd ..
$ pip install . # 安装 py 支持
$ cd .. && rm -r YDLidar-SDK # 删除 SDK 源码防止影响之后构建
```

> 请注意命令中 `.` 与 `..` 的区别不要敲错了

下载 limo 的 ROS2 镜像（当前目录 `/workspace`）：

```shell
$ git clone --recursive https://ghproxy.com/https://github.com/agilexrobotics/limo_ros2.git src # 使用了ghproxy.com 加速
```

修改 `/workspace/src/ydlidar_ros2/src/ydlidar_ros2_driver_node.cpp` 中源码参数：

```c++
# 61 行
int optval = 115200;
# 76 行
optval = 3;
# 108 行
b_optvalue = true;
# 134 行
f_optvalue = 12.f;
# 143 行
f_optvalue = 8.f;
```

安装必要的支持库并编译（当前目录`/workspace`）：

```shell
$ rosdepc install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install # 使用符号链接节省空间，如果要删除源码，则使用 --merge-install
```

![25](images/25.png)

会发现有一些标准错误输出，是正常的，不需要管。

之后 `source` 安装并启动雷达节点

```shell
$ source install/setup.bash
$ ros2 launch limo_bringup limo_start.launch.py
```

然后在我们本地的电脑上打开我们本地的开发环境（也是在 Docker 中，根据上文所讲，使用的是 https://github.com/athackst/vscode_ros2_workspace 的模板）

在本地开发环境的终端中启动 rviz:

```shell
$ rviz2
```

然后订阅雷达节点，按照图示进行配置

![26](images/26.png)

![27](images/27.png)

即可看到 limo 远程发来的雷达信息：

![24](images/24.png)

至此，我们便掌握了机器人远程开发的基础操作！熟悉这种方法可以极大的提升我们工作效率，降低机器人开发复杂度。并且您也可以将刚才的开发环境提交到 Docker 的服务器之上或者输出成压缩包供他人使用，让我们能够快速大批量的部署完全相同的开发环境！

最后，感谢您的阅读！如果您喜欢本文请点赞分享，如有任何疑问请在官方仓库或者评论区进行提问。

