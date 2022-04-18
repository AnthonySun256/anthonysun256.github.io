# 在 Docker 中使用 Nvidia 显卡


作为一名炼丹师，我们经常需要使用 Nvidia 显卡辅助我们加速计算，但是 Docker 并不直接支持访问 Nvidia 显卡，但好在 Nvidia 官方提供了提供了相关 Docker 插件可以让我们直接在容器中访问 Nvidia 显卡
<!--more-->

## 1. 查看安装专有驱动
想要使用 Nvidia 显卡，我们必须先要安装 Nvidia 官方驱动（默认情况下我们使用的是 开源驱动）
>  安装流程：{{< link "https://zhuanlan.zhihu.com/p/388970072" >}}

## 2. 安装 Nvidia-docker

> 官方仓库：{{< link "https://github.com/NVIDIA/nvidia-docker" >}}
>
> 安装指南：{{< link "https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker" >}}

{{< admonition >}}
本节使用 Ubuntu + docker2作为教程环境，其余发行版请参考[官方安装指南](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker[)
{{< /admonition >}}需要注意的是，在已经构建好的 容器 中改变这些环境变量不会有任何效果，如果您需要梗概配置还请 重新构建容器。

### 添加 GPG key 和 源

在终端执行如下指令：

```shell
$ distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

之后更新 apt:

```shell
$ sudo apt-get update
```

### 安装 Nvidia-docker 

使用如下指令安装：

```shell
$ sudo apt-get install -y nvidia-docker2
```

成功后重启 docker：

```shell
$ sudo systemctl restart docker
```

之后可以进行测试：

```shell
$ sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

应当能看到输出：

```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 450.51.06    Driver Version: 450.51.06    CUDA Version: 11.0     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  Tesla T4            On   | 00000000:00:1E.0 Off |                    0 |
| N/A   34C    P8     9W /  70W |      0MiB / 15109MiB |      0%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+

+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
|  No running processes found                                                 |
+-----------------------------------------------------------------------------+
```

## 3. 使用说明

> 官方文档：{{<link "https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#gpu-enumeration" >}}

与未使用 nvidia 显卡的容器不同之处在于 nvidia-docker 多了一个 ``--gpus `` 参数，这个参数标明我们容器中可以使用哪些显卡（如果你的机器上有多张显卡的话）

一般情况下我们使用 ``docker --gpus all``  即可，如果需要特别指名使用哪几张显卡，则可以使用类似 ``--gpus 0,1`` 指明使用 GPU UUID 为 0、1 的显卡

此外，根据不同情景需要我们还需要指名要在容器中安装哪些驱动程序或支持库，完整的列表见： {{< link "https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#driver-capabilities" >}}

比如我们需要 ``CUDA`` 和 ``nvidia-smi+NVML`` 支持，则需要这样

```shell
$ docker run --rm --gpus 'all,"capabilities=compute,utility"' \
    nvidia/cuda:11.0-base nvidia-smi
```

或者这样：
```shell
# 传递环境变量的方式
$ docker run --rm --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES=2,3 \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    nvidia/cuda nvidia-smi
```

{{< admonition title="注意">}}
需要注意的是，在已经构建好的 容器 中改变这些环境变量不会有任何效果，如果您需要梗概配置还请 重新构建容器。
{{< /admonition >}}

## 结语

至此，在 Docker 中使用 nvidia 显卡的方法就介绍完毕了，如果您需要在 容器 中使用 Nvidia 相关支持库或者其他功能，则请参考官方文档。

