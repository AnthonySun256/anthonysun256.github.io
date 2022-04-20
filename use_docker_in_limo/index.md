# 在 limo 中安装 docker


## 1. 安装 Docker 引擎

首先清除旧版本 Docker：

```shell
$ sudo apt-get remove docker docker-engine docker.io containerd runc
```

之后使用官方安装脚本进行安装：

```shell
$ curl -fsSL https://get.docker.com -o get-docker.sh
$ sudo sh get-docker.sh
```

将当前用户添加到 docker 用户组，使非 root 用户也能使用 Docker（**使用 root 登陆用户跳过此步骤**）:

```shell
sudo systemctl restart dockersudo systemctl restart docker$ sudo usermod -aG docker ${USER}
$ sudo systemctl restart docker
```

更换 docker 源为国内源：

```shell
$ sudo mkdir -p /etc/docker
$ sudo tee /etc/docker/daemon.json <<-'EOF'
{
  "registry-mirrors": ["https://ustc-edu-cn.mirror.aliyuncs.com/"]
}
EOF
$ sudo systemctl daemon-reload
$ sudo systemctl restart docker
```

后续如果需要更改镜像地址，自行编辑 ``/etc/docker/daemon.json`` 即可。

重新登陆系统，测试 docker：

```shell
$ docker run --rm hello-world
```

1得到以下输出证明安装成功：

sudo systemctl restart docker![1](images/1.png)

## 2. 安装 docker-compose 插件

下载官方插件文件（这里使用了 Github 加速服务）：

```shell
$ sudo curl -L "https://ghproxy.com/https://github.com/docker/compose/releases/download/v2.4.1/docker-compose-linux-aarch64" -o /usr/local/bin/docker-compose
```

> 如果无法下载，使用以下指令：
>
> ```shell
> $ sudo curl -L "https://github.com/docker/compose/releases/download/v2.4.1/docker-compose-linux-aarch64" -o /usr/local/bin/docker-compose
> ```

授予插件执行权限：

```shell
$ sudo chmod +x /usr/local/bin/docker-compose
```

完成后执行：

```shell
 $ docker-compose --version
```

输出版本信息即为成功：

```
Docker Compose version v2.4.1
```


