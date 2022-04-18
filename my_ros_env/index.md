# 我的 ROS 开发环境


本文主要介绍我所使用的 ROS 开发环境
我采用这个模式进行开发是受到了这篇文章的启发：{{< link "https://www.allisonthackston.com/articles/vscode-docker-ros2.html" >}}
<!--more-->

如果您用过 ROS 就会发现配置这玩意儿的开发环境是一件非常麻烦的事情，经常会出现缺少支持库或者支持库冲突的情况，尤其是一台电脑上需要做多种任务的情况（比如 深度学习、QT开发、ROS1、ROS2）则会更糟糕。
这个时候我们就有必要使用 Docker 进行开发，关于如何使用 `VSCode + Docker` 进行开发的流程已经写在：[Docker 配合 VSC 开发最佳实践]({{< ref "Docker-with-VSC_best-practice" >}})，本文主要介绍如何配置 ROS 相关开发环境

## 1. 设置 .devcontainer


