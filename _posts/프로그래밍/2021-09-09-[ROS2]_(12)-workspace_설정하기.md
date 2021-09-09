---
title:  "[ROS2] (12)-Workspace 설정하기"
excerpt: "Workspace는 ROS2 패키지를 포함하고 있는 디렉토리를 의미한다. ROS2를 사용하기 위해서는 ROS2 installation workspace를 작업하고자 하는 터미널에 source해야 한다. "
toc: True
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - 프로그래밍
tags:
  - ROS2
date: 2021-09-09 21:16:00
last_modified_at: 2021-09-09
---

## workspace 란
Workspace는 ROS2 패키지를 포함하고 있는 디렉토리를 의미한다. ROS2를 사용하기 위해서는 ROS2 installation workspace를 작업하고자 하는 터미널에 source해야 한다. <a href="https://sangteak601.github.io/%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D/ROS2-_(2)-ROS2_%ED%99%98%EA%B2%BD%EC%84%A4%EC%A0%95"> [ROS2] (2)-ROS2 환경설정 </a> 에서 터미널 창마다 입력해야 한다고 했던 `source /opt/ros/foxy/setup.bash` 명령어가 바로 여기서 말하는 ROS2 installation workspace를 source 하는 작업이다.

Workspace는 하나만 source 할 수 있는 것은 아니고, 동시에 여러 개의 workspace를 source 할 수 있다. 이 경우, 새롭게 source 되는 workspace를 "overlay"라고 하고, 기존의 workspace를 "underlay"라고 한다. underlay는 overlay의 패키지 dependency를 모두 포함해야 하고, overlay는 underlay의 패키지를 무효화하고 덮어쓴다. 여러 개의 overlay와 underlay가 있을 수도 있다.

## workspace 생성
새로운 폴더를 만들어서 workspace를 생성해보자. `mkdir -p ~/dev_ws/src` 를 통해 `dev_ws` 폴더와 `src` 폴더를 만든다.

생성한 폴더로 이동한 뒤, github에서 repo를 복제해온다.

```
cd dev_ws/src
git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
```

이제 `src` 폴더 하위에 `ros_tutorials`라고 하는 폴더가 새로 생겼다. workspace를 빌드하기 전에 먼저 패키지 dependency를 모두 설치해주자. 먼저 `dev_ws`폴더로 이동 후, 설치 명령어를 입력해준다. rosdep가 설치되어 있지 않은 경우, 먼저 설치해야 한다(`sudo apt install python3-rosdep2`, `rosdep update`).

```
cd ~dev_ws
rosdep install -i --from-path src --rosdistro foxy -y
```

이제 아래 명령어를 통해 workspace를 빌드할 수 있다. 먼저, `dev_ws` 폴더로 이동 후, 빌드해야 한다. colcon이 설치되어 있지 않은 경우, 먼저 설치해야 한다(`sudo apt install python3-colcon-common-extensions`).

```
cd ~dev_ws
colcon build
```

> `colcon build`에 `--packages-up-to`를 추가하면 전제 workspace가 아닌 원하는 패키지와 dependency만 빌드할 수 있다. 

빌드가 완료된 후, `dev_ws` 폴더에 가보면 `build`, `install`, `log` 폴더가 새로 생겨있는 것을 볼 수 있다.

## overlay와 underlay
overlay를 source 해보자. 여기서는 ROS2 환경이 underlay가 되고, 새로 빌드한 workspace가 overlay가 되는 것이다. 이전에 `.bashrc` 파일에 `source /opt/ros/foxy/setup.bash`를 추가해두었기 때문에 터미널 창을 새로 열 때마다 ROS2 installation workspace는 자동으로 source 된다.

새로운 터미널 창을 열고, 새로 빌드한 workspace를 source 해보자.

```
cd ~/dev_ws
. install/local_setup.bash
```

`install` 폴더에는 `local_setup.bash` 파일과 `setup.bash` 파일이 있는데, 전자는 overlay의 패키지만 환경에 추가해주는 것이고, 후자는 overlay 패키지 뿐만 아니라 해당 workspace가 빌드된 underlay 환경도 함께 source 해준다. 

즉, ROS2 installation workspace + dev_ws local_setup.bash(2가지를 각각 source 해주는 것)= dev_ws setup.bash(1가지만 source 해주는 것) 가 된다.

이제 turtlesim 패키지를 실행하면 기존에 실행되던 패키지(ROS2 installation workspace)가 아닌 overlay(dev_ws workspace)의 turtlesim 패키지가 실행된다. 실제로 overlay 패키지가 실행되는지 확인하기 위해 turtlesim 윈도우 이름을 바꿔보자.

`~/dev_ws/src/ros_tutorials/turtlesim/src` 폴더의 `turtle_frame.cpp` 파일을 텍스트 편집기로 열어 52번째 줄의 "TurtleSim"을 "MyTurtleSim"으로 변경한다.

다시 `dev_ws` 폴더에서 workspace를 빌드한 후, 새로운 터미널에서 해당 workspace를 source 한다. 이제 turtlesim 노드를 실행하면 window 창의 타이틀이 "MyTurtlesim"으로 바뀌어 있는 것을 확인할 수 있다. 비교를 위해 새로운 터미널 창에서 turtlesim 노드를 실행해보면 윈도우 타이틀이 "TurtleSim"으로 나오는 것을 확인할 수 있다.

이처럼 overlay의 수정은 underlay에는 영향을 미치지 않는다. 적은 수의 패키지를 작업할 때는 이처럼 overlay를 활용하면 전체 workspace를 매번 빌드하지 않아도 되는 이점을 얻을 수 있다.