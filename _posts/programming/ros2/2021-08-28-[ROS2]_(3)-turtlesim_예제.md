---
title:  "[ROS2] (3)-turtlesim 예제"
excerpt: "turtlesim 예제를 통해 ROS2가 어떠한 방식으로 동작하는지, ROS2를 활용해서 어떤 일들을 할 수 있는지를 느껴보기로 한다. 해당 예제에 사용된 명령어 및 개념들은 이후에 상술하기로 한다."
toc: True
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - programming
tags:
  - ROS2
last_modified_at: 2021-08-28
---

## ROS2 turtlesim 예제
turtlesim 예제를 통해 ROS2가 어떠한 방식으로 동작하는지, ROS2를 활용해서 어떤 일들을 할 수 있는지를 느껴보기로 한다. 해당 예제에 사용된 명령어 및 개념들은 이후에 상술하기로 한다.

## turtlesim 설치
먼저, turtlesim 패키지를 설치한다.

```
sudo apt update

sudo apt install ros-foxy-turtlesim
```

아래 명령어로 설치가 되었는지 확인한다. 설치가 되었다면 4가지 파일 list를 출력한다.

```
ros2 pkg executables turtlesim
```

## turtlesim 실행
아래 명령어로 turtlesim을 실행한다. 아래 사진과 같이 창이 하나 새로 뜨게 된다.

```
ros2 run turtlesim turtlesim_node
```

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-28-[ROS2]_(3)-turtlesim_예제/turtlesim.png" alt="image"> 

이제 turtle을 움직일 수 있는 새로운 노드를 실행한다. 새로운 터미널을 열고, 아래 명령어를 실행한다.

```
ros2 run turtlesim turtle_teleop_key
```

상기 명령어를 실행한 터미널 창을 활성화 시키고, 키보드의 화살표 키를 누르면 turtle이 움직이는 것을 확인할 수 있다. 앞뒤 화살표는 turtle이 이동하게 하고, 좌우 화살표는 turtle이 회전하도록 한다.

## rqt를 사용하여 예제 변경하기
이제 rqt를 통해 turtlesim 예제를 조금 변경해보자.

먼저, 아래 명령어로 rqt를 설치해준다.

```
sudo apt update

sudo apt install ~nros-foxy-rqt*
```

설치가 완료되면 터미널에 `rqt`라고 입력하여 rqt를 실행해준다. 상단의 메뉴바에서 **Plugins > Services > Service Caller**를 클릭한다. 좌측 상단의 새로고침 버튼을 눌러주고, 우측의 드롭다운 박스에서 `/spawn`을 선택한다. name Topic에서 Expression 부분을 'turtle2'라고 바꾸어 주고, 우측 상단의 'Call' 버튼을 눌러준다. 그러면 아까 열어둔 turtlesim 화면의 좌측 하단에 새로운 거북이가 생긴다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-28-[ROS2]_(3)-turtlesim_예제/rqt_spawn.png" alt="image"> 

이제 turtle2를 움직여보자. 아래의 명령어를 새로운 turminal 창에 실행해준다.

```
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

상기 터미널을 활성화 하고, 키보드를 입력하면 새로 만든 거북이가 움직인다. 

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-28-[ROS2]_(3)-turtlesim_예제/turtlesim_2turtles.png" alt="image"> 

