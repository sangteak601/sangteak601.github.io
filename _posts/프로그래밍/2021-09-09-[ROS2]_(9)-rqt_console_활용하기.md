---
title:  "[ROS2] (9)-rqt_console 활용하기"
excerpt: "rqt_console은 로그 메시지를 확인할 수 있는 GUI 툴이다. rqt_console을 이용하면 로그 메시지를 더 정돈된 형태로 확인할 수 있고, 메시지들을 저장하고 불러올 수도 있다.
"
toc: True
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - 프로그래밍
tags:
  - ROS2
date: 2021-09-09 12:16:00
last_modified_at: 2021-09-09
---

## rqt_console 실행하기
rqt_console은 로그 메시지를 확인할 수 있는 GUI 툴이다. rqt_console을 이용하면 로그 메시지를 더 정돈된 형태로 확인할 수 있고, 메시지들을 저장하고 불러올 수도 있다.

rqt_console을 실행하기 위해서는 다음 명령어를 터미널에 입력해준다.

```
ros2 run rqt_console rqt_console
```

다음과 같이 rqt_console 창이 뜬다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-09-09-[ROS2]_(9)-rqt_console_활용하기/rqt_console.png" alt="image">

첫 번째 영역은 시스템에서 출력하는 모든 로그 메시지를 보여주는 곳이고, 두 번째 영역은 특정 메시지를 제거하기 위한 필터를 설정할 수 있는 영역이다. 마지막 영역은 특정 조건을 만족하는 메시지만 하이라이트시키기 위한 조건을 설정할 수 있는 영역이다.

## logger level
먼저, 새로운 터미널 창에서 turtlesim 예제를 실행한다(`ros2 run turtlesim turtlesim_node`). 새로운 터미널 창에서 다음과 같이 토픽을 퍼블리시 한다.

```
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

거북이가 계속 같은 방향으로 이동하고, 벽에 부딪히게 된다. 이제 rqt_console을 보게 되면 로그 메시지가 첫 번째 영역에 출력되는 것을 확인할 수 있다. 메시지가 충분히 출력되고 나면 ctrl + c 로 토픽 퍼블리시를 중단하고, 출력된 메시지를 확인해보자.

ROS2의 로그 메시지는 다음과 같이 5개의 심각도 레벨을 갖고 있다. Fatal > Error > Warn > Info > Debug. 일반적으로 각각의 레벨은 다음의 의미를 갖는다고 해석할 수 있다.

- Fatal: 시스템 손상으로부터 보호하기 위해 시스템이 종료될 예정임을 나타냄
- Error: 시스템 손상은 아니지만, 제대로 기능이 동작하지 못하는 경우를 나타냄
- Warn: 기능이 동작하지 못하는 것은 아니지만, 예상치 못한 상황 또는 비이상적인 결과가 발생하는 경우를 나타냄
- Info: 현재 상태를 나타냄
- Debug: 시스템이 실행되는 전체 프로세스를 메시지로 출력함

기본 로그 메시지 레벨은 Info 이다. 기본 레벨보다 더 심각하거나 같은 수준의 메시지만 출력하게 되고, 더 낮은 레벨의 메시지는 출력되지 않는다. 예를 들어, 기본 로그 메시지 레벨이 Info인 경우, Debug 메시지만 출력되지 않는다.

아래 명령어를 통해 노드를 실행할 때, 기본 로그 메시지 레벨을 설정할 수 있다.

```
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```

