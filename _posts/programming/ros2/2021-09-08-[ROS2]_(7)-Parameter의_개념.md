---
title:  "[ROS2] (7)-Parameter의 개념"
excerpt: "parameter는 노드의 설정값과 비슷한 개념으로, 해당 노드에서 사용되는 변수라고 이해할 수 있다. 파라미터는 다양한 데이터 형식으로 저장될 수 있고, 동적으로 재설정될 수 있다."
toc: True
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - programming
tags:
  - ROS2
last_modified_at: 2021-09-08
---

## parameter란?
parameter는 노드의 설정값과 비슷한 개념으로, 해당 노드에서 사용되는 변수라고 이해할 수 있다. 파라미터는 다양한 데이터 형식으로 저장될 수 있고, 동적으로 재설정될 수 있다.

이번 포스팅에서 사용할 parameter 관련 명령어를 정리하면 다음과 같다.

```
ros2 param list
ros2 param get <node_name> <parameter_name>
ros2 param set <node_name> <parameter_name> <value>
ros2 param dump <node_name>
ros2 param load <node_name> <parameter_file>
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

## parameter 정보 확인
2개의 터미널 창에 각각 `ros2 run turtlesim turtlesim_node`, `ros2 run turtlesim turtle_teleop_key`를 입력하여 실행해준다.

`ros2 param list` 명령어를 통해 현재 실행중인 노드의 파라미터 리스트를 확인할 수 있다. 실행한 결과는 다음과 같다.

```
/teleop_turtle:
  scale_angular
  scale_linear
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  use_sim_time
```

`/teleop_turtle`, `/turtlesim` 2개의 노드가 실행중이고, 각각 3,4개의 파라미터를 갖고 있다. `use_sim_time` 파라미터는 2개 노드에 공통으로 포함되어 있다.

파라미터의 type과 값을 확인하기 위해서는 `ros2 param get <node_name> <parameter_name>` 명령어를 사용한다. `/turtlesim`노드의 `background_b` 파라미터 값을 확인해보면 다음과 같다.

```
ros2 param get /turtlesim background_b

Integer value is: 255
```

`background_b` 파라미터는 정수(integer)이고, 현재 값은 255인 것을 확인할 수 있다.

## parameter 변경하기
파라미터의 값을 터미널 창을 통해서 변경할 수 있다. `ros2 param set <node_name> <parameter_name> <value>` 를 입력한다. turtlesim 예제의 배경색을 변경해보자.

```
ros2 param set /turtlesim background_b 0
```

다음 사진과 같이 국방색으로 배경색이 변경된 것을 확인할 수 있다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-09-08-[ROS2]_(7)-Parameter의_개념/turtlesim_bg_color.png" alt="image"> 

`set` 명령어를 통해 파라미터를 변경하게되면 현재 session에서만 변경이 되는 것이고, 영구적으로 변경이 되는 것은 아니다.

## parameter 저장 및 불러오기
파라미터 값을 저장해두었다가 다음에 필요할 때 불러와서 사용할 수도 있다. 노드의 현재 파라미터 값을 저장하기 위해서는 `ros2 param dump <node_name>` 명령어를 사용한다. `/turtlesim` 노드의 파라미터를 저장해보자.

```
ros2 param dump /turtlesim
```

workspace 폴더에 turtlesim.yaml 파일이 저장되었다. 

이제 저장한 파라미터를 불러와서 다시 적용해보자. 먼저 실행중인 노드를 모두 종료하고 다시 실행한다. 다시 실행하면 배경색이 파란색으로 돌아와 있는 것을 확인할 수 있다. 이제 `ros2 param load <node_name> <parameter_file>` 명령어를 통해 파라미터를 불러와보자.

```
ros2 param load /turtlesim ./turtlesim.yaml
```

파라미터를 성공적으로 설정했다는 메시지가 출력되고, 배경색이 다시 바뀌어 있는 것을 확인할 수 있다.

`ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>` 명령어를 이용하면 노드를 실행할 때, 저장된 파라미터 값을 바로 불러올 수도 있다.