---
title:  "[ROS2] (21)-Python 클래스에서 parameter 사용하기"
excerpt: "이번 포스팅에서는 python 클래스 내에서 파라미터를 정의하고 설정하는 방법에 대해 알아볼 것이다."
toc: True
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - Programming
tags:
  - ROS2
date: 2021-10-04 12:00:00
last_modified_at: 2021-10-04
---

## Python 클래스에서 parameter 사용하기
이번 포스팅에서는 python 클래스 내에서 파라미터를 정의하고 설정하는 방법에 대해 알아볼 것이다.

## 패키지 만들기
먼저, `dev_ws/src` 폴더에 `python_parameters` 라는 패키지를 생성한다.

```
$ cd ~/dev_ws/src
$ ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy
```

`package.xml`에 의존성 부분은 추가해주었으므로(`--dependencies`를 통해), 필요한 경우, 설명, 라이센스 등의 정보만 업데이트 해주면 된다.

## 파이썬 노드 작성하기
`dev_ws/src/python_parameters/python_parameters` 폴더 안에 `python_parameters_node.py` 파일을 생성하고, 아래 코드를 붙여넣는다.

```py
import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('my_parameter', 'world')

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

코드를 살펴보면, 먼저 클래스와 생성자를 정의하고 있다. 생성자 내에서 `declare_parameter` 를 통해 파라미터를 정의하고 있다.

```py
class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('my_parameter', 'world')
```

`timer_callback` 함수에서는 `my_parameter`를 `my_param` 변수에 저장한다. 다시 `my_parameter`를 `world`로 설정해준다.

다음으로 `setup.py` 파일을 열고, 아래와 같이 `entry_points` 영역에 내용을 추가해준다.

```py
entry_points={
    'console_scripts': [
        'param_talker = python_parameters.python_parameters_node:main',
    ],
},
```

## 빌드 및 실행하기
의존성을 모두 설치하고, 빌드를 진행한다.

```
$ cd ~/dev_ws
$ rosdep install -i --from-path src --rosdistro foxy - y
$ colcon build --packages-select python_parameters
```

환경변수를 설정하고, 실행한다.

```
$ cd ~/dev_ws
$ . install/setup.bash
$ ros2 run python_parameters param_talker
```

## 파라미터 수정하기
먼저, 콘솔을 통해 파라미터를 수정해보자.

노드를 실행시킨다.

```
$ cd ~/dev_ws
$ . install/setup.bash
$ ros2 run python_parameters param_talker
```

새로운 터미널에서 아래와 같이 파라미터 리스트를 확인하고, `my_parameter`를 `earth`로 수정해준다.

```
$ cd ~/dev_ws
$ . install/setup.bash
$ ros2 param list
$ ros2 param set /minimal_param_node my_parameter earth
```

`Hello earth!`를 한 번 출력한 뒤에 다시 `Hello world!`를 출력하는 것을 확인할 수 있다.

이번에는 런치 파일을 통해 파라미터를 수정해보자. 먼저, `dev_ws/src/python_parameters/` 폴더 내에 `launch` 폴더를 생성하고, `python_parameters_launch.py` 파일을 만들어 아래 내용을 붙여넣는다.

```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_parameters',
            executable='param_talker',
            name='custom_parameter_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'my_parameter': 'earth'}
            ]
        )
    ])
```

다음으로 `setup.py` 파일에 아래 내용을 추가해준다. `import` 부분은 파일 상단에 추가한다.

```py
import os
from glob import glob
# ...

setup(
  # ...
  data_files=[
      # ...
      (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ]
  )
```

패키지를 다시 빌드한 후, 런치 파일을 실행한다.

```
$ cd ~/dev_ws
$ colcon build --packages-select python_parameters
$ . install/setup.bash
$ ros2 launch python_parameters python_parameters_launch.py
```

역시, 처음에 `Hello earth!`를 한 번 출력하고, `Hello world!`를 출력하는 것을 확인할 수 있다.