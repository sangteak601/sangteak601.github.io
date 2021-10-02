---
title:  "[ROS2] (20)-C++ 클래스에서 parameter 사용하기"
excerpt: "노드를 만들다보면 때로는 런치파일에서 설정할 수 있는 파라미터가 필요할 때가 있다."
toc: True
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - Programming
tags:
  - ROS2
date: 2021-10-02 16:00:00
last_modified_at: 2021-10-02
---

## 파라미터 사용하기
노드를 만들다보면 때로는 런치파일에서 설정할 수 있는 파라미터가 필요할 때가 있다. 여기서는 C++ 클래스에서 파라미터를 만든는 방법과 런치 파일에서 파라미터를 설정하는 방법을 알아볼 것이다.

## 패키지 만들기
`dev_ws/src` 폴더에 `cpp_parameters` 라는 패키지를 생성한다.

```
$ cd dev_ws/src
$ ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
```

`--dependencies`는 자동으로 필요한 의존성을 `package.xml`과 `CMakeLists.txt`에 추가해준다. 의존성은 추가되었으므로, 설명, 이름, 이메일, 라이센스 등의 정보만 `package.xml` 파일에서 추가해준다.

## C++ 노드 작성하기
`dev_ws/src/cpp_parameters/src` 폴더에 `cpp_parameters_node.cpp` 파일을 만들고, 아래 내용을 붙여넣는다.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;

class ParametersClass: public rclcpp::Node
{
  public:
    ParametersClass()
      : Node("parameter_node")
    {
      this->declare_parameter<std::string>("my_parameter", "world");
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&ParametersClass::respond, this));
    }
    void respond()
    {
      this->get_parameter("my_parameter", parameter_string_);
      RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
    }
  private:
    std::string parameter_string_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametersClass>());
  rclcpp::shutdown();
  return 0;
}
```

이제 코드 내용을 살펴보자. 

먼저, 클래스와 생성자를 정의하고 있다. 생성자의 첫 번째 줄에서 `my_parameter` 라는 이름의 파라미터를 생성하고 `world`라는 값을 부여해주고 있다. 그 다음 타이머를 정의해서 1초에 한 번씩 `respond` 함수를 실행하도록 하고 있다.

```cpp
class ParametersClass: public rclcpp::Node
{
  public:
    ParametersClass()
      : Node("parameter_node")
    {
      this->declare_parameter<std::string>("my_parameter", "world");
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&ParametersClass::respond, this));
    }
```

`respond` 함수의 첫 번째 줄에서 `my_parameter`를 불러와서 `parameter_string_`에 저장하고 있다.

```cpp
    void respond()
    {
      this->get_parameter("my_parameter", parameter_string_);
      RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
    }
```

다음으로 `CMakeLists.txt` 파일에 실행파일을 만드는 부분을 추가해준다.

```
add_executable(parameter_node src/cpp_parameters_node.cpp)
ament_target_dependencies(parameter_node rclcpp)

install(TARGETS
  parameter_node
  DESTINATION lib/${PROJECT_NAME}
)
```

## 빌드 및 실행하기
먼저, 의존성을 모두 설치해준다.

```
$ cd ~/dev_ws
$ rosdep install -i --from-path src --rosdistro foxy -y
```

새로운 패키지를 빌드한다.

```
$ cd ~/dev_ws
$ colcon build --packages-select cpp_parameters
```

환경변수를 설정하고, 노드를 실행한다.

```
$ cd ~/dev_ws
$ source install/setup.bash
$ ros2 run cpp_parameters parameter_node
```

1초마다 `Hello world`를 출력하는 것을 확인할 수 있다.

## 파라미터 수정하기
먼저, 콘솔 창을 통해 파라미터를 수정해보자. 노드를 실행한 상태에서 다음 내용을 진행한다.

```
$ ros2 run cpp_parameters parameter_node
```

새로운 터미널에서 아래 내용을 실행한다.

```
cd ~/dev_ws
$ source install/setup.bash
$ ros2 param list
```

`my_parameter`가 목록에 나오는 것을 확인할 수 있다. 같은 터미널 창에 아래 내용을 입력해준다.

```
ros2 param set /parameter_node my_parameter earth
```

출력하는 메시지가 `Hello world`에서 `Hello earth`로 바뀐 것을 확인할 수 있다.

다음으로 런치 파일을 통해 파라미터를 수정해보자. 먼저, `dev_ws/src/cpp_parameters/` 폴더 안에 `launch`라는 폴더를 생성하고, `cpp_parameters_launch.py`를 만들어 아래 내용을 붙여넣는다.

```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="parameter_node",
            name="custom_parameter_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
```

여기서 `parameters=` 를 통해 `my_parameter`에 `earth`라는 값을 할당해주고 있다.

`CMakeLists.txt` 파일을 열고, 아래 내용을 추가해준다.

```
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

다음으로 패키지를 다시 빌드하고, 런치파일로 실행해준다.

```
$ cd ~/dev_ws
$ colcon build --packages-select cpp_parameters
$ . install/setup.bash
$ ros2 launch cpp_parameters cpp_parameters_launch.py
```

`Hello earth`를 출력하는 것을 확인할 수 있다.