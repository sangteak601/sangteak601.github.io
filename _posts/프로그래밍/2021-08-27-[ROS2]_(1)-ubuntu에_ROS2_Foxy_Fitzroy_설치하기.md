---
title:  "[ROS2] (1)-ubuntu에 ROS2 Foxy Fitzroy 설치하기"
excerpt: "ubuntu에 ROS를 설치하는 방법에 대해 정리한다. 현재(2021년 8월 27일 기준)는 ROS2의 Foxy Fitzroy 버전이 가장 최신의 LTS(Long Term Support) 버전이다."
toc: True
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - 프로그래밍
tags:
  - ROS2
last_modified_at: 2021-08-27
---

## ubuntu에 ROS2 Foxy Fitzroy 설치하기
ubuntu에 ROS를 설치하는 방법에 대해 정리한다. 현재(2021년 8월 27일 기준)는 ROS2의 Foxy Fitzroy 버전이 가장 최신의 LTS(Long Term Support) 버전이다. Foxy버전이 ROS2의 최초의 LTS 버전이다. LTS라고는 하지만, ROS1(ROS가 정식 명칭이지만, ROS2와 구분하기 위해 ROS1이라고 함)에 비하면 지원하는 기간이 훨씬 짧다. ROS1은 5년인데 비해 ROS2는 3년에 불과하다. ROS1과 ROS2의 최신 LTS 버전을 비교하면 다음과 같다.

- ROS1 Noetic Ninjemys - 출시일: 2020-05-23, 서비스 종료일: 2025-05
- ROS2 Foxy Fitzroy - 출시일: 2020-06-05, 서비스 종료일: 2023-05

ROS1의 서비스 기간이 훨씬 길기도 하고, 참고할만한 자료도 훨씬 많을 것이다. ROS1은 출시한지 10년이 넘었고, ROS2는 출시한지 4년 정도밖에 되지 않았기 때문에 자료의 차이가 분명 있을 것이다.

그럼에도 불구하고, 나는 ROS2를 설치하고, 공부하기로 했다. 어차피 할거라면 최신버전으로 공부하는게 좋을 것 같기도 했고, ROS2는 ROS1과 달리 Windows 운영체제도 지원하기 때문에 훨씬 많은 사용자가 유입될 것이라고 생각했기 때문이다.

상세내용은 아래 링크를 참고하면 된다.  
<https://docs.ros.org/en/foxy/Installation.html>

## Debian 패키지로 설치하기
Windows와 mac은 제외하고, ubuntu에서 설치하는 방법에 대해서만 알아보기로 한다. 설치할 수 있는 방법은 패키지로 설치하는 방법, 소스 파일로부터 직접 빌드하는 방법이 있다. 

바이너리 패키지는 한 번에 전체 패키지를 설치하는 간편한 방법이고, 소스 파일로부터 직접 빌드하는 방법은 내 취향대로 설치되는 파일을 추가 및 생략할 수 있다는 장점이 있다. 아무래도 특별히 변경할 내용이 없다면 바이너리 패키지로 설치하는 것이 편할 것이다.

나는 Debian 패키지로 설치하기로 했다. ROS2 Foxy를 설치하기 위해서는 ubuntu가 20.04(Focal Fossa)버전이어야 한다. 

먼저, locale 설정을 확인한다. terminal 창에 `locale`을 입력한다. 나는 아래 사진과 같이 `LANG=ko_KR.UTF-8`로 나오는 것을 확인할 수 있었다. UTF-8 이기만 하면 문제없는 것으로 보인다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-27-[ROS2]_(1)-ubuntu에_ROS2_Foxy_Fitzroy_설치하기/check_locale.png" alt="image"> 

다음으로 아래 명령어를 차례로 terminal에 입력해준다. 

```
sudo apt update && sudo apt install curl gnupg2 lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
```
실행해보니 `sudo: curl: 명령이 없습니다.`라는 오류가 떴다. curl이 설치되어 있지 않아 발생하는 오류이다. 먼저, curl을 설치해야 한다. curl이 설치되어 있다면 아래 명령어는 생략해도 된다.

```
sudo apt install curl
```
다시 실행해보니 오류가 뜨지 않았다. 다음으로 아래 명령어를 차례로 입력한다. ROS2 apt 레포지토리를 시스템에 추가하고, 레포지토리를 업데이트 해주는 것이다.

```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

아래의 명령어를 실행하면 설치가 시작된다. 시간이 오래 걸린다.

```
sudo apt install ros-foxy-desktop
```

긴 기다림 끝에 설치가 완료되었다.

## 예제 실행
이제 설치는 완료되었고, 예제를 실행하는 것으로 마무리한다.

ROS2 명령어를 사용하기 위해서는 아래의 명령어를 먼저 실행해야 한다. 설정 파일을 읽어온 뒤, 적용하는 것이다.

```
source /opt/ros/foxy/setup.bash
```

이제 예제를 실행해보자. 아래 명령어를 2개의 터미널 창을 연 뒤, 각각의 터미널에 입력한다.

터미널 1
```
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
```

터미널2
```
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener
```

아래와 같이 하나의 터미널에서는 메세지를 보내고, 다른 터미널에서는 메세지를 받는 것을 확인할 수 있다. 이 예제가 정상 동작한다면 설치가 정상적으로 완료된 것이다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-27-[ROS2]_(1)-ubuntu에_ROS2_Foxy_Fitzroy_설치하기/ros2_example.png" alt="image"> 

참고로 설치된 ROS2를 제거하는 방법은 다음과 같다.

```
sudo apt remove ros-foxy-* && sudo apt autoremove
```
