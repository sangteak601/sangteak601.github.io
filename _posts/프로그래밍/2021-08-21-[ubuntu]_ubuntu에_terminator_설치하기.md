---
title:  "[ubuntu] ubuntu에 terminator 설치하기"
excerpt: "ubuntu에서 유용한 프로그램 중 하나인 terminator 설치방법에 대해 알아본다."
toc: False
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - 프로그래밍
tags:
  - ubuntu
last_modified_at: 2021-08-21
---

## ubuntu에 terminator 설치하기
ubuntu에서 유용한 프로그램 중 하나인 terminator 설치방법에 대해 알아본다. terminator는 terminal 창을 한 번에 여러 개 띄워주는 프로그램이다. linux 운영체제에서는 terminal을 자주 활용하게 되므로, 아주 활용도가 높은 프로그램이다.

terminal 창을 열고, 아래 명령어를 순차적으로 입력해주면 된다.

```
$ sudo add-apt-repository ppa:gnome-terminator
$ sudo apt-get update
$ sudo apt-get install terminator  
```
아래 사진과 같이 설치가 되는 것을 확인할 수 있다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-21-[ubuntu]_ubuntu에_terminator_설치하기/terminator_install.png" alt=""> 

설치가 완료되면 터미네이터를 실행한 후, 우클릭하여 환경설정으로 진입한다. 환경설정에서 본인의 취향에 맞게 여러가지 설정을 변경할 수 있다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-21-[ubuntu]_ubuntu에_terminator_설치하기/terminator_setting2.png" alt=""> 

여기에서는 창을 분할하는 방법에 대해 알아본다. 레이아웃 탭으로 이동 후, 새 레이아웃을 만들어 준다. 본인이 원하는 레이아웃으로 화면을 분할한 뒤, "새 레이아웃"의 이름을 default로 바꾸고, 덮어쓰면 기본값으로 적용이 된다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-21-[ubuntu]_ubuntu에_terminator_설치하기/terminator_setting3.png" alt=""> 

나는 세로로 한 번, 가로로 한 번 나눈 형태로 설정했다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-21-[ubuntu]_ubuntu에_terminator_설치하기/terminator_setting4.png" alt=""> 