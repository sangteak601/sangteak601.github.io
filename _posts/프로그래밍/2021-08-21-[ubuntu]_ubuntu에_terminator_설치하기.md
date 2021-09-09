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

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-21-[ubuntu]_ubuntu에_terminator_설치하기/terminator_install.png" alt="image"> 

설치가 완료되면 터미네이터를 실행한 후, 우클릭하여 환경설정으로 진입한다. 환경설정에서 본인의 취향에 맞게 여러가지 설정을 변경할 수 있다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-21-[ubuntu]_ubuntu에_terminator_설치하기/terminator_setting2.png" alt="image"> 

여기에서는 창을 분할하는 방법에 대해 알아본다. 레이아웃 탭으로 이동 후, 새 레이아웃을 만들어 준다. 본인이 원하는 레이아웃으로 화면을 분할한 뒤, "새 레이아웃"의 이름을 default로 바꾸고, 덮어쓰면 기본값으로 적용이 된다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-21-[ubuntu]_ubuntu에_terminator_설치하기/terminator_setting3.png" alt="image"> 

나는 세로로 한 번, 가로로 한 번 나눈 형태로 설정했다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-21-[ubuntu]_ubuntu에_terminator_설치하기/terminator_setting4.png" alt="image"> 

## apt 명령어 실행시 출력되는 에러 없애기
terminator를 설치하고 나면 apt 명령어를 실행할 때, 아래와 같은 에러가 뜰 때가 있다.

```
http://ppa.launchpad.net/gnome-terminator/ppa/ubuntu focal Release' does not have a Release file
```

이것은 레포지토리의 버전이 최신 ubuntu(focal)를 지원하지 않기 때문에 생기는 에러이다. 이를 해결하기 위해서는 아래의 방법을 따른다.

'소프트웨어 & 업데이트'를 연 뒤, '기타 소프트웨어' 탭에서 `http://ppa.launchpad.net/gnome-terminator/ppa/ubuntu focal` 항목을 클릭한 뒤, 삭제한다.

## 유용한 단축키 정리
terminator 또는 terminal에서 사용 가능한 유용한 단축키를 정리한다.

|단축키|내용|
|---|---|
|ctrl + shift + c or v|복사 or 붙여넣기|
|F11|풀 스크린 모드 <-> 창 모드 전환|
|ctrl + shift + O|수평으로 터미널 창 나누기|
|ctrl + shift + E|수직으로 터미널 창 나누기|
|ctrl + shift + W|현재 터미널 창 삭제(닫기)|
|ctrl + shift + T|새로운 탭 열기|
|ctrl + tap|다음창으로 이동|
|ctrl + shift + tap|이전창으로 이동|