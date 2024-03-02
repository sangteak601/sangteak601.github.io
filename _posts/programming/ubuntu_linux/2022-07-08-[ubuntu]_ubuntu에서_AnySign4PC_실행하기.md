---
title:  "[ubuntu] ubuntu에서 AnySign4PC 실행하기"
toc: True
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - Programming
tags:
  - Ubuntu
last_modified_at: 2022-07-08
---

## ubuntu에서 AnySign4PC 실행하기
국내 공공기관 웹사이트에 접속하다 보면 AnySign4PC를 설치해야 하는 경우가 종종 있다. 최근에 정부24 홈페이지에서 서류를 출력하려고 하니, AnySign4PC를 설치하지 않으면 로그인할 수 없는 문제가 있었다. Windows에서는 프로그램을 설치하는 것이 크게 어렵지 않지만, ubuntu 환경에서 해당 프로그램을 설치하고 실행하려다 보니 조금 어려움이 있었다. 이번에 알게 된 내용을 기록해두고자 한다.

## AnySign 설치하기
먼저, AnySign 프로그램을 설치하려면 CPU 타입(x86_64 또는 i386)을 알아야 한다. CPU 타입에 따라 설치파일이 다르기 때문이다. 아래 명령어를 이용하면 CPU 타입을 확인할 수 있다.

```
$ uname -m
```

CPU 타입에 맞는 프로그램을 다운로드 한 뒤, 아래와 같이 다운로드된 파일을 설치해준다.

```
$ cd ~/Downloads
$ sudo apt install ./anysign4pc_linux_x86_64.deb
```

설치 후, 설치된 경로를 확인한다.
기본 설치 경로는 `/opt/anysign4pc/` 이다.

## AnySign 실행하기
Windows와 달리 ubuntu에서는 프로그램 설치 이후에 직접 실행까지 시켜줘야 정상적으로 동작했다. 먼저, 모든 인터넷 브라우저를 종료한 뒤 아래 명령어를 실행한다. 설치경로가 다른 경우, 경로에 맞게 수정하도록 하자.

```
/opt/anysign4pc/amd64/config/AnySign.lunux.x64 start
```

명령어를 실행한 후, 다시 웹사이트에 접속하면 정상적으로 접속할 수 있을 것이다. 만약 되지 않는다면 재부팅 후, 상기 명령어를 다시 실행하고 접속해보자.

## AnySign 기타 명령어
프로그램을 시작하기 위해 사용한 `start` 외에 다른 명령어도 있다. 명령어는 `start/stop/restart/reload/status` 총 5개이다. 필요에 따라 프로그램을 정지하거나 재시작하기 위해 사용할 수 있을 것이다.