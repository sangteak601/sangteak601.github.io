---
title:  "[ubuntu] ubuntu에 VS Code 설치하기"
excerpt: "프로그래밍을 하기 위해서는 개발 환경을 구성하는 것이 필요하다. 나의 경우, visual studio code를 선호하기 때문에 VS Code로 개발 환경을 구축하기로 했다."
toc: False
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - Programming
tags:
  - Ubuntu
last_modified_at: 2021-08-21
---

## ubuntu에 VS Code 설치하기
프로그래밍을 하기 위해서는 개발 환경을 구성하는 것이 필요하다. 나의 경우, visual studio code를 선호하기 때문에 VS Code로 개발 환경을 구축하기로 했다.  
VS Code는 일종의 text 편집기이기 때문에 프로그램이 아주 가벼운 장점이 있다. 또, 필요한 경우 interpreter와 연결하여 바로 코드를 실행할 수도 있다. 다만, 직접 환경 설정을 해야하기 때문에 다른 IDE에 비하면 조금 번거로울 수 있다.

## 한글이 입력되지 않는 문제
visual studio code 사이트에 나와있는 것처럼 snap 형식으로 설치했더니 한글이 입력되지 않는 문제가 있었다.

```
$ sudo snap install --classic code
```

해당 문제에 대해 검색을 하다가 아래 웹사이트에서 해결 방법을 찾을 수 있었다.
<https://gist.github.com/philoskim/a79440bd51ae40f04a4d7cafa472caf1>

해결 방법은 snap 형식이 아닌 .deb 형식으로 VS Code를 설치하는 것이다. 설치 방법은 다음과 같다.

- 기존에 snap으로 설치된 VS Code 삭제
```
$ sudo snap remove code
```
- 다음의 웹사이트에서 .deb 형식의 VS Code를 다운받는다.(<https://code.visualstudio.com/download>)
- .deb 파일(버전에 따라 파일명은 다름)로 VS Code를 설치한다. (해당 파일이 위치한 폴더(다운로드)로 이동 후, 명령 실행)
```
$ cd 다운로드
$ sudo dpkg -i code_1.58.2-1626302803_amd64.deb
```

> 참고  
VS Code 뿐만 아니라, 다른 프로그램에서도 유사한 문제가 있는 것으로 보인다. 프로그램을 설치했는데 문제가 있다면 삭제 후, .deb 파일을 다운로드 받아 설치해보자.