---
title:  "[ubuntu] ubuntu에 sticky notes 설치하기"
excerpt: "나는 해야할 일들을 관리하고, 잊지 않기 위해서 주로 post-it 형태의 메모장을 사용한다. 이러한 형태의 프로그램인 sticky notes를 ubuntu에 설치하는 방법을 정리한다."
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

## ubuntu에 sticky notes 설치하기
나는 해야할 일들을 관리하고, 잊지 않기 위해서 주로 post-it 형태의 메모장을 사용한다. 이러한 형태의 프로그램인 sticky notes를 ubuntu에 설치하는 방법을 정리한다.

아래의 명령을 순서대로 입력해주면 설치는 완료된다.

```
$ sudo add-apt-repository ppa:gnome-terminator
$ sudo apt-get update
$ sudo apt-get install terminator  
```
설치가 완료된 모습은 아래와 같다. sticky notes를 실행하면 우측 상단에 메모장 모양의 아이콘이 뜨게 된다. 이를 클릭하면 메모 개수를 추가할 수도 있고, 색상도 바꿀 수 있다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-21-[ubuntu]_ubuntu에_sticky_notes_설치하기/sticky_notes.png" alt=""> 