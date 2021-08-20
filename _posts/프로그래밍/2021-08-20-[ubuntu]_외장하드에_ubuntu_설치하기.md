---
title:  "[ubuntu] 외장하드에 ubuntu 설치하기"
excerpt: "GitHub Blog 서비스인 github.io 블로그 시작하기로 했다."
toc: True
toc_label: "목차"
toc_sticky: True
categories:
  - 프로그래밍
tags:
  - ubuntu
last_modified_at: 2021-08-20
---

## 외장하드에 ubuntu 설치하기
외장하드에 ububtu를 설치하여 사용하는 방법에 대해 정리한다. 기존에도 외장하드에 ubuntu를 설치하여 사용하고 있었으나, 최신버전이 아닌 18.04.5 LTS 버전을 사용 중이었다. 그런데 종종 인터넷이 끊기는 문제가 발생했고, 최신버전으로 업데이트하면 개선되지 않을까? 하는 기대로 최신 LTS 버전인 20.04.2 LTS 버전으로 설치하기로 했다.

준비물: usb(4GB 이상), 외장하드

## 부팅 usb 만들기
1. ubuntu 다운로드  

ubuntu 홈페이지에 접속하여 Download > Ubuntu Desktop 경로로 들어간다. Download 버튼을 클릭하여 다운로드하면 된다. 버전은 LTS 버전으로 받는 것이 좋다. LTS란 'Long Term Support' 의 약자인데, 장기간동안 지원한다는 것을 의미한다. 현재는 5년 동안 지원한다고 한다.  

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_download.png" alt=""> 

2. 부팅 usb 만들기  

ubuntu로 부팅하기 위한 부팅 usb를 만들어야 한다. 1에서 받은 ubuntu 파일이 약 3GB 정도의 용량이므로, 4GB 이상의 usb가 필요하다.  
부팅 usb를 만들기 위해서는 'Linuxlive USB creator' 가 필요하다.  
<http://www.linuxliveusb.com> 에 접속하여 Download tab으로 이동 후, Download LiLi 버튼을 클릭하여 다운로드한다.  

//linuxliveusb 사이트 이미지  

usb를 pc에 연결하고, 다운로드한 Linuxlive USB creator를 실행한다. 아래 사진과 같이 설정해준 뒤, 설치를 시작하면 된다. 약 10분 정도 소요되고, 컴퓨터 사양에 따라 30분까지 소요될 수도 있다고 한다.  

//Linuxlive USB creator 설정사진

> **참고**  
1단계: 설치하고자 하는 위치 설정(usb 메모리)  
2단계: 설치하고자 하는 파일의 형식 설정(iso 파일)  
3단계: 퍼시스턴스 데이터 크기(live CD(OS 설치용 부팅 CD)는 일반적으로 재부팅시 수정사항이 모두 초기화되는데, 재부팅시에도 초기화되지 않는 데이터를 퍼시스턴스 데이터라 한다.)  
4단계:  
USB키에 생성된 파일 숨김 - 생성한 파일을 Windows 탐색기에서 볼 수 없는 숨김 파일로 변경  
FAT32로 USB 드라이브 포맷(USB 데이터 삭제됨.) - USB 포맷  
5단계: 설치 시작

3. usb 드라이브로 부팅

컴퓨터를 재부팅하면서 BIOS로 진입한다. 메인보드 제조사에 따라 BIOS 진입방법은 상이하므로, 각자 본인에게 맞는 방법을 찾아야 한다. 나는 GIGABYTE의 메인보드를 사용하고 있고, 부팅시에 F12 키를 눌러 부팅 디바이스를 선택하는 창으로 진입했다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_choose_disk.png" alt="">

ubuntu를 설치한 usb 드라이브(나의 경우 'SanDisk')를 선택한다. 이전에 ubuntu를 설치하여 사용했었다보니 ubuntu를 삭제했는데도 목록에 여전히 ubuntu가 남아있었다.  
어쨋든 usb 드라이브를 선택하여 부팅하면 바로 부팅이 되지 않고, 'checking disk' 라고 뜨면서 시간이 조금 소요되었다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_file_check.png" alt="">

4. ubuntu 설치하기

부팅이 완료되면 아래 사진과 같이 'Ubuntu 체험하기'와 'Ubuntu 설치' 를 선택할 수 있는 화면이 나온다. 바로 설치하기 위해서 'Ubuntu 설치'를 클릭한다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_language.png" alt="">

설치와 관련된 몇가지 설정을 할 수 있는 화면이 나온다. 나는 일반 설치를 선택하고, 업데이트도 어차피 할 것이기 때문에 업데이트도 동시에 진행할 수 있도록 'Ubuntu 설치 중 업데이트 다운로드' 도 체크해주었다. 그리고 이전에 Ubuntu를 설치할 때, 화면이 안나오는 문제가 있었고, 그 원인이 그래픽 카드였던 것으로 기억하고 있어서 '서드파티 소프트웨어 설치'도 체크해주었다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_install_choose.png" alt="">

설치형식은 기타로 선택하여 직접 설치 위치를 지정할 수 있도록 했다. 많은 사람들이 파티션을 나누어, 데이터 저장공간과 운영체제를 위한 공간을 분리하는 것 같았다. 그러나 나는 중요 데이터가 별로 없고, 이전에는 파티션을 나눠서 설치해봤으니 이번에는 파티션을 따로 나누지 않기로 했다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_install_choose2.png" alt="">

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_partition.png" alt="">

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_partition3.png" alt="">

바뀐 것을 덮어쓰고, 간단한 내용 몇가지를 입력해주면 본격적으로 설치가 시작된다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_partition2.png" alt="">

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_install.png" alt="">

꽤 오랜 시간이 지나고, 설치가 완료되면 재부팅을 하라고 알림이 뜬다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_install_finish.png" alt="">

이제 부팅을 하면 부팅방법을 선택할 수 있는 창이 뜬다. 아무것도 선택하지 않으면 자동으로 Ubuntu로 부팅이 된다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_choose_ubuntu.png" alt="">

부팅이 완료되면 로그인할 수 있는 화면이 뜬다.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/2021-08-20-[ubuntu] 외장하드에_ubuntu_설치하기/ubuntu_login.png" alt="">

이전 버전에 비해 UI가 훨씬 고급스러워진 것 같은 느낌이다.

이것으로 Ubuntu 설치가 완료되었다. 파티션을 나누는 부분에 대해서는 필요한 경우 업데이트하도록 하겠다.