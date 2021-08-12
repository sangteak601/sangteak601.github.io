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
last_modified_at: 2021-08-12
---

## 외장하드에 ubuntu 설치하기
외장하드에 ububtu를 설치하여 사용하는 방법에 대해 정리한다. 기존에도 외장하드에 ubuntu를 설치하여 사용하고 있었으나, 최신버전이 아닌 18.04.5 LTS 버전을 사용 중이었다. 그런데 종종 인터넷이 끊기는 문제가 발생했고, 최신버전으로 업데이트하면 개선되지 않을까? 하는 기대로 최신 LTS 버전인 20.04.2 LTS 버전으로 설치하기로 했다.

준비물: usb(4GB 이상), 외장하드

## 부팅 usb 만들기
1. ubuntu 다운로드  

ubuntu 홈페이지에 접속하여 Download > Ubuntu Desktop 경로로 들어간다. Download 버튼을 클릭하여 다운로드하면 된다. 버전은 LTS 버전으로 받는 것이 좋다. LTS란 'Long Term Support' 의 약자인데, 장기간동안 지원한다는 것을 의미한다. 현재는 5년 동안 지원한다고 한다.  

//ubuntu 사이트 이미지

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

//BIOS 사진

ubuntu를 설치한 usb 드라이브(나의 경우 'SanDisk')를 선택한다. 이전에 ubuntu를 설치하여 사용했었다보니 ubuntu를 삭제했는데도 목록에 여전히 ubuntu가 남아있었다.  
어쨋든 usb 드라이브를 선택하여 부팅하면 바로 부팅이 되지 않고, 'checking disk' 라고 뜨면서 시간이 조금 소요되었다.

// checking file 사진

4. ubuntu 설치하기

------------------수정 필요-------------------------------------