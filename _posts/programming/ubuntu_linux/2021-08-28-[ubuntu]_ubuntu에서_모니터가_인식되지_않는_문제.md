---
title:  "[ubuntu] ubuntu에서 모니터가 인식되지 않는 문제"
toc: True
toc_label: "목차"
toc_sticky: True
classes: wide
categories:
  - Programming
tags:
  - Ubuntu
last_modified_at: 2021-08-28
---

## ubuntu 모니터 인식 문제
어제까지 잘 사용했었는데, 오늘 컴퓨터를 켜보니 듀얼 모니터 중 1개는 화면이 아예 나오지 않고, 나머지 1개는 화면은 나오지만 해상도가 바뀌어 있었다. 설정-디스플레이에 들어가보니 이전에는 모니터의 정확한 모델명까지 인식했었는데 오늘은 '알수없는 모니터'라고 인식하고 있었고, 해상도 변경도 불가능했다.

원인이 무엇일까 고민해보니, 어제 apt update를 하고 나서 사용하지 않는 드라이버가 있으니 삭제하라는 안내 메시지가 떴고, 안내에 따라 autoremove를 했던게 떠올랐다. 그 사용하지 않는 드라이버 목록에 `nvidia ...` 라고 시작하는 이름의 드라이버가 있었던 것 같다. 따라서 nvidia 드라이버를 삭제한 것이 원인인 것으로 추정된다.

## 해결방법

아래 명령어를 순차적으로 입력해주는 것으로 nvidia 드라이버를 새로 설치할 수 있었다.

```
sudo apt update

sudo apt upgrade

sudo ubuntu-drivers autoinstall
```

설치에는 시간이 꽤 소요되었고, 설치 완료 후, 재부팅을 해주니 문제가 해결되었다.

```
sudo reboot
```

함부로 드라이버를 삭제하면 안되겠다는 것을 느낄 수 있었다. 

의문이 드는 것은 드라이버가 없으면 모니터 인식이 안되는 문제가 발생하는데 왜 사용하지 않는 드라이버이니 삭제하라는 메시지가 떴을까?

nvidia 드라이버가 이미 설치되어 있는 경우, 아래와 같이 기존에 설치된 드라이버를 삭제 후, 재설치 해준다.

```
sudo apt update

sudo apt upgrade

sudo apt autoremove

sudo apt-get remove --purge nvidia*

sudo ubuntu-drivers autoinstall

sudo reboot
```
