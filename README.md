# Regala 설치 및 운영 가이드
현재 경기장 영상을 외부에서 가져올 수 있습니다.

# 설치
NUC PC를 사용한다는 가정

기존 regala_ws 폴더 삭제 후,

```
# 워크스페이스 생성
$ mkdir -p ~/regala_ws/src
$ cd ~/regala_ws
$ catkin_make

# 소스코드 클론 후 빌드
$ cd ~/regala_ws/src
$ git clone https://github.com/Taemin0707/Regala.git
$ cd ~/regala_ws
$ catkin_make
```

# 사용 방법
1. 터미널에서 `roscore` 를 실행
2. 터미널 하나 더 생성 후 `roslaunch regala_ros ptz.launch` 를 실행
3. 터미널 하나 더 생성 후 `rqt` 를 실행해서 이미지 확인
4. 영상 저장은 `python3 video_recorder.py`
5. 영상 업로드는 `python3 google_drive_test.py`
6. 영상을 다루는 것은 ROS 이미지 받아서 코드 내부에서 CV 패키지를 사용하면 됩니다.
<br>[참고.1](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
<br>[참고.2](http://wiki.ros.org/image_transport/Tutorials)

7. 카톡으로 문의하시면 알려드릴게요.
