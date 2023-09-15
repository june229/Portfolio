# Ros-development
For studying
this is first edit3

- - -

## 2023_02_13

- - -

* first
  * turtlebot3
  * Laptop ubuntu 20.04.01 LTS

* second
  * ROS2 DDS explain

* third

```shell
  * ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 2.0, y: 0.0, z: 1.8}}'
  # I can control turtle1 and turtle2

  * ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 9, theta: 1.57, name: 'turtle2'}"
  # I can make service2 to control another turtlebot
```

# 개념정리

- .json파일 만들기 : 원하는 폴더 내의 터미널 창에 'touch settings.json'이라고 입력한다.

- 파일 안에는 숨김파일이라는 항목이 존재한다. 체크해주면 .bashrc 등의 파일이 나타난다.

- Terminator 사용법 - Ctrl + shift +
e : 좌우 분할, w : 터미널 닫기, o : 상하 분할, c : 복사, v : 붙여넣기

    Alt + 방향키 : 터미널 이동

- 명령어 힌트 찾기 : ros2 run turtlesim ~~~ ----> Tap키를 두 번 누른다.

- VisualStudio Code, Github와 연동하기 : git config --global user.email "    "

    git config --global user.name"    "

- 수정 개념 : commit(문서 상 수정), push(git으로 전송), pull(git에서 pc로 불러오기)

- cat : cat memoji - 파일 안의 내용 보기

- home 표현 : ~/.

- alias : alias testpub = ros2 run ~~~ // ros2 run 이하의 명령어를 testpub으로 단축

- 노드 이름 바꾸기 : ros2 run turtlesim turtlesim_node --ros -args -remap -node :=이름

    //그냥 실행하면 turtlesim이라는 이름으로 생성. 그러므로 여러대 동작시 이름 변경 필요

- 실행은 run

- 거북이 2마리 따로 조종시 서비스 이름 바꾸기

- rqt 모니터, rqt service caller 등 사용

- 노드 정보 확인 : ros2 node info/이름

- 파라미터 불러오기 및 설정 : ros2 param get /turtlesim background_r
//불러오기

  ros2 param set /turtlesim background_r//설정

  저장 : ros2 param dump /turtlesim ---->.yaml 형태로 저장
  ---->이를 확인하기 위해서는 cat turtlesim.yaml

  저장 파일 불러오기 : ros2 param load /turtlesim ./ turtlesim.yaml // turtlesim에 turtlesim.yaml의 정보를 불러온다.

  저장된 것을 바로 적용하여 실행 : ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml


- ros의 단위는 m 이다.
- https://github.com/freshmea
- https://freshmea.notion.site/freshmea/ROS2-5a5303ac2160454885498a52dfce26c4
#
#
# 2023.02.14.
# class
- class는 틀이다. class에는 여러 기능적인 함수 뿐만 아니라 변수들 또한 가지게 된다.

  ex) 학생정리 : 각각의 학생에게는 나이와 성격, 성적 등 다양한 것이 있다. 여러 학생들을 전부 선언하기 위하여 여러 변수를 만들게 되면 매우 힘들다. 이에 정해진 틀을 만들어 놓고 적용하기 위해 class를 사용한다.

    ob=Test(), ob2=Test()


# pkg
- pkg에는 노드, yaml 등이 있고 launch를 통해 이들을 조작한다.
- src 파일 안에서 pkg를 만든다. --> "ros2 pkg create --build-type ament_python my_package"

  빌드타입, 사용하는 파이썬, 패키지 이름

- 빌드는 'colcon build'라고 쓴다.

# pkg 파일 생성
- robot_ws에서 src에 다음과 같은 명령어를 입력한다. 'ros2 pkg create --build-type ament_python my_package'

# class
- class M_turtle(Node): #상속을 받기위해 가로 안에 부모 class를 작성한다.
  def __init__(self): #상속을 하기 위해서는 이를 작성해야 한다.
    super().__init__('move_turtle') # mpub는 토픽의 이름을 설정한 것이다.
    self.qos = QoSProfile(depth = 10)
    self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', self.qos) #message는 통신하는 토픽의 이름으로 pub과 sub의 내용이 동일하다.


# Publisher와 Subscriber
- 노드 등록 하는 방법 설명 : setup.py 수정 'mp = my_package.mpub:main'
- mpub.py, msub.py
- mpub.py 이용하여 터틀심 조종하기
- Publisher : class 이름 설정, topic 이름 설정, 통신하는 message 이름 설정, 내용을 publish로 설정
- Subscriber : 주고 받을 message 이름 동일화 그리고 내용을 subscribe로 설정

# launch
- run을 거의 사용하지 않고 대부분 launch를 사용한다. node를 하나하나 사용하지 않고 launch에 묶어서 여러 노드를 사용한다.

# 기타 수정사항
- 코드가 주고 받는 내용의 변수명을 꼭 제대로 확인한다.
- 파일을 실행시킬 때는 저장 후 사용한다.

# 2023.02.15
- 군집을 위해 드론에 부착해야 하는 작은 컴퓨터 : STM32, ESP32 -->microros(아두이노와 유사)를 사용

  --> ros middleware(rmw)는 라즈베리파이 같은 고사양 컴퓨터에서 사용되는데 센서의 정보를 토픽으로 보내기 위해 OpenCV 등과 같은 소형 컴퓨터에 rmw의 중요사항만 뽑아낸 microros를 설치하여 rmw에 토픽 메시지를 보낸다.

  이와 같은 형태는 모든 형태의 드론에 적용된다. 센서 같은 정보는 작은 컴퓨터(OpenCV)에, 통신과 같은 주요 작업은 메인 컴퓨터(라즈베리파이 등)에서 사용한다.

    또한 드론은 라즈베리파이 대신 노트북을 사용한다.

- SD카드 설정 : 우분투는 두 가지 유형이 있다. GUI버전과 Text버전이 있다.(Desktop, Server)

  로봇에는 Text버전(Server)을 사용한다.

- 로봇 조작 위한 pc 설정

  ROS 의 설치는 이미 되어 있으므로 turtlebot3 에 관련된 설치 파일만을 진행 한다.

- 가제보는 ROS2에서 제공하는 시뮬레이션 모듈이다. 매우 유용한 기능이 많으므로 사용법을 숙지해 두는게 좋고 로봇 시뮬레이션을 사용하기 위해서는 URDF 작성법도 같이 익혀 두는게 좋다.

```bash
sudo apt-get install ros-foxy-gazebo-*
```

- 카토그래퍼는 ROS2에서 지원하는 SLAM 의 하나로 터틀봇3 에서는 라이다 센서 를 기반으로 지도를 제작하게 된다.

```bash
sudo apt install ros-foxy-cartographer
sudo apt install ros-foxy-cartographer-ros
```

- 제작한 지도를 바탕으로 터틀봇을 움직이기 위해 설치하는 모듈이다. BT(behavior Tree )를 기반으로 잘 짜여진 네비게이션 모듈이다.

```bash
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup
```

- 터틀봇을 구동하기 위해 필요한 패키지들을 설치한다. 데비안 패키지 형식의 터틀봇3 와 터틀봇3 메시지 그리고 다이나믹셀 sdk 를 설치한다.

```bash
source ~/.bashrc
sudo apt install ros-foxy-dynamixel-sdk
sudo apt install ros-foxy-turtlebot3-msgs
sudo apt install ros-foxy-turtlebot3
```

1. 터틀봇3 도메인 아이디를 설정한다.( .bashrc 파일을 에디터로 열어서 중복되지 않게 설정을 확인한다.)

  ```bash
  echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc$ source ~/.bashrc
  ```


# sd카드 설정
- 디스크에 들어가 sd카드를 선택하고 우측 상단에 점 3개를 눌로 디스크 이미지 복구, 이후 다운 받은 Server파일을 선택하여 복구를 시작한다.

    이후 ros를 설치하여 용도에 맞게 Robot을 조작할 수 있도록 구성한다.

# wifi 설정
- 터미널에 'sudo nano 50-cloud-init.yaml'을 입력하여 설정한다.

  이후 파일이 있는 폴더(etc/netplain)에서 와이파이 아이디(제목)와 비밀번호를 설정하고 sd카드를 로봇에 장착한다.

- GCS와 Robot의 ip확인.

  ifconfig를 터미널에 입력하면 와이파이에 연결된 노트북의 ip번호를 확인할 수 있다.

  Robot의 ip를 확인하기 위해서는 192.168.0.1을 통해 연결된 Robot의 ip를 확인한다.

- ip를 통한 Robot조작 : 터미널에 'ssh ubuntu@로봇ip'를 입력하여 연결시킨다.

  터미널을 통해 Robot과 연결되면 터미널에서 Robot의 터미널을 조작할 수 있다.

  sudo nano .bashrc에서 도메인 id를 설정할 수 있다.(다중 접속시 따로 조종 위해)

  GCS의 .bashrc로 들어가 도메인 id를 동일화 한다.

  또한 source .bashrc를 robot 터미널에 입력한다.

- turtlebot 모델 설정

  export TURTLEBOT3_MODEL=burger

- Turtlebot 설정

  ros2 launch turtlebot3_bringup robot.launch.py


- 통신 확인 : 'ros2 topic list'

- 키보드로 조작 : 'ros2 run turtlebot3_teleop  teleop_keyboard'

- move_turtle을 통한 turtlebot 제어

# 2023.02.16

- turtlebot 움직임의 상호작용 : 위치 정보를 위해 imu, odom이라는 토픽을 사용한다.

  처음 실행이 되는 장소는 x, y,가 0에서 시작한다.

IMU : 각속도에 의한 계산값

ODOM : 이동한 바퀴에 의한 계산값.

        둘 중에 원하는 값을 사용함.


- SLAM : 실제지도, 실제 상황을 만든다. 즉, 특정한 상황만이 아닌 실제 상황을 반영한다.
  - 사용할 SLAM : cartographer

# Lidar 사용하기
- 시뮬레이션을 킨 후에 cartographer를 실행시키는 launch파일을 통해 rviz로 turtlebot의 상황을 파악한다.('ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True'-launch 파일 내 노드 분석하기!!)-https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/

- 지도 완성 후 navigating 하기 : 'ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml'

# 2023.02.17

- usb를 꽂고 확인하기 위해서는 터미널 창에 'ls /dev'를 입력한다. 그러면 인식된 usb를 확인할 수 있다.

- 8-5-3. Turtlebot3_manipulation 을 이용한 연결 : turtlebot을 통해 그리퍼를 조종가능

- 상황에 따른 자동 손 조작 : opencv

- Moveit : moveit 2 foxy 홈페이지.
