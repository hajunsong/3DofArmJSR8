1. ~/catkin_ws/src 로 이동해서 github 에서 프로젝트 다운로드
~~~
cd ~/catkin_ws/src
git clone git clone https://github.com/KETI-AN/Robot_System.git
~~~
![move_dir_catkin_workspace](./image/move_dir_catkin_workspace.png)

2. ~/catkin_ws 위치에서 catkin_make 로 빌드
~~~
cd ~/catkin_ws/src
catkin_make
~~~
![catkin_make_1](./image/catkin_make_1.png)
![catkin_make_2](./image/catkin_make_2.png)

3. sawyer client ui node 실행
~~~
rosrun sawyer_control sawyer_client_ui
~~~
![sawyer_client_ui_node_run](./image/sawyer_client_ui_node_run.png)

4. 현재 PC의 IP 주소 입력 후 Listen 클릭
5. 터미널 실행 후 ~/ros_ws 폴더로 이동 후 sawyer 로봇 팔에 접속
~~~
cd ~/ros_ws
./intera.sh sawyer_IP_address
~~~
![sawyer_connect_exam](./image/sawyer_connect_exam.png)

6. sawyer client 실행
- 이 때 XX 는 앞서 입력한 sawyer_IP_address의 끝 번호 입력
~~~
source ~/catkin_ws/devel/setup.bash
rosrun sawyer_control sawyer_client.py XX
~~~

7. 5 ~ 6 을 반복하여 또다른 sawyer 로봇 팔에 접속
- sawyer client 실행 시 ui와 바로 연결 되므로 ip address 10 부터 시행하고 그 다음 11을 실행한다.

8. 두 개의 sawyer에 정상적으로 접속되면 ui에서 비활성화 
![ui_enabled](./image/ui_enabled.png)

9. Joint 1 ~ 7 에 원하는 값을 입력하고 Move 버튼을 누르면 입력 된 값에 따라서 두 sawyer 로봇 팔이 움직인다.
![sawyer_move_1](./image/sawyer_move_1.png)
![sawyer_move_2](./image/sawyer_move_2.png)