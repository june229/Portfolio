Position
다음 코드는 드론이 고도 10m에 위치한 지점에서, x방향으로 5m, y방향으로 10m 이동하는 자율비행 코드입니다.

python
Copy code
trajectory_msg = TrajectorySetpoint()
trajectory_msg.position[0] = 5.0  # x
trajectory_msg.position[1] = 10.0  # y
trajectory_msg.position[2] = -10.0  # z (고도, 음수는 위쪽)
Velocity
다음 코드는 드론이 x방향으로 2m/s, y방향으로 4m/s의 속도로 이동하는 자율비행 코드입니다.

python
Copy code
trajectory_msg = TrajectorySetpoint()
trajectory_msg.velocity[0] = 2.0  # vx
trajectory_msg.velocity[1] = 4.0  # vy
trajectory_msg.velocity[2] = 0.0  # vz
Acceleration
다음 코드는 드론이 x방향으로 2m/s^2, y방향으로 4m/s^2의 가속도로 이동하는 자율비행 코드입니다.

python
Copy code
trajectory_msg = TrajectorySetpoint()
trajectory_msg.acceleration[0] = 2.0  # ax
trajectory_msg.acceleration[1] = 4.0  # ay
trajectory_msg.acceleration[2] = 0.0  # az
Jerk
다음 코드는 드론이 x방향으로 2m/s^3, y방향으로 4m/s^3의 제르크로 이동하는 자율비행 코드입니다.

python
Copy code
trajectory_msg = TrajectorySetpoint()
trajectory_msg.jerk[0] = 2.0  # jx
trajectory_msg.jerk[1] = 4.0  # jy
trajectory_msg.jerk[2] = 0.0  # jz
Thrust
다음 코드는 드론이 50%의 추력으로 비행하는 자율비행 코드입니다.

python
Copy code
trajectory_msg = TrajectorySetpoint()
trajectory_msg.thrust = 0.5  # 0 ~ 1 사이의 값
Yaw
다음 코드는 드론이 45도 방향을 바라보도록 하는 자율비행 코드입니다.

python
Copy code
trajectory_msg = TrajectorySetpoint()
trajectory_msg.yaw = math.radians(45)  # yaw 값은 라디안 단위로 입력
Yaw Rate
다음 코드는 드론이 30도/초의 각속도로 회전하는 자율비행 코드입니다.

python
Copy code
trajectory_msg = TrajectorySetpoint()
trajectory_msg.yaw_rate = math.radians(30)  # yaw_rate 값은 라디안/초 단위로 입력
위의 예시 코드는 TrajectorySetpoint