clear all
bag = rosbag("/home/gopisainath/EECE5554/LAB3/src/Data/data.bag");
topic = select(bag, 'Topic', '/imu');
msgs = readMessages(topic , 'DataFormat','struct');

accel_x = cellfun(@(i) double(i.IMU.LinearAcceleration.X),msgs);
accel_y = cellfun(@(i) double(i.IMU.LinearAcceleration.Y),msgs);
accel_z = cellfun(@(i) double(i.IMU.LinearAcceleration.Z),msgs);
timestamp = cellfun(@(i) double(i.Header.Stamp.Sec),msgs);
accel_x = accel_x(6442:6527)    %this is for video snippet of 3:50 to 3:53 mins 
accel_y = accel_y(6442:6527)    % considered only the part for which we took video snippets
accel_z = accel_z(6442:6527) 
timestamp = timestamp(6442:6527)
figure(1);
subplot(2,2,1);
plot(timestamp,accel_x);
xlabel('time in sec')
ylabel(' GYRO X in rad/sec^2')
title('stationary data time series plot for gyro X')
grid on

subplot(2,2,2);
plot(timestamp,accel_y);
xlabel('time in sec')
ylabel(' GYRO Y in rad/sec^2')
title('stationary data time series plot for gyro y')
grid on

subplot(2,2,3);
plot(timestamp,accel_z);
xlabel('time in sec')
ylabel(' GYRO Z in rad/sec^2')
title('stationary data time series plot for gyro z')
grid on

subplot(2,2,4);
plot(timestamp,accel_x, 'r',timestamp,accel_y,'g',timestamp,accel_z,'b');
xlabel('time in sec')
ylabel(' accel X, Y, Z in m/s^2')
title('stationary data time series plot for accel x, y, z')
grid on