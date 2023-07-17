bag = rosbag("/home/gopisainath/EECE5554/LAB3/src/Data/data.bag");
topic = select(bag, 'Topic', '/imu');
msgs = readMessages(topic , 'DataFormat','struct');
% msgs.IMU
gyro_x = cellfun(@(i) double(i.IMU.AngularVelocity.X),msgs);
gyro_y = cellfun(@(i) double(i.IMU.AngularVelocity.Y),msgs);
gyro_z = cellfun(@(i) double(i.IMU.AngularVelocity.Z),msgs);
timestamp = cellfun(@(i) double(i.Header.Stamp.Sec),msgs);

figure(1);
subplot(2,2,1);
plot(timestamp,gyro_x);
xlabel('time in sec')
ylabel(' GYRO X in rad/sec^2')
title('stationary data time series plot for gyro X')
grid on

subplot(2,2,2);
plot(timestamp,gyro_y);
xlabel('time in sec')
ylabel(' GYRO Y in rad/sec^2')
title('stationary data time series plot for gyro y')
grid on

subplot(2,2,3);
plot(timestamp,gyro_z);
xlabel('time in sec')
ylabel(' GYRO Z in rad/sec^2')
title('stationary data time series plot for gyro z')
grid on

subplot(2,2,4);
plot(timestamp,gyro_x, 'r',timestamp,gyro_y,'g',timestamp,gyro_z,'b');
xlabel('time in sec')
ylabel(' GYRO X, Y, Z in rad/sec^2')
title('stationary data time series plot for gyro x, y, z')
grid on
