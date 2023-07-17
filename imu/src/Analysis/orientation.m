clear all
bag = rosbag("/home/gopisainath/EECE5554/LAB3/src/Data/data.bag");
topic = select(bag, 'Topic', '/imu');
msgs = readMessages(topic , 'DataFormat','struct');
msgs{1}.IMU.Orientation.X
ox = cellfun(@(i) double(i.IMU.Orientation.X),msgs);
oy = cellfun(@(i) double(i.IMU.Orientation.Y),msgs);
oz = cellfun(@(i) double(i.IMU.Orientation.Z),msgs);
ow = cellfun(@(i) double(i.IMU.Orientation.W),msgs);
timestamp = cellfun(@(i) double(i.Header.Stamp.Sec),msgs);

qt = [ow,ox,oy,oz];
eulXYZ = quat2eul(qt, "XYZ");
eulXYZ = rad2deg(eulXYZ);
X = eulXYZ(:,1);
Y = eulXYZ(:,2);
Z = eulXYZ(:,3);

figure(1);
subplot(2,2,1);
plot(timestamp,X);
xlabel('time in sec')
ylabel(' orientation X in rad/sec')
title('stationary data time series plot for X')
grid on

subplot(2,2,2);
plot(timestamp,Y);
xlabel('time in sec')
ylabel('X in rad/sec')
title('stationary data time series plot for Y')
grid on

subplot(2,2,3);
plot(timestamp,Z);
xlabel('time in sec')
ylabel('Y in rad/sec')
title('stationary data time series plot for z')
grid on

subplot(2,2,4);
plot(timestamp,X, 'r',timestamp,Y,'g',timestamp,Z,'b');
xlabel('time in sec')
ylabel('Z in rad/sec')
title('stationary data time series plot for x, y, z')
grid on