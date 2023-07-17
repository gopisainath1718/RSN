bag = rosbag("/home/gopisainath/EECE5554/LAB3/src/Data/LocationA.bag");

topic = select(bag, 'Topic', '/vectornav');
msgs = readMessages(topic, 'DataFormat', 'Struct');

gyroData = zeros(length(msgs), 3);
timestamps = zeros(length(msgs), 1);
for i = 1:length(msgs)
   msg = msgs{i}.Data;
   C = strsplit(msg, ',');
   if length(C) >= 13 % Check if C has at least 6 elements
      x = C{13}(1:10);
      gyroData(i,:) = [str2double(C{11}), str2double(C{12}), str2double(x)];
      timestamps(i) = msgs{i}.Header.Stamp.Sec + msgs{i}.Header.Stamp.Nsec*1e-9;
   end
end

figure();
subplot(2,2,1);
plot(timestamps,gyroData(:,1));
xlabel('time in sec')
ylabel(' GYRO X in rad/sec^2')
title('stationary data time series plot for gyro X')
grid on

subplot(2,2,2);
plot(timestamps,gyroData(:,2));
xlabel('time in sec')
ylabel(' GYRO Y in rad/sec^2')
title('stationary data time series plot for gyro y')
grid on

subplot(2,2,3);
plot(timestamps,gyroData(:,2));
xlabel('time in sec')
ylabel(' GYRO Z in rad/sec^2')
title('stationary data time series plot for gyro z')
grid on

subplot(2,2,4);
plot(timestamps,gyroData(:,1), 'r',timestamps,gyroData(:,2),'g',timestamps,gyroData(:,3),'b');
xlabel('time in sec')
ylabel(' GYRO X, Y, Z in rad/sec^2')
title('stationary data time series plot for gyro x, y, z')
grid on