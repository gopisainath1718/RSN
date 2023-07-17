bag = rosbag("/home/gopisainath/EECE5554/LAB3/src/Data/data.bag");
topic = select(bag, 'Topic', '/imu');
msgs = readMessages(topic , 'DataFormat','struct');

acc_x = cellfun(@(i) double(i.IMU.LinearAcceleration.X),msgs);

figure(1)
histogram(acc_x)
xlabel('histogram accel X (m/s^2)')
ylabel('frequency count')
