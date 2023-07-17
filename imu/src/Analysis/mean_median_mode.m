bag = rosbag("/home/gopisainath/EECE5554/LAB3/src/Data/data.bag");
topic = select(bag, 'Topic', '/imu');
msgs = readMessages(topic , 'DataFormat','struct');

accel_x = cellfun(@(i) double(i.IMU.LinearAcceleration.X),msgs);

medain_value = median(accel_x)
mean_value = mean(accel_x)
std_deviation = std(accel_x)