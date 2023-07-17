clear;
close all;
circle_bag = rosbag("/home/gopisainath/EECE5554/LAB4/src/Data/data_going_in_circles.bag");
driving_bag = rosbag("/home/gopisainath/EECE5554/LAB4/src/Data/data_driving.bag");

circle_topic = select(circle_bag, 'Topic', '/imu');
driving_topic_imu = select(driving_bag, 'Topic', '/imu');
driving_topic_gps = select(driving_bag, 'Topic', '/gps');
circle_msgs = readMessages(circle_topic, 'DataFormat', 'Struct');
driving_msgs_imu = readMessages(driving_topic_imu, 'DataFormat', 'Struct');
driving_msgs_gps = readMessages(driving_topic_gps, 'DataFormat', 'Struct');

mag_x_vals = cellfun(@(i) double(i.MagField.MagneticField_.X),circle_msgs);
mag_y_vals = cellfun(@(i) double(i.MagField.MagneticField_.Y),circle_msgs);
mag_z_vals = cellfun(@(i) double(i.MagField.MagneticField_.Z),circle_msgs);

mag_x_vals_dr = cellfun(@(i) double(i.MagField.MagneticField_.X),driving_msgs_imu);
mag_y_vals_dr = cellfun(@(i) double(i.MagField.MagneticField_.Y),driving_msgs_imu);
mag_z_vals_dr = cellfun(@(i) double(i.MagField.MagneticField_.Z),driving_msgs_imu);


figure(1)
[xfit,yfit,Rfit] = circfit(mag_x_vals,mag_y_vals);
plot(mag_x_vals,mag_y_vals, 'color','green', 'Marker','*')
hold on
rectangle('position',[xfit-Rfit,yfit-Rfit,Rfit*2,Rfit*2],'curvature',[1,1],'linestyle','-','edgecolor','b');
axis equal
grid on;

xlabel('magnetic field x (Gauss)')
ylabel('magnetic field y (Gauss)')
title('magnetic field in y vs magnetic field in x')

ellipse_t = fit_ellipse(mag_x_vals,mag_y_vals);
syms x y 
% ellipse axis 
a=ellipse_t.long_axis/2;   
b=ellipse_t.short_axis/2;
%ellipse center
h=ellipse_t.X0_in; k=ellipse_t.Y0_in;
%ellipse equation
ellipse= (((x-h)^2)/(a^2))+(((y-k)^2)/(b^2))==1;
%plot the ellipse
% plotZoom=(max(a,b)+max(abs(h), abs(k)))+1;
% fimplicit(ellipse, [-plotZoom plotZoom],'color', 'black'); 
%plot([-plotZoom plotZoom], [0 0], '-k ');
%plot([0 0 ], [-plotZoom plotZoom], '-k');
%plot(h, k, 'b*');

xlabel('magnetic field x (Gauss)')
ylabel('magnetic field y (Gauss)')
title('magnetic field in y vs magnetic field in x')

axis equal;

%______________________________________

offsetx = ellipse_t.X0_in;
offsety = ellipse_t.Y0_in;
mag_x_vals_transl = mag_x_vals-offsetx;
mag_y_vals_transl = mag_y_vals-offsety;

ellipse_t = fit_ellipse(mag_x_vals_transl,mag_y_vals_transl);
angle = ellipse_t.phi;

rotationmat = [cos(angle), sin(angle);...
-sin(angle), cos(angle)];
Mag_x_valy = [mag_x_vals_transl, mag_y_vals_transl];
Mag_x_valy_rot = Mag_x_valy * rotationmat;
Mag_x_val_rot = Mag_x_valy_rot(:,1);
Mag_y_val_rot = Mag_x_valy_rot(:,2);
ellipse_t = fit_ellipse(Mag_x_val_rot,Mag_y_val_rot);

tau = (ellipse_t.short_axis/2)/(ellipse_t.long_axis/2);
rescaling_mat = [tau,0;0,1];

Mag_x_valy_rot = Mag_x_valy_rot*rescaling_mat;

Mag_x_val_final = Mag_x_valy_rot(:,1);
Mag_y_val_final = Mag_x_valy_rot(:,2);
plot(Mag_x_val_final, Mag_y_val_final, 'color', 'yellow', 'Marker','o')
grid on;
axis equal;
[xfit,yfit,Rfit] = circfit(Mag_x_val_final,Mag_y_val_final);
rectangle('position',[xfit-Rfit,yfit-Rfit,Rfit*2,Rfit*2],'curvature',[1,1],'linestyle','-','edgecolor','r');

legend('Before correction', 'After correction');
hold off;

%----------------time series mag data before and after correction--------------------------------
Mag_x_valdr_tarnsl = mag_x_vals_dr - offsetx;
Mag_y_valdr_transl = mag_y_vals_dr - offsety;

temp = [Mag_x_valdr_tarnsl,Mag_y_valdr_transl];

Mag_x_valydr_corr = (temp*rotationmat)*rescaling_mat;
Mag_x_val_corr_dr = Mag_x_valydr_corr(:,1);
Mag_y_val_corr_dr = Mag_x_valydr_corr(:,2);

magdr_yaw_raw = (atan2(-mag_y_vals_dr,mag_x_vals_dr));
magdr_yaw_corr = (atan2(-Mag_y_val_corr_dr,Mag_x_val_corr_dr));

secs = cellfun(@(i) double(i.Header.Stamp.Sec), driving_msgs_imu);
nsecs = cellfun(@(i) double(i.Header.Stamp.Nsec), driving_msgs_imu);
time = secs + 10^(-9)*nsecs;
time = time - time(1);

figure(2)
plot(time, unwrap(magdr_yaw_corr));
grid on;
hold on;
plot(time, magdr_yaw_raw);
hold off;
legend('corrected magnetometer yaw','raw magnetometer yaw');
xlabel('time (seconds)');
ylabel('yaw (radians)');
title('Yaw vs Time');
%----------------------yaw angle comparision between mag, yaw integrated from gyro----------------------------
gyroz = cellfun(@(i) double(i.Imu.AngularVelocity.Z), driving_msgs_imu);
gyro_yaw = cumtrapz(time,gyroz);
magdr_yaw_corr = magdr_yaw_corr - magdr_yaw_corr(1);

figure(3)
plot(time,unwrap(gyro_yaw))
hold on;
grid on;
plot(time, unwrap(magdr_yaw_corr))
legend('gyroscope yaw', 'magnetometer yaw')
xlabel('time (seconds)')
ylabel('yaw (radians)')
title('Yaw from Magnetometer and Yaw integrated from Gyro')
hold off;
%--------------------------LPF-----------------------------
lowpass_magdr_yaw = lowpass(unwrap(magdr_yaw_corr),0.0001,40);
figure(4);

plot(time,unwrap(lowpass_magdr_yaw));

grid on;
hold on;
%--------------------------HPF-----------------------------
highpass_gyro_yaw = highpass(unwrap(gyro_yaw),0.07,40);

plot(time,unwrap(highpass_gyro_yaw))

%Euler Angles
qx= cellfun(@(i) double(i.Imu.Orientation.X), driving_msgs_imu);
qy= cellfun(@(i) double(i.Imu.Orientation.Y), driving_msgs_imu);
qz= cellfun(@(i) double(i.Imu.Orientation.Z), driving_msgs_imu);
qw= cellfun(@(i) double(i.Imu.Orientation.W), driving_msgs_imu);

qt = [qw,qx,qy,qz];
eulXYZ = quat2eul(qt, "XYZ");
imu_yaw = eulXYZ(:,3);
%--------------------------Complimentry-----------------------------
added_mag_gyro = highpass_gyro_yaw + lowpass_magdr_yaw;

plot(time, unwrap(added_mag_gyro))

legend('LP mag Yaw','HP Gyro Yaw', 'Compl. Filter Yaw')
xlabel('time (seconds)')
ylabel('yaw (radians)')
title('LPF Mag-Yaw HPF Gyro-Yaw CPF-Yaw')
hold off;
%Velocity imu--------------------------------------------------------------
accx = cellfun(@(i) double(i.Imu.LinearAcceleration.X), driving_msgs_imu);
accy = cellfun(@(i) double(i.Imu.LinearAcceleration.Y), driving_msgs_imu);
accz = cellfun(@(i) double(i.Imu.LinearAcceleration.Z), driving_msgs_imu);
velocity_imu = cumtrapz(time,accx);
%GPS imu------------------------------------------------------------------

secs_gps = cellfun(@(i) double(i.Header.Stamp.Sec), driving_msgs_gps);
nsecs_gps = cellfun(@(i) double(i.Header.Stamp.Nsec), driving_msgs_gps);
time_gps = secs_gps + 10^(-9)*nsecs_gps;
time_gps = time_gps - time_gps(1); 

utm_east = cellfun(@(i) double(i.UTMEasting), driving_msgs_gps);
utm_north = cellfun(@(i) double(i.UTMNorthing), driving_msgs_gps);

utm_east = utm_east - utm_east(1);
utm_north = utm_north - utm_north(1);

utm_comdine = [utm_east,utm_north];

velocity_gps= zeros(length(time_gps)-1,1);
for i = 1:length(time_gps)-1
    velocity_gps(i) = norm(utm_comdine(i+1,:)-utm_comdine(i,:))/(time_gps(i+1)-time_gps(i));
end
velocity_gps = [0,transpose(velocity_gps)];
velocity_gps= transpose(velocity_gps);
vel_gps_truesiz = velocity_gps;
velocity_gps = interp(velocity_gps,40);

figure(5)

plot(velocity_gps)
grid on;
hold on;
plot(velocity_imu)  
xlabel('time (seconds)')
ylabel('velocity (meter/second)')
title('Velocity estimate from GPS and IMU defore adjustment')
legend('velocity gps','velocity imu')
hold off;
%Removing acclerometer bias------------------------------------------------
bias_pos = [0,5067,6317,7357,8263,10226,12133,13291,14721,15933,19639,21899,23883,25706,27637,27895,28638,29235,29966,31789,33102,33734,34555,35900,37182,43248];
% bias_pos = [0,43248];
accx_corrected = zeros(size(accx));
for i = 1:length(bias_pos)
    if i==length(bias_pos)-1
        mean_bias = mean(accx(bias_pos(1,i):bias_pos(1,i+1)));
        accx_corrected(bias_pos(1,i):bias_pos(1,i+1)) = accx(bias_pos(1,i):bias_pos(1,i+1)) - mean_bias; 
        break
    end
    if i == 1
        mean_bias = mean(accx(1:bias_pos(1,2)));
        accx_corrected(1:bias_pos(1,3)) = accx(1:bias_pos(1,3))-mean_bias;
    else 
        mean_bias = mean(accx(bias_pos(1,i):bias_pos(1,i+1)));
        accx_corrected(bias_pos(1,i):bias_pos(1,i+2)) = accx(bias_pos(1,i):bias_pos(1,i+2))-mean_bias;
    end
end

velocity_imu_corr = cumtrapz(accx_corrected*(1/40));
figure(6)
plot(velocity_gps)
hold on;
plot(velocity_imu_corr)         
grid on;
legend('velocity GPS','velocity IMU')
xlabel('time (seconds)')
ylabel('velocity (meter/second)')
title('Velocity estimate from GPS and IMU after adjustment')
hold off;
%Displacement calculation GPS and IMU--------------------------------------

displacement_imu = cumtrapz(velocity_imu_corr);
displacement_gps = cumtrapz(velocity_gps);

figure(7)
plot(displacement_imu)
grid on;
hold on;
plot(displacement_gps)
legend('displacement imu','displacement gps')
xlabel('time (seconds)')
ylabel('displacement (meter)')
title('Displacement vs Time')
hold off;
%Dead Reckoning------------------------------------------------------------
x_dd_ods = accx_corrected;
x_d = velocity_imu_corr;
w_xd = gyroz.*x_d;

y_dd_ods = accy + w_xd;

xddods_filt = lowpass(x_dd_ods,0.001,40);
yddods_filt = lowpass(y_dd_ods,0.001,40);

figure(8)
plot(time, w_xd )
grid on;
hold on;
plot(time,yddods_filt)
legend('𝜔𝑋̇','𝑦̈𝑜𝑏𝑠')
xlabel('time (seconds)')
ylabel('acceleration (meter/second^2)')
title('𝜔𝑋̇ and 𝑦̈𝑜𝑏𝑠')
hold off;
%------------------------------------------------
compl_yaw = added_mag_gyro - 0.15;

ve = velocity_gps.*sin(compl_yaw(1:43240));
vn = velocity_gps.*cos(compl_yaw(1:43240));

vi = velocity_imu(3:end);

ve1 = vi.*sin(gyro_yaw(1:43246));
vn2 = vi.*cos(gyro_yaw(1:43246));

vic = velocity_imu_corr(3:end);

ve3 = vic.*sin(gyro_yaw(1:43246));
vn4 = vic.*cos(gyro_yaw(1:43246));

xe = cumtrapz(ve.*(1/40));
xn = cumtrapz(vn.*(1/40));

xe1 = cumtrapz(ve1.*(1/40));
xn2 = cumtrapz(vn2.*(1/40));

xe3 = cumtrapz(ve3.*(1/40));
xn4 = cumtrapz(vn4.*(1/40));


figure(9)
plot(xe,xn)
grid on;
hold on;
plot(utm_east,utm_north)
legend('path estimated using complementary filter yaw','Path followed shown dy GPS', 'Location','southeast')
xlabel('easting (m)')
ylabel('northing (m)')
title('Comparing the Path estimated from GPS & complementary filter output')
hold off;

