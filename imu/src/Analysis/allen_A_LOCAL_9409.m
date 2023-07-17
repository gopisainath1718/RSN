close all
clc
<<<<<<< HEAD
bag = rosbag("/home/gopisainath/EECE5554/LAB3/src/Data/LocationC.bag");
=======
bag = rosbag("/home/gopisainath/EECE5554/LAB3/src/Data/LocationA.bag");
>>>>>>> a78af53c36486e192fb1d9631593ead7ecd9464c

topic = select(bag, 'Topic', '/vectornav');
msgs = readMessages(topic, 'DataFormat', 'Struct');

gyroData = zeros(length(msgs), 3);
<<<<<<< HEAD
for i = 1:length(msgs)
   msg = msgs{i}.Data
=======

for i = 1:length(msgs)
   msg = msgs{i}.Data;
>>>>>>> a78af53c36486e192fb1d9631593ead7ecd9464c
   C = strsplit(msg, ',');
   if length(C) >= 13 % Check if C has at least 6 elements
      x = C{13}(1:10);
      gyroData(i,:) = [str2double(C{11}), str2double(C{12}), str2double(x)];
      
   end
end


Fs = 40;
t0 = 1/Fs;
<<<<<<< HEAD
theta = cumsum(gyroData(:,3), 1)*t0;
=======
theta = cumsum(gyroData(:,2), 1)*t0;
>>>>>>> a78af53c36486e192fb1d9631593ead7ecd9464c
maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.

tau = m*t0;

avar = zeros(numel(m), 1);
for i = 1:numel(m)
    mi = m(i);
    avar(i,:) = sum( ...
        (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
end
avar = avar ./ (2*tau.^2 .* (L - 2*m));
adev = sqrt(avar);

slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN

% Plot the results.
tauN = 1;
lineN = N ./ sqrt(tau);

slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK

% Plot the results.
tauK = 3;
lineK = K .* sqrt(tau/3);

slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB

% Plot the results.
tauB = tau(i);
lineB = B * scfB * ones(size(tau));

tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')

% loglog(tau, avar)

title('Allan Deviation with Noise Parameters of Angular Velocity about Y')
% title('Allan Variance of Angular Velocity about Z')

xlabel('\tau (sec)')
ylabel('\sigma(\tau)')
legend('$\sigma (rad/s)^2$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
    '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')

text(tauParams, params, {'N', 'K', 'B'})
grid on
axis equal


figure
loglog(tau, adev, '-')
title('Allan Deviation of Angular Velocity about Y')
xlabel('\tau (sec)');
ylabel('\sigma(\tau)')
legend('HW')
grid on
axis equal


