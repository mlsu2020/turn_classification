%% Load data
folder = 'C:\Users\jknaup3\throttle\2020-03-09\';
bag = rosbag([folder,'beta_localhost_2020-03-09-11-17-00_0.bag']);
chassis_states = select(bag, 'Topic', '/chassisState');
[chassis_state_ts, chassis_state_cols] = timeseries(chassis_states);
wheel_speeds = select(bag, 'Topic', '/wheelSpeeds');
[wheel_speed_ts, wheel_speed_cols] = timeseries(wheel_speeds);
%% Format

r = 0.095;
w_lf = wheel_speed_ts.Data(:,1)/r;
w_lr = wheel_speed_ts.Data(:,3)/r;
w_rf = wheel_speed_ts.Data(:,2)/r;
w_rr = wheel_speed_ts.Data(:,4)/r;

throttle = chassis_state_ts.Data(:,2);

time_chassis_state = chassis_state_ts.time - chassis_state_ts.time(1);
time_wheel_speed = wheel_speed_ts.time - chassis_state_ts.time(1);

tf=floor(max([time_chassis_state(end), time_wheel_speed(end)]));
ti = floor(min([time_chassis_state(1), time_wheel_speed(1)]));
tf = tf-ti;
dt=0.001;
t3=( 0:dt:tf )';
lth=length(t3);
mthd='pchip';

w_lf=interp1(time_wheel_speed,w_lf,t3,mthd);
w_lf = medfilt1(w_lf, 20);
w_lr=interp1(time_wheel_speed,w_lr,t3,mthd);
w_lr = medfilt1(w_lr, 20);
w_rf=interp1(time_wheel_speed,w_rf,t3,mthd);
w_rf = medfilt1(w_rf, 20);
w_rr=interp1(time_wheel_speed,w_rr,t3,mthd);
w_rr = medfilt1(w_rr, 20);

throttle = interp1(time_chassis_state, throttle, t3, mthd);

% figure;
% subplot(3,2,1);
% plot(t3, w_lf);
% subplot(3,2,2);
% plot(t3, w_rf);
% subplot(3,2,3);
% plot(t3, w_lr);
% subplot(3,2,4);
% plot(t3, w_rr);
% subplot(3,2,5);
% plot(t3, throttle);

w_lrs = w_lr;
w_rrs = w_rr;
throttles = throttle;
t4 = t3;
%% Load data
folder = 'C:\Users\jknaup3\throttle\2020-03-09\';
bag = rosbag([folder,'beta_localhost_2020-03-09-11-16-17_0.bag']);
chassis_states = select(bag, 'Topic', '/chassisState');
[chassis_state_ts, chassis_state_cols] = timeseries(chassis_states);
wheel_speeds = select(bag, 'Topic', '/wheelSpeeds');
[wheel_speed_ts, wheel_speed_cols] = timeseries(wheel_speeds);
%% Format
r = 0.095;
w_lf = wheel_speed_ts.Data(:,1)/r;
w_lr = wheel_speed_ts.Data(:,3)/r;
w_rf = wheel_speed_ts.Data(:,2)/r;
w_rr = wheel_speed_ts.Data(:,4)/r;

throttle = chassis_state_ts.Data(:,2);

time_chassis_state = chassis_state_ts.time - chassis_state_ts.time(1);
time_wheel_speed = wheel_speed_ts.time - chassis_state_ts.time(1);

tf=floor(max([time_chassis_state(end), time_wheel_speed(end)]));
ti = floor(min([time_chassis_state(1), time_wheel_speed(1)]));
tf = tf-ti;
dt=0.001;
t3=( 0:dt:tf )';
lth=length(t3);
mthd='pchip';

w_lf=interp1(time_wheel_speed,w_lf,t3,mthd);
w_lf = medfilt1(w_lf, 20);
w_lr=interp1(time_wheel_speed,w_lr,t3,mthd);
w_lr = medfilt1(w_lr, 20);
w_rf=interp1(time_wheel_speed,w_rf,t3,mthd);
w_rf = medfilt1(w_rf, 20);
w_rr=interp1(time_wheel_speed,w_rr,t3,mthd);
w_rr = medfilt1(w_rr, 20);

throttle = interp1(time_chassis_state, throttle, t3, mthd);

w_lrs = [w_lrs; w_lr];
w_rrs = [w_rrs; w_rr];
throttles = [throttles; throttle];
t4 = [t4; t3+t4(end)];
%% Load data
folder = 'C:\Users\jknaup3\throttle\2020-03-09\';
bag = rosbag([folder,'beta_localhost_2020-03-09-11-17-52_0.bag']);
chassis_states = select(bag, 'Topic', '/chassisState');
[chassis_state_ts, chassis_state_cols] = timeseries(chassis_states);
wheel_speeds = select(bag, 'Topic', '/wheelSpeeds');
[wheel_speed_ts, wheel_speed_cols] = timeseries(wheel_speeds);
%% Format

r = 0.095;
w_lf = wheel_speed_ts.Data(:,1)/r;
w_lr = wheel_speed_ts.Data(:,3)/r;
w_rf = wheel_speed_ts.Data(:,2)/r;
w_rr = wheel_speed_ts.Data(:,4)/r;

throttle = chassis_state_ts.Data(:,2);

time_chassis_state = chassis_state_ts.time - chassis_state_ts.time(1);
time_wheel_speed = wheel_speed_ts.time - chassis_state_ts.time(1);

tf=floor(max([time_chassis_state(end), time_wheel_speed(end)]));
ti = floor(min([time_chassis_state(1), time_wheel_speed(1)]));
tf = tf-ti;
dt=0.001;
t3=( 0:dt:tf )';
lth=length(t3);
mthd='pchip';

w_lf=interp1(time_wheel_speed,w_lf,t3,mthd);
w_lf = medfilt1(w_lf, 20);
w_lr=interp1(time_wheel_speed,w_lr,t3,mthd);
w_lr = medfilt1(w_lr, 20);
w_rf=interp1(time_wheel_speed,w_rf,t3,mthd);
w_rf = medfilt1(w_rf, 20);
w_rr=interp1(time_wheel_speed,w_rr,t3,mthd);
w_rr = medfilt1(w_rr, 20);

throttle = interp1(time_chassis_state, throttle, t3, mthd);

w_lrs = [w_lrs; w_lr];
w_rrs = [w_rrs; w_rr];
throttles = [throttles; throttle];
t4 = [t4; t3+t4(end)];

%% plotting
figure;
subplot(2,1ddd,1);
hold on
plot(t4, w_lrs);
% subplot(2,2,2);
% hold on
% plot(t4, w_rrs);
subplot(2,1,2);
hold on
plot(t4, throttles);
legend('throttle command')

%% simulaion
kT = 1239.714;%204.1;
cT = 0.1601;
ka = 0.1577;
Iw = 0.0373;
sw_lrs = zeros(length(w_lrs), 1);
sw_lrs(1) = w_lrs(1);
for ii = 2:length(w_lrs)
    if throttles(ii,1) < 0.1
        T = 0;
    else
        T = throttles(ii,1) - cT;
    end
    sw_lrs(ii,1) = sw_lrs(ii-1,1) + dt * ka * (kT * T - sw_lrs(ii-1,1)) / Iw; 
end
subplot(2,1,1);
plot(t4, sw_lrs);
legend('measured wheel speed', 'simulated wheel speed');

%% fitting
% line = [0.17 12.25; 0.19 37.29; 0.2 49.38];
% figure;
% scatter(line(:,1), line(:,2))
% Fit = polyfit(line(:,1),line(:,2),1);
% plot(polyval(Fit,line(:,1)))
