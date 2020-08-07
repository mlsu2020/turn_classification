%% Load data
bag = rosbag('C:\Users\jknaup3\Downloads\beta_autorally7_2019-07-19-10-43-36.bag');
% bag = rosbag('C:\Users\jknaup3\Downloads\beta_autorally7_2019-07-24-09-49-15.bag');
% chassis_states = select(bag, 'Topic', '/chassisState');
% [chassis_state_ts, chassis_state_cols] = timeseries(chassis_states);
% wheel_odoms = select(bag, 'Topic', '/wheel_odom');
% [wheel_odom_ts, wheel_odom_cols] = timeseries(wheel_odoms);
% wheel_speeds = select(bag, 'Topic', '/wheelSpeeds');
% [wheel_speed_ts, wheel_speed_cols] = timeseries(wheel_speeds);
% poses = select(bag, 'Topic', '/pose_estimate');
% [pose_ts, pose_cols] = timeseries(poses);

CA = select(bag, 'Topic', '/MAP_CA/mapCA');
[CA_ts, CA_cols] = timeseries(CA);
chassis = select(bag, 'Topic', '/chassisState');
[chassis_ts, chassis_cols] = timeseries(chassis);
%% Map CA
Rw=0.359;
wheel_radius = 0.095;

vx = CA_ts.Data(:,1);
vy = CA_ts.Data(:,2);
wF = CA_ts.Data(:,3) / wheel_radius;
wR = CA_ts.Data(:,4) / wheel_radius;
wz = CA_ts.Data(:,5);
s_raw = CA_ts.Data(:,6);
lap = CA_ts.Data(:,14);
track_length = CA_ts.Data(1,16);
s = s_raw + lap * track_length - lap(1)*track_length; %- s_raw(1)
ey = CA_ts.Data(:,7);
epsi = CA_ts.Data(:,8);
X = CA_ts.Data(:,11);
Y = CA_ts.Data(:,12);
Yaw = CA_ts.Data(:,13);
Yaw = unwrap(Yaw);

time_ca = CA_ts.Time - CA_ts.Time(1);
curvature = CA_ts.Data(:,9);

% figure;
% subplot(4,2,1);
% plot(time_ca, vx);
% subplot(4,2,2);
% plot(time_ca, vy);
% subplot(4,2,3);
% plot(time_ca, wz);
% subplot(4,2,4);
% plot(time_ca, wF);
% subplot(4,2,5);
% plot(time_ca, wR);
% subplot(4,2,6);
% plot(time_ca, epsi);
% subplot(4,2,7);
% plot(time_ca, ey);

% subplot(4,2,8);
% plot(time_ca, s);
% plot(CA_ts.Data(:,11), CA_ts.Data(:,12), '.')

%% Control
steering = -pi / 180 * chassis_ts.Data(:,1);
throttle = chassis_ts.Data(:,2);
time_chassis = chassis_ts.Time - CA_ts.Time(1);
% figure;
% plot(time_chassis, steering);
% hold on
% plot(time_chassis, throttle);
% hold off

%% resample
tf=floor(max([time_ca(end), time_chassis(end)]));
ti = floor(min([time_ca(1), time_chassis(1)]));
tf = tf-ti;
dt=0.001;
t3=( 0:dt:tf )';
lth=length(t3);
mthd='pchip';

vx=interp1(time_ca,vx,t3,mthd);
vy=interp1(time_ca,vy,t3,mthd);
wz=interp1(time_ca,wz,t3,mthd);
wF=interp1(time_ca,wF,t3,mthd);
wR=interp1(time_ca,wR,t3,mthd);
epsi=interp1(time_ca,epsi,t3,mthd);
ey=interp1(time_ca,ey,t3,mthd);
s=interp1(time_ca,s,t3,mthd);
X=interp1(time_ca,X,t3,mthd);
Y=interp1(time_ca,Y,t3,mthd);
Yaw=interp1(time_ca,Yaw,t3,mthd);

curvature=interp1(time_ca,curvature,t3,'nearest');
curvature(isnan(curvature)) = 0;

steering=interp1(time_chassis,steering,t3,mthd);
throttle=interp1(time_chassis,throttle,t3,mthd);

steering=medfilt1(steering,500);
throttle=medfilt1(throttle,500);
wz=medfilt1(wz, 500);
wF=medfilt1(wF,500);
wR=medfilt1(wR,500);
%%
N=1000;
figure;
subplot(4,2,1);
plot(t3(N:end), vx(N:end));
legend('True vx','Simulated vx');
subplot(4,2,2);
plot(t3(N:end), vy(N:end));
legend('True vy','Simulated vy');
subplot(4,2,3);
plot(t3(N:end), wz(N:end));
legend('True wz','Simulated wz');
subplot(4,2,4);
plot(t3(N:end), wF(N:end));
legend('True wF','Simulated wF');
subplot(4,2,5);
plot(t3(N:end), wR(N:end));
legend('True wR','Simulated wR');
subplot(4,2,6);
plot(t3(N:end), epsi(N:end));
legend('True epsi','Simulated epsi');
subplot(4,2,7);
plot(t3(N:end), ey(N:end));
legend('True ey','Simulated ey');
subplot(4,2,8);
plot(t3(N:end), s(N:end));
legend('True s','Simulated s');
% figure;
% plot(t3, steering);
% hold on
% plot(t3, throttle);
% hold off

%%
states = [vx'; vy'; wz'; wF'; wR'; epsi'; ey'; s'; X'; Y'; Yaw'];
inputs = [steering'; throttle'];

%%
subplot(4,2,1);
legend('True vx','Simulated vx');
xlabel('t (s)');
ylabel('m/s')
subplot(4,2,2);
legend('True vy','Simulated vy');
xlabel('t (s)');
ylabel('m/s');
subplot(4,2,3);
legend('True wz','Simulated wz');
xlabel('t (s)');
ylabel('rad/s');
subplot(4,2,4);
legend('True wF','Simulated wF');
xlabel('t (s)');
ylabel('rad/s');
subplot(4,2,5);
legend('True wR','Simulated wR');
xlabel('t (s)');
ylabel('rad/s');
subplot(4,2,6);
legend('True epsi','Simulated epsi');
xlabel('t (s)');
ylabel('rad');
subplot(4,2,7);
legend('True ey','Simulated ey');
xlabel('t (s)');
ylabel('m');
subplot(4,2,8);
legend('True s','Simulated s');
xlabel('t (s)');
ylabel('m');