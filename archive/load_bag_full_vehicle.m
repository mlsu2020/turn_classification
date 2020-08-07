%% Load data
bag = rosbag('C:\Users\jknaup3\Downloads\beta_localhost_2019-08-01-13-34-33_0.bag');
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
pose = select(bag, 'Topic', '/pose_estimate');
[pose_ts, pose_cols] = timeseries(pose);
%% Map CA
Rw=0.359;
wheel_radius = 0.095;

vx = CA_ts.Data(:,1);
vy = CA_ts.Data(:,2);
w_lf = CA_ts.Data(:,3) / wheel_radius;
w_rf = w_lf;
w_lr = CA_ts.Data(:,4) / wheel_radius;
w_rr = w_lr;

wz = CA_ts.Data(:,5);
% s_raw = CA_ts.Data(:,6);
% lap = CA_ts.Data(:,14);
% track_length = CA_ts.Data(1,16);
% s = s_raw + lap * track_length - s_raw(1) - lap(1)*track_length;
% ey = CA_ts.Data(:,7);
% epsi = CA_ts.Data(:,8);

time_ca = CA_ts.Time - CA_ts.Time(1);
% curvature = CA_ts.Data(:,9);

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


%% pose
X = pose_ts.Data(:,4);
Y = pose_ts.Data(:,5);
Zs = pose_ts.Data(:,6);

vz = pose_ts.Data(:,13);

quats = pose_ts.Data(:,7:10);
euls = quat2eul(quats);
PitchAngle = abs(euls(:,1))-pi;
RollAngle = euls(:,2);
YawAngle = euls(:,3);
YawAngle = unwrap(YawAngle);

vPitch = pose_ts.Data(:,14);
vRoll = pose_ts.Data(:,15);
vYaw = pose_ts.Data(:,16);

time_pose = pose_ts.Time - CA_ts.Time(1);

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
w_lf=interp1(time_ca,w_lf,t3,mthd);
w_rf=interp1(time_ca,w_rf,t3,mthd);
w_lr=interp1(time_ca,w_lr,t3,mthd);
w_rr=interp1(time_ca,w_rr,t3,mthd);

% curvature=interp1(time_ca,curvature,t3,'nearest');
% curvature(isnan(curvature)) = 0;

steering=interp1(time_chassis,steering,t3,mthd);
throttle=interp1(time_chassis,throttle,t3,mthd);

vz=interp1(time_pose,vz,t3,mthd);
X=interp1(time_pose,X,t3,mthd);
Y=interp1(time_pose,Y,t3,mthd);
Zs=interp1(time_pose,Zs,t3,mthd);
PitchAngle=interp1(time_pose,PitchAngle,t3,mthd);
YawAngle=interp1(time_pose,YawAngle,t3,mthd);
RollAngle=interp1(time_pose,RollAngle,t3,mthd);
vPitch=interp1(time_pose,vPitch,t3,mthd);
vYaw=interp1(time_pose,vYaw,t3,mthd);
vRoll=interp1(time_pose,vRoll,t3,mthd);

steering=medfilt1(steering,100);
throttle=medfilt1(throttle,100);
wz=medfilt1(wz, 100);
w_lf=medfilt1(w_lf,200);
w_rf=medfilt1(w_rf,200);
w_lr=medfilt1(w_lr,100);
w_rr=medfilt1(w_rr,100);
PitchAngle=medfilt1(PitchAngle,200);
RollAngle=medfilt1(RollAngle,200);

%%
states = [vx'; vy'; wz'; X'; Y'; YawAngle'; vz'; vPitch'; vRoll'; Zs'; PitchAngle'; RollAngle'; w_lf'; w_rf'; w_lr'; w_rr'];
inputs = [steering'; throttle'];
%%
dt = 0.001;
N0 = 1;
Nf = 24000;
state = states(:,1);
figure;
time = (1:27000) * dt;
for ii = 1:6
    subplot(3, 2, ii);
    hold on
    plot(time(N0:Nf), states(ii, N0:Nf))
end
%%
% y = Y_opt(1,:);
% figure;
% time = 0:Nf;
for ii = 1:6
    subplot(3, 2, ii);
%     hold on
    plot(time(N0:Nf), analytic_states(ii, N0:Nf))
end
%% plot nn sim results
load('states_out.mat');
nn_states = [nn_states(5,:); nn_states(6,:); nn_states(7,:); nn_states(1,:); nn_states(2,:); nn_states(3,:)];

for ii = 1:6
    subplot(3, 2, ii);
%     hold on
    plot(time(N0:Nf), nn_states(ii, N0:Nf))
end
%%
subplot(3,2,1);
legend('Measured vx','Analytic vx', 'NN vx');
xlabel('t (s)');
ylabel('vx (m/s)')
subplot(3,2,2);
legend('Measured vy','Analytic vy', 'NN vy');
xlabel('t (s)');
ylabel('vy (m/s)')
subplot(3,2,3);
legend('Measured wz','Analytic wz', 'NN wz');
xlabel('t (s)');
ylabel('Yaw rate (rad/s)')
subplot(3,2,4);
legend('Measured X','Analytic X', 'NN X');
xlabel('t (s)');
ylabel('global X (m)')
subplot(3,2,5);
legend('Measured Y','Analytic Y', 'NN Y');
xlabel('t (s)');
ylabel('global Y (m)')
subplot(3,2,6);
legend('Measured Yaw','Analytic Yaw', 'NN Yaw');
xlabel('t (s)');
ylabel('Yaw angle (rad)')
% subplot(4,2,7);
% legend('True ey','Simulated ey');
% subplot(4,2,8);
% legend('True s','Simulated s');