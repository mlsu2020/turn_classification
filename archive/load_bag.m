%% Load data
bag = rosbag('beta_autorally4_2018-03-29-12-24-56.bag');
chassis_states = select(bag, 'Topic', '/chassisState');
[chassis_state_ts, chassis_state_cols] = timeseries(chassis_states);
wheel_odoms = select(bag, 'Topic', '/wheel_odom');
[wheel_odom_ts, wheel_odom_cols] = timeseries(wheel_odoms);
wheel_speeds = select(bag, 'Topic', '/wheelSpeeds');
[wheel_speed_ts, wheel_speed_cols] = timeseries(wheel_speeds);
poses = select(bag, 'Topic', '/pose_estimate');
[pose_ts, pose_cols] = timeseries(poses);

% bag = rosbag('2020-02-21-16-13-47.bag');
% CA = select(bag, 'Topic', '/MAP_CA/mapCA');
% [CA_ts, CA_cols] = timeseries(CA);
%% Format
data = zeros(21646, 11);
Rw=0.095;

% quats = pose_ts.Data(:,7:10);
% euls = quat2eul(quats);
% data(:,1) = euls(:,1)*180/pi;
% data(:,2) = euls(:,2)*180/pi;
% data(:,3) = euls(:,3)*180/pi;
% 
% data(:,4) = pose_ts.Data(:,11)*3.6;
% data(:,5) = pose_ts.Data(:,12)*3.6;
% 
% data(:,6) = pose_ts.Data(:,4);
% data(:,7) = pose_ts.Data(:,5);
% 
% data(:,8) = wheel_speed_ts.Data(:,1)*Rw*3.6;
% data(:,9) = wheel_speed_ts.Data(:,3)*Rw*3.6;
% data(:,10) = wheel_speed_ts.Data(:,2)*Rw*3.6;
% data(:,11) = wheel_speed_ts.Data(:,4)*Rw*3.6;


%SWA = chassis_state_ts(:,)
%Time = pose_ts(
%r = ?

quats = pose_ts.Data(:,7:10);
euls = quat2eul(quats);
PitchAngle = abs(euls(:,1))-pi;
RollAngle = euls(:,2);
YawAngle = euls(:,3);
YawAngle = unwrap(YawAngle) - YawAngle(1);

VxI = pose_ts.Data(:,11);
VyI = pose_ts.Data(:,12);
VI = sqrt(VxI.*VxI + VyI.*VyI);
beta = atan2(VyI, VxI) - YawAngle;
% Vx = CA_ts.Data(:,1);
Vx = +(VxI.*cos(YawAngle) + VyI.*sin(YawAngle));
% Vy = CA_ts.Data(:,2);
Vy = VxI.*-sin(YawAngle) + VyI.*cos(YawAngle);

XI = pose_ts.Data(:,4);
YI = pose_ts.Data(:,5);

r = 2*pi/pi/0.19;
w_lf = wheel_speed_ts.Data(:,1)*r;
w_lr = wheel_speed_ts.Data(:,3)*r;
w_rf = wheel_speed_ts.Data(:,2)*r;
% w_rf = w_lf;
w_rr = wheel_speed_ts.Data(:,4)*r;

steering = chassis_state_ts.Data(:,1);
r = pose_ts.Data(:,16);

time_chassis_state = chassis_state_ts.time - chassis_state_ts.time(1);
time_pose = pose_ts.time - pose_ts.time(1);
time_wheel_speed = wheel_speed_ts.time - wheel_speed_ts.time(1);
% time_r = linspace(0,max(time_pose), 20000);
% time_CA = CA_ts.time - CA_ts.time(1);

% plot(time_pose, Vy, '.')

tf=floor(max([time_chassis_state(end), time_pose(end), time_wheel_speed(end)]));
ti = floor(min([time_chassis_state(1), time_pose(1), time_wheel_speed(1)]));
tf = tf-ti;
dt=0.01;
t3=( 0:dt:tf )';
lth=length(t3);
mthd='pchip';

PitchAngle=interp1(time_pose,PitchAngle,t3,mthd);
RollAngle=interp1(time_pose,RollAngle,t3,mthd);
YawAngle=interp1(time_pose,YawAngle,t3,mthd);

steering0=interp1(time_chassis_state,steering,t3,mthd);
steering=medfilt1(steering0,200);
steering2=steering;
% steering2(steering2<0)=0;
% steering2([64:1044, 1515:2492, 2962:3936, 4403:5366, 5840:6806, 7275:8239, 8709:9661, 1.014e4:1.11e4, 1.157e4:1.253e4, 1.3005e4:1.396e4, 1.4437e4:1.54e4, 1.587e4:1.684e4, 1.73e4:end])=1.332;
% steering2([64:1044, 1515:2492, 2962:3936, 4403:5366, 5840:6806, 7275:8239, 8709:9661, 1.014e4:1.11e4, 1.157e4:1.253e4, 1.3005e4:1.396e4, 1.4437e4:1.54e4, 1.587e4:1.684e4, 1.73e4:end])=1.33;
% steering2([64:1044, 1515:2492, 2962:3936, 4403:5366, 5840:6806, 7275:8239, 8709:9661, 1.014e4:1.11e4, 1.157e4:1.253e4, 1.3005e4:1.396e4, 1.4437e4:1.54e4, 1.587e4:1.684e4, 1.73e4:end])=1.331;
% steering2([1145:1421, 2606:2873, 4031:4325, 5474:5706, 6918:7176, 8343:8592, 9777:1.004e4, 1.121e4:1.148e4, 1.264e4:1.29e4, 1.409e4:1.432e4, 1.552e4:1.576e4, 1.696e4:1.719e4])=0;

r=interp1(time_pose,r,t3,mthd);

Vx=interp1(time_pose,Vx,t3,mthd);
Vy=interp1(time_pose,Vy,t3,mthd);
XI=interp1(time_pose,XI,t3,mthd);
YI=interp1(time_pose,YI,t3,mthd);

w_lf=interp1(time_wheel_speed,w_lf,t3,mthd);
w_lf = medfilt1(w_lf, 20);
w_lr=interp1(time_wheel_speed,w_lr,t3,mthd);
w_lr = medfilt1(w_lr, 20);
w_rf=interp1(time_wheel_speed,w_rf,t3,mthd);
w_rf = medfilt1(w_rf, 20);
w_rr=interp1(time_wheel_speed,w_rr,t3,mthd);
w_rr = medfilt1(w_rr, 20);

%figure;plot(t3,Vy);
%figure; plot(t3,Vx,'b',t3,w_lf*Rw,'r',t3,w_lr*Rw,'g',t3,w_rf*Rw,'c',t3,w_rr*Rw,'m');
%  figure;plot(t3,PitchAngle);
%  figure;plot(t3,RollAngle);
%  figure;plot(t3,YawAngle);
% figure;plot(t3,steering);
%   figure;plot(t3,r);
%  figure;plot(XI,YI);
