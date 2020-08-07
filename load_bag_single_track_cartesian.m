%% Load data
% bag = rosbag('C:\Users\jknaup3\Downloads\beta_autorally7_2019-07-19-10-43-36.bag');
% bag = rosbag('C:\Users\jknaup3\Downloads\beta_autorally7_2019-07-24-09-49-15.bag');
bag = rosbag('C:\Users\jknaup3\Downloads\beta_autorally4_2018-03-29-15-34-10_split0.bag');
% bag = rosbag('C:\Users\jknaup3\Downloads\beta_autorally4_2018-03-29-11-57-43.bag');
%%
% chassis_states = select(bag, 'Topic', '/chassisState');
% [chassis_state_ts, chassis_state_cols] = timeseries(chassis_states);
% wheel_odoms = select(bag, 'Topic', '/wheel_odom');
% [wheel_odom_ts, wheel_odom_cols] = timeseries(wheel_odoms);
wheel_speeds = select(bag, 'Topic', '/wheelSpeeds');
[wheel_speed_ts, wheel_speed_cols] = timeseries(wheel_speeds);
poses = select(bag, 'Topic', '/pose_estimate');
[pose_ts, pose_cols] = timeseries(poses);
imu = select(bag, 'Topic', 'imu/imu');
[imu_ts, imu_cols] = timeseries(imu);

% CA = select(bag, 'Topic', '/MAP_CA/mapCA');
% [CA_ts, CA_cols] = timeseries(CA);
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
% time_chassis = chassis_ts.Time - CA_ts.Time(1);
% figure;
% plot(time_chassis, steering);
% hold on
% plot(time_chassis, throttle);
% hold off

%% acceleration
a_x = imu_ts.Data(:, 11);
a_y = imu_ts.Data(:, 12);
a_z = imu_ts.Data(:, 13);
% time_imu = imu_ts.Time - CA_ts.Time(1);

%% others
wheel_radius = 0.095;
wF = (wheel_speed_ts.Data(:,1) + wheel_speed_ts.Data(:,2)) / 2 / wheel_radius;
wR = (wheel_speed_ts.Data(:,3) + wheel_speed_ts.Data(:,4)) / 2 / wheel_radius;
X = pose_ts.Data(:, 4);
Y = pose_ts.Data(:, 5);
wz = pose_ts.Data(:,16);
quats = pose_ts.Data(:,7:10);
euls = quat2eul(quats);
PitchAngle = abs(euls(:,1))-pi;
RollAngle = euls(:,2);
YawAngle = euls(:,3);
Yaw = unwrap(YawAngle) + 2*pi;
% Yaw = YawAngle;
vx = pose_ts.Data(:,11).*cos(Yaw)+pose_ts.Data(:,12).*sin(Yaw);
vy = pose_ts.Data(:,11).*-sin(YawAngle)+pose_ts.Data(:,12).*cos(YawAngle);
time_pose = pose_ts.Time - pose_ts.Time(1);
time_ca = time_pose;
time_imu = imu_ts.Time - pose_ts.Time(1);
time_wheel_speed = wheel_speed_ts.Time - pose_ts.Time(1);
time_chassis = chassis_ts.Time - pose_ts.Time(1);

%% resample
tf=floor(max([time_ca(end), time_chassis(end), time_wheel_speed(end), time_imu(end)]));
ti = floor(min([time_ca(1), time_chassis(1), time_wheel_speed(1), time_imu(1)]));
tf = tf-ti;
dt=0.001;
t3=( 0:dt:tf )';
lth=length(t3);
mthd='pchip';

vx=interp1(time_ca,vx,t3,mthd);
vy=interp1(time_ca,vy,t3,mthd);
wz=interp1(time_ca,wz,t3,mthd);
wF=interp1(time_wheel_speed,wF,t3,mthd);
wR=interp1(time_wheel_speed,wR,t3,mthd);
% epsi=interp1(time_ca,epsi,t3,mthd);
% ey=interp1(time_ca,ey,t3,mthd);
% s=interp1(time_ca,s,t3,mthd);
X=interp1(time_ca,X,t3,mthd);
Y=interp1(time_ca,Y,t3,mthd);
Yaw=interp1(time_ca,Yaw,t3,mthd);

% curvature=interp1(time_ca,curvature,t3,'nearest');
% curvature(isnan(curvature)) = 0;

steering=interp1(time_chassis,steering,t3,mthd);
throttle=interp1(time_chassis,throttle,t3,mthd);

a_x=interp1(time_imu,a_x,t3,mthd);
a_y=interp1(time_imu,a_y,t3,mthd);
a_z=interp1(time_imu,a_z,t3,mthd);

% steering=medfilt1(steering,200);
% throttle=medfilt1(throttle,500);
wz=medfilt1(wz, 500);
wF=medfilt1(wF,500);
wR=medfilt1(wR,500);
vx=medfilt1(vx,500);
vy=medfilt1(vy,500);
a_x=medfilt1(a_x,500);
a_y=medfilt1(a_y,200);
a_z=medfilt1(a_z,1000);

%%
N=1;
figure;
subplot(4,2,1);
plot(t3(N:end-1000), vx(N:end-1000));
legend('True vx','Simulated vx');
subplot(4,2,2);
plot(t3(N:end-1000), vy(N:end-1000));
legend('True vy','Simulated vy');
subplot(4,2,3);
plot(t3(N:end-1000), wz(N:end-1000));
legend('True wz','Simulated wz');
subplot(4,2,4);
plot(t3(N:end-1000), wF(N:end-1000));
legend('True wF','Simulated wF');
subplot(4,2,5);
plot(t3(N:end-1000), wR(N:end-1000));
legend('True wR','Simulated wR');
subplot(4,2,6);
plot(t3(N:end-1000), Yaw(N:end-1000));
legend('True Yaw','Simulated Yaw');
subplot(4,2,7);
plot(t3(N:end-1000), X(N:end-1000));
legend('True X','Simulated X');
subplot(4,2,8);
plot(t3(N:end-1000), Y(N:end-1000));
legend('True Y','Simulated Y');
figure;
plot(t3, steering);
hold on
plot(t3, throttle);
hold on

%%
states = [vx'; vy'; wz'; wF'; wR'; Yaw'; X'; Y'];
inputs = [steering'; throttle'];

%%
subplot(4,2,1);
legend('True vx','Simulated vx', 'Carsim vx');
xlabel('t (s)');
ylabel('m/s')
subplot(4,2,2);
legend('True vy','Simulated vy', 'Carsim vy');
xlabel('t (s)');
ylabel('m/s');
subplot(4,2,3);
legend('True wz','Simulated yaw-rate', 'Carsim yaw-rate');
xlabel('t (s)');
ylabel('rad/s');
subplot(4,2,4);
legend('True wF','Simulated wF','Carsim wF');
xlabel('t (s)');
ylabel('rad/s');
subplot(4,2,5);
legend('True wR','Simulated wR','Carsim wR');
xlabel('t (s)');
ylabel('rad/s');
subplot(4,2,6);
legend('True Yaw','Simulated Yaw','Carsim Yaw');
xlabel('t (s)');
ylabel('rad');
subplot(4,2,7);
legend('True X','Simulated X','Carsim X');
xlabel('t (s)');
ylabel('m');
subplot(4,2,8);
legend('True Y','Simulated Y','Carsim Y');
xlabel('t (s)');
ylabel('m');

%% acceleration plotting
% figure;
% subplot(2,2,1);
% plot(t3(N:end-1000), a_x(N:end-1000));
% subplot(2,2,2);
% plot(t3(N:end-1000), a_y(N:end-1000));
% subplot(2,2,3);
% plot(t3(N:end-1000), a_z(N:end-1000));

a = [a_x'; a_y'; a_z'];
% forces = medfilt1(forces,500);
figure;
subplot(2, 1, 1);
hold on
plot(t3(N:end-1000), a(1, N:end-1000))
forces(1, :)=medfilt1(forces(1,:),200);
forces(4, :)=medfilt1(forces(2,:),200);
% plot(t3(N:end-1000), forces(1, N:end-1000) + forces(4, N:end-1000))
plot(t3(N:end-1000), forces(7, N:end-1000))
legend('m*a_x', 'f_x')
xlabel('t (s)')
ylabel('F (N)')
subplot(2, 1, 2);
hold on
forces(2, :)=medfilt1(forces(2,:),200);
forces(5, :)=medfilt1(forces(5,:),200);
plot(t3(N:end-1000), -a(2, N:end-1000))
% plot(t3(N:end-1000), forces(2, N:end-1000) + forces(5, N:end-1000))
plot(t3(N:end-1000), forces(8, N:end-1000));
legend('m*a_y', 'f_y')
xlabel('t (s)')
ylabel('F (N)')

%%
thresh = 0.2;
straights = abs(wz) < thresh;
left = wz > thresh;
right = wz < -thresh;
wz2 = wz;
wz2(straights) = 0;
wz2(left) = 1;
wz2(right) = -1;
wz2 = medfilt1(wz2, 1000);
figure;
plot(t3(N:end-1000), wz2(N:end-1000));
hold on
plot(t3(N:end-1000), Yaw(N:end-1000));

%%
left_images = select(bag, 'Topic', '/left_camera/image_color/compressed');
% [left_images_ts, left_images_cols] = timeseries(left_images);
msgs = readMessages(left_images, 'DataFormat','struct');
%%
time_images = msgs2.Time - msgs2.Time(1);
length(time_images);
turns = interp1(t3,wz2,time_images,'nearest');
vx = interp1(t3,vx,time_images,mthd);
vy = interp1(t3,vy,time_images,mthd);
wz = interp1(t3,wz,time_images,mthd);
Yaw = interp1(t3,Yaw,time_images,mthd);
X = interp1(t3,X,time_images,mthd);
Y = interp1(t3,Y,time_images,mthd);
steering = interp1(t3,steering,time_images,mthd);
wR = interp1(t3,wR,time_images,mthd);
states = [vx';vy';wz';Yaw';X';Y'];
controls = [steering'; wR'];
left = turns > thresh;
right = turns < -thresh;
turns(left) = 1;
turns(right) = -1;
figure
plot(t3(N:end-1000), wz2(N:end-1000));
hold on
plot(time_images(N:end-50), turns(N:end-50));
% csvwrite('turn_labels.csv', turns)
% csvwrite('states.csv', states);
% csvwrite('controls.csv',controls);


%%
for ii = 1:length(msgs)
    image = readImage(copyImage(msg{ii}));
    imshow(image);
    path = sprintf('images/img%d.png', ii);
    imwrite(image, path);
end

function msg = copyImage(msgStruct)
    msg = rosmessage(msgStruct.MessageType);
    fNames = fieldnames(msg);
    for k = 1:numel(fNames)
        if ~any(strcmp(fNames{k}, {'MessageType', 'Header'}))
            msg.(fNames{k}) = msgStruct.(fNames{k});
        end
    end
end

function Seq = loadImages(imgPath, imgType)
    %imgPath = 'path/to/images/folder/';
    %imgType = '*.png'; % change based on image type
    images  = dir([imgPath imgType]);
    N = length(images);

    % check images
    if( ~exist(imgPath, 'dir') || N<1 )
        display('Directory not found or no matching images found.');
    end

    % preallocate cell
    Seq{N,1} = []

    for idx = 1:N
        Seq{idx} = imread([imgPath images(idx).name]);
    end
end
