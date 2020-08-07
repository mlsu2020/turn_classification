
% load('matlab.mat');
load('matlab.mat');

Rw=0.359; %wheel radius

% data=data(2:end,:); %remove first line with all 0
% Time=Time(1:end-1);

PitchAngle=data(:,1)/180*pi;
RollAngle=data(:,2)/180*pi;
YawAngle=data(:,3)/180*pi;
steering=SWA/180*pi;
Vx=data(:,4)/3.6;
Vy=data(:,5)/3.6;
XI=data(:,6);  %check unit
YI=data(:,7);  %check unit
r=r/180*pi;

w_lf=data(:,8)/3.6/Rw;  
w_lr=data(:,9)/3.6/Rw; 
w_rf=data(:,10)/3.6/Rw;  
w_rr=data(:,11)/3.6/Rw; 

% figure;plot(Time,Vy);

tf=  floor(Time(end));
dt=0.01;
t3=( 0:dt:tf )';
lth=length(t3);

mthd='pchip';

PitchAngle=interp1(Time,PitchAngle,t3,mthd);
RollAngle=interp1(Time,RollAngle,t3,mthd);
YawAngle=interp1(Time,YawAngle,t3,mthd);

steering0=interp1(Time,steering,t3,mthd);
steering=medfilt1(steering0,200);
steering2=steering;
steering2(steering2<0)=0;
steering2([64:1044, 1515:2492, 2962:3936, 4403:5366, 5840:6806, 7275:8239, 8709:9661, 1.014e4:1.11e4, 1.157e4:1.253e4, 1.3005e4:1.396e4, 1.4437e4:1.54e4, 1.587e4:1.684e4, 1.73e4:end])=1.332;
steering2([64:1044, 1515:2492, 2962:3936, 4403:5366, 5840:6806, 7275:8239, 8709:9661, 1.014e4:1.11e4, 1.157e4:1.253e4, 1.3005e4:1.396e4, 1.4437e4:1.54e4, 1.587e4:1.684e4, 1.73e4:end])=1.33;
steering2([64:1044, 1515:2492, 2962:3936, 4403:5366, 5840:6806, 7275:8239, 8709:9661, 1.014e4:1.11e4, 1.157e4:1.253e4, 1.3005e4:1.396e4, 1.4437e4:1.54e4, 1.587e4:1.684e4, 1.73e4:end])=1.331;
steering2([1145:1421, 2606:2873, 4031:4325, 5474:5706, 6918:7176, 8343:8592, 9777:1.004e4, 1.121e4:1.148e4, 1.264e4:1.29e4, 1.409e4:1.432e4, 1.552e4:1.576e4, 1.696e4:1.719e4])=0;

r=interp1(Time,r,t3,mthd);

Vx=interp1(Time,Vx,t3,mthd);
Vy=interp1(Time,Vy,t3,mthd);
XI=interp1(Time,XI,t3,mthd);
YI=interp1(Time,YI,t3,mthd);

w_lf=interp1(Time,w_lf,t3,mthd);
w_lr=interp1(Time,w_lr,t3,mthd);
w_rf=interp1(Time,w_rf,t3,mthd);
w_rr=interp1(Time,w_rr,t3,mthd);

% figure;plot(t3,Vy);
% figure; plot(t3,Vx,'b',t3,w_lf*Rw,'r',t3,w_lr*Rw,'g',t3,w_rf*Rw,'c',t3,w_rr*Rw,'m');
% figure;plot(t3,PitchAngle);
% figure;plot(t3,RollAngle);
%figure;plot(t3,YawAngle);
% figure;plot(t3,steering);
%figure;plot(t3,r);
 figure;plot(XI,YI);