mass = 50;
tf = 200;
dt = 0.01;
nt = tf/dt;

time = [0:dt:tf-dt];
T = ones(nt,1) + 0.05*randn([nt,1]);
S = zeros(nt,1);
Vx = zeros(nt,1);
Vy = zeros(nt,1);
X = zeros(nt,1);
Y = zeros(nt,1);
% size(time)
% size(T)
% size(V)
for ii = 2:nt
    Vx(ii) = Vx(ii-1) + (T(ii) + 1*randn())/mass*dt;
    Vy(ii) = Vy(ii-1) + (S(ii-1))*dt;
    X(ii) = X(ii-1) + Vx(ii)*dt + 0.1*randn();
    Y(ii) = Y(ii-1) + Vy(ii)*dt;
end

figure;plot(time,T);
figure; plot(time,Vx);
figure;plot(time,X);
figure;plot(time,Y);
