vx = states(1, 1:10:end);
vy = states(2, 1:10:end);
wz = states(3, 1:10:end);
wF = states(4, 1:10:end);
wR = states(5, 1:10:end);
steering = inputs(1, 1:10:end);
vx_dot = (vx(2:end) - vx(1:end-1))/0.01;
vy_dot = (vy(2:end) - vy(1:end-1))/0.01;
wz_dot = (wz(2:end) - wz(1:end-1))/0.01;
vx = vx(1, 2:end);
vy = vy(1, 2:end);
wz = wz(1, 2:end);
wF = wF(1, 2:end);
wR = wR(1, 2:end);
steering = steering(2:end);
vx_dot = vx_dot(1, 1:end);
vy_dot = vy_dot(1, 1:end);
wz_dot = wz_dot(1, 1:end);

% vx_dot = medfilt1(vx_dot, 75);
% vy_dot = medfilt1(vy_dot, 75);
% wz_dot = medfilt1(wz_dot, 75);
% steering = medfilt1(steering, 75);

m_Vehicle_m = 21.7562;%1270;%21.7562;
m_Vehicle_Iz = 1.124;%2000;%2.5;
lFR = 0.57;%2.91;%0.57;
m_Vehicle_lF = 0.34;%1.022;%0.4;
m_Vehicle_lR = lFR-m_Vehicle_lF;%0.57
m_Vehicle_rF = 0.095;

m_Vehicle_kSteering = 18.7861;%23.0811;%34
m_Vehicle_cSteering = 0.0109;
delta = m_Vehicle_kSteering .* steering + m_Vehicle_cSteering;
% delta = (input(2) + input(3))/2;
% delta = input(1);
% delta = medfilt1(delta, 25);

beta = atan2(vy, vx);

V = sqrt(vx .* vx + vy .* vy);
vFx = V .* cos(beta - delta) + wz .* m_Vehicle_lF .* sin(delta);
vFy = V .* sin(beta - delta) + wz .* m_Vehicle_lF .* cos(delta);
vRx = vx;
vRy = vy - wz .* m_Vehicle_lR;

alphaF = -atan(vFy ./ abs(vFx));
alphaR = -atan(vRy ./ abs(vRx));

slipF = -(vFx - m_Vehicle_rF) ./ vFx;
slipR = -(vRx - m_Vehicle_rF) ./ vRx;

fRy = (m_Vehicle_m * m_Vehicle_lF*(vy_dot + vx.*wz) - m_Vehicle_Iz * wz_dot) / lFR;
fFy = m_Vehicle_m * (vy_dot + vx .* wz) - fRy;

% ax = a(1,2:10:end);

% fFy = medfilt1(fFy, 3);
% fFy = interp1(1:length(fFy), fFy,1:10:length(fFy));
% fRy = interp1(1:length(fRy), fRy,1:10:length(fRy));
% alphaF = medfilt1(alphaF, 75);
% fRy = medfilt1(fRy, 3);
% alphaR = medfilt1(alphaR, 75);
% vy_dot = medfilt1(vy_dot, 20);

% wz_dot2 = (fFy * 0.34 - fRy * 0.23) / 1.124;

% wz_dot2 = medfilt1(wz_dot2, 10);

% plot(fFy(100:end-200), '.');
% hold on
% plot(wz_dot2(100:end-200));

wz_dot = (fFy * 0.34 - fRy * 0.23) / 1.124;
plot(wz(100:end-200))
hold on;
wz = cumsum(wz_dot * 0.01);
plot(wz(100:end-200))


% hold on;
% plot(ax(100:end-200));
% plot(wz_dot(10:end-20));
% hold on;
% plot(vx_dot(100:end-200));

% new_wz = zeros(1, length(wz));
% new_wz(1) = wz(1);
% for ii = 2:length(new_wz)
%     new_wz(ii) = new_wz(ii-1) + 0.001 * ((fFy(ii)) * m_Vehicle_lF - fRy(ii) * m_Vehicle_lR) / m_Vehicle_Iz;
% end
% sum(wz - new_wz)
% plot(wz(1000:end-1000));
% hold on
% plot(new_wz(1000:end-1000), '.');
% plot3(vx_dot(100:end-200), alphaF(100:end-200), fFy(100:end-200), '.');

% steering = steering(1:10:end);
% vx = vx(1:10:end);
% vy = vy(1:10:end);
% wz = wz(1:10:end);
% vx_dot = vx_dot(1:10:end);
% wF = wF(1:10:end);

ff = [steering; vx; vy; wz; vx_dot; wF; wR; fFy; fRy; vy_dot; wz_dot];