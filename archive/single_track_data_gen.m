state = states(:,N);%[0.2; 0; 0; 0.2/0.095+1e-4; 0.2/0.095+1e-4; 0;0;0];
input = [-0.2;0.5];
% curvature = -1;

time = t3;%1:20000;
% states = zeros(length(state), length(time));
dots = zeros(length(state), length(time));
% inputs = 0.5*sin(time/1000) + input;
% figure;
% plot(time, inputs)
for ii = N:length(time)
% if ii == 16001
%     fprintf("h");
% end

[dotii, state] = update_dynamics(state, inputs(:,ii), curvature(ii));
% state(1) = states(1,ii);
% state(2) = states(2,ii);
% state(6) = states(6,ii);
states(:,ii) = state;
dots(:,ii) = dotii;
end
% figure;
for ii = 1:length(state)
    subplot(length(state)/2, 2, ii);
    hold on
    plot(time(N:end), states(ii, N:end))
end
% subplot(length(state)/2+1, 2, length(state)+1);
% plot(states(9, :), states(10, :));

function [dots, next_state] = update_dynamics(state, input, curvature)
m_Vehicle_m = 22.0262;% 21.88;
m_Vehicle_Iz = 1.2311;%1.124;
m_Vehicle_lF = .3092;%0.34;
m_Vehicle_lR = 0.57-.3092;%0.23;
m_Vehicle_IwF = .0416;%0.048;
m_Vehicle_IwR = .0373;%0.044;
m_Vehicle_rF = 0.095;
m_Vehicle_rR = 0.095;
m_Vehicle_mu1 = 0.75;
m_Vehicle_mu2 = 0.90;
m_Vehicle_h = .1159;%0.2;    
m_g = 9.80665;

m_Vehicle_kSteering = 30.0811;
m_Vehicle_kThrottle = 204.0922;
m_Vehicle_kTorque = 0.1577;

tire_B = 1.1433;%1.1559;
tire_C = 1.1277;%1.1924;
tire_D = 1.4031;%0.9956;
tire_E = 0.1368;
tire_Sh = 0.7;
tire_Sv = 0.999;

m_nx = 8;
m_nu = 2;
dots = zeros(m_nx, 1);

vx = state(1, 1);
vy = state(2, 1);
wz = state(3, 1);
wF = state(4, 1);
wR = state(5, 1);
epsi = state(6, 1);
ey = state(7, 1);
s = state(8, 1);
% X = state(9, 1);
% Y = state(10,1);

delta = m_Vehicle_kSteering * input(1, 1) + 0.00*randn();
T = m_Vehicle_kThrottle * input(2, 1) + 0.0*randn();

min_velo = 0.1;
deltaT = 0.001;

if (vx < min_velo)
  vx = min_velo;
end
if (wF < min_velo / m_Vehicle_rF)
    wF = min_velo / m_Vehicle_rF + 1e-4;
end
if wR < min_velo / m_Vehicle_rR
    wR = min_velo / m_Vehicle_rR + 1e-4;
end
if (vx ~= 0.0)
  beta = atan2(vy, vx);
else
  beta = 0.0;
end

V = sqrt(vx * vx + vy * vy);
vFx = V * cos(beta - delta) + wz * m_Vehicle_lF * sin(delta);
vFy = V * sin(beta - delta) + wz * m_Vehicle_lF * cos(delta);
vRx = vx;
vRy = vy - wz * m_Vehicle_lR;

if (wF ~= 0.0)
  sFx = (vFx - wF * m_Vehicle_rF) / (wF * m_Vehicle_rF);
else
  sFx = 0.0;
end
if (wR ~= 0.0)
  sRx = (vRx - wR * m_Vehicle_rR) / (wR * m_Vehicle_rR);
else
  sRx = 0.0;
end
if (vFx ~= 0.0)
  sFy = vFy / (wF * m_Vehicle_rF);
%   sFy = (1 + sFx) * vFy / vFx;
else
  sFy = 0.0;
end
if (vRx ~= 0.0)
  sRy = vRy / (wR * m_Vehicle_rR);
%   sRy = (1 + sRx) * vRy / vRx;
else
  sRy = 0.0;
end

sF = sqrt(sFx * sFx + sFy * sFy);
sR = sqrt(sRx * sRx + sRy * sRy);

sEF = sF - tire_Sh;
sER = sR - tire_Sh;

muF = tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv; 
muR = tire_D*sin( tire_C*atan( tire_B*sER - tire_E*(tire_B*sER - atan(tire_B*sER) ) ) ) + tire_Sv;
% muF = tire_D * sin(tire_C * atan(tire_B * sF)); 
% muR = tire_D * sin(tire_C * atan(tire_B * sR));

fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * m_Vehicle_mu1) / (m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h * (m_Vehicle_mu1 * cos(delta) - m_Vehicle_mu2 * sin(delta) - m_Vehicle_mu1));
fRz = m_Vehicle_m * m_g - fFz;

fFx = -sFx / sF * muF * fFz;
fFy = -sFy / sF * muF * fFz;
fRx = -sRx / sR * muR * fRz;
fRy = -sRy / sR * muR * fRz;

next_state = zeros(m_nx, 1);
next_state(1, 1) = vx + deltaT * ((fFx * cos(delta) - fFy * sin(delta) + fRx) / m_Vehicle_m + vy * wz);
next_state(2, 1) = vy + deltaT * ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m - vx * wz);
next_state(3, 1) = wz + deltaT * ((fFy * cos(delta) + fFx * sin(delta)) * m_Vehicle_lF - fRy * m_Vehicle_lR) / m_Vehicle_Iz;
next_state(4, 1) = wF - deltaT * m_Vehicle_rF / m_Vehicle_IwF * fFx;
next_state(5, 1) = wR + deltaT * (m_Vehicle_kTorque * (T-wR) - m_Vehicle_rR * fRx) / m_Vehicle_IwR;
next_state(6, 1) = epsi + deltaT * (wz - (vx * cos(epsi) - vy * sin(epsi)) / (1 - curvature * ey) * curvature);
epsi = epsi + 0.20;
next_state(7, 1) = ey + deltaT * (vx * sin(epsi) + vy * cos(epsi));
next_state(8, 1) = s + deltaT * (vx * cos(epsi) - vy * sin(epsi)) / (1 - curvature * ey);
% next_state(9, 1) = X + deltaT * vx * cos(epsi) - vy * sin(epsi);
% next_state(10, 1) = Y + deltaT * vx * sin(epsi) + vy * cos(epsi);

% dots(1,1) = ((fFx * cos(delta) - fFy * sin(delta) + fRx) / m_Vehicle_m + vy * wz);
% dots(2,1) = ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m - vx * wz);
% dots(3,1) = ((fFy * cos(delta) + fFx * sin(delta)) * m_Vehicle_lF - fRy * m_Vehicle_lR) / m_Vehicle_Iz;
% dots(4,1) = - m_Vehicle_rF / m_Vehicle_IwF * fFx;
% dots(5,1) = 1/ m_Vehicle_IwR * (T - m_Vehicle_rR * fRx);
% dots(6,1) = (wz - (vx * cos(epsi) - vy * sin(epsi)) / (1 - curvature * ey) * curvature);
% dots(7,1) = (vx * sin(epsi) + vy * cos(epsi));
% dots(8,1) = (vx * cos(epsi) - vy * sin(epsi)) / (1 - curvature * ey);
% dots(9,1) = vx * cos(epsi) - vy * sin(epsi);
% dots(10,1) = vx * sin(epsi) + vy * cos(epsi);
end


