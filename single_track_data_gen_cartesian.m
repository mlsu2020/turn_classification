N = 2000;
Nf = 2000;
subset_states = states(:,N:10:end-Nf);
subset_inputs = inputs(:,N:10:end-Nf);
% subset_ff = ff(:,N/10:end-Nf/10);
state = subset_states(:,1);
% carsim_states(7:8,N) = states(7:8,N);
time = (1:length(subset_states))/100;%1:20000;
analytic_states = zeros(length(state), length(time));
forces = zeros(8, length(time));
ff_training_data = zeros(5, length(time));
% figure;
% plot(time, inputs)
for ii = 1:length(time)
% state(3) = states(3,ii);
[force, state] = update_dynamics(state, subset_inputs(:,ii), 0);
% state(1) = states(1,ii);
% state(2) = states(2,ii);
% state(3) = state(3) - 0.0279;
% state(4) = states(4,ii);
state(5) = subset_states(5,ii);
% state(1:5) = carsim_states(1:5,ii);
% state(6) = wzs(ii,4);
analytic_states(:,ii) = state;
forces(:,ii) = force;
% Yawi = carsim_states(6,ii);
% T_rotation=[ cos(Yawi) sin(Yawi); -sin(Yawi) cos(Yawi) ];
% dot_pos=T_rotation'*[ carsim_states(1,ii); carsim_states(2,ii) ];
% dot_X=dot_pos(1);
% dot_Y=dot_pos(2); 
% carsim_states(7,ii) = carsim_states(7,ii-1) + 0.001 * dot_X;
% carsim_states(8, ii) = carsim_states(8,ii-1) + 0.001 * dot_Y;
% dots(:,ii) = dotii;
% state = states(:,ii);
end
% yaw_error = states(6,N+1:end-Nf) - analytic_states(6,N:end-Nf-1);
% yaw_error = yaw_error(end-Nf)
% wz_error = abs(states(3,N+1:end-Nf) - analytic_states(3,N:end-Nf-1));
% wz_error_int = sum(wz_error(1:end) * 0.01)
% wz_error_mean = mean(wz_error)
% analytic_states(2,:)=-1*analytic_states(2,:);
figure;
for ii = 1:length(state)
    subplot(length(state)/2, 2, ii);
    hold on
        plot(time, subset_states(ii, :))
%     if ii < 7
%     else
        plot(time, analytic_states(ii, :))
%     end
%         plot(time(N:end-Nf), carsim_states(ii, N:end-Nf))

end
% figure;
% plot(time, cos(18.8*subset_ff(1,:)+0.01) .* forces(2, :) + sin(18.8*subset_ff(1,:)+0.01) .* forces(1,:));
% hold on;
% plot(time, subset_ff(7, :));
% plot(time(N:end-1001), wz_error);
% hold on
% plot(time(N:end-1001), wz_error_mean*ones(length(time(N:end-1001)),1));
% ylabel('error in wz (rad/s)');
% xlabel('time (s)');
% legend(strcat('error (mean = ', num2str(wz_error_mean),')'));
% for ii = 1:6
%     subplot(6/2, 2, ii);
%     hold on
%     plot(time(N:end-1000), forces(ii, N:end-1000))
% end

function [forces, next_state] = update_dynamics(state, input, wz_error_mean)
m_Vehicle_m = 21.7562;%1270;%21.7562;
m_Vehicle_Iz = 1.124;%2000;%2.5;
lFR = 0.57;%2.91;%0.57;
m_Vehicle_lF = 0.34;%1.022;%0.4;
m_Vehicle_lR = lFR-m_Vehicle_lF;%0.57
m_Vehicle_IwF = 0.05;%8.0;%4.02;
m_Vehicle_IwR = 0.5;%3.73;
m_Vehicle_rF = 0.095;%0.325;%0.095;
m_Vehicle_rR = 0.09;%0.325;%0.090;
m_Vehicle_mu1 = 0.75;
m_Vehicle_mu2 = 0.90;
m_Vehicle_h = 0.12;%0.54;%.2;%0.2;    
m_g = 9.80665;

m_Vehicle_kSteering = 18.7861;%23.0811;%34
m_Vehicle_cSteering = 0.0109;
m_Vehicle_kThrottle = 165.0922;
m_Vehicle_kTorque = 0.07577;

tire_B = 4;%1.5;
tire_C = 1;%1.5;
tire_D = 1.0;
tire_a = 1;%0.75;
tire_E = 0.97;
tire_Sh = -0.0;
tire_Sv = 0.00;
tire_By = tire_B;
tire_Cy = tire_C;
tire_Dy = tire_C;
tire_Bx = tire_B;
tire_Cx = tire_C;
tire_Dx = tire_D;

m_nx = 8;
m_nu = 2;

vx = state(1, 1);
vy = state(2, 1);
wz = state(3, 1);
wF = state(4, 1);
wR = state(5, 1);
Yaw = state(6, 1);
X = state(7, 1);
Y = state(8, 1);

delta = m_Vehicle_kSteering * input(1, 1) + m_Vehicle_cSteering;
% delta = (input(2) + input(3))/2;
% delta = input(1);
% T = m_Vehicle_kThrottle * input(2, 1);

min_velo = 0.1;
deltaT = 0.01;

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

alphaF = -atan(vFy / abs(vFx));
alphaR = -atan(vRy / abs(vRx));

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

% muF = tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;
% muR = tire_D*sin( tire_C*atan( tire_B*sER - tire_E*(tire_B*sER - atan(tire_B*sER) ) ) ) + tire_Sv;
muF = tire_D * sin(tire_C * atan(tire_B * sF)); 
muR = tire_D * sin(tire_C * atan(tire_B * sR));

muFx = -sFx / sF * muF;
muFy = -sFy / sF * muF;
muRx = -sRx / sR * muR;
muRy = -sRy / sR * muR;

fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * muRx) / (m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h * (muFx * cos(delta) - muFy * sin(delta) - muRx));
% fFz = m_Vehicle_m * m_g * (m_Vehicle_lR / lFR);
fRz = m_Vehicle_m * m_g - fFz;

fFx = muFx * fFz;
fFy = tire_a*muFy * fFz;
fRx = muRx * fRz;
fRy = tire_a*muRy * fRz;

%%%%%%%%%%%%%%%%%%%
% sEF = -(vFx - wF * m_Vehicle_rF) / (vFx) + tire_Sh;
% muFx = tire_Dx*sin( tire_Cx*atan( tire_Bx*sEF - tire_E*(tire_Bx*sEF - atan(tire_Bx*sEF) ) ) ) + tire_Sv;
% sEF = -(vRx - wR * m_Vehicle_rR) / (vRx) + tire_Sh;
% muRx = tire_Dx*sin( tire_Cx*atan( tire_Bx*sEF - tire_E*(tire_Bx*sEF - atan(tire_Bx*sEF) ) ) ) + tire_Sv;
% 
% sEF = atan(vFy / abs(vFx)) + tire_Sh;
% muFy = -tire_Dy*sin( tire_Cy*atan( tire_By*sEF - tire_E*(tire_By*sEF - atan(tire_By*sEF) ) ) ) + tire_Sv;
% sEF = atan(vRy / abs(vRx)) + tire_Sh;
% muRy = -tire_Dy*sin( tire_Cy*atan( tire_By*sEF - tire_E*(tire_By*sEF - atan(tire_By*sEF) ) ) ) + tire_Sv;

% fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * muRx) / (m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h * (muFx * cos(delta) - muFy * sin(delta) - muRx));
% fFz = m_Vehicle_m * m_g * (m_Vehicle_lR / lFR);
% fRz = m_Vehicle_m * m_g - fFz;

% fFx = fFz * muFx;
% fRx = fRz * muRx;
% fFy = fFz * muFy;
% fRy = fRz * muRy;

% fFx = input(6);
% fFy = input(7);
% fRx = input(8);
% fRy = input(9);
%%%%%%%%%%%%%%%%%%%

dot_X = vx .* cos(Yaw) - vy .* sin(Yaw);
dot_Y = vx .* sin(Yaw) + vy .* cos(Yaw);

next_state = zeros(m_nx, 1);
next_state(1, 1) = vx + deltaT * ((fFx * cos(delta) - fFy * sin(delta) + fRx) / m_Vehicle_m + vy * wz);
next_state(2, 1) = vy + deltaT * ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m - vx * wz);
next_state(3, 1) = wz + deltaT * ((fFy*cos(delta) + fFx*sin(delta)) * m_Vehicle_lF - fRy * m_Vehicle_lR) / m_Vehicle_Iz;

% fafy = input(7,1);
% fary = input(8,1);
% % next_state(2, 1) = vy + deltaT * ((fafy + fary) / m_Vehicle_m - vx * wz);
% next_state(3, 1) = wz + deltaT * ((fafy) * m_Vehicle_lF - fary * m_Vehicle_lR) / m_Vehicle_Iz;
% wz_dot = input(9,:);
% next_state(3, 1) = wz + deltaT * wz_dot;

next_state(4, 1) = wF - deltaT * m_Vehicle_rF / m_Vehicle_IwF * fFx;
% next_state(5, 1) = wR + deltaT * (m_Vehicle_kTorque * (T-wR) - m_Vehicle_rR * fRx) / m_Vehicle_IwR;
next_state(6, 1) = Yaw + deltaT * (wz);
next_state(7, 1) = X + deltaT * dot_X;
next_state(8, 1) = Y + deltaT * dot_Y;
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
forces = zeros(8,1);
forces(1,1) = fFx;
forces(2,1) = fFy;
forces(3,1) = fFz;
forces(4,1) = fRx;
forces(5,1) = fRy;
forces(6,1) = fRz;
forces(7,1) = (fFx * cos(delta) - fFy * sin(delta) + fRx) / m_Vehicle_m - vy * wz;
forces(8,1) = ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m + vx * wz);

% forces = zeros(5,1);
% forces(2,1) = alphaF;
% forces(3,1) = alphaR;
end


