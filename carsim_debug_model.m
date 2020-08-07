N = 1;
Nf = 0;
state = states(:,N);
time = (1:length(states))/100;%1:20000;
analytic_states = zeros(length(state), length(time));
forces = zeros(8, length(time));
% figure;
% plot(time, inputs)
for ii = N:length(time)
% state(3) = states(3,ii);
[state] = update_dynamics(state, inputs(:,ii), 0);
% state(1) = states(1,ii);
% state(3) = states(3,ii);
% state(3) = state(3) - 0.0279;
state(1) = states(1,ii);
state(2) = states(2,ii);
% state(3) = wzs(ii);
% state(6) = wzs(ii,4);
analytic_states(:,ii) = state;
% forces(:,ii) = force;
% dots(:,ii) = dotii;
% state = states(:,ii);
end
% yaw_error = states(6,N+1:end-Nf) - analytic_states(6,N:end-Nf-1);
% yaw_error = yaw_error(end-Nf)
wz_error = abs(states(3,N+1:end-Nf) - analytic_states(3,N:end-Nf-1));
wz_error_int = sum(wz_error(1:end) * 0.01)
% wz_error_mean = mean(wz_error)
% analytic_states(2,:)=-1*analytic_states(2,:);
figure;
for ii = 1:length(state)
    subplot(length(state)/2, 2, ii);
    hold on
    plot(time(N:end-Nf), states(ii, N:end-Nf))
    plot(time(N:end-Nf), analytic_states(ii, N:end-Nf))
end
% figure;
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

function [next_state] = update_dynamics(state, input, wz_error_mean)
m_Vehicle_m = 1270;%21.7562;
m_Vehicle_Iz = 1537;%2.5;
lFR = 2.910;%0.57;
m_Vehicle_lF = 1.015;%0.4;
m_Vehicle_lR = m_Vehicle_lF - lFR;%0.57
m_Vehicle_rF = 0.325;%0.095;
m_Vehicle_rR = 0.325;%0.090;
m_Vehicle_h = 0.54;%.2;%0.2;
m_Vehicle_w = 1.54;
m_g = 9.80665;
dt = 0.01;

tire_By = 10;%1.5;
tire_Cy = 1.9;%1.5;
tire_Dy = 1.0;
tire_E = 0.97;
tire_Sh = -0.00;
tire_Sv = 0.00;
tire_Bx = tire_By;
tire_Cx = tire_Cy;
tire_Dx = tire_Dy;

sw = input(1);
delta = (input(2) + input(3))/2;
phiL = input(2);
phiR = input(3);
% % eps = 1e-;
% % if abs(phiL) < eps
% %     phiL = eps;
% % end
% % if abs(phiR) < eps
% %     phiR = eps;
% % end
% if sw < 0
%     phiI = phiL;
%     phiO = phiR;
% else
%     phiI = phiR;
%     phiO = phiL;
% end
% % 
% phi1 = acot(cot(phiI) + m_Vehicle_w / (2*lFR));
% phi2 = acot(cot(phiO) - m_Vehicle_w / (2*lFR));
% % 
% delta = (phi1 + phi2)/2;
% % if abs(delta) > 0.2
% %     delta = 0;
% % end
C=4000;
Ff = -C*(input(4)+input(5))/2;
Fr = -C*(input(6)+input(7))/2;

vx = state(1, 1);
vy = state(2, 1);
v = sqrt(vx.^2 + vy.^2);
beta1 = atan2(vy,vx);
beta = atan(m_Vehicle_lR/lFR * tan(delta));
wz = state(3, 1);
yaw = state(4, 1);
% wF = state(4, 1);
% wR = state(5, 1);
% Yaw = state(6, 1);
% X = state(7, 1);
% Y = state(8, 1);

next_state = zeros(length(state), 1);
next_state(2) = delta;
% next_state(3) = v / lFR * cos(beta) * tan(delta);
next_state(3) = 1/m_Vehicle_Iz*(m_Vehicle_lF*Ff-m_Vehicle_lR*Fr);
next_state(4) = yaw + dt*wz;

end


