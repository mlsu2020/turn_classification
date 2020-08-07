clc
N = 2000;
Nf = 150000;
ddd = 0.000001;
subset_states = states(:,N:10:end-Nf);
subset_inputs = [inputs(1,N:10:end-Nf); states(5,N:10:end-Nf)];
state = subset_states(:,1)
state(4) = state(5);
control = subset_inputs(:,1)
% time = (1:10000)/100;
time = (1:length(subset_states))/100;
sim_states = zeros(length(state), length(time));
% x_target = subset_states(:,end);
x_target = subset_states(:,2);
[control, xs, us] = LTIMPC(state, control, x_target);
state = update_dynamics(state, control)
xs{1} = state;
control
for ii = 1:length(time)-2
    x_target = subset_states(:,ii+2);
    [control, xs, us] = LTIMPC(xs{1}, control, x_target);
    if isnan(control)
        break
    end
    state = update_dynamics(state, control);
    xs{1} = state;
    sim_states(:,ii) = state;
end
%%
plot_subset = subset_states([1,2,3,6,7,8], :);
plot_sim = sim_states([1,2,3,6,7,8], :);
state = plot_subset(:,1);
figure;
for ii = 1:length(state)
    subplot(length(state)/2, 2, ii);
    hold on
    plot(time(:,1:end-2), plot_subset(ii, 1:end-2))
    plot(time(:,1:end-2), plot_sim(ii, 1:end-2))
end
subplot(length(state)/2,2,1);
legend('Goal vx','Realized vx');
xlabel('t (s)');
ylabel('m/s')
subplot(length(state)/2,2,2);
legend('Goal vy','Realized vy');
xlabel('t (s)');
ylabel('m/s');
subplot(length(state)/2,2,3);
legend('Goal yaw-rate','Realized yaw-rate');
xlabel('t (s)');
ylabel('rad/s');
% subplot(4,2,4);
% legend('Goal wF','Simulated wF','Carsim wF');
% xlabel('t (s)');
% ylabel('rad/s');
% subplot(4,2,5);
% legend('True wR','Simulated wR','Carsim wR');
% xlabel('t (s)');
% ylabel('rad/s');
subplot(length(state)/2,2,4);
legend('Goal Yaw','Realized Yaw');
xlabel('t (s)');
ylabel('rad');
subplot(length(state)/2,2,5);
legend('Goal X','Realized X');
xlabel('t (s)');
ylabel('m');
subplot(length(state)/2,2,6);
legend('Goal Y','Realized Y');
xlabel('t (s)');
ylabel('m');

% test_models(states, inputs);

function [control, xs, us] = LTVMPC(xs, us)
    Q = zeros(8);
    Q(1,1) = 25;
    Q(2,2) = 1;
    Q(3,3) = 5;
    QT = zeros(8);
    R = zeros(2);
    R(1,1) = 200;
    R(2,2) = 0.00001;
    x_target = [5;0;0;50;50;0;0;0];
    [A_0, B_0, d_0] = linearize_dynamics(xs{1}, us{1}, 0.1);
    A = cell(12,1);
    B = cell(12,1);
    d = cell(12,1);
    for ii = 2:12
        [A{ii-1}, B{ii-1}, d{ii-1}] = linearize_dynamics(xs{ii}, us{ii}, 0.1);
    end
    [A{12}, B{12}, d{12}] = linearize_dynamics(xs{13}, us{12}, 0.1);
    params.A_0 = A_0;
    A = zeros(8);
    params.A = {A_0,A,A,A,A,A,A,A,A,A,A,A};
    params.B_0 = B_0;
%     B = zeros([8,2]);
    B = B_0;
    params.B = {B,B,B,B,B,B,B,B,B,B,B,B};
%     params.B = B;
    params.Q = Q;
    params.QT = QT;
    params.R = R;
    params.d_0 = d_0;
%     d = zeros(8,1);
    d = d_0;
    params.d = {d,d,d,d,d,d,d,d,d,d,d,d};
%     params.d = d;
    params.half_road_width = 900;
    params.target = x_target;
    params.umax = [0.044; 80];
    params.x_0 = xs{1};
    settings.verbose = 0;
%     settings.max_iters = 100;
%     settings.eps = 0.01;
%     settings.resid_tol = 1e-2;
    [vars, status] = csolve(params, settings);
    if status.converged
        control = vars.u_0;
        us = vars.u;
        xs = vars.x;
        if control(2) < 0
            control(2) = 10;
        end
    else
        control = nan;
        us = nan;
        xs = nan;
%         'nonconvered'
    end
end

function [control, xs, us] = LTIMPC(x_0, u_0, x_target)
    Q = zeros(8);
    Q(1,1) = 25;
    Q(2,2) = 0;
    Q(3,3) = 5;
%     Q(6,6) = 100;
%     Q(7,7) = 10;
%     Q(8,8) = 10;
    QT = Q;
    R = zeros(2);
    R(1,1) = 10;
    R(2,2) = 0.0001;
%     x_target = [5;0;0;50;50;0;0;0];
    % x_0 = [3;0;0;30;30;0;0;0];
    % u_0 = [0; 30];
    [A, B, d] = linearize_dynamics(x_0, u_0, 0.000001)
    params.A_0 = A;
%     A0 = eye(8);
    A0 = A;
    params.A = {A,A,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0};
    params.B_0 = B;
%     B0 = zeros([8,2]);
    B0 = B;
    params.B = {B,B,B0,B0,B0,B0,B0,B0,B0,B0,B0,B0};
    params.Q = Q;
    params.QT = QT;
    params.R = R;
    params.d_0 = d;
%     d0 = zeros([8,1]);
    d0 = d;
    params.d = {d,d,d0,d0,d0,d0,d0,d0,d0,d0,d0,d0};
    params.half_road_width = 900;
    params.target = x_target;
    params.umax = [0.02; 50];
    params.x_0 = x_0;
    settings.verbose = 0;
%     settings.max_iters = 100;
%     settings.eps = 0.01;
%     settings.resid_tol = 1e-2;
    [vars, status] = csolve(params, settings);
    if status.converged
        control = vars.u_0;
        xs = vars.x;
        us = vars.u;
        if control(2) < 0
            control(2) = 10;
        end
    else
        control = nan;
        us = vars.u;
        xs = vars.x;
%         'nonconvered'
    end
end

function test_models(states, inputs)
    N = 2000;
    Nf = 2000;
    ddd = 0.000001;
    subset_states = states(:,N:10:end-Nf);
    subset_inputs = [inputs(1,N:10:end-Nf); states(5,N:10:end-Nf)];
    state = subset_states(:,1);
    time = (1:length(subset_states))/100;
    nonlinear_states = zeros(length(state), length(time));
    linear_states = zeros(length(state), length(time));
    non_state = state;
    lin_state = state;

    for ii = 1:length(time)
    non_state = update_dynamics(non_state, subset_inputs(:,ii));
    nonlinear_states(:,ii) = non_state;
    [A, B, d] = linearize_dynamics(lin_state, subset_inputs(:,ii), ddd);
    lin_state = A*lin_state + B*subset_inputs(:,ii) + d;
    linear_states(:,ii) = lin_state;
    end

    figure;
    for ii = 1:length(state)
        subplot(length(state)/2, 2, ii);
        hold on
        plot(time, subset_states(ii, :))
        plot(time, nonlinear_states(ii, :))
        plot(time, linear_states(ii, :))
    end
end

function [A, B, d] = linearize_dynamics(state, control, delta)
dx = length(state);
du = length(control);

x = repmat(state, [1,dx]);
u = repmat(control, [1,dx]);
delta_x = eye(dx) * delta;
x_plus = x + delta_x;
x_minus = x - delta_x;
f_plus = update_dynamics(x_plus, u);
f_minus = update_dynamics(x_minus, u);
A = (f_plus - f_minus) / (2*delta);

x = repmat(state, [1,du]);
u = repmat(control, [1,du]);
delta_u = eye(du) * delta;
u_plus = u + delta_u;
u_minus = u - delta_u;
f_plus = update_dynamics(x, u_plus);
f_minus = update_dynamics(x, u_minus);
B = (f_plus - f_minus) / (2*delta);

next_state = update_dynamics(state, control);
d = next_state - A*state - B*control;
end

function next_state = update_dynamics(state, control)
m_Vehicle_m = 21.7562;%1270;%21.7562;
m_Vehicle_Iz = 1.124;%2000;%2.5;
lFR = 0.57;%2.91;%0.57;
m_Vehicle_lF = 0.34;%1.022;%0.4;
m_Vehicle_lR = lFR-m_Vehicle_lF;%0.57
m_Vehicle_IwF = 0.01;%8.0;%4.02;
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
N = size(state,2);

vx = state(1, :);
vy = state(2, :);
wz = state(3, :);
wF = state(4, :);
wR = control(2, :);
Yaw = state(6, :);
X = state(7, :);
Y = state(8, :);  

delta = m_Vehicle_kSteering * control(1, :) + m_Vehicle_cSteering;
% delta = (input(2) + input(3))/2;
% delta = input(1);
% T = m_Vehicle_kThrottle * input(2, 1);

min_velo = 0.1;
deltaT = 0.01;

% if (vx < min_velo)
%   vx = min_velo;
% end
% if (wF < min_velo / m_Vehicle_rF)
%     wF = min_velo / m_Vehicle_rF + 1e-4;
% end
% if wR < min_velo / m_Vehicle_rR
%     wR = min_velo / m_Vehicle_rR + 1e-4;
% end
% if (vx ~= 0.0)
beta = atan2(vy, vx);
% else
%   beta = 0.0;
% end

V = sqrt(vx .* vx + vy .* vy);
vFx = V .* cos(beta - delta) + wz .* m_Vehicle_lF .* sin(delta);
vFy = V .* sin(beta - delta) + wz .* m_Vehicle_lF .* cos(delta);
vRx = vx;
vRy = vy - wz .* m_Vehicle_lR;

alphaF = -atan(vFy ./ abs(vFx));
alphaR = -atan(vRy ./ abs(vRx));

% if (wF ~= 0.0)
  sFx = (vFx - wF .* m_Vehicle_rF) ./ (wF .* m_Vehicle_rF);
% else
%   sFx = 0.0;
% end
% if (wR ~= 0.0)
  sRx = (vRx - wR .* m_Vehicle_rR) ./ (wR .* m_Vehicle_rR);
% else
%   sRx = 0.0;
% end
% if (vFx ~= 0.0)
  sFy = vFy ./ (wF .* m_Vehicle_rF);
%   sFy = (1 + sFx) * vFy / vFx;
% else
%   sFy = 0.0;
% % end
% if (vRx ~= 0.0)
  sRy = vRy ./ (wR .* m_Vehicle_rR);
%   sRy = (1 + sRx) * vRy / vRx;
% else
%   sRy = 0.0;
% end

sF = sqrt(sFx .* sFx + sFy .* sFy);
sR = sqrt(sRx .* sRx + sRy .* sRy);

sEF = sF - tire_Sh;
sER = sR - tire_Sh;

% muF = tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;
% muR = tire_D*sin( tire_C*atan( tire_B*sER - tire_E*(tire_B*sER - atan(tire_B*sER) ) ) ) + tire_Sv;
muF = tire_D .* sin(tire_C .* atan(tire_B .* sF)); 
muR = tire_D .* sin(tire_C .* atan(tire_B .* sR));

muFx = -sFx ./ sF .* muF;
muFy = -sFy ./ sF .* muF;
muRx = -sRx ./ sR .* muR;
muRy = -sRy ./ sR .* muR;

fFz = m_Vehicle_m .* m_g .* (m_Vehicle_lR - m_Vehicle_h .* muRx) ./ (m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h .* (muFx .* cos(delta) - muFy .* sin(delta) - muRx));
% fFz = m_Vehicle_m * m_g * (m_Vehicle_lR / lFR);
fRz = m_Vehicle_m .* m_g - fFz;

fFx = muFx .* fFz;
fFy = tire_a.*muFy .* fFz;
fRx = muRx .* fRz;
fRy = tire_a.*muRy .* fRz;

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

next_state = zeros(m_nx, N);
next_state(1, :) = vx + deltaT .* ((fFx .* cos(delta) - fFy .* sin(delta) + fRx) ./ m_Vehicle_m + vy .* wz);
next_state(2, :) = vy + deltaT .* ((fFx .* sin(delta) + fFy .* cos(delta) + fRy) ./ m_Vehicle_m - vx .* wz);
next_state(3, :) = wz + deltaT .* ((fFy.*cos(delta) + fFx.*sin(delta)) .* m_Vehicle_lF - fRy .* m_Vehicle_lR) ./ m_Vehicle_Iz;

% fafy = input(7,1);
% fary = input(8,1);
% % next_state(2, 1) = vy + deltaT * ((fafy + fary) / m_Vehicle_m - vx * wz);
% next_state(3, 1) = wz + deltaT * ((fafy) * m_Vehicle_lF - fary * m_Vehicle_lR) / m_Vehicle_Iz;
% wz_dot = input(9,:);
% next_state(3, 1) = wz + deltaT * wz_dot;

next_state(4, :) = wR;%wF - deltaT .* m_Vehicle_rF ./ m_Vehicle_IwF .* fFx;
next_state(5, :) = wR;
next_state(6, :) = Yaw + deltaT .* (wz);
next_state(7, :) = X + deltaT .* dot_X;
next_state(8, :) = Y + deltaT .* dot_Y;

% state = next_state;
end
