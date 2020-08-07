N = 1;
Nf = 0;
state = states(1:8,N);
state = [state; 0;0;0; 0;0;0; states(11,N); states(12,N)];
time = (1:length(states))/1000;%1:20000;
analytic_states = zeros(length(state), length(time));

% dots = zeros(length(state), length(time));
% inputs = 0.5*sin(time/1000) + input;
% figure;
% plot(time, inputs)
for ii = N:length(time)
state = update_dynamics(state, inputs(:,ii));
% state(3) = wzs(ii);
state([4]) = states(9,ii);
state([5]) = states(10,ii);
state(15) = states(11,ii);
state(16) = states(12,ii);
analytic_states(:,ii) = state;
end

figure;
for ii = 1:8
    subplot(8/2, 2, ii);
    hold on
    if ii <= 8
        plot(time(N:end-Nf), states(ii, N:end-Nf))
    end
    plot(time(N:end-Nf), analytic_states(ii, N:end-Nf))
end

function next_state = update_dynamics(state, input)
m_Vehicle_m = 1270;%22.0262;
Iz = 1537;%1.2311;
lFR = 2.910;
lf = 1.015;%.3092;
lr = lFR-lf;
h = 0.54;%.1159;
m_Vehicle_IwF = 8;%.0416;
m_Vehicle_IwR = 8;%.0373;

Cd=0.3*0;
Ix=Iz/3;
Iy=Iz;
Kf=27000;
Kr=27000;
Cf=10000;
Cr=10000;
L_axis=lFR;
ms=m_Vehicle_m;%22.0262;
mu1=71/2;%.75;
mu2=71/2;%0.9;
m=ms+mu1+mu2;
Wa=1.540;%0.44;
Wb=1.540;%0.46;
mf=mu1/2;
mr=mu2/2;
g=9.80665;
r_F=0.325;%0.095;
r_R=0.325;%0.095;
rho_A=1.206;
Area=2.2;

m_Vehicle_kSteering = 19.01;
m_Vehicle_cSteering = 0.0122;
m_Vehicle_kThrottle = 204.0922;
m_Vehicle_kTorque = 0.1577;

B = 1.1433;%1.1559;
C = 1.1277;%1.1924;
D = 1.4031;%0.9956;
E = 0.1368;
Sh = 0.7;
Sv = 0.999;

tire_By = 10;%1.9;
tire_Cy = 2;%1.5;
tire_Dy = 1;%1.1;
tire_E = 1.0;
tire_Sh = -0.00;
tire_Sv = 0.00;
tire_Bx = tire_By;
tire_Cx = tire_Cy;
tire_Dx = tire_Dy;

m_nx = 8;
m_nu = 2;
dots = zeros(m_nx, 1);

vx = state(1, 1);
vy = state(2, 1);
wz = state(3, 1);
w_lf = state(4,1);
w_lr = state(5,1);
psi = state(6, 1);
X = state(7, 1);
Y = state(8, 1);
vz = state(9, 1);
vtheta = state(10, 1);
vphi = state(11, 1);
z = state(12,1);
theta = state(13,1);
phi = state(14,1);
w_rf = state(15,1);
w_rr = state(16,1);

delta = m_Vehicle_kSteering * input(1, 1) + m_Vehicle_cSteering;
delta = (input(1,1));
T = 0;
% w_lf=T;
% w_rf=T;
% w_lr=T;
% w_rr=T;

min_velo = 0.1;
dt = 0.001;

if (vx < min_velo)
  vx = min_velo;
end
if (w_lf < min_velo / r_F)
    w_lf = min_velo / r_F + 1.2e-4;
end
if w_rf < min_velo / r_F
    w_rf = min_velo / r_F + 0.9e-4;
end
if (w_lr < min_velo / r_R)
    w_lr = min_velo / r_R + 1.1e-4;
end
if w_rr < min_velo / r_R
    w_rr = min_velo / r_R + 1e-4;
end
if (vx ~= 0.0)
  beta = atan2(vy, vx);
else
  beta = 0.0;
end

   % wheel velocity in body-fixed frame
    Vlf=[vx  vy vz]'+ cross([vphi vtheta wz]', [lf Wa/2 r_F-h]'); % beta is measured here. I may use beta=atan(Vy/Vx);
    Vrf=[vx  vy vz]'+ cross([vphi vtheta wz]', [lf -Wa/2 r_F-h]');
    Vlr=[vx vy vz]'+ cross([vphi vtheta wz]', [-lr Wb/2 r_R-h]');
    Vrr=[vx  vy vz]'+ cross([vphi vtheta wz]', [-lr -Wb/2 r_R-h]');
    
   % wheel velocity in wheel-fixed frame
   Tran=[cos(delta) sin(delta) 0; -sin(delta) cos(delta) 0; 0 0 1];
   Vlf=Tran*Vlf;
   Vrf=Tran*Vrf;
   
   Vlf_x=Vlf(1); Vlf_y=Vlf(2); % don't consider the verticle vibration of the wheel here
   Vrf_x=Vrf(1); Vrf_y=Vrf(2);
   Vlr_x=Vlr(1); Vlr_y=Vlr(2);
   Vrr_x=Vrr(1); Vrr_y=Vrr(2);   
   
   slf_x=( Vlf_x-w_lf*r_F )/( w_lf*r_F );
   slf_y=Vlf_y/( w_lf*r_F );
   slf= ( slf_x^2+slf_y^2 )^0.5;
 
   srf_x=( Vrf_x-w_rf*r_F )/( w_rf*r_F );
   srf_y=Vrf_y/( w_rf*r_F );
   srf= ( srf_x^2+srf_y^2 )^0.5;
   
   slr_x=( Vlr_x-w_lr*r_R )/( w_lr*r_R );
   slr_y=Vlr_y/( w_lr*r_R );
   slr= ( slr_x^2+slr_y^2 )^0.5;
 
   srr_x=( Vrr_x-w_rr*r_R )/( w_rr*r_R );
   srr_y=Vrr_y/( w_rr*r_R );
   srr= ( srr_x^2+srr_y^2 )^0.5;
     
   mu_slf=D*sin( C*atan( B*( (1-E)*(slf+Sh)+E/B*atan(B*(slf+Sh)) ) ) ) +Sv;
   mu_srf=D*sin( C*atan( B*( (1-E)*(srf+Sh)+E/B*atan(B*(srf+Sh)) ) ) ) +Sv;
   mu_slr=D*sin( C*atan( B*( (1-E)*(slr+Sh)+E/B*atan(B*(slr+Sh)) ) ) ) +Sv;
   mu_srr=D*sin( C*atan( B*( (1-E)*(srr+Sh)+E/B*atan(B*(srr+Sh)) ) ) ) +Sv;
   
   mu_lfx=-slf_x*mu_slf/slf;
   mu_lfy=-slf_y*mu_slf/slf;
   mu_rfx=-srf_x*mu_srf/srf;
   mu_rfy=-srf_y*mu_srf/srf;    
   mu_lrx=-slr_x*mu_slr/slr;
   mu_lry=-slr_y*mu_slr/slr;
   mu_rrx=-srr_x*mu_srr/srr;
   mu_rry=-srr_y*mu_srr/srr;
   
    %%%
    sEF = -(Vlf_x - w_lf * r_F) / (Vlf_x) + tire_Sh;
    mu_lfx = tire_Dx*sin( tire_Cx*atan( tire_Bx*sEF - tire_E*(tire_Bx*sEF - atan(tire_Bx*sEF) ) ) ) + tire_Sv;
    sEF = -(Vrf_x - w_rf * r_F) / (Vrf_x) + tire_Sh;
    mu_rfx = tire_Dx*sin( tire_Cx*atan( tire_Bx*sEF - tire_E*(tire_Bx*sEF - atan(tire_Bx*sEF) ) ) ) + tire_Sv;
    sEF = -(Vlr_x - w_lr * r_R) / (Vlr_x) + tire_Sh;
    mu_lrx = tire_Dx*sin( tire_Cx*atan( tire_Bx*sEF - tire_E*(tire_Bx*sEF - atan(tire_Bx*sEF) ) ) ) + tire_Sv;
    sEF = -(Vrr_x - w_rr * r_R) / (Vrr_x) + tire_Sh;
    mu_rrx = tire_Dx*sin( tire_Cx*atan( tire_Bx*sEF - tire_E*(tire_Bx*sEF - atan(tire_Bx*sEF) ) ) ) + tire_Sv;

    sEF = atan(Vlf_y / abs(Vlf_x)) + tire_Sh;
    mu_lfy = -tire_Dy*sin( tire_Cy*atan( tire_By*sEF - tire_E*(tire_By*sEF - atan(tire_By*sEF) ) ) ) + tire_Sv;
    sEF = atan(Vrf_y / abs(Vrf_x)) + tire_Sh;
    mu_rfy = -tire_Dy*sin( tire_Cy*atan( tire_By*sEF - tire_E*(tire_By*sEF - atan(tire_By*sEF) ) ) ) + tire_Sv;
    sEF = atan(Vlr_y / abs(Vlr_x)) + tire_Sh;
    mu_lry = -tire_Dy*sin( tire_Cy*atan( tire_By*sEF - tire_E*(tire_By*sEF - atan(tire_By*sEF) ) ) ) + tire_Sv;
    sEF = atan(Vrr_y / abs(Vrr_x)) + tire_Sh;
    mu_rry = -tire_Dy*sin( tire_Cy*atan( tire_By*sEF - tire_E*(tire_By*sEF - atan(tire_By*sEF) ) ) ) + tire_Sv;
    %%%
   
   Flf0=m*g*lr/L_axis/2;
   Frf0=Flf0;
   Flr0=m*g*lf/L_axis/2;
   Frr0=Flr0;  

   dFlf=Kf*(lf*theta-Wa/2*phi-z)+Cf*(lf*vtheta-Wa/2*vphi-vz);
   dFrf=Kf*(lf*theta+Wa/2*phi-z)+Cf*(lf*vtheta+Wa/2*vphi-vz);
   dFlr=Kr*(-lr*theta-Wb/2*phi-z)+Cr*(-lr*vtheta-Wb/2*vphi-vz);
   dFrr=Kr*(-lr*theta+Wb/2*phi-z)+Cr*(-lr*vtheta+Wb/2*vphi-vz);
   
   TireForce_z=[ Flf0+dFlf Frf0+dFrf Flr0+dFlr Frr0+dFrr ];

   f_lfx=mu_lfx*TireForce_z(1);
   f_lfy=mu_lfy*TireForce_z(1);
   f_rfx=mu_rfx*TireForce_z(2);
   f_rfy=mu_rfy*TireForce_z(2);
   f_lrx=mu_lrx*TireForce_z(3);
   f_lry=mu_lry*TireForce_z(3);
   f_rrx=mu_rrx*TireForce_z(4);
   f_rry=mu_rry*TireForce_z(4);
%    f_lfx = input(2);
%    f_lrx = input(3);
%    f_rfx = input(4);
%    f_rrx = input(5);
%    f_lfy = input(6);
%    f_lry = input(7);
%    f_rfy = input(8);
%    f_rry = input(9);
   
   dot_Vx=( ( f_lfx+f_rfx )*cos( delta )-( f_lfy+f_rfy )*sin( delta )+( f_lrx+f_rrx ) )/m+vy*wz-0.5*Cd*rho_A*Area*vx^2;  %no aerodynamics
   dot_Vy=( ( f_lfx+f_rfx )*sin( delta )+( f_lfy+f_rfy )*cos( delta )+( f_lry+f_rry ) )/m-vx*wz;  
   dot_r=( ( ( f_lfy+f_rfy )*cos( delta )+( f_lfx+f_rfx )*sin( delta ) )*lf-( f_lry+f_rry )*lr )/Iz; 
   
   T_rotation=[ cos(psi) sin(psi); -sin(psi) cos(psi) ];
   dot_pos=T_rotation'*[ vx; vy ];
   
   dot_XI=dot_pos(1);
   dot_YI=dot_pos(2); 
   
  %% calcualte x and  y acceleration of each wheel center, ignore vertical acceleration
   Acc_lf=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [lf Wa/2 r_F-h]')+cross([vphi vtheta wz]',cross([vphi vtheta wz]', [lf Wa/2 r_F-h]'));
   Acc_rf=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [lf -Wa/2 r_F-h]')+cross([vphi vtheta wz]',cross([vphi vtheta wz]', [lf -Wa/2 r_F-h]'));
   Acc_lr=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [-lr Wb/2 r_F-h]')+cross([vphi vtheta wz]',cross([vphi vtheta wz]', [-lr Wb/2 r_R-h]'));
   Acc_rr=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [-lr -Wb/2 r_F-h]')+cross([vphi vtheta wz]',cross([vphi vtheta wz]', [-lr -Wb/2 r_R-h]')); 
   
   F_slfx=f_lfx*cos( delta )-f_lfy*sin( delta )-mf*Acc_lf(1);    F_slfy=f_lfx*sin( delta )+f_lfy*cos( delta )-mf*Acc_lf(2);
   F_srfx=f_rfx*cos( delta )-f_rfy*sin( delta )-mf*Acc_rf(1);    F_srfy=f_rfx*sin( delta )+f_rfy*cos( delta )-mf*Acc_rf(2);    
   F_slrx=f_lrx-mr*Acc_lr(1);    F_slry=f_lry-mr*Acc_lr(2);   
   F_srrx=f_rrx-mr*Acc_rr(1);    F_srry=f_rry-mr*Acc_rr(2);   
   
   dot_VZs=( -2*(Kf+Kr)*z-2*(Cf+Cr)*vz+2*(lf*Kf-lr*Kr)*theta+2*(lf*Cf-lr*Cr)*vtheta )/ms;
   dot_Vtheta=( 2*(lf*Kf-lr*Kr)*z+2*(lf*Cf-lr*Cr)*vz-2*(lf^2*Kf+lr^2*Kr)*theta-2*(lf^2*Cf+lr^2*Cr)*vtheta-(F_slfx+F_srfx)*(h-r_F)-(F_slrx+F_srrx)*(h-r_R) )/Iy;
   dot_Vphi=( -Wa^2*Kf*phi/2-Wa^2*Cf*vphi/2-Wb^2*Kr*phi/2-Wb^2*Cr*vphi/2 + (F_slfy+F_srfy)*(h-r_F)+(F_slry+F_srry)*(h-r_R) )/Ix;
   
   dot_w_lf = - r_F * f_lfx / m_Vehicle_IwF;
   dot_w_rf = - r_F * f_rfx / m_Vehicle_IwF;
   dot_w_lr = (m_Vehicle_kTorque * (T-w_lr) - r_R * f_lrx) / m_Vehicle_IwR;
   dot_w_rr = (m_Vehicle_kTorque * (T-w_rr) - r_R * f_rrx) / m_Vehicle_IwR;
   
   next_state = state+[ dot_Vx; dot_Vy; dot_r; dot_w_lf; dot_w_lr; wz; dot_XI; dot_YI; dot_VZs; dot_Vtheta; dot_Vphi; vz; vtheta; vphi; dot_w_rf; dot_w_rr]*dt;
   

% next_state = zeros(m_nx, 1);
% next_state(1, 1) = vx + deltaT * ((fFx * cos(delta) - fFy * sin(delta) + fRx) / m_Vehicle_m + vy * wz);
% next_state(2, 1) = vy + deltaT * ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m - vx * wz);
% next_state(3, 1) = wz + deltaT * ((fFy * cos(delta) + fFx * sin(delta)) * m_Vehicle_lF - fRy * m_Vehicle_lR) / m_Vehicle_Iz;
% next_state(4, 1) = wF - deltaT * m_Vehicle_rF / m_Vehicle_IwF * fFx;
% next_state(5, 1) = wR + deltaT * (m_Vehicle_kTorque * (T-wR) - m_Vehicle_rR * fRx) / m_Vehicle_IwR;
% next_state(6, 1) = epsi + deltaT * (wz - (vx * cos(epsi) - vy * sin(epsi)) / (1 - curvature * ey) * curvature);
% epsi = epsi + 0.1258;
% next_state(7, 1) = ey + deltaT * (vx * sin(epsi) + vy * cos(epsi));
% next_state(8, 1) = s + deltaT * (vx * cos(epsi) - vy * sin(epsi)) / (1 - curvature * ey);
% next_state(9, 1) = X + deltaT * vx * cos(epsi) - vy * sin(epsi);
% next_state(10, 1) = Y + deltaT * vx * sin(epsi) + vy * cos(epsi);
end


