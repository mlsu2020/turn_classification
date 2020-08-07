function [ para_opt, J, X_train, Y_opt, Y_train ] = XXVII_Adaptive_Limit_Memory_opt_Joint_UKF_11Dof (XI, YI, YawAngle2, Vx, Vy, r, RollAngle, PitchAngle, steering, steering2, w_lf, w_lr, w_rf, w_rr, dt)
%OPTIMAL_NOISE_SEARCHING Summary of this function goes here

%   Data setup for Carsim

%% scaling using logarithm

L_axis=3.048;
ms=1653;
mu1=90;
mu2=90;
m=ms+mu1+mu2;

Wa=1.6;
Wb=1.602;
% m=21.5;
mf=mu1/2;
mr=mu2/2;
g=9.81;
r_F=0.359;
r_R=0.359;
rho_A=1.206; % air density
Area=1.8; %(reference area =0.138)

%% Known parameters
% lf=1.402;
% Ix=614;
% Iy=2765;
% Iz=2765;
% h=0.59;
% scl=14.336;
% Kf=34000;
% Cf=2500: %not accurate
% Kr=46000;
% Cr=2500;

Eps=1e-12;

Index=find(Vx>2); %10
window0=60; %112
% window0=37;
window=window0/dt; % take part of data for estimation and validation
Vx=Vx( Index(1):window+Index(1) );
Vy=Vy( Index(1):window+Index(1) );
r=r( Index(1):window+Index(1) );
XI=XI( Index(1):window+Index(1) );
YI=YI( Index(1):window+Index(1) );
YawAngle=YawAngle2( Index(1):window+Index(1) );
RollAngle=RollAngle( Index(1):window+Index(1) );
PitchAngle=PitchAngle( Index(1):window+Index(1) );

steering=steering( Index(1):window+Index(1) );
steering2=steering2( Index(1):window+Index(1) );
% steering2=steering;
w_lf=w_lf( Index(1):window+Index(1) );
w_lr=w_lr( Index(1):window+Index(1) );
w_rf=w_rf( Index(1):window+Index(1) );
w_rr=w_rr( Index(1):window+Index(1) );

dT=5;
dN=dT/dt; %left data of dT for validation
resample=5;
dts=resample*dt;
dNs=dT/dts;

Nr=10/dt;
Nq=10/dt;
Fade=0;
N0=max(Nr,Nq)+5/dt;

tail=0;
Omit=tail/dt;
lth=length(r);
M=window0/((window0-28)+dT); % data split into 2 parts, first half for training
N=round((lth-rem(lth,M))/M)-dN;

Qmax=1e-2;
Qmin=1e-14;
Rmax=1e-2;
Rmin=1e-14;
Pmax=1e4;
Pmin=1e-12;
Pyy_max=1e4;
dK=1;

%%
%           Iz            lf                 h                  B               C                D                  E                  Sh                   Sv                    scl               Cd                Ix                 Iy             Kf              Kr                  Cf              Cr          
para0 = [  2500,          1.5,               0.5,       7.79817410543529,       1.6,              1,         -0.526797954883659,  -0.0514327220075633,  -0.0454297657839960,       14,        0.466447502050303,       500,              2500,         30000,          40000,                2100            2100];
%          Iz         lf        h        B       C       D       E       Sh      Sv     scl      Cd     Ix      Iy        Kf       Kr       Cf       Cr 
para_lb=[  2000       1        0.3       1       0      0.4      -1     -0.2    -0.1    10      0.01    200     2000     30000    40000     500      500   ];   %lower boundry
para_ub=[  3000       2        0.7       10      4      1.05     1       0.2     0.1    20       0.5    1000    3000     40000    50000     4000     4000  ];   %upper boundry
% para_ub=[  3000      1.8       0.7       10      4      1.05     1       0.2     0.1    20       0.5    1000    3000     40000    50000     4000     4000  ];   %upper boundry

%% rescale the parameters
scales=para_ub/2;
para0=para0./scales;
para_lb=para_lb./scales;
para_ub=para_ub./scales;

%%
x0=[zeros(12,1); para0'];
x0(1:6)=[ Vx(1) Vy(1) r(1) XI(1) YI(1) YawAngle(1)]'; x0(11:12)=[ PitchAngle(1) RollAngle(1) ]';
                                                 
Y=[ Vx; Vy; r ; XI; YI; YawAngle; PitchAngle; RollAngle];  % use column vector

% w10=1e-5; w20=1e-6; v0=1e-3; P10=1e-5; P20=1e-4;
w10=1e-6; w20=3e-10; v0=1e-3; P10=1e-5; P20=11e-4;  
w10=1e-6; w20=1e-8; v0=1e-3; P10=1e-5; P20=1e-4;   w21=5e-5; %best results; j=3.48e6


%          state           para        meas          state         para
filter=[w10*ones(1,12) w20*ones(1,17) v0*ones(1,8) P10*ones(1,12) P20*ones(1,17)];   % noise is upto 1% of its covariance   
%%
%   Detailed explanation goes here
   
   v=( filter(30:37) )';
   P01=diag( filter(38:49).^2 );
   P02=diag( filter(50:66).^2 );

   alf=1e-3;
   bet=2;
   L=66;
   kap=0;%3-L;
   
   lam=alf^2*(L+kap)-L;
   Wm0=lam/(L+lam);
   Wc0=lam/(L+lam)+(1-alf^2+bet);
   Wmi=1/(L+lam)/2;
   Wci=Wmi;

   Y=[ Vx  Vy  r  XI  YI  YawAngle PitchAngle RollAngle ]; 
   Ye=Y;
   Yeb=Ye;

   P1=[ P01 0*ones(12,17); 0*ones(17,12) P02 ];  % ok results

   Qw1=diag( filter(1:12).^2 );
   Qw2=diag( filter(13:29).^2 );
   Qw=[ Qw1 0*ones(12,17); 0*ones(17,12) Qw2 ];
   Rv=diag( v.^2 );

   X=zeros(lth,29);
   X(1,:)=x0';
   Xc=X;

   Xa=zeros(lth,66);
   Xa(1,1:29)=x0'; 

   Xe=zeros(lth,29);
   Xe(1,:)=x0';
   Xeb=Xe;
   xv=zeros(dNs,12);
   Xs=zeros(lth,12); %Xs(1,:)=[ Vx(1) Vy(1) r(1)  XI(1)  YI(1)  YawAngle(1) 0 0 0];
  
   Sgm_x2=zeros(29,2*L+1);  
   Sgm_x2s=Sgm_x2;
   Sgm_x2b=Sgm_x2;
   
   q0=zeros(29,1);
   r0=zeros(8,1);
   
   q_buf=zeros(29,N);
   bar_q_buf=zeros(29,N);
   r_buf=zeros(8,N);
   bar_r_buf=zeros(8,N);
   
   Del_buf=zeros(29,29,N);
   Gam_buf=zeros(8,8,N);
   Q_buf=zeros(29,29,N);
   R_buf=zeros(8,8,N);
   
   P1_buf=zeros(29,29,N);
   
   q_buf(:,1)=q0;
   r_buf(:,1)=r0;
   Q_buf(:,:,1)=Qw;
   R_buf(:,:,1)=Rv;   
   P1_buf(:,:,1)=P1;
     
   Q=Qw;
   R=Rv;
   
%%  state estimate   
    for k=1: N0-1

% prediction and observation
    
    root_Pa=(L+lam)^0.5*[ real(sqrtm(P1)) zeros(29,37); zeros(29,29) real(sqrtm(Q)) zeros(29,8); zeros(8,58) real(sqrtm(R)) ];
    
    if ~isreal(root_Pa)
       fprintf('Pa not real \n'); 
    end
    
    Sgm_a=[Xa(k,:)' krone( ones(1,L),Xa(k,:)' )+root_Pa krone( ones(1,L),Xa(k,:)' )-root_Pa ];
    
    Sgm_x1=Sgm_a(1:29,:);
    Sgm_w=Sgm_a(30:58,:);
    Sgm_v=Sgm_a(59:66,:);

   for jj=1:2*L+1 
   
%    Iz   lf    h    B    C    D    E   Sh   Sv   scl   Cd   Ix   Iy   Kf   Kr   Cf    Cr 
   Sgm_x1_aux=Sgm_x1(13:29,jj).*scales';
   Iz=Sgm_x1_aux(1);
   lf=Sgm_x1_aux(2); lr=L_axis-lf;
   h=Sgm_x1_aux(3);
   B=Sgm_x1_aux(4);
   C=Sgm_x1_aux(5);   
   D=Sgm_x1_aux(6);   
   E=Sgm_x1_aux(7);
   Sh=Sgm_x1_aux(8);
   Sv=Sgm_x1_aux(9);
   scl=Sgm_x1_aux(10);
   Cd=Sgm_x1_aux(11);
   Ix=Sgm_x1_aux(12);
   Iy=Sgm_x1_aux(13);  
   Kf=Sgm_x1_aux(14);   
   Kr=Sgm_x1_aux(15);   
   Cf=Sgm_x1_aux(16);   
   Cr=Sgm_x1_aux(17);   
   
   delta=scl*steering;

   % wheel velocity in body-fixed frame
    Vlf=[Sgm_x1(1,jj)  Sgm_x1(2,jj) Sgm_x1(7,jj)]'+ cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [lf Wa/2 r_F-h]'); % beta is measured here. I may use beta=atan(Vy/Vx);
    Vrf=[Sgm_x1(1,jj)  Sgm_x1(2,jj) Sgm_x1(7,jj)]'+ cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [lf -Wa/2 r_F-h]');
    Vlr=[Sgm_x1(1,jj)  Sgm_x1(2,jj) Sgm_x1(7,jj)]'+ cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [-lr Wb/2 r_R-h]');
    Vrr=[Sgm_x1(1,jj)  Sgm_x1(2,jj) Sgm_x1(7,jj)]'+ cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [-lr -Wb/2 r_R-h]');
    
   % wheel velocity in wheel-fixed frame
   Tran=[cos(delta(k)) sin(delta(k)) 0; -sin(delta(k)) cos(delta(k)) 0; 0 0 1];
   Vlf=Tran*Vlf;
   Vrf=Tran*Vrf;
   
   Vlf_x=Vlf(1); Vlf_y=Vlf(2); % don't consider the verticle vibration of the wheel here
   Vrf_x=Vrf(1); Vrf_y=Vrf(2);
   Vlr_x=Vlr(1); Vlr_y=Vlr(2);
   Vrr_x=Vrr(1); Vrr_y=Vrr(2);   
   
   slf_x=( Vlf_x-w_lf(k)*r_F )/( w_lf(k)*r_F );
   slf_y=Vlf_y/( w_lf(k)*r_F );
   slf= ( slf_x^2+slf_y^2 )^0.5;
 
   srf_x=( Vrf_x-w_rf(k)*r_F )/( w_rf(k)*r_F );
   srf_y=Vrf_y/( w_rf(k)*r_F );
   srf= ( srf_x^2+srf_y^2 )^0.5;
   
   slr_x=( Vlr_x-w_lr(k)*r_R )/( w_lr(k)*r_R );
   slr_y=Vlr_y/( w_lr(k)*r_R );
   slr= ( slr_x^2+slr_y^2 )^0.5;
 
   srr_x=( Vrr_x-w_rr(k)*r_R )/( w_rr(k)*r_R );
   srr_y=Vrr_y/( w_rr(k)*r_R );
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
   
   Flf0=m*g*lr/L_axis/2;
   Frf0=Flf0;
   Flr0=m*g*lf/L_axis/2;
   Frr0=Flr0;  

   dFlf=Kf*(lf*Sgm_x1(11,jj)-Wa/2*Sgm_x1(12,jj)-Sgm_x1(10,jj))+Cf*(lf*Sgm_x1(8,jj)-Wa/2*Sgm_x1(9,jj)-Sgm_x1(7,jj));
   dFrf=Kf*(lf*Sgm_x1(11,jj)+Wa/2*Sgm_x1(12,jj)-Sgm_x1(10,jj))+Cf*(lf*Sgm_x1(8,jj)+Wa/2*Sgm_x1(9,jj)-Sgm_x1(7,jj));
   dFlr=Kr*(-lr*Sgm_x1(11,jj)-Wb/2*Sgm_x1(12,jj)-Sgm_x1(10,jj))+Cr*(-lr*Sgm_x1(8,jj)-Wb/2*Sgm_x1(9,jj)-Sgm_x1(7,jj));
   dFrr=Kr*(-lr*Sgm_x1(11,jj)+Wb/2*Sgm_x1(12,jj)-Sgm_x1(10,jj))+Cr*(-lr*Sgm_x1(8,jj)+Wb/2*Sgm_x1(9,jj)-Sgm_x1(7,jj));
   
   TireForce_z=[ Flf0+dFlf Frf0+dFrf Flr0+dFlr Frr0+dFrr ];

   f_lfx=mu_lfx*TireForce_z(1);
   f_lfy=mu_lfy*TireForce_z(1);
   f_rfx=mu_rfx*TireForce_z(2);
   f_rfy=mu_rfy*TireForce_z(2);
   f_lrx=mu_lrx*TireForce_z(3);
   f_lry=mu_lry*TireForce_z(3);
   f_rrx=mu_rrx*TireForce_z(4);
   f_rry=mu_rry*TireForce_z(4);
   
   dot_Vx=( ( f_lfx+f_rfx )*cos( delta(k) )-( f_lfy+f_rfy )*sin( delta(k) )+( f_lrx+f_rrx ) )/m+Sgm_x1(2,jj)*Sgm_x1(3,jj)-0.5*Cd*rho_A*Area*Sgm_x1(1,jj)^2;  %no aerodynamics
   dot_Vy=( ( f_lfx+f_rfx )*sin( delta(k) )+( f_lfy+f_rfy )*cos( delta(k) )+( f_lry+f_rry ) )/m-Sgm_x1(1,jj)*Sgm_x1(3,jj);  
   dot_r=( ( ( f_lfy+f_rfy )*cos( delta(k) )+( f_lfx+f_rfx )*sin( delta(k) ) )*lf-( f_lry+f_rry )*lr )/Iz; 
   
   T_rotation=[ cos(Sgm_x1(6,jj)) sin(Sgm_x1(6,jj)); -sin(Sgm_x1(6,jj)) cos(Sgm_x1(6,jj)) ];
   dot_pos=T_rotation'*[ Sgm_x1(1,jj); Sgm_x1(2,jj) ];
   
   dot_XI=dot_pos(1);
   dot_YI=dot_pos(2); 
   
  %% calcualte x and  y acceleration of each wheel center, ignore vertical acceleration
   Acc_lf=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [lf Wa/2 r_F-h]')+cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]',cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [lf Wa/2 r_F-h]'));
   Acc_rf=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [lf -Wa/2 r_F-h]')+cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]',cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [lf -Wa/2 r_F-h]'));
   Acc_lr=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [-lr Wb/2 r_F-h]')+cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]',cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [-lr Wb/2 r_R-h]'));
   Acc_rr=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [-lr -Wb/2 r_F-h]')+cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]',cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [-lr -Wb/2 r_R-h]')); 
   
   F_slfx=f_lfx*cos( delta(k) )-f_lfy*sin( delta(k) )-mf*Acc_lf(1);    F_slfy=f_lfx*sin( delta(k) )+f_lfy*cos( delta(k) )-mf*Acc_lf(2);
   F_srfx=f_rfx*cos( delta(k) )-f_rfy*sin( delta(k) )-mf*Acc_rf(1);    F_srfy=f_rfx*sin( delta(k) )+f_rfy*cos( delta(k) )-mf*Acc_rf(2);    
   F_slrx=f_lrx-mr*Acc_lr(1);    F_slry=f_lry-mr*Acc_lr(2);   
   F_srrx=f_rrx-mr*Acc_rr(1);    F_srry=f_rry-mr*Acc_rr(2);   
   
   dot_VZs=( -2*(Kf+Kr)*Sgm_x1(10,jj)-2*(Cf+Cr)*Sgm_x1(7,jj)+2*(lf*Kf-lr*Kr)*Sgm_x1(11,jj)+2*(lf*Cf-lr*Cr)*Sgm_x1(8,jj) )/ms;
   dot_Vtheta=( 2*(lf*Kf-lr*Kr)*Sgm_x1(10,jj)+2*(lf*Cf-lr*Cr)*Sgm_x1(7,jj)-2*(lf^2*Kf+lr^2*Kr)*Sgm_x1(11,jj)-2*(lf^2*Cf+lr^2*Cr)*Sgm_x1(8,jj)-(F_slfx+F_srfx)*(h-r_F)-(F_slrx+F_srrx)*(h-r_R) )/Iy;
   dot_Vphi=( -Wa^2*Kf*Sgm_x1(12,jj)/2-Wa^2*Cf*Sgm_x1(9,jj)/2-Wb^2*Kr*Sgm_x1(12,jj)/2-Wb^2*Cr*Sgm_x1(9,jj)/2 + (F_slfy+F_srfy)*(h-r_F)+(F_slry+F_srry)*(h-r_R) )/Ix;
   
   Sgm_x2(:,jj)=Sgm_x1(:,jj)+[ dot_Vx; dot_Vy; dot_r; dot_XI; dot_YI; Sgm_x1(3,jj); dot_VZs; dot_Vtheta; dot_Vphi; Sgm_x1(7,jj); Sgm_x1(8,jj); Sgm_x1(9,jj); 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]*dt+Sgm_w(:,jj)*sqrt(dt);
   Sgm_x2s(:,jj)=Sgm_x1(:,jj)+[ dot_Vx; dot_Vy; dot_r; dot_XI; dot_YI; Sgm_x1(3,jj); dot_VZs; dot_Vtheta; dot_Vphi; Sgm_x1(7,jj); Sgm_x1(8,jj); Sgm_x1(9,jj); 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]*dt;
   
   end

   
    Xe(k+1,:)= (  Sgm_x2*[ Wm0 ;(ones(2*L,1)*Wmi)]   )';

    P2=squeeze(Q_buf(:,:,k));
    for i=1:2*L+1
        if i==1
        P2=P2+Wc0*(Sgm_x2(:,i)- Xe(k+1,:)')*(Sgm_x2(:,i)- Xe(k+1,:)')';
        else
        P2=P2+Wci*(Sgm_x2(:,i)- Xe(k+1,:)')*(Sgm_x2(:,i)- Xe(k+1,:)')'; 
        end
    end
    
    P2=0.5*(P2+P2');
    fprintf('%d\n',k);
    
    P2(isnan(P2))=Pmin;
    P2(isinf(P2))=Pmax;
    
    [U,S,V] = svd(P2); 
    Aux=diag(S);  %get diagonal elements
    if max(abs(Aux))>Pmax %||min(Aux)<Rmin
       Aux(Aux>Pmax)=Pmax;
       P2= U*diag(Aux)*V';
    end
    P2=nearestSPD(P2);

    Cdz=[ eye(6) zeros(6,23); zeros(1,10) 1 zeros(1,18); zeros(1,11) 1 zeros(1,17)];
    Sgm_y=Cdz*Sgm_x2+Sgm_v;
    Sgm_ys=Cdz*Sgm_x2;
      
    Ye(k+1,:)= (  Sgm_y*[ Wm0 ;(ones(2*L,1)*Wmi)]  )';
    Ye(k+1,:)=( Cdz*Xe(k+1,:)' )'; % UKFz method
    
%     Pyy=0;
    Pyy=R;
    for ii=1:2*L+1
        if ii==1
        Pyy=Pyy+ Wc0*(Sgm_y(:,ii)-Ye(k+1,:)' )*(Sgm_y(:,ii)-Ye(k+1,:)' )';
        else
        Pyy=Pyy+ 2*Wci*(Sgm_y(:,ii)-Ye(k+1,:)' )*(Sgm_y(:,ii)-Ye(k+1,:)' )';
        end
    end

    Pyy(isnan(Pyy))=Pmin;
    Pyy(isinf(Pyy))=Pmax; 

    [U,S,V] = svd(Pyy); 
    Aux=diag(S);  %get diagonal elements
    if max(abs(Aux))>Pyy_max %||min(Aux)<Rmin
       Aux(Aux>Pyy_max)=Pyy_max;
       Pyy= U*diag(Aux)*V';
    end
    Pyy=nearestSPD(Pyy);

    Pxy=0;
    for ii=1:2*L+1
        if ii==1
        Pxy=Pxy+ Wc0*(Sgm_x2(:,ii)-Xe(k+1,:)')*(Sgm_y(:,ii)-Ye(k+1,:)' )';
        else
        Pxy=Pxy+ Wci*(Sgm_x2(:,ii)-Xe(k+1,:)')*(Sgm_y(:,ii)-Ye(k+1,:)' )';
        end
    end

    %% Measure noise
    r_buf(:,k+1)=( Y(k+1,:)-Ye(k+1,:) )';
    
    Ehx2=0;
    for ii=1:2*L+1        
        if ii==1
        Ehx2=Ehx2+ Wm0*Sgm_ys(:,ii)*Sgm_ys(:,ii)';
        else
        Ehx2=Ehx2+ Wmi*Sgm_ys(:,ii)*Sgm_ys(:,ii)';
        end          
    end
       
    Gam_buf(:,:,k+1)=Ehx2-Ye(k+1,:)'*Ye(k+1,:);
    
    if k<N0-1        
       R_buf(:,:,k+1)=R_buf(:,:,k);  
%        R_buf2(:,:,k+1)=R_buf(:,:,k+1);
    end
    
    if k==N0-1       
%         wk=(k-0.1*N0)*(k-0.3*N0)*(k-0.5*N0)*(k-0.7*N0)*(k-0.9*N0)/k^Fade;
        wk=1;
        r_buf(:,k+1)=wk*r_buf(:,k+1);
        bar_r=r_buf(:,N0-Nr+1:N0)*ones(Nr,1)/Nr; 
        bar_r_buf(:,k+1)=bar_r;
        
        R_aux=0;
            for j=N0-Nr+1:N0       
            R_aux=R_aux+(r_buf(:,j)-bar_r)*(r_buf(:,j)-bar_r)'/(Nr-1)-squeeze(Gam_buf(:,:,j))/Nr;    
%             R_aux=R_aux+(r_buf(:,j)-0)*(r_buf(:,j)-0)'/(Nr-1)-squeeze(Gam_buf(:,:,j))/Nr;
            end
% 
        R=squeeze(R_buf(:,:,k+1));
        [U,S,V] = svd(R); 
        Aux=diag(S);  %get diagonal elements
        if max(abs(Aux))>Rmax%||min(Aux)<Rmin
%            Aux(Aux<Rmin)=Rmin;
           Aux(Aux>Rmax)=Rmax;
           R_buf(:,:,k+1)= U*diag(Aux)*V';
        end
    R_buf(:,:,k+1)=nearestSPD(squeeze(R_buf(:,:,k+1)));         
        
    end
    
    
    %% Kalman gain 
     K=Pxy/Pyy;
%      K=Pxy/(Pyy-R);
    
    %% state update & covariance update
    
%     X(k+1,:)=( Xe(k+1,:)'+K*(Y(k+1,:)-Ye(k+1,:))' )';
    X(k+1,:)=( Xe(k+1,:)'+K*(Y(k+1,:)-Ye(k+1,:))')';%.*[ones(12,1);dK*ones(17,1)] )';
    P1=P2-K*Pyy*K'; 

    P1=(P1+P1')/2;

    P1(isnan(P1))=Pmin;
    P1(isinf(P1))=Pmax; 

    [U,S,V] = svd(P1); 
    Aux=diag(S);  %get diagonal elements
    if max(abs(Aux))>Pmax %||min(Aux)<Rmin
       Aux(Aux>Pmax)=Pmax;
       P1= U*diag(Aux)*V';
    end
    P1=nearestSPD(P1);

    %% add constraints

%       m1=ms-X(k+1,14)/L_axis*ms; % front sprung mass 
%       m2=X(k+1,14)/L_axis*ms; % rear sprung mass 
%        
%       if m1<=0
%           m1=1;
%           m2=ms-m1;
%       elseif m2<=0
%           m2=1;
%           m1=ms-m2;
%       end
%       
%       Df1_x0=[zeros(1,13), X(k+1,28)/(2*X(k+1,26)*m1)^1.5*(-2*X(k+1,26)*ms)/L_axis, zeros(1,11), X(k+1,28)/(2*X(k+1,26)*m1)^1.5*(2*m1), 0, 1/(2*X(k+1,26)*m1)^0.5, 0]; % make damping ratio 0.1-1
%       Df2_x0=[zeros(1,13), X(k+1,29)/(2*X(k+1,27)*m2)^1.5*(2*X(k+1,27)*ms)/L_axis, zeros(1,12), X(k+1,29)/(2*X(k+1,27)*m2)^1.5*(2*m2), 0, 1/(2*X(k+1,27)*m2)^0.5];
%       
%       f1_x0=X(k+1,28)/(2*X(k+1,26)*m1)^0.5;
%       f2_x0=X(k+1,29)/(2*X(k+1,27)*m2)^0.5;
%       
%       D_g=[ zeros(17,12) eye(17); [-1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; zeros(17,12) -eye(17); Df1_x0; -Df1_x0; Df2_x0; -Df2_x0]; % let -Vx<0 -> Vx>0
%     
%       B_cons=[ para_ub'; 0; -para_lb'; 1+Df1_x0*X(k+1,:)'-f1_x0; -0.1-Df1_x0*X(k+1,:)'+f1_x0; 1+Df2_x0*X(k+1,:)'-f2_x0; -0.1-Df2_x0*X(k+1,:)'+f2_x0 ];

      D_g=[ zeros(17,12) eye(17); [-1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; zeros(17,12) -eye(17)] % let -Vx<0 -> Vx>0
    
      B_cons=[ para_ub'; 0; -para_lb']
      
    if ~isreal(D_g)
       fprintf('D_g not real \n'); 
    end
      
      
      P_aux=P1+Eps*eye(29);
      Inv_P=eye(29)/(P_aux); % inv(P_aux)
      
      
        if isreal(Inv_P)
        opts = optimoptions('quadprog','Algorithm','interior-point-convex ','Display','off');
        Aux= quadprog(Inv_P+Inv_P',-(Inv_P+Inv_P')*X(k+1,:)',D_g,B_cons,[],[],[],[],[],opts);
        Xc(k+1,:)=Aux'; 
        X(k+1,:)=Xc(k+1,:);
        else
            fprintf('P_aux is not real here! k= %d. \n', k)
            return;
        end 
        
 
%%  process noise
%     r(:,k+1)=( Y(k+1,:)-Ye(k+1,:) )';
    q_buf(:,k+1)=( X(k+1,:)-Xe(k+1,:) )';
    
    Efx2=0;
        for ii=1:2*L+1        
            if ii==1
            Efx2=Efx2+ Wm0*Sgm_x2s(:,ii)*Sgm_x2s(:,ii)';
            else
            Efx2=Efx2+ Wmi*Sgm_x2s(:,ii)*Sgm_x2s(:,ii)';
            end          
        end
       
    Del_buf(:,:,k+1)=Efx2-Xe(k+1,:)'*Xe(k+1,:)-P1;
    
    if k<N0-1      
        Q_buf(:,:,k+1)=Q_buf(:,:,k);        
    end
    
    
    if k==N0-1       
%         wk=(k-0.1*N0)*(k-0.3*N0)*(k-0.5*N0)*(k-0.7*N0)*(k-0.9*N0)/k^Fade;
        wk=1;
        q_buf(:,k+1)=wk*q_buf(:,k+1);
        bar_q=q_buf(:,N0-Nq+1:N0)*ones(Nq,1)/Nq; 
        bar_q_buf(:,k+1)=bar_q;
        
        Q_aux=0;
            for j=N0-Nq+1:N0       
            Q_aux=Q_aux+(q_buf(:,j)-bar_q)*(q_buf(:,j)-bar_q)'/(Nq-1)-squeeze(Del_buf(:,:,j))/Nq; 
%             Q_aux=Q_aux+(q_buf(:,j)-0)*(q_buf(:,j)-0)'/(Nq-1)-squeeze(Del_buf(:,:,j))/Nq; 
            end
                    
        Q=squeeze(Q_buf(:,:,k+1));
        [U,S,V] = svd(Q);  
        Aux=diag(S);  %get diagonal elements       
        if max(abs(Aux))>Qmax%||min(Aux)<Qmin
%            Aux(Aux<Rmin)=Qmin;
           Aux(Aux>Rmax)=Qmax;
           Q_buf(:,:,k+1)= U*diag(Aux)*V';
        end
        Q_buf(:,:,k+1)=nearestSPD(squeeze(Q_buf(:,:,k+1)));
    end 
     
     Q=squeeze(Q_buf(:,:,k+1));
     R=squeeze(R_buf(:,:,k+1));
     Xa(k+1,:)= [X(k+1,:) bar_q_buf(:,k+1)'  bar_r_buf(:,k+1)'];  
%      Xa(k+1,:)= [X(k+1,:) 0*bar_q_buf(:,k+1)'  0*bar_r_buf(:,k+1)'];  
     P1_buf(:,:,k+1)=P1;
     
    end  
    
%% for big k

     Q=Qw;
     R=Rv;     
     
     
     for k=N0: N-1

% prediction and observation


    root_Pa=(L+lam)^0.5*[ real(sqrtm(P1)) zeros(29,37); zeros(29,29) real(sqrtm(Q)) zeros(29,8); zeros(8,58) real(sqrtm(R)) ];
%     root_Pb=(L+lam)^0.5*[ real(sqrtm(P1)) zeros(29,37); zeros(29,29) real(sqrtm(squeeze(Q_buf(:,:,k-1)))) zeros(29,8); zeros(8,58) real(sqrtm(squeeze(R_buf(:,:,k-1)))) ];
    root_Pb=(L+lam)^0.5*[ real(sqrtm(P1)) zeros(29,37); zeros(29,29) real(sqrtm(squeeze(Q_buf(:,:,k-1)))) zeros(29,8); zeros(8,58) real(sqrtm(R)) ];
    
    if ~isreal(root_Pa)
       fprintf('Pa not real \n'); 
    end 
    
    Sgm_a=[Xa(k,:)' krone( ones(1,L),Xa(k,:)' )+root_Pa krone( ones(1,L),Xa(k,:)' )-root_Pa ];
    Sgm_b=[Xa(k,:)' krone( ones(1,L),Xa(k,:)' )+root_Pb krone( ones(1,L),Xa(k,:)' )-root_Pb ];
    
    Sgm_x1=Sgm_a(1:29,:);
    Sgm_w=Sgm_a(30:58,:);   Sgm_wb=Sgm_b(30:58,:);    
    Sgm_v=Sgm_a(59:66,:);   Sgm_vb=Sgm_b(59:66,:);

   for jj=1:2*L+1 
   
%    Iz   lf    h    B    C    D    E   Sh   Sv   scl   Cd   Ix   Iy   Kf   Kr   Cf    Cr 
   Sgm_x1_aux=Sgm_x1(13:29,jj).*scales';
   Iz=Sgm_x1_aux(1);
   lf=Sgm_x1_aux(2); lr=L_axis-lf;
   h=Sgm_x1_aux(3);
   B=Sgm_x1_aux(4);
   C=Sgm_x1_aux(5);   
   D=Sgm_x1_aux(6);   
   E=Sgm_x1_aux(7);
   Sh=Sgm_x1_aux(8);
   Sv=Sgm_x1_aux(9);
   scl=Sgm_x1_aux(10);
   Cd=Sgm_x1_aux(11);
   Ix=Sgm_x1_aux(12);
   Iy=Sgm_x1_aux(13);  
   Kf=Sgm_x1_aux(14);   
   Kr=Sgm_x1_aux(15);   
   Cf=Sgm_x1_aux(16);   
   Cr=Sgm_x1_aux(17);   
 
   delta=scl*steering;

   % wheel velocity in body-fixed frame
    Vlf=[Sgm_x1(1,jj)  Sgm_x1(2,jj) Sgm_x1(7,jj)]'+ cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [lf Wa/2 r_F-h]'); % beta is measured here. I may use beta=atan(Vy/Vx);
    Vrf=[Sgm_x1(1,jj)  Sgm_x1(2,jj) Sgm_x1(7,jj)]'+ cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [lf -Wa/2 r_F-h]');
    Vlr=[Sgm_x1(1,jj)  Sgm_x1(2,jj) Sgm_x1(7,jj)]'+ cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [-lr Wb/2 r_R-h]');
    Vrr=[Sgm_x1(1,jj)  Sgm_x1(2,jj) Sgm_x1(7,jj)]'+ cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [-lr -Wb/2 r_R-h]');
    
   % wheel velocity in wheel-fixed frame
   Tran=[cos(delta(k)) sin(delta(k)) 0; -sin(delta(k)) cos(delta(k)) 0; 0 0 1];
   Vlf=Tran*Vlf;
   Vrf=Tran*Vrf;
   
   Vlf_x=Vlf(1); Vlf_y=Vlf(2); % don't consider the verticle vibration of the wheel here
   Vrf_x=Vrf(1); Vrf_y=Vrf(2);
   Vlr_x=Vlr(1); Vlr_y=Vlr(2);
   Vrr_x=Vrr(1); Vrr_y=Vrr(2);   
   
   slf_x=( Vlf_x-w_lf(k)*r_F )/( w_lf(k)*r_F );
   slf_y=Vlf_y/( w_lf(k)*r_F );
   slf= ( slf_x^2+slf_y^2 )^0.5;
 
   srf_x=( Vrf_x-w_rf(k)*r_F )/( w_rf(k)*r_F );
   srf_y=Vrf_y/( w_rf(k)*r_F );
   srf= ( srf_x^2+srf_y^2 )^0.5;
   
   slr_x=( Vlr_x-w_lr(k)*r_R )/( w_lr(k)*r_R );
   slr_y=Vlr_y/( w_lr(k)*r_R );
   slr= ( slr_x^2+slr_y^2 )^0.5;
 
   srr_x=( Vrr_x-w_rr(k)*r_R )/( w_rr(k)*r_R );
   srr_y=Vrr_y/( w_rr(k)*r_R );
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
   
   Flf0=m*g*lr/L_axis/2;
   Frf0=Flf0;
   Flr0=m*g*lf/L_axis/2;
   Frr0=Flr0;  

   dFlf=Kf*(lf*Sgm_x1(11,jj)-Wa/2*Sgm_x1(12,jj)-Sgm_x1(10,jj))+Cf*(lf*Sgm_x1(8,jj)-Wa/2*Sgm_x1(9,jj)-Sgm_x1(7,jj));
   dFrf=Kf*(lf*Sgm_x1(11,jj)+Wa/2*Sgm_x1(12,jj)-Sgm_x1(10,jj))+Cf*(lf*Sgm_x1(8,jj)+Wa/2*Sgm_x1(9,jj)-Sgm_x1(7,jj));
   dFlr=Kr*(-lr*Sgm_x1(11,jj)-Wb/2*Sgm_x1(12,jj)-Sgm_x1(10,jj))+Cr*(-lr*Sgm_x1(8,jj)-Wb/2*Sgm_x1(9,jj)-Sgm_x1(7,jj));
   dFrr=Kr*(-lr*Sgm_x1(11,jj)+Wb/2*Sgm_x1(12,jj)-Sgm_x1(10,jj))+Cr*(-lr*Sgm_x1(8,jj)+Wb/2*Sgm_x1(9,jj)-Sgm_x1(7,jj));
   
   TireForce_z=[ Flf0+dFlf Frf0+dFrf Flr0+dFlr Frr0+dFrr ];

   f_lfx=mu_lfx*TireForce_z(1);
   f_lfy=mu_lfy*TireForce_z(1);
   f_rfx=mu_rfx*TireForce_z(2);
   f_rfy=mu_rfy*TireForce_z(2);
   f_lrx=mu_lrx*TireForce_z(3);
   f_lry=mu_lry*TireForce_z(3);
   f_rrx=mu_rrx*TireForce_z(4);
   f_rry=mu_rry*TireForce_z(4);
   
   dot_Vx=( ( f_lfx+f_rfx )*cos( delta(k) )-( f_lfy+f_rfy )*sin( delta(k) )+( f_lrx+f_rrx ) )/m+Sgm_x1(2,jj)*Sgm_x1(3,jj)-0.5*Cd*rho_A*Area*Sgm_x1(1,jj)^2;  %no aerodynamics
   dot_Vy=( ( f_lfx+f_rfx )*sin( delta(k) )+( f_lfy+f_rfy )*cos( delta(k) )+( f_lry+f_rry ) )/m-Sgm_x1(1,jj)*Sgm_x1(3,jj);  
   dot_r=( ( ( f_lfy+f_rfy )*cos( delta(k) )+( f_lfx+f_rfx )*sin( delta(k) ) )*lf-( f_lry+f_rry )*lr )/Iz; 
   
   T_rotation=[ cos(Sgm_x1(6,jj)) sin(Sgm_x1(6,jj)); -sin(Sgm_x1(6,jj)) cos(Sgm_x1(6,jj)) ];
   dot_pos=T_rotation'*[ Sgm_x1(1,jj); Sgm_x1(2,jj) ];
   
   dot_XI=dot_pos(1);
   dot_YI=dot_pos(2); 
   
   %calcualte x and  y acceleration of each wheel center, ignore verticle acceleration
   Acc_lf=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [lf Wa/2 r_F-h]')+cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]',cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [lf Wa/2 r_F-h]'));
   Acc_rf=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [lf -Wa/2 r_F-h]')+cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]',cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [lf -Wa/2 r_F-h]'));
   Acc_lr=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [-lr Wb/2 r_F-h]')+cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]',cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [-lr Wb/2 r_R-h]'));
   Acc_rr=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [-lr -Wb/2 r_F-h]')+cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]',cross([Sgm_x1(9,jj) Sgm_x1(8,jj) Sgm_x1(3,jj)]', [-lr -Wb/2 r_R-h]')); 
   
   F_slfx=f_lfx*cos( delta(k) )-f_lfy*sin( delta(k) )-mf*Acc_lf(1);    F_slfy=f_lfx*sin( delta(k) )+f_lfy*cos( delta(k) )-mf*Acc_lf(2);
   F_srfx=f_rfx*cos( delta(k) )-f_rfy*sin( delta(k) )-mf*Acc_rf(1);    F_srfy=f_rfx*sin( delta(k) )+f_rfy*cos( delta(k) )-mf*Acc_rf(2);    
   F_slrx=f_lrx-mr*Acc_lr(1);    F_slry=f_lry-mr*Acc_lr(2);   
   F_srrx=f_rrx-mr*Acc_rr(1);    F_srry=f_rry-mr*Acc_rr(2);   
   
   dot_VZs=( -2*(Kf+Kr)*Sgm_x1(10,jj)-2*(Cf+Cr)*Sgm_x1(7,jj)+2*(lf*Kf-lr*Kr)*Sgm_x1(11,jj)+2*(lf*Cf-lr*Cr)*Sgm_x1(8,jj) )/ms;
   dot_Vtheta=( 2*(lf*Kf-lr*Kr)*Sgm_x1(10,jj)+2*(lf*Cf-lr*Cr)*Sgm_x1(7,jj)-2*(lf^2*Kf+lr^2*Kr)*Sgm_x1(11,jj)-2*(lf^2*Cf+lr^2*Cr)*Sgm_x1(8,jj)-(F_slfx+F_srfx)*(h-r_F)-(F_slrx+F_srrx)*(h-r_R) )/Iy;
   dot_Vphi=( -Wa^2*Kf*Sgm_x1(12,jj)/2-Wa^2*Cf*Sgm_x1(9,jj)/2-Wb^2*Kr*Sgm_x1(12,jj)/2-Wb^2*Cr*Sgm_x1(9,jj)/2 + (F_slfy+F_srfy)*(h-r_F)+(F_slry+F_srry)*(h-r_R) )/Ix;
   
   Sgm_x2(:,jj)=Sgm_x1(:,jj)+[ dot_Vx; dot_Vy; dot_r; dot_XI; dot_YI; Sgm_x1(3,jj); dot_VZs; dot_Vtheta; dot_Vphi; Sgm_x1(7,jj); Sgm_x1(8,jj); Sgm_x1(9,jj); 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]*dt+Sgm_w(:,jj)*sqrt(dt);
   Sgm_x2b(:,jj)=Sgm_x1(:,jj)+[ dot_Vx; dot_Vy; dot_r; dot_XI; dot_YI; Sgm_x1(3,jj); dot_VZs; dot_Vtheta; dot_Vphi; Sgm_x1(7,jj); Sgm_x1(8,jj); Sgm_x1(9,jj); 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]*dt+Sgm_wb(:,jj)*sqrt(dt);
   Sgm_x2s(:,jj)=Sgm_x1(:,jj)+[ dot_Vx; dot_Vy; dot_r; dot_XI; dot_YI; Sgm_x1(3,jj); dot_VZs; dot_Vtheta; dot_Vphi; Sgm_x1(7,jj); Sgm_x1(8,jj); Sgm_x1(9,jj); 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]*dt;
   
   end
   
    Xe(k+1,:)= (  Sgm_x2*[ Wm0 ;(ones(2*L,1)*Wmi)]   )';
    Xeb(k+1,:)= (  Sgm_x2b*[ Wm0 ;(ones(2*L,1)*Wmi)]   )';
    
    P2=squeeze(Q_buf(:,:,k));
    P2b=squeeze(Q_buf(:,:,k-1));
    for i=1:2*L+1
        if i==1
        P2=P2+Wc0*(Sgm_x2(:,i)- Xe(k+1,:)')*(Sgm_x2(:,i)- Xe(k+1,:)')';
        P2b=P2b+Wc0*(Sgm_x2b(:,i)- Xeb(k+1,:)')*(Sgm_x2b(:,i)- Xeb(k+1,:)')';
        else
        P2=P2+Wci*(Sgm_x2(:,i)- Xe(k+1,:)')*(Sgm_x2(:,i)- Xe(k+1,:)')'; 
        P2b=P2b+Wci*(Sgm_x2b(:,i)- Xeb(k+1,:)')*(Sgm_x2b(:,i)- Xeb(k+1,:)')'; 
        end
    end
       
%     P2=0.5*(P2+P2');
    fprintf('%d\n',k);
    
    P2(isnan(P2))=Pmin;
    P2(isinf(P2))=Pmax;
    P2b(isnan(P2b))=Pmin;
    P2b(isinf(P2b))=Pmax;
    
    [U,S,V] = svd(P2); 
    Aux=diag(S);  %get diagonal elements
    if max(abs(Aux))>Pmax %||min(Aux)<Rmin
       Aux(Aux>Pmax)=Pmax;
       P2= U*diag(Aux)*V';
    end
    if min(S)<Eps
    P2=nearestSPD(P2);
    end
    
    [U,S,V] = svd(P2b); 
    Aux=diag(S);  %get diagonal elements
    if max(abs(Aux))>Pmax %||min(Aux)<Rmin
       Aux(Aux>Pmax)=Pmax;
       P2b= U*diag(Aux)*V';
    end
    if min(S)<Eps
    P2b=nearestSPD(P2b);
    end   
      
    Cdz=[ eye(6) zeros(6,23); zeros(1,10) 1 zeros(1,18); zeros(1,11) 1 zeros(1,17)];
    Sgm_y=Cdz*Sgm_x2+Sgm_v;
    Sgm_yb=Cdz*Sgm_x2b+Sgm_vb;
    Sgm_ys=Cdz*Sgm_x2;
      
    Ye(k+1,:)= (  Sgm_y*[ Wm0 ;(ones(2*L,1)*Wmi)]  )';
    Yeb(k+1,:)= (  Sgm_yb*[ Wm0 ;(ones(2*L,1)*Wmi)]  )';    
%     Ye(k+1,:)=( Cdz*Xe(k+1,:)' )'; % UKFz method
    
%     Pyy=0;
    Pyy=R;
    Pyyb=squeeze(R_buf(:,:,k-1));
    for ii=1:2*L+1
        if ii==1
        Pyy=Pyy+ Wc0*(Sgm_y(:,ii)-Ye(k+1,:)' )*(Sgm_y(:,ii)-Ye(k+1,:)' )';
        Pyyb=Pyyb+ Wc0*(Sgm_yb(:,ii)-Yeb(k+1,:)' )*(Sgm_yb(:,ii)-Yeb(k+1,:)' )';        
        else
        Pyy=Pyy+ Wci*(Sgm_y(:,ii)-Ye(k+1,:)' )*(Sgm_y(:,ii)-Ye(k+1,:)' )';
        Pyyb=Pyyb+ Wci*(Sgm_yb(:,ii)-Yeb(k+1,:)' )*(Sgm_yb(:,ii)-Yeb(k+1,:)' )';     
        end
    end

    Pyy(isnan(Pyy))=Pmin;
    Pyy(isinf(Pyy))=Pmax; 
    Pyyb(isnan(Pyyb))=Pmin;
    Pyyb(isinf(Pyyb))=Pmax;

    [U,S,V] = svd(Pyy); 
    Aux=diag(S);  %get diagonal elements
    if max(abs(Aux))>Pyy_max %||min(Aux)<Rmin
       Aux(Aux>Pyy_max)=Pyy_max;
       Pyy= U*diag(Aux)*V';
    end
    if min(S)<Eps
    Pyy=nearestSPD(Pyy);
    end
    
    [U,S,V] = svd(Pyyb); 
    Aux=diag(S);  %get diagonal elements
    if max(abs(Aux))>Pyy_max %||min(Aux)<Rmin
       Aux(Aux>Pyy_max)=Pyy_max;
       Pyyb= U*diag(Aux)*V';
    end
    
    if min(S)<Eps
    Pyyb=nearestSPD(Pyyb);
    end
    
    Pxy=0;
    Pxyb=0;
    for ii=1:2*L+1
        if ii==1
        Pxy=Pxy+ Wc0*(Sgm_x2(:,ii)-Xe(k+1,:)')*(Sgm_y(:,ii)-Ye(k+1,:)' )';
        Pxyb=Pxyb+ Wc0*(Sgm_x2b(:,ii)-Xeb(k+1,:)')*(Sgm_yb(:,ii)-Yeb(k+1,:)' )';        
        else
        Pxy=Pxy+ Wci*(Sgm_x2(:,ii)-Xe(k+1,:)')*(Sgm_y(:,ii)-Ye(k+1,:)' )';
        Pxyb=Pxyb+ Wci*(Sgm_x2b(:,ii)-Xeb(k+1,:)')*(Sgm_yb(:,ii)-Yeb(k+1,:)' )';        
        end
    end

    %% Measure noise
    r_buf(:,k+1)=( Y(k+1,:)-Ye(k+1,:) )';
    r_bufb=( Y(k+1,:)-Yeb(k+1,:) )';
    
    Ehx2=0;
    for ii=1:2*L+1        
        if ii==1
        Ehx2=Ehx2+ Wm0*Sgm_ys(:,ii)*Sgm_ys(:,ii)';
        else
        Ehx2=Ehx2+ Wmi*Sgm_ys(:,ii)*Sgm_ys(:,ii)';
        end          
    end
       
    Gam_buf(:,:,k+1)=Ehx2-Ye(k+1,:)'*Ye(k+1,:);
    Gam_bufb=Ehx2-Yeb(k+1,:)'*Yeb(k+1,:);
       
%     wk=(k-0.1*N0)*(k-0.3*N0)*(k-0.5*N0)*(k-0.7*N0)*(k-0.9*N0)/k^Fade; 
    wk=1;
    r_buf(:,k+1)=wk*r_buf(:,k+1);
    bar_r_buf(:,k+1)=bar_r_buf(:,k)+(r_buf(:,k+1)-r_buf(:,k+1-Nr))/Nr;
    R_buf(:,:,k+1)=squeeze(R_buf(:,:,k))+( (r_buf(:,k+1)-bar_r_buf(:,k+1))*(r_buf(:,k+1)-bar_r_buf(:,k+1))'-(r_buf(:,k+1-Nr)-bar_r_buf(:,k+1))*(r_buf(:,k+1-Nr)-bar_r_buf(:,k+1))'+(r_buf(:,k+1)-r_buf(:,k+1-Nr))*(r_buf(:,k+1)-r_buf(:,k+1-Nr))'/Nr )/(Nr-1)+squeeze(( Gam_buf(:,:,k+1-Nr)-Gam_buf(:,:,k+1) ))/Nr;
%     R_buf(:,:,k+1)=squeeze(R_buf(:,:,k))+( (r_buf(:,k+1)-0)*(r_buf(:,k+1)-0)'-(r_buf(:,k+1-Nr)-0)*(r_buf(:,k+1-Nr)-0)'+(r_buf(:,k+1)-r_buf(:,k+1-Nr))*(r_buf(:,k+1)-r_buf(:,k+1-Nr))'/Nr )/(Nr-1)+squeeze(( Gam_buf(:,:,k+1-Nr)-Gam_buf(:,:,k+1) ))/Nr;
    
    r_bufb=wk*r_bufb;
    bar_r_bufb=bar_r_buf(:,k)+(r_bufb-r_buf(:,k+1-Nr))/Nr;
    R_bufb=squeeze(R_buf(:,:,k))+( (r_bufb-bar_r_bufb)*(r_bufb-bar_r_bufb)'-(r_buf(:,k+1-Nr)-bar_r_bufb)*(r_buf(:,k+1-Nr)-bar_r_bufb)'+(r_bufb-r_buf(:,k+1-Nr))*(r_bufb-r_buf(:,k+1-Nr))'/Nr )/(Nr-1)+squeeze(( Gam_buf(:,:,k+1-Nr)-Gam_bufb ))/Nr;
%     R_bufb=squeeze(R_buf(:,:,k))+( (r_bufb-0)*(r_bufb-0)'-(r_buf(:,k+1-Nr)-0)*(r_buf(:,k+1-Nr)-0)'+(r_bufb-r_buf(:,k+1-Nr))*(r_bufb-r_buf(:,k+1-Nr))'/Nr )/(Nr-1)+squeeze(( Gam_buf(:,:,k+1-Nr)-Gam_bufb ))/Nr;
    
        R=squeeze(R_buf(:,:,k+1));
        R(isnan(R))=Rmin;
        R(isinf(R))=Rmax; 
        R_buf(:,:,k+1)=R; %remove Nan and inf in R_buf
        [U,S,V] = svd(R); 
        Aux=diag(S);  %get diagonal elements
        if max(abs(Aux))>Rmax %||min(Aux)<Rmin
%            Aux(Aux<Rmin)=Rmin;
           Aux(Aux>Rmax)=Rmax;
           R_buf(:,:,k+1)= U*diag(Aux)*V';
        end
        
        if min(S)<Eps        
        R_buf(:,:,k+1)=nearestSPD(squeeze(R_buf(:,:,k+1)));  
        end     
        
% 	    R=squeeze(R_buf(:,:,k+1));
        R_bufb(isnan(R_bufb))=Rmin;
        R_bufb(isinf(R_bufb))=Rmax; 
        [U,S,V] = svd(R_bufb); 
        Aux=diag(S);  %get diagonal elements
        if max(abs(Aux))>Rmax %||min(Aux)<Rmin
%            Aux(Aux<Rmin)=Rmin;
           Aux(Aux>Rmax)=Rmax;
           R_bufb= U*diag(Aux)*V';
        end
        
        if min(S)<Eps
        R_bufb=nearestSPD(R_bufb); 
        end
    
    
    %% Kalman gain 
     K=Pxy/Pyy;
     Kb=Pxyb/Pyyb;
%      K=Pxy/(Pyy-R);
     K(13:end,:)=dK*K(13:end,:);
     Kb(13:end,:)=dK*Kb(13:end,:);

    %% state update & covariance update
    
%     X(k+1,:)=( Xe(k+1,:)'+K*(Y(k+1,:)-Ye(k+1,:))' )';
    X(k+1,:)=( Xe(k+1,:)'+K*(Y(k+1,:)-Ye(k+1,:))')';%.*[ones(12,1);dK*ones(17,1)] )';
    Xb=( Xeb(k+1,:)'+Kb*(Y(k+1,:)-Yeb(k+1,:))')';%.*[ones(12,1);dK*ones(17,1)] )';
    
    P1=P2-K*Pyy*K'; 
    P1b=P2b-Kb*Pyyb*Kb'; 
    
%     P1=(P1+P1')/2;

    P1(isnan(P1))=Pmin;
    P1(isinf(P1))=Pmax; 
    P1b(isnan(P1b))=Pmin;
    P1b(isinf(P1b))=Pmax;    
    
    [U,S,V] = svd(P1); 
    Aux=diag(S);  %get diagonal elements
    if max(abs(Aux))>Pmax %||min(Aux)<Rmin
       Aux(Aux>Pmax)=Pmax;
       P1= U*diag(Aux)*V';
    end
    P1=nearestSPD(P1);

    [U,S,V] = svd(P1b); 
    Aux=diag(S);  %get diagonal elements
    if max(abs(Aux))>Pmax %||min(Aux)<Rmin
       Aux(Aux>Pmax)=Pmax;
       P1b= U*diag(Aux)*V';
    end
    P1b=nearestSPD(P1b);
    
    
    %% add constraints

%       m1=ms-X(k+1,14)/L_axis*ms; % front sprung mass 
%       m2=X(k+1,14)/L_axis*ms; % rear sprung mass 
%        
%       if m1<=0
%           m1=1;
%           m2=ms-m1;
%       elseif m2<=0
%           m2=1;
%           m1=ms-m2;
%       end
%       
%       Df1_x0=[zeros(1,13), X(k+1,28)/(2*X(k+1,26)*m1)^1.5*(-2*X(k+1,26)*ms)/L_axis, zeros(1,11), X(k+1,28)/(2*X(k+1,26)*m1)^1.5*(2*m1), 0, 1/(2*X(k+1,26)*m1)^0.5, 0]; % make damping ratio 0.1-1
%       Df2_x0=[zeros(1,13), X(k+1,29)/(2*X(k+1,27)*m2)^1.5*(2*X(k+1,27)*ms)/L_axis, zeros(1,12), X(k+1,29)/(2*X(k+1,27)*m2)^1.5*(2*m2), 0, 1/(2*X(k+1,27)*m2)^0.5];
%       
%       f1_x0=X(k+1,28)/(2*X(k+1,26)*m1)^0.5;
%       f2_x0=X(k+1,29)/(2*X(k+1,27)*m2)^0.5;
%       
%       D_g=[ zeros(17,12) eye(17); [-1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; zeros(17,12) -eye(17); Df1_x0; -Df1_x0; Df2_x0; -Df2_x0]; % let -Vx<0 -> Vx>0
%     
%       B_cons=[ para_ub'; 0; -para_lb'; 1+Df1_x0*X(k+1,:)'-f1_x0; -0.1-Df1_x0*X(k+1,:)'+f1_x0; 1+Df2_x0*X(k+1,:)'-f2_x0; -0.1-Df2_x0*X(k+1,:)'+f2_x0 ];

      D_g=[ zeros(17,12) eye(17); [-1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; zeros(17,12) -eye(17)]; % let -Vx<0 -> Vx>0
    
      B_cons=[ para_ub'; 0; -para_lb'];
      
    if ~isreal(D_g)
       fprintf('D_g not real \n'); 
    end
      
      
      P_aux=P1+Eps*eye(29);
      Inv_P=eye(29)/(P_aux); % inv(P_aux)
      P_auxb=P1b+Eps*eye(29);
      Inv_Pb=eye(29)/(P_auxb); % inv(P_aux)
      
      Inv_P(isnan(Inv_P))=Pmin;
      Inv_P(isinf(Inv_P))=Pmax; 
      Inv_Pb(isnan(Inv_Pb))=Pmin;
      Inv_Pb(isinf(Inv_Pb))=Pmax;  

      X(k+1,(isinf(X(k+1,:))))=1e5;
      X(k+1,(isnan(X(k+1,:))))=-1e5;
      Xb(isinf(Xb))=1e5;
      Xb(isnan(Xb))=-1e5;
      Xe(isnan(Xe))=-1e5;
      Xeb(isnan(Xeb))=-1e5;
%         if ~isreal(Inv_P)
%             fprintf('P_aux is not real here! k= %d. \n', k)
%             return;
%         end

      if k >2000
%       fprintf("here")
%           Inv_P+Inv_P'
%           X(k+1,:)
%             Xe(k+1,:)
%             Xeb(k+1,:)
%           -(Inv_P+Inv_P')*X(k+1,:)'
%           D_g
%           B_cons
     end
        
      opts = optimoptions('quadprog','Algorithm','interior-point-convex ','Display','off');
      Aux= quadprog(Inv_P+Inv_P',-(Inv_P+Inv_P')*X(k+1,:)',D_g,B_cons,[],[],[],[],[],opts);
      Auxb= quadprog(Inv_Pb+Inv_Pb',-(Inv_Pb+Inv_Pb')*Xb',D_g,B_cons,[],[],[],[],[],opts);      
      Xc(k+1,:)=Aux'; 
      X(k+1,:)=Aux';
      Xb=Auxb';
        
 
%%  process noise
%     r(:,k+1)=( Y(k+1,:)-Ye(k+1,:) )';
    q_buf(:,k+1)=( X(k+1,:)-Xe(k+1,:) )';
    q_bufb=( Xb-Xeb(k+1,:) )';
    
    Efx2=0;
        for ii=1:2*L+1        
            if ii==1
            Efx2=Efx2+ Wm0*Sgm_x2s(:,ii)*Sgm_x2s(:,ii)';
            else
            Efx2=Efx2+ Wmi*Sgm_x2s(:,ii)*Sgm_x2s(:,ii)';
            end          
        end
       
    Del_buf(:,:,k+1)=Efx2-Xe(k+1,:)'*Xe(k+1,:)-P1;
    Del_bufb=Efx2-Xeb(k+1,:)'*Xeb(k+1,:)-P1b;

%     wk=(k-0.1*N0)*(k-0.3*N0)*(k-0.5*N0)*(k-0.7*N0)*(k-0.9*N0)/k^Fade; 
        wk=1;
        q_buf(:,k+1)=wk*q_buf(:,k+1);
        bar_q_buf(:,k+1)=bar_q_buf(:,k)+(q_buf(:,k+1)-q_buf(:,k+1-Nq))/Nq;
        Q_buf(:,:,k+1)=squeeze(Q_buf(:,:,k))+( (q_buf(:,k+1)-bar_q_buf(:,k+1))*(q_buf(:,k+1)-bar_q_buf(:,k+1))'-(q_buf(:,k+1-Nq)-bar_q_buf(:,k+1))*(q_buf(:,k+1-Nq)-bar_q_buf(:,k+1))'+(q_buf(:,k+1)-q_buf(:,k+1-Nq))*(q_buf(:,k+1)-q_buf(:,k+1-Nq))'/Nq )/(Nq-1)+squeeze(( Del_buf(:,:,k+1-Nq)-Del_buf(:,:,k+1) ))/Nq;
%         Q_buf(:,:,k+1)=squeeze(Q_buf(:,:,k))+( (q_buf(:,k+1)-0)*(q_buf(:,k+1)-0)'-(q_buf(:,k+1-Nq)-0)*(q_buf(:,k+1-Nq)-0)'+(q_buf(:,k+1)-q_buf(:,k+1-Nq))*(q_buf(:,k+1)-q_buf(:,k+1-Nq))'/Nq )/(Nq-1)+squeeze(( Del_buf(:,:,k+1-Nq)-Del_buf(:,:,k+1) ))/Nq;

        q_bufb=wk*q_bufb;
        bar_q_bufb=bar_q_buf(:,k)+(q_bufb-q_buf(:,k+1-Nq))/Nq;
        Q_bufb=squeeze(Q_buf(:,:,k))+( (q_bufb-bar_q_bufb)*(q_bufb-bar_q_bufb)'-(q_buf(:,k+1-Nq)-bar_q_bufb)*(q_buf(:,k+1-Nq)-bar_q_bufb)'+(q_bufb-q_buf(:,k+1-Nq))*(q_bufb-q_buf(:,k+1-Nq))'/Nq )/(Nq-1)+( squeeze(Del_buf(:,:,k+1-Nq))-Del_bufb )/Nq;
%         Q_bufb=squeeze(Q_buf(:,:,k))+( (q_bufb-0)*(q_bufb-0)'-(q_buf(:,k+1-Nq)-0)*(q_buf(:,k+1-Nq)-0)'+(q_bufb-q_buf(:,k+1-Nq))*(q_bufb-q_buf(:,k+1-Nq))'/Nq )/(Nq-1)+( squeeze(Del_buf(:,:,k+1-Nq))-Del_bufb )/Nq;

        Q=squeeze(Q_buf(:,:,k+1));
        Q(isnan(Q))=Qmin;
        Q(isinf(Q))=Qmax; 
        Q_buf(:,:,k+1)=Q; %remove Nan and inf in Q_buf
        [U,S,V] = svd(Q);  
        Aux=diag(S);  %get diagonal elements       
        if max(abs(Aux))>Qmax%||min(Aux)<Qmin
%            Aux(Aux<Rmin)=Qmin;
           Aux(Aux>Rmax)=Qmax;
           Q_buf(:,:,k+1)= U*diag(Aux)*V';
        end           
        Q_buf(:,:,k+1)=nearestSPD(squeeze(Q_buf(:,:,k+1)));
        
        Q_bufb(isnan(Q_bufb)) = -1e5;
     
        [U,S,V] = svd(Q_bufb);  
        Aux=diag(S);  %get diagonal elements       
        if max(abs(Aux))>Qmax%||min(Aux)<Qmin
%            Aux(Aux<Rmin)=Qmin;
           Aux(Aux>Rmax)=Qmax;
           Q_bufb= U*diag(Aux)*V';
        end           
        Q_bufb=nearestSPD(Q_bufb);
     
%      J1=ValidityCheck( k, X(k+1,:) );
%      J2=ValidityCheck( k, Xb );
% %      
%      if J2<J1 % || trace(P1)>trace(P1b)    % means old noise is better
%        
%       r_buf(:,k+1)=r_bufb;
%       bar_r_buf(:,k+1)=bar_r_bufb;
%       R_buf(:,:,k+1)=R_bufb;
%       Gam_buf(:,:,k+1)=Gam_bufb;
%       
%       q_buf(:,k+1)=q_bufb;
%       bar_q_buf(:,k+1)=bar_q_bufb;
%       Q_buf(:,:,k+1)=Q_bufb;
%       Del_buf(:,:,k+1)=Del_bufb;
%       
%       X(k+1,:)=Xb;
%       P1=P1b;
%          
%      end
    
% %      Q=intp*squeeze(Q_buf(:,:,k+1))+(1-intp)*(Q_bufb);
% %      R=intp*squeeze(R_buf(:,:,k+1))+(1-intp)*(R_bufb);
% %      Xa(k+1,:)= intp*[X(k+1,:) bar_q_buf(:,k+1)'  bar_r_buf(:,k+1)']+(1-intp)*[Xb bar_q_bufb'  bar_r_bufb'];

     Qaux=squeeze(Q_buf(:,:,k+1));
     R=squeeze(R_buf(:,:,k+1));
   
     diagQ=diag(Qaux);
%      factor=(filter(50:end)'./diagQ(13:end)).^0.5;
     factor=w21./(diagQ(13:end)).^0.5;
     Qaux(13:end,:)=Qaux(13:end,:).*krone(ones(1,29),factor);
     Qaux(:,13:end)=Qaux(:,13:end).*krone(ones(29,1),factor');
     Q=Qaux;
     Q_buf(:,:,k+1)=Q;
     
     bar_q_buf(13:end,k+1)=0*bar_q_buf(13:end,k+1);% mean value of the parameter is set to be 0
     
     Xa(k+1,:)= [X(k+1,:) bar_q_buf(:,k+1)'  bar_r_buf(:,k+1)']; 
%      Xa(k+1,:)= [X(k+1,:) 0*bar_q_buf(:,k+1)'  0*bar_r_buf(:,k+1)'];  
     P1_buf(:,:,k+1)=P1;
     
    end
    

 %%  results   

    %         Iz   lf   h    B   C   D   E   Sh   Sv   scl   Cd 
   X_aux=X(N,13:29).*scales;
   Iz=X_aux(1);
   lf=X_aux(2); lr=L_axis-lf;
   h=X_aux(3);
   B=X_aux(4);
   C=X_aux(5);   
   D=X_aux(6);   
   E=X_aux(7);
   Sh=X_aux(8);
   Sv=X_aux(9);
   scl=X_aux(10);
   Cd=X_aux(11);
   Ix=X_aux(12);
   Iy=X_aux(13);  
   Kf=X_aux(14);   
   Kr=X_aux(15);   
   Cf=X_aux(16);   
   Cr=X_aux(17);  
    
 %% simulated results

   Xs(N,:)=X(N,1:12);
   Xs(N,1:6)=[ Vx(N) Vy(N) r(N) XI(N) YI(N) YawAngle(N)];  Xs(N,11:12)=[ PitchAngle(N) RollAngle(N) ];
   delta=scl*steering2;
   
   for k=N:lth-1

%    V=( Xs(k,1)^2 + Xs(k,2)^2 )^0.5;  

   % wheel velocity in body-fixed frame
    Vlf=[Xs(k,1)  Xs(k,2) Xs(k,7)]'+ cross([Xs(k,9) Xs(k,8) Xs(k,3)]', [lf Wa/2 r_F-h]');
    Vrf=[Xs(k,1)  Xs(k,2) Xs(k,7)]'+ cross([Xs(k,9) Xs(k,8) Xs(k,3)]', [lf -Wa/2 r_F-h]');
    Vlr=[Xs(k,1)  Xs(k,2) Xs(k,7)]'+ cross([Xs(k,9) Xs(k,8) Xs(k,3)]', [-lr Wb/2 r_R-h]');
    Vrr=[Xs(k,1)  Xs(k,2) Xs(k,7)]'+ cross([Xs(k,9) Xs(k,8) Xs(k,3)]', [-lr -Wb/2 r_R-h]');
    
   % wheel velocity in wheel-fixed frame
   Tran=[cos(delta(k)) sin(delta(k)) 0; -sin(delta(k)) cos(delta(k)) 0; 0 0 1];
   Vlf=Tran*Vlf;
   Vrf=Tran*Vrf;
   
   Vlf_x=Vlf(1); Vlf_y=Vlf(2);
   Vrf_x=Vrf(1); Vrf_y=Vrf(2);
   Vlr_x=Vlr(1); Vlr_y=Vlr(2);
   Vrr_x=Vrr(1); Vrr_y=Vrr(2);   
   
   slf_x=( Vlf_x-w_lf(k)*r_F )/( w_lf(k)*r_F );
   slf_y=Vlf_y/( w_lf(k)*r_F );
   slf= ( slf_x^2+slf_y^2 )^0.5;
 
   srf_x=( Vrf_x-w_rf(k)*r_F )/( w_rf(k)*r_F );
   srf_y=Vrf_y/( w_rf(k)*r_F );
   srf= ( srf_x^2+srf_y^2 )^0.5;
   
   slr_x=( Vlr_x-w_lr(k)*r_R )/( w_lr(k)*r_R );
   slr_y=Vlr_y/( w_lr(k)*r_R );
   slr= ( slr_x^2+slr_y^2 )^0.5;
 
   srr_x=( Vrr_x-w_rr(k)*r_R )/( w_rr(k)*r_R );
   srr_y=Vrr_y/( w_rr(k)*r_R );
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
   
   Flf0=m*g*lr/L_axis/2;
   Frf0=Flf0;
   Flr0=m*g*lf/L_axis/2;
   Frr0=Flr0;  

   dFlf=Kf*(lf*Xs(k,11)-Wa/2*Xs(k,12)-Xs(k,10))+Cf*(lf*Xs(k,8)-Wa/2*Xs(k,9)-Xs(k,7));
   dFrf=Kf*(lf*Xs(k,11)+Wa/2*Xs(k,12)-Xs(k,10))+Cf*(lf*Xs(k,8)+Wa/2*Xs(k,9)-Xs(k,7));
   dFlr=Kr*(-lr*Xs(k,11)-Wb/2*Xs(k,12)-Xs(k,10))+Cr*(-lr*Xs(k,8)-Wb/2*Xs(k,9)-Xs(k,7));
   dFrr=Kr*(-lr*Xs(k,11)+Wb/2*Xs(k,12)-Xs(k,10))+Cr*(-lr*Xs(k,8)+Wb/2*Xs(k,9)-Xs(k,7));
   
   TireForce_z=[ Flf0+dFlf Frf0+dFrf Flr0+dFlr Frr0+dFrr ];

   f_lfx=mu_lfx*TireForce_z(1);
   f_lfy=mu_lfy*TireForce_z(1);
   f_rfx=mu_rfx*TireForce_z(2);
   f_rfy=mu_rfy*TireForce_z(2);
   f_lrx=mu_lrx*TireForce_z(3);
   f_lry=mu_lry*TireForce_z(3);
   f_rrx=mu_rrx*TireForce_z(4);
   f_rry=mu_rry*TireForce_z(4);
   
   dot_Vx=( ( f_lfx+f_rfx )*cos( delta(k) )-( f_lfy+f_rfy )*sin( delta(k) )+( f_lrx+f_rrx ) )/m+Xs(k,2)*Xs(k,3)-0.5*Cd*rho_A*Area*Xs(k,1)^2;  %no aerodynamics
   dot_Vy=( ( f_lfx+f_rfx )*sin( delta(k) )+( f_lfy+f_rfy )*cos( delta(k) )+( f_lry+f_rry ) )/m-Xs(k,1)*Xs(k,3);  
   dot_r=( ( ( f_lfy+f_rfy )*cos( delta(k) )+( f_lfx+f_rfx )*sin( delta(k) ) )*lf-( f_lry+f_rry )*lr )/Iz;

   T_rotation=[ cos(Xs(k,6)) sin(Xs(k,6)); -sin(Xs(k,6)) cos(Xs(k,6)) ];
   dot_pos=T_rotation'*[ Xs(k,1); Xs(k,2) ];
   
   dot_XI=dot_pos(1);
   dot_YI=dot_pos(2);  
   
   %calcualte x and  y acceleration of each wheel center, ignore verticle acceleration
   Acc_lf=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [lf Wa/2 r_F-h]')+cross([Xs(k,9) Xs(k,8) Xs(k,3)]',cross([Xs(k,9) Xs(k,8) Xs(k,3)]', [lf Wa/2 r_F-h]'));
   Acc_rf=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [lf -Wa/2 r_F-h]')+cross([Xs(k,9) Xs(k,8) Xs(k,3)]',cross([Xs(k,9) Xs(k,8) Xs(k,3)]', [lf -Wa/2 r_F-h]'));
   Acc_lr=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [-lr Wb/2 r_F-h]')+cross([Xs(k,9) Xs(k,8) Xs(k,3)]',cross([Xs(k,9) Xs(k,8) Xs(k,3)]', [-lr Wb/2 r_R-h]'));
   Acc_rr=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [-lr -Wb/2 r_F-h]')+cross([Xs(k,9) Xs(k,8) Xs(k,3)]',cross([Xs(k,9) Xs(k,8) Xs(k,3)]', [-lr -Wb/2 r_R-h]')); 
   
   F_slfx=f_lfx*cos( delta(k) )-f_lfy*sin( delta(k) )-mf*Acc_lf(1);    F_slfy=f_lfx*sin( delta(k) )+f_lfy*cos( delta(k) )-mf*Acc_lf(2);
   F_srfx=f_rfx*cos( delta(k) )-f_rfy*sin( delta(k) )-mf*Acc_rf(1);    F_srfy=f_rfx*sin( delta(k) )+f_rfy*cos( delta(k) )-mf*Acc_rf(2);   
   F_slrx=f_lrx-mr*Acc_lr(1);    F_slry=f_lry-mr*Acc_lr(2);   
   F_srrx=f_rrx-mr*Acc_rr(1);    F_srry=f_rry-mr*Acc_rr(2);   
   
   dot_Zs=( -2*(Kf+Kr)*Xs(k,10)-2*(Cf+Cr)*Xs(k,7)+2*(lf*Kf-lr*Kr)*Xs(k,11)+2*(lf*Cf-lr*Cr)*Xs(k,8) )/ms;
   dot_theta=( 2*(lf*Kf-lr*Kr)*Xs(k,10)+2*(lf*Cf-lr*Cr)*Xs(k,7)-2*(lf^2*Kf+lr^2*Kr)*Xs(k,11)-2*(lf^2*Cf+lr^2*Cr)*Xs(k,8)-(F_slfx+F_srfx)*(h-r_F)-(F_slrx+F_srrx)*(h-r_R) )/Iy;
   dot_phi=( -Wa^2*Kf*Xs(k,12)/2-Wa^2*Cf*Xs(k,9)/2-Wb^2*Kr*Xs(k,12)/2-Wb^2*Cr*Xs(k,9)/2+(F_slfy+F_srfy)*(h-r_F)+(F_slry+F_srry)*(h-r_R) )/Ix;   
   
   Xs(k+1,:)=Xs(k,:)+[ dot_Vx dot_Vy dot_r dot_XI dot_YI Xs(k,3) dot_Zs dot_theta dot_phi Xs(k,7) Xs(k,8) Xs(k,9)]*dt;  
%    Xs(k+1,:)=Xs(k,:)+[ dot_Vx dot_Vy dot_r dot_XI dot_YI Xs(k,3) dot_Zs dot_theta dot_phi Xs(k,7) Xs(k,8) Xs(k,9)]*dt+bar_q_buf(1:12,N)';  
   
   end   
   
    w_1=1/8; w_2=1/8; w_3=1/8;
    w_4=1/2; w_5=1/2; w_6=1/4;
    J = w_1*(Xs(N:lth-1-Omit,1)-Y(N:lth-1-Omit,1))'*(Xs(N:lth-1-Omit,1)-Y(N:lth-1-Omit,1))+w_2*(Xs(N:lth-1-Omit,2)-Y(N:lth-1-Omit,2))'*(Xs(N:lth-1-Omit,2)-Y(N:lth-1-Omit,2))+w_3*(Xs(N:lth-1-Omit,3)-Y(N:lth-1-Omit,3))'*(Xs(N:lth-1-Omit,3)-Y(N:lth-1-Omit,3))+...
        w_4*(Xs(N:lth-1-Omit,4)-Y(N:lth-1-Omit,4))'*(Xs(N:lth-1-Omit,4)-Y(N:lth-1-Omit,4))+w_5*(Xs(N:lth-1-Omit,5)-Y(N:lth-1-Omit,5))'*(Xs(N:lth-1-Omit,5)-Y(N:lth-1-Omit,5))+w_6*(Xs(N:lth-1-Omit,6)-Y(N:lth-1-Omit,6))'*(Xs(N:lth-1-Omit,6)-Y(N:lth-1-Omit,6));
    
    fprintf('The cost is %e\n', J);
    
    para_opt=[ Iz lf h B C D E Sh Sv scl Cd Ix Iy Kf Kr Cf Cr ];

    Y_opt=Xs;
    Y_train=X;
    

%% checking validity function

    function [J]=ValidityCheck( k2, XN )  %XN is a vector contains the parameters
    
       X_aux=XN(13:29).*scales;
       Iz=X_aux(1);
       lf=X_aux(2); lr=L_axis-lf;
       h=X_aux(3);
       B=X_aux(4);
       C=X_aux(5);   
       D=X_aux(6);   
       E=X_aux(7);
       Sh=X_aux(8);
       Sv=X_aux(9);
       scl=X_aux(10);
       Cd=X_aux(11);
       Ix=X_aux(12);
       Iy=X_aux(13);  
       Kf=X_aux(14);   
       Kr=X_aux(15);   
       Cf=X_aux(16);   
       Cr=X_aux(17);  

       xv(1,:)=X(k2+1,1:12);
       xv(1,1:6)=[ Vx(k2+1) Vy(k2+1) r(k2+1) XI(k2+1) YI(k2+1) YawAngle(k2+1)];  xv(1,11:12)=[ PitchAngle(k2+1) RollAngle(k2+1) ];
       delta=scl*steering;
        
        for k3=1:dNs-1

       % wheel velocity in body-fixed frame
        Vlf=[xv(k3,1)  xv(k3,2) xv(k3,7)]'+ cross([xv(k3,9) xv(k3,8) xv(k3,3)]', [lf Wa/2 r_F-h]');
        Vrf=[xv(k3,1)  xv(k3,2) xv(k3,7)]'+ cross([xv(k3,9) xv(k3,8) xv(k3,3)]', [lf -Wa/2 r_F-h]');
        Vlr=[xv(k3,1)  xv(k3,2) xv(k3,7)]'+ cross([xv(k3,9) xv(k3,8) xv(k3,3)]', [-lr Wb/2 r_R-h]');
        Vrr=[xv(k3,1)  xv(k3,2) xv(k3,7)]'+ cross([xv(k3,9) xv(k3,8) xv(k3,3)]', [-lr -Wb/2 r_R-h]');

       % wheel velocity in wheel-fixed frame
       Tran=[cos(delta(k3*resample+k2)) sin(delta(k3*resample+k2)) 0; -sin(delta(k3*resample+k2)) cos(delta(k3*resample+k2)) 0; 0 0 1];
       Vlf=Tran*Vlf;
       Vrf=Tran*Vrf;

       Vlf_x=Vlf(1); Vlf_y=Vlf(2);
       Vrf_x=Vrf(1); Vrf_y=Vrf(2);
       Vlr_x=Vlr(1); Vlr_y=Vlr(2);
       Vrr_x=Vrr(1); Vrr_y=Vrr(2);   

       slf_x=( Vlf_x-w_lf(k3*resample+k2)*r_F )/( w_lf(k3*resample+k2)*r_F );
       slf_y=Vlf_y/( w_lf(k3*resample+k2)*r_F );
       slf= ( slf_x^2+slf_y^2 )^0.5;

       srf_x=( Vrf_x-w_rf(k3*resample+k2)*r_F )/( w_rf(k3*resample+k2)*r_F );
       srf_y=Vrf_y/( w_rf(k3*resample+k2)*r_F );
       srf= ( srf_x^2+srf_y^2 )^0.5;

       slr_x=( Vlr_x-w_lr(k3*resample+k2)*r_R )/( w_lr(k3*resample+k2)*r_R );
       slr_y=Vlr_y/( w_lr(k3*resample+k2)*r_R );
       slr= ( slr_x^2+slr_y^2 )^0.5;

       srr_x=( Vrr_x-w_rr(k3*resample+k2)*r_R )/( w_rr(k3*resample+k2)*r_R );
       srr_y=Vrr_y/( w_rr(k3*resample+k2)*r_R );
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

       Flf0=m*g*lr/L_axis/2;
       Frf0=Flf0;
       Flr0=m*g*lf/L_axis/2;
       Frr0=Flr0;  

       dFlf=Kf*(lf*xv(k3,11)-Wa/2*xv(k3,12)-xv(k3,10))+Cf*(lf*xv(k3,8)-Wa/2*xv(k3,9)-xv(k3,7));
       dFrf=Kf*(lf*xv(k3,11)+Wa/2*xv(k3,12)-xv(k3,10))+Cf*(lf*xv(k3,8)+Wa/2*xv(k3,9)-xv(k3,7));
       dFlr=Kr*(-lr*xv(k3,11)-Wb/2*xv(k3,12)-xv(k3,10))+Cr*(-lr*xv(k3,8)-Wb/2*xv(k3,9)-xv(k3,7));
       dFrr=Kr*(-lr*xv(k3,11)+Wb/2*xv(k3,12)-xv(k3,10))+Cr*(-lr*xv(k3,8)+Wb/2*xv(k3,9)-xv(k3,7));

       TireForce_z=[ Flf0+dFlf Frf0+dFrf Flr0+dFlr Frr0+dFrr ];

       f_lfx=mu_lfx*TireForce_z(1);
       f_lfy=mu_lfy*TireForce_z(1);
       f_rfx=mu_rfx*TireForce_z(2);
       f_rfy=mu_rfy*TireForce_z(2);
       f_lrx=mu_lrx*TireForce_z(3);
       f_lry=mu_lry*TireForce_z(3);
       f_rrx=mu_rrx*TireForce_z(4);
       f_rry=mu_rry*TireForce_z(4);

       dot_Vx=( ( f_lfx+f_rfx )*cos( delta(k3*resample+k2) )-( f_lfy+f_rfy )*sin( delta(k3*resample+k2) )+( f_lrx+f_rrx ) )/m+xv(k3,2)*xv(k3,3)-0.5*Cd*rho_A*Area*xv(k3,1)^2;  %no aerodynamics
       dot_Vy=( ( f_lfx+f_rfx )*sin( delta(k3*resample+k2) )+( f_lfy+f_rfy )*cos( delta(k3*resample+k2) )+( f_lry+f_rry ) )/m-xv(k3,1)*xv(k3,3);  
       dot_r=( ( ( f_lfy+f_rfy )*cos( delta(k3*resample+k2) )+( f_lfx+f_rfx )*sin( delta(k3*resample+k2) ) )*lf-( f_lry+f_rry )*lr )/Iz;

       T_rotation=[ cos(xv(k3,6)) sin(xv(k3,6)); -sin(xv(k3,6)) cos(xv(k3,6)) ];
       dot_pos=T_rotation'*[ xv(k3,1); xv(k3,2) ];

       dot_XI=dot_pos(1);
       dot_YI=dot_pos(2);  

       %calcualte x and  y acceleration of each wheel center, ignore verticle acceleration
       Acc_lf=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [lf Wa/2 r_F-h]')+cross([xv(k3,9) xv(k3,8) xv(k3,3)]',cross([xv(k3,9) xv(k3,8) xv(k3,3)]', [lf Wa/2 r_F-h]'));
       Acc_rf=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [lf -Wa/2 r_F-h]')+cross([xv(k3,9) xv(k3,8) xv(k3,3)]',cross([xv(k3,9) xv(k3,8) xv(k3,3)]', [lf -Wa/2 r_F-h]'));
       Acc_lr=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [-lr Wb/2 r_F-h]')+cross([xv(k3,9) xv(k3,8) xv(k3,3)]',cross([xv(k3,9) xv(k3,8) xv(k3,3)]', [-lr Wb/2 r_R-h]'));
       Acc_rr=[dot_Vx dot_Vy 0]'+ cross([0 0 dot_r]', [-lr -Wb/2 r_F-h]')+cross([xv(k3,9) xv(k3,8) xv(k3,3)]',cross([xv(k3,9) xv(k3,8) xv(k3,3)]', [-lr -Wb/2 r_R-h]')); 

       F_slfx=f_lfx*cos( delta(k3*resample+k2) )-f_lfy*sin( delta(k3*resample+k2) )-mf*Acc_lf(1);    F_slfy=f_lfx*sin( delta(k3*resample+k2) )+f_lfy*cos( delta(k3*resample+k2) )-mf*Acc_lf(2);
       F_srfx=f_rfx*cos( delta(k3*resample+k2) )-f_rfy*sin( delta(k3*resample+k2) )-mf*Acc_rf(1);    F_srfy=f_rfx*sin( delta(k3*resample+k2) )+f_rfy*cos( delta(k3*resample+k2) )-mf*Acc_rf(2);   
       F_slrx=f_lrx-mr*Acc_lr(1);    F_slry=f_lry-mr*Acc_lr(2);   
       F_srrx=f_rrx-mr*Acc_rr(1);    F_srry=f_rry-mr*Acc_rr(2);   

       dot_Zs=( -2*(Kf+Kr)*xv(k3,10)-2*(Cf+Cr)*xv(k3,7)+2*(lf*Kf-lr*Kr)*xv(k3,11)+2*(lf*Cf-lr*Cr)*xv(k3,8) )/ms;
       dot_theta=( 2*(lf*Kf-lr*Kr)*xv(k3,10)+2*(lf*Cf-lr*Cr)*xv(k3,7)-2*(lf^2*Kf+lr^2*Kr)*xv(k3,11)-2*(lf^2*Cf+lr^2*Cr)*xv(k3,8)-(F_slfx+F_srfx)*(h-r_F)-(F_slrx+F_srrx)*(h-r_R) )/Iy;
       dot_phi=( -Wa^2*Kf*xv(k3,12)/2-Wa^2*Cf*xv(k3,9)/2-Wb^2*Kr*xv(k3,12)/2-Wb^2*Cr*xv(k3,9)/2+(F_slfy+F_srfy)*(h-r_F)+(F_slry+F_srry)*(h-r_R) )/Ix;   

       xv(k3+1,:)=xv(k3,:)+[ dot_Vx dot_Vy dot_r dot_XI dot_YI xv(k3,3) dot_Zs dot_theta dot_phi xv(k3,7) xv(k3,8) xv(k3,9)]*dts;  

       end
        
    w_1=1/6; w_2=1/6; w_3=1/6;
    w_4=1/6; w_5=1/6; w_6=1/6;
    SelectRow=k2+(1:dNs)*resample;
    J = w_1*(xv(1:dNs,1)-Y(SelectRow,1))'*(xv(1:dNs,1)-Y(SelectRow,1))+w_2*(xv(1:dNs,2)-Y(SelectRow,2))'*(xv(1:dNs,2)-Y(SelectRow,2))+w_3*(xv(1:dNs,3)-Y(SelectRow,3))'*(xv(1:dNs,3)-Y(SelectRow,3))+...
        w_4*(xv(1:dNs,4)-Y(SelectRow,4))'*(xv(1:dNs,4)-Y(SelectRow,4))+w_5*(xv(1:dNs,5)-Y(SelectRow,5))'*(xv(1:dNs,5)-Y(SelectRow,5))+w_6*(xv(1:dNs,6)-Y(SelectRow,6))'*(xv(1:dNs,6)-Y(SelectRow,6)); 
       
    end
    
    
    
%% plot & save results

X_train=X(1:N,:);

t=0:dt:(dt*(N-1));
    figure;
    subplot(3,1,1);
    plot(t,Y(1:N,1),'b',t,Y_train(1:N,1),'r');
    legend('True Vx','Estimated Vx');     
    subplot(3,1,2);
    plot(t,Y(1:N,2),'b',t,Y_train(1:N,2),'r');
    legend('True Vy','Estimated Vy');  
    subplot(3,1,3);
    plot(t,Y(1:N,3),'b',t,Y_train(1:N,3),'r');
    legend('True r','Estimated r');  

    figure;
    plot(Y(1:N,4),Y(1:N,5),'b', Y_train(1:N,4),Y_train(1:N,5),'r');
    legend('True trajectory','Estimated trajectory'); 
    
    figure;
    plot(t,Y(1:N,6),'b',t,Y_train(1:N,6),'r');
    legend('True yaw angle','Estimated yaw angle');
    
    
    figure;
    subplot(3,2,1);
    plot(t,X(1:N,13)*scales(1));legend('Iz');
    subplot(3,2,2);
    plot(t,X(1:N,14)*scales(2));legend('lf');    
    subplot(3,2,3);
    plot(t,X(1:N,15)*scales(3));legend('h');    
    subplot(3,2,4);
    plot(t,X(1:N,18)*scales(6));legend('D');
    subplot(3,2,5);
    plot(t,X(1:N,22)*scales(10));legend('Gear ratio');    
    subplot(3,2,6);
    plot(t,X(1:N,23)*scales(11));legend('Air Coef'); 
       
    t=dt*N:dt:(dt*(lth-1));
    figure;
    subplot(3,1,1);
    plot(t,Y(N:lth-1,1),'b',t,Y_opt(N:lth-1,1),'r');
    legend('True Vx','Simulated Vx');
    subplot(3,1,2);
    plot(t,Y(N:lth-1,2),'b',t,Y_opt(N:lth-1,2),'r');
    legend('True Vy','Simulated Vy');
    subplot(3,1,3);
    plot(t,Y(N:lth-1,3),'b',t,Y_opt(N:lth-1,3),'r');
    legend('True r','Simulated r');
    
    figure;
    plot(Y(N:lth-1,4),Y(N:lth-1,5),'b', Y_opt(N:lth-1,4),Y_opt(N:lth-1,5),'r');
    legend('True trajectory','Simulated trajectory'); 
    
    figure;
    plot(t,Y(N:lth-1,6),'b',t,Y_opt(N:lth-1,6),'r');
    legend('True yaw angle','Simulated yaw angle');

    filename='XXVII_Adaptive_Limit_Memory_opt_Joint_UKF_11Dof.mat';
    save(filename, 'para_opt', 'J', 'X_train', 'Y_opt', 'Y_train', 'Q_buf', 'R_buf', 'bar_q_buf', 'bar_r_buf');     
  
    
    

end

