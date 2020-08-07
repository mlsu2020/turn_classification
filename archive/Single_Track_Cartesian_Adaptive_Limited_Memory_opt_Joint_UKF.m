function [ para_opt, X_train, Y_opt, Y_train ] = Single_Track_Cartesian_Adaptive_Limited_Memory_opt_Joint_UKF (vxj, vyj, wzj, wFj, wRj, Yawj, Xj, Yj, deltaj, Tj, dt)

Eps=1e-12;
min_velo = 0.1;

Index=2000; %10
window0=100; %112
% window0=37;
window=window0/dt; % take part of data for estimation and validation
vxj=vxj( Index(1):window+Index(1) );
vyj=vyj( Index(1):window+Index(1) );
wzj=wzj( Index(1):window+Index(1) );
wFj=wFj( Index(1):window+Index(1) );
wRj=wRj( Index(1):window+Index(1) );
Yawj=Yawj( Index(1):window+Index(1) );
Xj=Xj( Index(1):window+Index(1) );
Yj=Yj( Index(1):window+Index(1) );
% Xj=Xj( Index(1):window+Index(1) );
% Yj=Yj( Index(1):window+Index(1) );
deltaj=deltaj( Index(1):window+Index(1) );
Tj=Tj( Index(1):window+Index(1) );

dT=1;
dN=dT/dt; %left data of dT for validation
resample=50;
dts=resample*dt;
dNs=dT/dts;

Nr=1/dt;
Nq=1/dt;
Fade=0;
N0=max(Nr,Nq)+3/dt

tail=0;
Omit=tail/dt;
lth=length(Tj);
M=window0/((window0-78)+dT); % data split into 2 parts, first half for training
N=round((lth-rem(lth,M))/M)-dN

Qmax=1e-2;
Qmin=1e-14;
Rmax=1e-2;
Rmin=1e-14;
Pmax=1e4;
Pmin=1e-12;
Pyy_max=1e4;
dK=1;

%%
lFR = 0.57;
%          m   Iz   lF    IwF   IwR   h   B   C   D  st    mus E..
para0 = [  22  1.2 0.30  0.040 0.040 0.1 1.5 1.5 0.7 12.5 0.75 0.9 1 0 0 .008];
%          Iz         lf            h                  B               C      D       E                    Sh                 Sv                     scl              Cd                  Ix               Iy              Kf       Kr       Cf       Cr 
para_lb=[  20  1.0 0.28  0.030 0.030 0.09 0.5 0.5 0.1 8.0 0.7  0.7 0.8 -.1 -.1 .006];
para_ub=[  24  1.4 0.32  0.050 0.050 0.15 3.0 3.0 2.0 40.0 1.0  1.0  1.2 0.1 0.1 0.011];
% para_ub=[  3000      1.8       0.7       10      4      1.05     1       0.2     0.1    20       0.5    1000    3000     40000    50000     4000     4000  ];   %upper boundry

%% rescale the parameters
scales=para_ub/2;
para0=para0./scales;
para_lb=para_lb./scales;
para_ub=para_ub./scales;

%%
num_states = 8;%12;
num_params = 16;%17;
num_meas = 8;%8;
num_states_and_params = num_states + num_params;%29;
num_total = 2*num_states_and_params + num_meas;%66;

x0=[zeros(num_states,1); para0'];
x0(1:num_states)=[ vxj(1) vyj(1) wzj(1) wFj(1) wRj(1) Yawj(1) Xj(1) Yj(1)]'
% Y=[vxj; vyj; wzj; wFj; wRj; epsij; eyj; sj];  % use column vector

% w10=1e-5; w20=1e-6; v0=1e-3; P10=1e-5; P20=1e-4;
w10=1e-6; w20=3e-10; v0=1e-3; P10=1e-5; P20=11e-4;  
w10=1e-6; w20=1e-8; v0=1e-3; P10=1e-5; P20=1e-4;   w21=5e-5; %best results; j=3.48e6

%          state           para        meas          state         para
filter=[w10*ones(1,num_states) w20*ones(1,num_params) v0*ones(1,num_meas) P10*ones(1,num_states) P20*ones(1,num_params)];   % noise is upto 1% of its covariance   


%%
%   Detailed explanation goes here
   
   v=( filter(num_states+num_params+1:num_states+num_params+num_meas) )';
   P01=diag( filter(num_states_and_params+num_meas+1:num_states_and_params+num_meas+num_states).^2 );
   P02=diag( filter(num_states_and_params+num_meas+num_states+1:num_total).^2 );

   alf=1e-4;
   bet=2;
   L=num_total;
   kap=0;%3-L;
   
   lam=alf^2*(L+kap)-L;
   Wm0=lam/(L+lam);
   Wc0=lam/(L+lam)+(1-alf^2+bet);
   Wmi=1/(L+lam)/2;
   Wci=Wmi;

   Y=[ vxj vyj wzj wFj wRj Yawj Xj Yj ]; 
   Ye=Y;
   Yeb=Ye;

   P1=[ P01 0*ones(num_states,num_params); 0*ones(num_params,num_states) P02 ];  % ok results

   Qw1=diag( filter(1:num_states).^2 );
   Qw2=diag( filter(num_states+1:num_states+num_params).^2 );
   Qw=[ Qw1 0*ones(num_states,num_params); 0*ones(num_params,num_states) Qw2 ];
   Rv=diag( v.^2 );

   X=zeros(lth,num_states_and_params);
   X(1,:)=x0';
   Xc=X;

   Xa=zeros(lth,num_total);
   Xa(1,1:num_states_and_params)=x0'; 

   Xe=zeros(lth,num_states_and_params);
   Xe(1,:)=x0';
   Xeb=Xe;
   xv=zeros(dNs,num_states);
   Xs=zeros(lth,num_states); %Xs(1,:)=[ Vx(1) Vy(1) r(1)  XI(1)  YI(1)  YawAngle(1) 0 0 0];
  
   Sgm_x2=zeros(num_states_and_params,2*L+1);  
   Sgm_x2s=Sgm_x2;
   Sgm_x2b=Sgm_x2;
   
   q0=zeros(num_states_and_params,1);
   r0=zeros(num_meas,1);
   
   q_buf=zeros(num_states_and_params,N);
   bar_q_buf=zeros(num_states_and_params,N);
   r_buf=zeros(num_meas,N);
   bar_r_buf=zeros(num_meas,N);
   
   Del_buf=zeros(num_states_and_params,num_states_and_params,N);
   Gam_buf=zeros(num_meas,num_meas,N);
   Q_buf=zeros(num_states_and_params,num_states_and_params,N);
   R_buf=zeros(num_meas,num_meas,N);
   
   P1_buf=zeros(num_states_and_params,num_states_and_params,N);
   
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
    
    root_Pa=(L+lam)^0.5*[ real(sqrtm(P1)) zeros(num_states_and_params,num_states_and_params+num_meas); zeros(num_states_and_params,num_states_and_params) real(sqrtm(Q)) zeros(num_states_and_params,num_meas); zeros(num_meas,num_total-num_meas) real(sqrtm(R)) ];
    
    if ~isreal(root_Pa)
       fprintf('Pa not real \n'); 
    end

    Sgm_a=[Xa(k,:)' krone( ones(1,L),Xa(k,:)' )+root_Pa krone( ones(1,L),Xa(k,:)' )-root_Pa ];
    
    Sgm_x1=Sgm_a(1:num_states_and_params,:);
    Sgm_w=Sgm_a(num_states_and_params+1:2*num_states_and_params,:);
    Sgm_v=Sgm_a(2*num_states_and_params+1:num_total,:);

   for jj=1:2*L+1 
   
   Sgm_x1_aux=Sgm_x1(num_states+1:num_states_and_params,jj).*scales';
   mj=Sgm_x1_aux(1);
   Izj=Sgm_x1_aux(2);
   lFj=Sgm_x1_aux(3);
   IwFj=Sgm_x1_aux(4);
   IwRj=Sgm_x1_aux(5);
   hj=Sgm_x1_aux(6);
   Bj=Sgm_x1_aux(7);
   Cj=Sgm_x1_aux(8);
   Dj=Sgm_x1_aux(9);
   kSteerj=Sgm_x1_aux(10);
%    kThrottlej=Sgm_x1_aux(11);
%    kTorquej=Sgm_x1_aux(12);
   mu1j=Sgm_x1_aux(11);
   mu2j=Sgm_x1_aux(12);
   Ej=Sgm_x1_aux(13);
   Shj=Sgm_x1_aux(14);
   Svj=Sgm_x1_aux(15);
   cSteerj=Sgm_x1_aux(16);

    m_Vehicle_m = mj;
    m_Vehicle_Iz = Izj;
    m_Vehicle_lF = lFj;
    m_Vehicle_lR = lFR - lFj;
    m_Vehicle_IwF = IwFj;
    m_Vehicle_IwR = IwRj;
    m_Vehicle_rF = 0.095;
    m_Vehicle_rR = 0.095;
    m_Vehicle_mu1 = mu1j;
    m_Vehicle_mu2 = mu2j;
    m_Vehicle_h = hj;    
    m_g = 9.80665;
    
    m_Vehicle_kSteering = kSteerj;
%     m_Vehicle_kThrottle = kThrottlej;
%     m_Vehicle_kTorque = kTorquej;
    m_Vehicle_cSteering = cSteerj;

    tire_B = Bj;
    tire_C = Cj;
    tire_D = Dj;
    tire_E = Ej;
    tire_Sh = Shj;
    tire_Sv = Svj;
    
    vxk = Sgm_x1(1, jj);
    vyk = Sgm_x1(2, jj);
    wzk = Sgm_x1(3, jj);
    wFk = Sgm_x1(4, jj);
    wRk = Sgm_x1(5, jj);
    Yawk = Sgm_x1(6, jj);
    Xk = Sgm_x1(7, jj);
    Yk = Sgm_x1(8, jj);
%     Xk = Sgm_x1(9, jj);
%     Yk = Sgm_x1(10,jj);
    deltak = m_Vehicle_kSteering * deltaj(k) + m_Vehicle_cSteering;
%     Tk = m_Vehicle_kThrottle * Tj(k);
    
    fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * m_Vehicle_mu1) / (m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h * (m_Vehicle_mu1 * cos(deltak) - m_Vehicle_mu2 * sin(deltak) - m_Vehicle_mu1));
    fRz = m_Vehicle_m * m_g - fFz;
    if (vxk < min_velo)
      vxk = min_velo;
    end
    if (wFk < min_velo / m_Vehicle_rF)
        wFk = min_velo / m_Vehicle_rF + 1e-4;
    end
    if wRk < min_velo / m_Vehicle_rR
        wRk = min_velo / m_Vehicle_rR + 1e-4;
    end
    if (vxk ~= 0.0)
      beta = atan2(vyk, vxk);
    else
      beta = 0.0;
    end
    V = sqrt(vxk * vxk + vyk * vyk);
    vFx = V * cos(beta - deltak) + wzk * m_Vehicle_lF * sin(deltak);
    vFy = V * sin(beta - deltak) + wzk * m_Vehicle_lF * cos(deltak);
    vRx = vxk;
    vRy = vyk - wzk * m_Vehicle_lR;
    if (wFk ~= 0.0)
      sFx = (vFx - wFk * m_Vehicle_rF) / (wFk * m_Vehicle_rF);
    else
      sFx = 0.0;
    end
    if (wRk ~= 0.0)
      sRx = (vRx - wRk * m_Vehicle_rR) / (wRk * m_Vehicle_rR);
    else
      sRx = 0.0;
    end
    if (vFx ~= 0.0)
      sFy = (1 + sFx) * vFy / vFx;
    else
      sFy = 0.0;
    end
    if (vRx ~= 0.0)
      sRy = (1 + sRx) * vRy / vRx;
    else
      sRy = 0.0;
    end
    sF = sqrt(sFx * sFx + sFy * sFy);
    sR = sqrt(sRx * sRx + sRy * sRy);
    sEF = sF - tire_Sh;
    sER = sR - tire_Sh;
    muF = tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(sEF) ) ) ) + tire_Sv;
    muR = tire_D*sin( tire_C*atan( tire_B*sER - tire_E*(tire_B*sER - atan(sER) ) ) ) + tire_Sv;
%     muF = tire_D * sin(tire_C * atan(tire_B * sF)); 
%     muR = tire_D * sin(tire_C * atan(tire_B * sR));
    fFx = -sFx / sF * muF * fFz;
    fFy = -sFy / sF * muF * fFz;
    fRx = -sRx / sR * muR * fRz;
    fRy = -sRy / sR * muR * fRz;
    
    %%%%%%%%%%%%%%%%%%%
    sEF = -(vFx - wFk * m_Vehicle_rF) / (vFx) + tire_Sh;
    fFx = tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;
    fFx = fFz * fFx;
    sEF = -(vRx - wRk * m_Vehicle_rF) / (vRx) + tire_Sh;
    fRx = fFz * tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;

    sEF = atan(vFy / abs(vFx)) + tire_Sh;
    fFy = -fRz * tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;
    sEF = atan(vRy / abs(vRx)) + tire_Sh;
    fRy = -fRz * tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;
    %%%%%%%%%%%%%%%%%%%

    dot_vx = ((fFx * cos(deltak) - fFy * sin(deltak) + fRx) / m_Vehicle_m + vyk * wzk);
    dot_vy = ((fFx * sin(deltak) + fFy * cos(deltak) + fRy) / m_Vehicle_m - vxk * wzk);
    dot_wz = ((fFy * cos(deltak) + fFx * sin(deltak)) * m_Vehicle_lF - fRy * m_Vehicle_lR) / m_Vehicle_Iz;
    dot_wF = -1* m_Vehicle_rF / m_Vehicle_IwF * fFx;
%     dot_wR = (m_Vehicle_kTorque * (Tk-wRk) - m_Vehicle_rR * fRx) / m_Vehicle_IwR;
    dot_Yaw = wzk;
    dot_X = (vxk * cos(Yawk) - vyk * sin(Yawk));
    dot_Y = (vxk * sin(Yawk) + vyk * cos(Yawk));
   
   Sgm_x2(1,jj)=Sgm_x1(1,jj)+dot_vx*dt;
   Sgm_x2(2,jj)=Sgm_x1(2,jj)+dot_vy*dt;
   Sgm_x2(3,jj)=Sgm_x1(3,jj)+dot_wz*dt;
   Sgm_x2(4,jj)=Sgm_x1(4,jj)+dot_wF*dt; 
   Sgm_x2(5,jj)=wRj(k);
   Sgm_x2(6,jj)=Sgm_x1(6,jj)+dot_Yaw*dt;
   Sgm_x2(7,jj)=Sgm_x1(7,jj)+dot_X*dt;
   Sgm_x2(8,jj)=Sgm_x1(8,jj)+dot_Y*dt;
   Sgm_x2(num_states+1:num_states_and_params,jj)=Sgm_x1(num_states+1:num_states_and_params,jj)+zeros(num_params,1);
   
   Sgm_x2s(:,jj)=Sgm_x2(:,jj);
   Sgm_x2(:,jj) = Sgm_x2(:,jj)+Sgm_w(:,jj)*sqrt(dt);   
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

    Cdz=[eye(num_meas, num_states) zeros(num_meas,num_params)];
%     Cdz(1,1)=1;
%     Cdz(2,2)=1;
%     Cdz(3,3)=1;
%     Cdz(4,4)=1;
%     Cdz(5,5)=1;
%     Cdz(6,7)=1;
%     Cdz(7,8)=1;
    %     size(Sgm_x2)
%     size(Sgm_v)
%     size(Cdz)
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

      D_g=[ zeros(num_params,num_states) eye(num_params); zeros(1,num_states_and_params); zeros(num_params,num_states) -eye(num_params)]; % let -Vx<0 -> Vx>0
    
      B_cons=[ para_ub'; 0; -para_lb'];
      
    if ~isreal(D_g)
       fprintf('D_g not real \n'); 
    end
      
      
      P_aux=P1+Eps*eye(num_states_and_params);
      Inv_P=eye(num_states_and_params)/(P_aux); % inv(P_aux)
%     X(k+1,:)';
            
        if isreal(Inv_P)
        opts = optimoptions('quadprog','Algorithm','interior-point-convex ','Display','final');
        Aux = quadprog(Inv_P+Inv_P',-(Inv_P+Inv_P')*X(k+1,:)',D_g,B_cons,[],[],[],[],[],opts);
        size(Aux);
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
    fprintf("here!!!!!!!!!!!!!!!!!!\n");
%% for big k

     Q=Qw;
     R=Rv;     
     
     
     for k=N0: N-1

% prediction and observation


    root_Pa=(L+lam)^0.5*[ real(sqrtm(P1)) zeros(num_states_and_params,num_states_and_params+num_meas); zeros(num_states_and_params,num_states_and_params) real(sqrtm(Q)) zeros(num_states_and_params,num_meas); zeros(num_meas,num_total-num_meas) real(sqrtm(R)) ];
%     root_Pb=(L+lam)^0.5*[ real(sqrtm(P1)) zeros(29,37); zeros(29,29) real(sqrtm(squeeze(Q_buf(:,:,k-1)))) zeros(29,8); zeros(8,58) real(sqrtm(squeeze(R_buf(:,:,k-1)))) ];
    root_Pb=(L+lam)^0.5*[ real(sqrtm(P1)) zeros(num_states_and_params,num_states_and_params+num_meas); zeros(num_states_and_params,num_states_and_params) real(sqrtm(squeeze(Q_buf(:,:,k-1)))) zeros(num_states_and_params,num_meas); zeros(num_meas,num_total-num_meas) real(sqrtm(R)) ];
    
    if ~isreal(root_Pa)
       fprintf('Pa not real \n'); 
    end 
    
    Sgm_a=[Xa(k,:)' krone( ones(1,L),Xa(k,:)' )+root_Pa krone( ones(1,L),Xa(k,:)' )-root_Pa ];
    Sgm_b=[Xa(k,:)' krone( ones(1,L),Xa(k,:)' )+root_Pb krone( ones(1,L),Xa(k,:)' )-root_Pb ];
    
    Sgm_x1=Sgm_a(1:num_states_and_params,:);
    Sgm_w=Sgm_a(num_states_and_params+1:num_total-num_meas,:);   Sgm_wb=Sgm_b(num_states_and_params+1:num_total-num_meas,:);    
    Sgm_v=Sgm_a(num_total-num_meas+1:num_total,:);   Sgm_vb=Sgm_b(num_total-num_meas+1:num_total,:);

   for jj=1:2*L+1 
   
   Sgm_x1_aux=Sgm_x1(num_states+1:num_states_and_params,jj).*scales';
   mj=Sgm_x1_aux(1);
   Izj=Sgm_x1_aux(2);
   lFj=Sgm_x1_aux(3);
   IwFj=Sgm_x1_aux(4);
   IwRj=Sgm_x1_aux(5);
   hj=Sgm_x1_aux(6);
   Bj=Sgm_x1_aux(7);
   Cj=Sgm_x1_aux(8);
   Dj=Sgm_x1_aux(9);
   kSteerj=Sgm_x1_aux(10);
%    kThrottlej=Sgm_x1_aux(11);
%    kTorquej=Sgm_x1_aux(12);   
   mu1j=Sgm_x1_aux(11);
   mu2j=Sgm_x1_aux(12);
   Ej=Sgm_x1_aux(13);
   Shj=Sgm_x1_aux(14);
   Svj=Sgm_x1_aux(15);
   cSteerj=Sgm_x1_aux(16);

    m_Vehicle_m = mj;
    m_Vehicle_Iz = Izj;
    m_Vehicle_lF = lFj;
    m_Vehicle_lR = lFR - lFj;
    m_Vehicle_IwF = IwFj;
    m_Vehicle_IwR = IwRj;
    m_Vehicle_rF = 0.095;
    m_Vehicle_rR = 0.095;
    m_Vehicle_mu1 = mu1j;
    m_Vehicle_mu2 = mu2j;
    m_Vehicle_h = hj;    
    m_g = 9.80665;
    
    m_Vehicle_kSteering = kSteerj;
%     m_Vehicle_kThrottle = kThrottlej;
%     m_Vehicle_kTorque = kTorquej;
    m_Vehicle_cSteering = cSteerj;

    tire_B = Bj;
    tire_C = Cj;
    tire_D = Dj;
    tire_E = Ej;
    tire_Sh = Shj;
    tire_Sv = Svj;
    
    vxk = Sgm_x1(1, jj);
    vyk = Sgm_x1(2, jj);
    wzk = Sgm_x1(3, jj);
    wFk = Sgm_x1(4, jj);
    wRk = Sgm_x1(5, jj);
    Yawk = Sgm_x1(6, jj);
    Xk = Sgm_x1(7, jj);
    Yk = Sgm_x1(8, jj);
%     Xk = Sgm_x1(9, jj);
%     Yk = Sgm_x1(10,jj);
    deltak = m_Vehicle_kSteering * deltaj(k) + m_Vehicle_cSteering;
%     Tk = m_Vehicle_kThrottle * Tj(k);
    
    fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * m_Vehicle_mu1) / (m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h * (m_Vehicle_mu1 * cos(deltak) - m_Vehicle_mu2 * sin(deltak) - m_Vehicle_mu1));
    fRz = m_Vehicle_m * m_g - fFz;
    if (vxk < min_velo)
      vxk = min_velo;
    end
    if (wFk < min_velo / m_Vehicle_rF)
        wFk = min_velo / m_Vehicle_rF + 1e-4;
    end
    if wRk < min_velo / m_Vehicle_rR
        wRk = min_velo / m_Vehicle_rR + 1e-4;
    end
    if (vxk ~= 0.0)
      beta = atan2(vyk, vxk);
    else
      beta = 0.0;
    end
    V = sqrt(vxk * vxk + vyk * vyk);
    vFx = V * cos(beta - deltak) + wzk * m_Vehicle_lF * sin(deltak);
    vFy = V * sin(beta - deltak) + wzk * m_Vehicle_lF * cos(deltak);
    vRx = vxk;
    vRy = vyk - wzk * m_Vehicle_lR;
    if (wFk ~= 0.0)
      sFx = (vFx - wFk * m_Vehicle_rF) / (wFk * m_Vehicle_rF);
    else
      sFx = 0.0;
    end
    if (wRk ~= 0.0)
      sRx = (vRx - wRk * m_Vehicle_rR) / (wRk * m_Vehicle_rR);
    else
      sRx = 0.0;
    end
    if (vFx ~= 0.0)
      sFy = (1 + sFx) * vFy / vFx;
    else
      sFy = 0.0;
    end
    if (vRx ~= 0.0)
      sRy = (1 + sRx) * vRy / vRx;
    else
      sRy = 0.0;
    end
    sF = sqrt(sFx * sFx + sFy * sFy);
    sR = sqrt(sRx * sRx + sRy * sRy);
    sEF = sF - tire_Sh;
    sER = sR - tire_Sh;
    muF = tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(sEF) ) ) ) + tire_Sv;
    muR = tire_D*sin( tire_C*atan( tire_B*sER - tire_E*(tire_B*sER - atan(sER) ) ) ) + tire_Sv;
%     muF = tire_D * sin(tire_C * atan(tire_B * sF)); 
%     muR = tire_D * sin(tire_C * atan(tire_B * sR));
    fFx = -sFx / sF * muF * fFz;
    fFy = -sFy / sF * muF * fFz;
    fRx = -sRx / sR * muR * fRz;
    fRy = -sRy / sR * muR * fRz;
    
    %%%%%%%%%%%%%%%%%%%
    sEF = -(vFx - wFk * m_Vehicle_rF) / (vFx) + tire_Sh;
    fFx = tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;
    fFx = fFz * fFx;
    sEF = -(vRx - wRk * m_Vehicle_rF) / (vRx) + tire_Sh;
    fRx = fFz * tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;

    sEF = atan(vFy / abs(vFx)) + tire_Sh;
    fFy = -fRz * tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;
    sEF = atan(vRy / abs(vRx)) + tire_Sh;
    fRy = -fRz * tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;
    %%%%%%%%%%%%%%%%%%%

    dot_vx = ((fFx * cos(deltak) - fFy * sin(deltak) + fRx) / m_Vehicle_m + vyk * wzk);
    dot_vy = ((fFx * sin(deltak) + fFy * cos(deltak) + fRy) / m_Vehicle_m - vxk * wzk);
    dot_wz = ((fFy * cos(deltak) + fFx * sin(deltak)) * m_Vehicle_lF - fRy * m_Vehicle_lR) / m_Vehicle_Iz;
    dot_wF = -1* m_Vehicle_rF / m_Vehicle_IwF * fFx;
%     dot_wR = (m_Vehicle_kTorque * (Tk-wRk) - m_Vehicle_rR * fRx) / m_Vehicle_IwR;
    dot_Yaw = wzk;
    dot_X = (vxk * cos(Yawk) - vyk * sin(Yawk));
    dot_Y = (vxk * sin(Yawk) + vyk * cos(Yawk));

   Sgm_x2(1,jj)=Sgm_x1(1,jj)+dot_vx*dt;
   Sgm_x2(2,jj)=Sgm_x1(2,jj)+dot_vy*dt;
   Sgm_x2(3,jj)=Sgm_x1(3,jj)+dot_wz*dt;
   Sgm_x2(4,jj)=Sgm_x1(4,jj)+dot_wF*dt; 
   Sgm_x2(5,jj)=wRj(k); 
   Sgm_x2(6,jj)=Sgm_x1(6,jj)+dot_Yaw*dt;
   Sgm_x2(7,jj)=Sgm_x1(7,jj)+dot_X*dt;
   Sgm_x2(8,jj)=Sgm_x1(8,jj)+dot_Y*dt;
   Sgm_x2(num_states+1:num_states_and_params,jj)=Sgm_x1(num_states+1:num_states_and_params,jj)+zeros(num_params,1);
   
   Sgm_x2s(:,jj)=Sgm_x2(:,jj);
   Sgm_x2b(:,jj)=Sgm_x2(:,jj)+Sgm_wb(:,jj)*sqrt(dt);
   Sgm_x2(:,jj) = Sgm_x2(:,jj)+Sgm_w(:,jj)*sqrt(dt); 
   
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
      
    Cdz=[eye(num_meas, num_states) zeros(num_meas,num_params)];
%     Cdz(1,1)=1;
%     Cdz(2,2)=1;
%     Cdz(3,3)=1;
%     Cdz(4,4)=1;
%     Cdz(5,5)=1;
%     Cdz(6,7)=1;
%     Cdz(7,8)=1;
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
     K(num_states+1:end,:)=dK*K(num_states+1:end,:);
     Kb(num_states+1:end,:)=dK*Kb(num_states+1:end,:);

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

      D_g=[ zeros(num_params,num_states) eye(num_params); zeros(1,num_states_and_params); zeros(num_params,num_states) -eye(num_params)]; % let -Vx<0 -> Vx>0
    
      B_cons=[ para_ub'; 0; -para_lb'];
      
    if ~isreal(D_g)
       fprintf('D_g not real \n'); 
    end
      
      
      P_aux=P1+Eps*eye(num_states_and_params);
      Inv_P=eye(num_states_and_params)/(P_aux); % inv(P_aux)
      P_auxb=P1b+Eps*eye(num_states_and_params);
      Inv_Pb=eye(num_states_and_params)/(P_auxb); % inv(P_aux)
      
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
     factor=w21./(diagQ(num_states+1:end)).^0.5;
     Qaux(num_states+1:end,:)=Qaux(num_states+1:end,:).*krone(ones(1,num_states_and_params),factor);
     Qaux(:,num_states+1:end)=Qaux(:,num_states+1:end).*krone(ones(num_states_and_params,1),factor');
     Q=Qaux;
     Q_buf(:,:,k+1)=Q;
     
     bar_q_buf(num_states+1:end,k+1)=0*bar_q_buf(num_states+1:end,k+1);% mean value of the parameter is set to be 0
     
     Xa(k+1,:)= [X(k+1,:) bar_q_buf(:,k+1)'  bar_r_buf(:,k+1)']; 
%      Xa(k+1,:)= [X(k+1,:) 0*bar_q_buf(:,k+1)'  0*bar_r_buf(:,k+1)'];  
     P1_buf(:,:,k+1)=P1;
     
    end
    
    fprintf("here2222?????\n");
 %%  results   

    %         Iz   lf   h    B   C   D   E   Sh   Sv   scl   Cd 
   X_aux=X(N,num_states+1:num_states_and_params).*scales;
   mj=X_aux(1);
   Izj=X_aux(2);
   lFj=X_aux(3);
   IwFj=X_aux(4);
   IwRj=X_aux(5);
   hj=X_aux(6);
   Bj=X_aux(7);
   Cj=X_aux(8);
   Dj=X_aux(9);
   kSteerj=Sgm_x1_aux(10);
%    kThrottlej=Sgm_x1_aux(11);
%    kTorquej=Sgm_x1_aux(12);
   mu1j=Sgm_x1_aux(11);
   mu2j=Sgm_x1_aux(12);
   Ej=Sgm_x1_aux(13);
   Shj=Sgm_x1_aux(14);
   Svj=Sgm_x1_aux(15);
   cSteerj=Sgm_x1_aux(16);
    
 %% simulated results

   Xs(N,:)=X(N,1:num_states);
   Xs(N,1:num_states)=[ vxj(N) vyj(N) wzj(N) wFj(N) wRj(N) Yawj(N) Xj(N) Yj(N)];  %Xs(N,11:12)=[ PitchAngle(N) RollAngle(N) ];
%    Xs(N:lth-1,num_states-1:num_states) = [deltaj(N:lth-1) Tj(N:lth-1)];
%    delta=scl*steering2;
   
   for k=N:lth-1

    m_Vehicle_m = 21.88;%mj;
    m_Vehicle_Iz = 1.124;%Izj;
    m_Vehicle_lF = 0.34;%lFj;
    m_Vehicle_lR = lFR - m_Vehicle_lF;
    m_Vehicle_IwF = IwFj;
    m_Vehicle_IwR = IwRj;
    m_Vehicle_rF = 0.095;
    m_Vehicle_rR = 0.095;
    m_Vehicle_mu1 = mu1j;
    m_Vehicle_mu2 = mu2j;
    m_Vehicle_h = hj;    
    m_g = 9.80665;
    
    m_Vehicle_kSteering = kSteerj;
    m_Vehicle_cSteering = cSteerj;
%     m_Vehicle_kThrottle = kThrottlej;
%     m_Vehicle_kTorque = kTorquej;

    tire_B = Bj;
    tire_C = Cj;
    tire_D = Dj;
    tire_E = Ej;
    tire_Sh = Shj;
    tire_Sv = Svj;
    
    vxk = Xs(k, 1);
    vyk = Xs(k, 2);
    wzk = Xs(k, 3);
    wFk = Xs(k, 4);
    wRk = Xs(k, 5);
    Yawk = Xs(k, 6);
    Xk = Xs(k, 7);
    Yk = Xs(k, 8);
%     Xk = Xs(k, 9);
%     Yk = Xs(k, 10);
    deltak = m_Vehicle_kSteering * deltaj(k) + m_Vehicle_cSteering;
%     Tk = m_Vehicle_kThrottle * Tj(k);
    
    fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * m_Vehicle_mu1) / (m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h * (m_Vehicle_mu1 * cos(deltak) - m_Vehicle_mu2 * sin(deltak) - m_Vehicle_mu1));
    fRz = m_Vehicle_m * m_g - fFz;
    if (vxk < min_velo)
      vxk = min_velo;
    end
    if (wFk < min_velo / m_Vehicle_rF)
        wFk = min_velo / m_Vehicle_rF + 1e-4;
    end
    if wRk < min_velo / m_Vehicle_rR
        wRk = min_velo / m_Vehicle_rR + 1e-4;
    end
    if (vxk ~= 0.0)
      beta = atan2(vyk, vxk);
    else
      beta = 0.0;
    end
    V = sqrt(vxk * vxk + vyk * vyk);
    vFx = V * cos(beta - deltak) + wzk * m_Vehicle_lF * sin(deltak);
    vFy = V * sin(beta - deltak) + wzk * m_Vehicle_lF * cos(deltak);
    vRx = vxk;
    vRy = vyk - wzk * m_Vehicle_lR;
    
    if (wFk ~= 0.0)
      sFx = (vFx - wFk * m_Vehicle_rF) / (wFk * m_Vehicle_rF);
    else
      sFx = 0.0;
    end
    if (wRk ~= 0.0)
      sRx = (vRx - wRk * m_Vehicle_rR) / (wRk * m_Vehicle_rR);
    else
      sRx = 0.0;
    end
    if (vFx ~= 0.0)
      sFy = (1 + sFx) * vFy / vFx;
    else
      sFy = 0.0;
    end
    if (vRx ~= 0.0)
      sRy = (1 + sRx) * vRy / vRx;
    else
      sRy = 0.0;
    end
    
    sF = sqrt(sFx * sFx + sFy * sFy);
    sR = sqrt(sRx * sRx + sRy * sRy);
    sEF = sF - tire_Sh;
    sER = sR - tire_Sh;
    muF = tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(sEF) ) ) ) + tire_Sv;
    muR = tire_D*sin( tire_C*atan( tire_B*sER - tire_E*(tire_B*sER - atan(sER) ) ) ) + tire_Sv;
%     muF = tire_D * sin(tire_C * atan(tire_B * sF)); 
%     muR = tire_D * sin(tire_C * atan(tire_B * sR));
    fFx = -sFx / sF * muF * fFz;
    fFy = -sFy / sF * muF * fFz;
    fRx = -sRx / sR * muR * fRz;
    fRy = -sRy / sR * muR * fRz;
    
    %%%%%%%%%%%%%%%%%%%
    sEF = -(vFx - wFk * m_Vehicle_rF) / (vFx) + tire_Sh;
    fFx = tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;
    fFx = fFz * fFx;
    sEF = -(vRx - wRk * m_Vehicle_rF) / (vRx) + tire_Sh;
    fRx = fFz * tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;

    sEF = atan(vFy / abs(vFx)) + tire_Sh;
    fFy = -fRz * tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;
    sEF = atan(vRy / abs(vRx)) + tire_Sh;
    fRy = -fRz * tire_D*sin( tire_C*atan( tire_B*sEF - tire_E*(tire_B*sEF - atan(tire_B*sEF) ) ) ) + tire_Sv;
    %%%%%%%%%%%%%%%%%%%

    dot_vx = ((fFx * cos(deltak) - fFy * sin(deltak) + fRx) / m_Vehicle_m + vyk * wzk);
    dot_vy = ((fFx * sin(deltak) + fFy * cos(deltak) + fRy) / m_Vehicle_m - vxk * wzk);
    dot_wz = ((fFy * cos(deltak) + fFx * sin(deltak)) * m_Vehicle_lF - fRy * m_Vehicle_lR) / m_Vehicle_Iz;
    dot_wF = -1* m_Vehicle_rF / m_Vehicle_IwF * fFx;
%     dot_wR = (m_Vehicle_kTorque * (Tk-wRk) - m_Vehicle_rR * fRx) / m_Vehicle_IwR;
    dot_Yaw = wzk;
    dot_X = (vxk * cos(Yawk) - vyk * sin(Yawk));
    dot_Y = (vxk * sin(Yawk) + vyk * cos(Yawk));

   Xs(k+1,1)=Xs(k,1)+dot_vx*dt;
   Xs(k+1,2)=Xs(k,2)+dot_vy*dt;
   Xs(k+1,3)=Xs(k,3)+dot_wz*dt;
   Xs(k+1,4)=Xs(k,4)+dot_wF*dt; 
   Xs(k+1,5)=wRj(k); 
   Xs(k+1,6)=Xs(k,6)+dot_Yaw*dt;
   Xs(k+1,7)=Xs(k,7)+dot_X*dt;
   Xs(k+1,8)=Xs(k,8)+dot_Y*dt;
   
   end   
   
%     w_1=1/8; w_2=1/8; w_3=1/8;
%     w_4=1/2; w_5=1/2; w_6=1/4;
%     J = w_1*(Xs(N:lth-1-Omit,1)-Y(N:lth-1-Omit,1))'*(Xs(N:lth-1-Omit,1)-Y(N:lth-1-Omit,1))+w_2*(Xs(N:lth-1-Omit,2)-Y(N:lth-1-Omit,2))'*(Xs(N:lth-1-Omit,2)-Y(N:lth-1-Omit,2))+w_3*(Xs(N:lth-1-Omit,3)-Y(N:lth-1-Omit,3))'*(Xs(N:lth-1-Omit,3)-Y(N:lth-1-Omit,3))+...
%         w_4*(Xs(N:lth-1-Omit,4)-Y(N:lth-1-Omit,4))'*(Xs(N:lth-1-Omit,4)-Y(N:lth-1-Omit,4))+w_5*(Xs(N:lth-1-Omit,5)-Y(N:lth-1-Omit,5))'*(Xs(N:lth-1-Omit,5)-Y(N:lth-1-Omit,5))+w_6*(Xs(N:lth-1-Omit,6)-Y(N:lth-1-Omit,6))'*(Xs(N:lth-1-Omit,6)-Y(N:lth-1-Omit,6));
    
%     fprintf('The cost is %e\n', J);
    
    para_opt=[ mj Izj lFj IwFj IwRj hj Bj Cj Dj kSteerj mu1j mu2j Ej Shj Svj cSteerj];

    Y_opt=Xs;
    Y_train=X;
   
    
%% plot & save results

X_train=X(1:N,:);

t=0:dt:(dt*(N-1));
    figure;
    subplot(4,2,1);
    plot(t,Y(1:N,1),'b',t,Y_train(1:N,1),'r');
    legend('True vx','Estimated vx');     
    subplot(4,2,2);
    plot(t,Y(1:N,2),'b',t,Y_train(1:N,2),'r');
    legend('True vy','Estimated vy');  
    subplot(4,2,3);
    plot(t,Y(1:N,3),'b',t,Y_train(1:N,3),'r');
    legend('True wz','Estimated wz');  
    subplot(4,2,4);
    plot(t,Y(1:N,4),'b',t,Y_train(1:N,4),'r');
    legend('True wF','Estimated wF');
    subplot(4,2,5);
    plot(t,Y(1:N,5),'b',t,Y_train(1:N,5),'r');
    legend('True wR','Estimated wR');
    subplot(4,2,6);
    plot(t,Y(1:N,6),'b',t,Y_train(1:N,6),'r');
    legend('True Yaw','Estimated Yaw');
    subplot(4,2,7);
    plot(t,Y(1:N,7),'b',t,Y_train(1:N,7),'r');
    legend('True X','Estimated X');
    subplot(4,2,8);
    plot(t,Y(1:N,8),'b',t,Y_train(1:N,8),'r');
    legend('True Y','Estimated Y');
%     subplot(4,2,9);
%     plot(t,Y(1:N,9),'b',t,Y_train(1:N,9),'r');
%     legend('True X','Estimated X');
%     subplot(5,2,10);
%     plot(t,Y(1:N,10),'b',t,Y_train(1:N,10),'r');
%     legend('True Y','Estimated Y');

%     figure;
%     plot(Y(1:N,9),Y(1:N,10),'b', Y_train(1:N,9),Y_train(1:N,10),'r');
%     legend('True trajectory','Estimated trajectory'); 
    
%     figure;
%     plot(t,Y(1:N,6),'b',t,Y_train(1:N,6),'r');
%     legend('True yaw angle','Estimated yaw angle');
    
    
    figure;
    subplot(6,2,1);
    plot(t,X(1:N,num_states+1)*scales(1));legend('m');
    subplot(6,2,2);
    plot(t,X(1:N,num_states+2)*scales(2));legend('Iz');    
    subplot(6,2,3);
    plot(t,X(1:N,num_states+16)*scales(16));legend('cSteering');    
    subplot(6,2,4);
    plot(t,X(1:N,num_states+13)*scales(13));legend('E');
    subplot(6,2,5);
    plot(t,X(1:N,num_states+14)*scales(14));legend('Sh');
    subplot(6,2,6);
    plot(t,X(1:N,num_states+15)*scales(15));legend('Sv'); 
    subplot(6,2,7);
    plot(t,X(1:N,num_states+7)*scales(7));legend('B');
    subplot(6,2,8);
    plot(t,X(1:N,num_states+8)*scales(8));legend('C'); 
    subplot(6,2,9);
    plot(t,X(1:N,num_states+9)*scales(9));legend('D');
    subplot(6,2,10);
    plot(t,X(1:N,num_states+10)*scales(10));legend('kSteering'); 
    subplot(6,2,11);
    plot(t,X(1:N,num_states+11)*scales(11));legend('kThrottle');
    subplot(6,2,12);
    plot(t,X(1:N,num_states+12)*scales(12));legend('kTorque')
     %%  
    t=dt*N:dt:(dt*(lth-1));
    figure;
    subplot(4,2,1);
    plot(t,Y(N:lth-1,1),'b.',t,Y_opt(N:lth-1,1),'r');
    legend('True vx','Simulated vx');
    subplot(4,2,2);
    plot(t,Y(N:lth-1,2),'b.',t,Y_opt(N:lth-1,2),'r');
    legend('True vy','Simulated vy');
    subplot(4,2,3);
    plot(t,Y(N:lth-1,3),'b.',t,Y_opt(N:lth-1,3),'r');
    legend('True wz','Simulated wz');
    subplot(4,2,4);
    plot(t,Y(N:lth-1,4),'b.',t,Y_opt(N:lth-1,4),'r');
    legend('True wF','Simulated wF');
    subplot(4,2,5);
    plot(t,Y(N:lth-1,5),'b.',t,Y_opt(N:lth-1,5),'r');
    legend('True wR','Simulated wR');
    subplot(4,2,6);
    plot(t,Y(N:lth-1,6),'b.',t,Y_opt(N:lth-1,6),'r');
    legend('True Yaw','Simulated Yaw');
    subplot(4,2,7);
    plot(t,Y(N:lth-1,7),'b.',t,Y_opt(N:lth-1,7),'r');
    legend('True X','Simulated X');
    subplot(4,2,8);
    plot(t,Y(N:lth-1,8),'b.',t,Y_opt(N:lth-1,8),'r');
    legend('True Y','Simulated Y');
%     subplot(5,2,9);
%     plot(t,Y(N:lth-1,9),'b.',t,Y_opt(N:lth-1,9),'r');
%     legend('True X','Simulated X');
%     subplot(5,2,10);
%     plot(t,Y(N:lth-1,10),'b.',t,Y_opt(N:lth-1,10),'r');
%     legend('True Y','Simulated Y');
%     figure; plot(t,Y_opt(N:lth-1,4));
    
%     figure;
%     plot(Y(N:lth-1,9),Y(N:lth-1,10),'b', Y_opt(N:lth-1,9),Y_opt(N:lth-1,10),'r');
%     legend('True trajectory','Simulated trajectory'); 
%     
%     figure;
%     plot(t,Y(N:lth-1,2),'b',t,Y_opt(N:lth-1,6),'r');
%     legend('True yaw angle','Simulated yaw angle');
% 
%     filename='XXVII_Adaptive_Limit_Memory_opt_Joint_UKF_11Dof.mat';
%     save(filename, 'para_opt', 'J', 'X_train', 'Y_opt', 'Y_train', 'Q_buf', 'R_buf', 'bar_q_buf', 'bar_r_buf');     
  
    
    

end

