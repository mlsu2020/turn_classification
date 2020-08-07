function [ para_opt, Y, Xa, Y_opt ] = Single_Track_Cartesian_AJUKF (states, inputs, dt)

starti=1;
window_seconds=70;
window=window_seconds/dt;
vxj=states(1, starti:window+starti )';
vyj=states(2, starti:window+starti )';
wzj=states(3, starti:window+starti )';
wFj=states(4, starti:window+starti )';
wRj=states(5, starti:window+starti )';
Yawj=states(6, starti:window+starti )';
Xj=states(7, starti:window+starti )';
Yj=states(8, starti:window+starti )';
deltaj=inputs(1, starti:window+starti )';
% Tj=inputs(2, starti:window+starti )';

Nr=10/dt;
Nq=10/dt;
N0=max(Nr,Nq);

lth=length(vxj);
N=50/dt;

Qmax=1e4;
Qmin=1e-14;
Rmax=1e4;
Rmin=1e-14;
Pmax=1e4;
Pmin=1e-14;
Pyy_max=1e4;

%%
%          m   Iz   lF    lR    h   B   C   D   E    St
% para0 = [  22  1.2 0.35  0.040 0.20 1.5 1.5 1.5 1.0];
% para_lb=[  10  0.8 0.20  0.020 0.05 0.5 0.5 0.1 0.5];
% para_ub=[  28  2.0 0.40  0.060 0.50 12. 2.3 2.0 1.5];
para0 = [  21.75 1.12 0.34 0.23 0.12 4 1.0 1.0 1.0];
para_lb=[  15  0.9 0.3  0.2 0.1 0.5 0.5 0.1 0.5];
para_ub=[  30  1.3 0.4  0.3 0.2 12. 2.5 2.0 1.5];

% rescale the parameters
scales=para_ub/2;
para0=para0./scales;
para_lb=para_lb./scales;
para_ub=para_ub./scales;

num_states = 7;%12;
num_params = 9;%17;
num_meas = 7;%8;
num_states_and_params = num_states + num_params;%29;
num_total = 2*num_states_and_params + num_meas;%66;

w10=1e-5; w20=1e-6; v0=1e-3; P10=1e-4; P20=1e-4;   w21=1e-5;

%% Setup UT parameters
   
    alf=1e-2;
    bet=2;
    L=num_total;
    kap=0;%3-L;

    lam=alf^2*(L+kap)-L;
    Wm0=lam/(L+lam);
    Wc0=lam/(L+lam)+(1-alf^2+bet);
    Wmi=1/(L+lam)/2;
    Wci=Wmi;
    sigma_mean_weights = [Wm0 ;(ones(2*L,1)*Wmi)];
    sigma_cov_weights = [Wc0 ;(ones(2*L,1)*Wci)];

    Y=[ vxj vyj wzj wFj Yawj Xj Yj];
   
    state0 = Y(1,:);
    x0=[state0'; para0'];
    q0=zeros(num_states_and_params,1);
    r0=zeros(num_meas,1);

    init_state_cov = diag((P10*ones(1,num_states)).^2);
    init_param_cov = diag((P20*ones(1,num_params)).^2);
    P0=blkdiag(init_state_cov, init_param_cov);

    Qw1=diag((w10*ones(1,num_states)).^2); % state noise
    Qw2=diag(w20^2*ones(1,num_params)); % param noise
    Q0=blkdiag(Qw1, Qw2);
    R0=diag(v0^2*ones(1,num_meas));
   
    Xa=zeros(lth,num_total);
    Xa(1,:)= [x0; q0; r0]';  
    augmented_covariance = blkdiag(P0, Q0, R0);

    Y_opt = zeros(lth,num_meas);
    observation_noise_samples = zeros(num_meas, lth);
    gammas = zeros(num_meas, num_meas, lth);
    observation_noise_means = zeros(num_meas, lth);
    observation_noise_covariances = zeros(num_meas, num_meas, lth);
    process_noise_samples = zeros(num_states_and_params, lth);
    pies = zeros(num_states_and_params, num_states_and_params, lth);
    process_noise_means = zeros(num_states_and_params, lth);
    process_noise_covariances = zeros(num_states_and_params,num_states_and_params,lth);
    
%%  state estimate   
for k=1:N-1

    %% calc sigma predict
    root_Pa=(L+lam)^0.5 * real(sqrtm(augmented_covariance));
    
    if ~isreal(root_Pa)
       fprintf('Pa not real \n'); 
    end
    
    sigma_augmented_prev=[Xa(k,:)', (repmat(Xa(k,:),L,1)+root_Pa)', (repmat(Xa(k,:),L,1)-root_Pa)'];
    
    sigma_state_params_prev=sigma_augmented_prev(1:num_states_and_params,:);
    sigma_process_noise_prev=sigma_augmented_prev(num_states_and_params+1:2*num_states_and_params,:);
    sigma_observation_noise_prev=sigma_augmented_prev(2*num_states_and_params+1:num_total,:);
    sigma_params_prev = sigma_state_params_prev(num_states+1:num_states_and_params,:).*scales';
    sigma_states_prev = sigma_state_params_prev(1:num_states,:);
    control = [deltaj(k), wRj(k), wFj(k)];
    next_state = predict_next_state(sigma_states_prev, control, sigma_params_prev, dt);
    fundisturbed_sigma_states_params_new = [next_state; sigma_state_params_prev(num_states+1:num_states_and_params,:)];
    sigma_states_params_new = fundisturbed_sigma_states_params_new + sigma_process_noise_prev*sqrt(dt);

    mean_states_params_new = (sigma_states_params_new*sigma_mean_weights);

    covariance_states_params_new = (sigma_cov_weights' .* (sigma_states_params_new - mean_states_params_new)) * (sigma_states_params_new - mean_states_params_new)';
    
    covariance_states_params_new = ensure_wellformed(covariance_states_params_new, Pmax, Pmin);
    covariance_states_params_new = ensureSPD(covariance_states_params_new, Pmax);

    hundisturbed_measurements_sigma = sigma_states_params_new(1:num_states,:);
    measurements_sigma = hundisturbed_measurements_sigma + sigma_observation_noise_prev;
    hundisturbed_measurements_mean = hundisturbed_measurements_sigma*sigma_mean_weights;
    measurements_mean = (measurements_sigma*sigma_mean_weights);
    % alternately the old code just took the mean states
    
    %% estimate observation noise
    observation_noise_sample = (Y(k+1,:)' - hundisturbed_measurements_mean);
    % these indexes are how it was in the code, but should they be prev?
    observation_noise_samples(:,k+1) = observation_noise_sample;
    gamma = (sigma_mean_weights' .* hundisturbed_measurements_sigma) * (hundisturbed_measurements_sigma)' - (hundisturbed_measurements_mean * hundisturbed_measurements_mean');
    gammas(:,:,k+1) = gamma;
    
    if k < N0
        observation_noise_mean = r0;
        observation_noise_covariance = R0;
    else
        observation_noise_mean = observation_noise_means(:,k)+(observation_noise_samples(:,k+1)-observation_noise_samples(:,k+1-Nr))/Nr;
        obs_cov = (observation_noise_sample-observation_noise_mean) * (observation_noise_sample-observation_noise_mean)';
        old_obs_cov = (observation_noise_samples(:,k+1-Nr)-observation_noise_mean)*(observation_noise_samples(:,k+1-Nr)-observation_noise_mean)';
        mix_obs_cov = (observation_noise_sample-observation_noise_samples(:,k+1-Nr))*(observation_noise_sample-observation_noise_samples(:,k+1-Nr))'/Nr;
        observation_noise_covariance = squeeze(observation_noise_covariances(:,:,k))+(obs_cov-old_obs_cov+mix_obs_cov)/(Nr-1)+(squeeze(gammas(:,:,k+1-Nr))-gamma)/Nr;
    end
    observation_noise_means(:,k+1) = observation_noise_mean;
    observation_noise_covariance = ensure_wellformed(observation_noise_covariance, Rmax, Rmin);
    observation_noise_covariance = ensureSPD(observation_noise_covariance, Rmax);
    observation_noise_covariances(:,:,k+1) = observation_noise_covariance;
    
    %% Update Measurement

    measurements_covariance = (sigma_cov_weights' .* (measurements_sigma - measurements_mean)) * (measurements_sigma - measurements_mean)';
    % Wci weights were doubled here why?

    measurements_covariance = ensure_wellformed(measurements_covariance, Pmax, Pmin); 
    measurements_covariance = ensureSPD(measurements_covariance, Pyy_max);

    covariance_states_measurements = (sigma_cov_weights' .* (sigma_states_params_new - mean_states_params_new)) * (measurements_sigma - measurements_mean)'; 
    
    kalman_gain = covariance_states_measurements / measurements_covariance;
    
    adjusted_states_params = mean_states_params_new + kalman_gain * (Y(k+1,:)'-measurements_mean);        
    adjusted_covariance = covariance_states_params_new - kalman_gain * measurements_covariance * kalman_gain'; 

    adjusted_covariance = ensure_wellformed(adjusted_covariance, Pmax, Pmin);
    adjusted_covariance = ensureSPD(adjusted_covariance, Pmax);
    
    params = adjusted_states_params(num_states+1:num_states_and_params,:);
%     params(params > para_ub') = para_ub(params > para_ub');
%     params(params < para_lb') = para_lb(params < para_lb');
%     adjusted_states_params(num_states+1:num_states_and_params,:) = params;
    if (any(params > para_ub') || any(params < para_lb') || any(isnan(params)))
        bounds = [para_ub'; -para_lb'];
        adjusted_states_params = satisfy_constraints(adjusted_states_params, adjusted_covariance, bounds);
    end
    
%%  Estimate process noise

    fundisturbed_mean_states_params = fundisturbed_sigma_states_params_new*sigma_mean_weights;
    process_noise_sample = adjusted_states_params - fundisturbed_mean_states_params;
    %this was using the disturbed state params mean, x_hat instead of f_hat
    process_noise_samples(:,k+1) = process_noise_sample;
    pie = (sigma_mean_weights' .* fundisturbed_sigma_states_params_new) * fundisturbed_sigma_states_params_new' - fundisturbed_mean_states_params*fundisturbed_mean_states_params' - adjusted_covariance;
    pies(:,:,k+1) = pie;
    
    if k < N0
        process_noise_mean = q0;
        process_noise_covariance = Q0;
    else
        process_noise_mean=process_noise_means(:,k)+(process_noise_sample-process_noise_samples(:,k+1-Nq))/Nq;
        noise_cov = (process_noise_sample-process_noise_mean)*(process_noise_sample-process_noise_mean)';
        old_noise_cov = (process_noise_samples(:,k+1-Nq)-process_noise_mean)*(process_noise_samples(:,k+1-Nq)-process_noise_mean)';
        mix_noise_cov = (process_noise_sample-process_noise_samples(:,k+1-Nq))*(process_noise_sample-process_noise_samples(:,k+1-Nq))'/Nq;
        process_noise_covariance=squeeze(process_noise_covariances(:,:,k))+(noise_cov-old_noise_cov+mix_noise_cov)/(Nq-1)+(squeeze(pies(:,:,k+1-Nq))-pie)/Nq;
    
        % fix num instability in Q
        % !!Check to make sure this is setting the diag like expected
        diagQ=diag(process_noise_covariance);
        factor=w21./(diagQ(num_states+1:end)).^0.5;
        factor_matrix = diag(ones(num_params,1).*(factor-1)) + ones(num_params);
        process_noise_covariance(num_states+1:end,num_states+1:end) = process_noise_covariance(num_states+1:end,num_states+1:end) .* factor_matrix;
%         process_noise_covariance(num_states+1:end,:)=process_noise_covariance(num_states+1:end,:).*kron(ones(1,num_states_and_params),factor);
%         process_noise_covariance(:,num_states+1:end)=process_noise_covariance(:,num_states+1:end).*kron(ones(num_states_and_params,1),factor');
        % mean value of the parameter noise was set to 0
%         process_noise_mean(num_states+1:end,:) = zeros(num_params,1);
    end
    process_noise_means(:,k+1) = process_noise_mean;
    process_noise_covariance = ensure_wellformed(process_noise_covariance, Qmax, Qmin);
    process_noise_covariance = ensureSPD(process_noise_covariance, Qmax);
    process_noise_covariances(:,:,k+1) = process_noise_covariance;  

    %% update augmented
    Xa(k+1,:)= [adjusted_states_params; process_noise_mean; observation_noise_mean];  
    augmented_covariance = blkdiag(adjusted_covariance, process_noise_covariance, observation_noise_covariance);
    if rem(k,100) == 0
        fprintf('%d\n',k);
    end

end
    
 %%  results 

    para_opt = adjusted_states_params(num_states+1:num_states_and_params, 1).*scales';
    state = Y(N,:)';

    for k=N:lth-1
        control = [deltaj(k) wRj(k) wFj(k)];
        state = predict_next_state(state, control, para_opt, dt);
        Y_opt(k,:) = state';
    end

%% plot & save results


    t=0:dt:(dt*(N-1));
    figure;
    subplot(4,2,1);
    plot(t,Y(1:N,1),'b',t,Xa(1:N,1),'r');
    legend('True vx','Estimated vx');     
    subplot(4,2,2);
    plot(t,Y(1:N,2),'b',t,Xa(1:N,2),'r');
    legend('True vy','Estimated vy');  
    subplot(4,2,3);
    plot(t,Y(1:N,3),'b',t,Xa(1:N,3),'r');
    legend('True wz','Estimated wz');  
    subplot(4,2,4);
    plot(t,Y(1:N,4),'b',t,Xa(1:N,4),'r');
    legend('True wF','Estimated wF');
    subplot(4,2,5);
%     plot(t,Y(1:N,5),'b',t,Xa(1:N,5),'r');
%     legend('True wR','Estimated wR');
%     subplot(4,2,6);
%     plot(t,Y(1:N,6),'b',t,Xa(1:N,6),'r');
%     legend('True Yaw','Estimated Yaw');
%     subplot(4,2,7);
%     plot(t,Y(1:N,7),'b',t,Xa(1:N,7),'r');
%     legend('True X','Estimated X');
%     subplot(4,2,8);
%     plot(t,Y(1:N,8),'b',t,Xa(1:N,8),'r');
%     legend('True Y','Estimated Y');
  
    figure;
    subplot(6,2,1);
    plot(t,Xa(1:N,num_states+1)*scales(1));legend('m');
    subplot(6,2,2);
    plot(t,Xa(1:N,num_states+2)*scales(2));legend('Iz');    
    subplot(6,2,3);
    plot(t,Xa(1:N,num_states+3)*scales(3));legend('lF');    
    subplot(6,2,4);
    plot(t,Xa(1:N,num_states+4)*scales(4));legend('Iw');
    subplot(6,2,5);
    plot(t,Xa(1:N,num_states+5)*scales(5));legend('h');
    subplot(6,2,6);
    plot(t,Xa(1:N,num_states+6)*scales(6));legend('B'); 
    subplot(6,2,7);
    plot(t,Xa(1:N,num_states+7)*scales(7));legend('C');
    subplot(6,2,8);
    plot(t,Xa(1:N,num_states+8)*scales(8));legend('D'); 
    subplot(6,2,9);
    plot(t,Xa(1:N,num_states+9)*scales(9));legend('E');
    subplot(6,2,10);
%     plot(t,Xa(1:N,num_states+10)*scales(10));legend('kSteering'); 
%     subplot(6,2,11);
%     plot(t,Xa(1:N,num_states+11)*scales(11));legend('cSteering');
%     subplot(6,2,12);
%     plot(t,Xa(1:N,num_states+11)*scales(11));legend('muR')
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
%     subplot(4,2,8);
%     plot(t,Y(N:lth-1,8),'b.',t,Y_opt(N:lth-1,8),'r');
%     legend('True Y','Simulated Y');
    

end

function next_state = predict_next_state(states, control, params, dt)
%     min_velo = 0.1;
    lFR = 0.57;
    m_Vehicle_m = params(1, :);
    m_Vehicle_Iz = params(2, :);
    m_Vehicle_lF = params(3, :);
    m_Vehicle_lR = params(4, :);
    lFR = m_Vehicle_lF + m_Vehicle_lR;
%     m_Vehicle_IwF = params(4, :);
%     m_Vehicle_IwR = params(5, :);
    m_Vehicle_rF = 0.095;
    m_Vehicle_rR = 0.095;
%     m_Vehicle_mu1 = params(10, :);
%     m_Vehicle_mu2 = m_Vehicle_mu1;%params(11, :);
    m_Vehicle_h = params(5, :);    
    m_g = 9.80665;
    
%     m_Vehicle_kSteering = params(10, :);
%     m_Vehicle_kThrottle = kThrottlej;
%     m_Vehicle_kTorque = kTorquej;
%     m_Vehicle_cSteering = params(11, :);

    tire_B = params(6, :);
    tire_C = params(7, :);
    tire_D = params(8, :);
    tire_E = params(9, :);
    tire_Sh = 0;%params(13, :);
    tire_Sv = 0;%params(14, :);
    
    vxk = states(1, :);
    vyk = states(2, :);
    wzk = states(3, :);
    wFk = control(3);
    wRk = control(2);
    Yawk = states(5, :);
    Xk = states(6, :);
    Yk = states(7, :);

    deltak = 18.7861 .* control(1)+0.0109;
%     Tk = m_Vehicle_kThrottle * Tj(k);
    
%     if (vxk < min_velo)
%       vxk = min_velo;
%     end
%     if (wFk < min_velo / m_Vehicle_rF)
%         wFk = min_velo / m_Vehicle_rF + 1e-4;
%     end
%     if wRk < min_velo / m_Vehicle_rR
%         wRk = min_velo / m_Vehicle_rR + 1e-4;
%     end
%     if (vxk ~= 0.0)
      beta = atan2(vyk, vxk);
%     else
%       beta = 0.0;
%     end
    V = sqrt(vxk .* vxk + vyk .* vyk);
    vFx = V .* cos(beta - deltak) + wzk .* m_Vehicle_lF .* sin(deltak);
    vFy = V .* sin(beta - deltak) + wzk .* m_Vehicle_lF .* cos(deltak);
    vRx = vxk;
    vRy = vyk - wzk .* m_Vehicle_lR;
    
    sFx = (vFx - wFk .* m_Vehicle_rF) ./ (wFk .* m_Vehicle_rF);
    sRx = (vRx - wRk .* m_Vehicle_rR) ./ (wRk .* m_Vehicle_rR);
    sFy = vFy ./ (wFk .* m_Vehicle_rF);
    sRy = vRy ./ (wRk .* m_Vehicle_rR);

    sF = sqrt(sFx .* sFx + sFy .* sFy);
    sR = sqrt(sRx .* sRx + sRy .* sRy);

    sEF = sF - tire_Sh;
    sER = sR - tire_Sh;

    muF = tire_D.*sin( tire_C.*atan( tire_B.*sEF - tire_E.*(tire_B.*sEF - atan(tire_B.*sEF) ) ) ) + tire_Sv;
    muR = tire_D.*sin( tire_C.*atan( tire_B.*sER - tire_E.*(tire_B.*sER - atan(tire_B.*sER) ) ) ) + tire_Sv;
    muF = tire_D .* sin(tire_C .* atan(tire_B .* sF)); 
    muR = tire_D .* sin(tire_C .* atan(tire_B .* sR));

    muFx = -sFx ./ sF .* muF;
    muFy = -sFy ./ sF .* muF;
    muRx = -sRx ./ sR .* muR;
    muRy = -sRy ./ sR .* muR;

    fFz = m_Vehicle_m .* m_g .* (m_Vehicle_lR - m_Vehicle_h .* muRx) ./ (m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h .* (muFx .* cos(deltak) - muFy .* sin(deltak) - muRx));
    % fFz = m_Vehicle_m * m_g * (m_Vehicle_lR / lFR);
    fRz = m_Vehicle_m .* m_g - fFz;

    fFx = muFx .* fFz;
    fFy = muFy .* fFz;
    fRx = muRx .* fRz;
    fRy = muRy .* fRz;
    
    %%%%%%%%%%%%%%%%%%%
%     sEF = -(vFx - wFk .* m_Vehicle_rF) ./ (vFx) + tire_Sh;
%     muFx = tire_D.*sin( tire_C.*atan( tire_B.*sEF - tire_E.*(tire_B.*sEF - atan(tire_B.*sEF) ) ) ) + tire_Sv;
%     sEF = -(vRx - wRk .* m_Vehicle_rR) ./ (vRx) + tire_Sh;
%     muRx = tire_D.*sin( tire_C.*atan( tire_B.*sEF - tire_E.*(tire_B.*sEF - atan(tire_B.*sEF) ) ) ) + tire_Sv;
%     
%     sEF = atan(vFy ./ abs(vFx)) + tire_Sh;
%     muFy = -tire_D.*sin( tire_C.*atan( tire_B.*sEF - tire_E.*(tire_B.*sEF - atan(tire_B.*sEF) ) ) ) + tire_Sv;
%     sEF = atan(vRy ./ abs(vRx)) + tire_Sh;
%     muRy = -tire_D.*sin( tire_C.*atan( tire_B.*sEF - tire_E.*(tire_B.*sEF - atan(tire_B.*sEF) ) ) ) + tire_Sv;
%     
%     fFz = m_Vehicle_m .* m_g .* (m_Vehicle_lR - m_Vehicle_h .* muRx) / (m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h .* (muFx .* cos(deltak) - muFy .* sin(deltak) - muRx));
%     fRz = m_Vehicle_m .* m_g - fFz;
% 
%     fFx = fFz .* muFx;
%     fRx = fRz .* muRx;
%     fFy = fFz .* muFy;
%     fRy = fRz .* muRy;
    %%%%%%%%%%%%%%%%%%%

    dot_vx = ((fFx .* cos(deltak) - fFy .* sin(deltak) + fRx) ./ m_Vehicle_m + vyk .* wzk);
    dot_vy = ((fFx .* sin(deltak) + fFy .* cos(deltak) + fRy) ./ m_Vehicle_m - vxk .* wzk);
    dot_wz = ((fFy .* cos(deltak) + fFx .* sin(deltak)) .* m_Vehicle_lF - fRy .* m_Vehicle_lR) ./ m_Vehicle_Iz;
%     dot_wF = -1* m_Vehicle_rF ./ m_Vehicle_IwF .* fFx;
%     dot_wR = (m_Vehicle_kTorque * (Tk-wRk) - m_Vehicle_rR * fRx) / m_Vehicle_IwR;
    dot_Yaw = wzk;
    dot_X = (vxk .* cos(Yawk) - vyk .* sin(Yawk));
    dot_Y = (vxk .* sin(Yawk) + vyk .* cos(Yawk));
    
    next_state = zeros(size(states));
    next_state(1, :)=vxk+dot_vx*dt;
    next_state(2, :)=vyk+dot_vy*dt;
    next_state(3, :)=wzk+dot_wz*dt;
    next_state(4, :)=wFk;%+dot_wF*dt; 
    next_state(5, :)=wRk;
    next_state(5, :)=Yawk+dot_Yaw*dt;
    next_state(6, :)=Xk+dot_X*dt;
    next_state(7, :)=Yk+dot_Y*dt;
end

function A_hat = ensureSPD(A, Amax)
    [U,S,V] = svd(A); 
    Aux=diag(S);  %get diagonal elements
    if max(abs(Aux))>Amax %||min(Aux)<Rmin
       Aux(Aux>Amax)=Amax;
       A= U*diag(Aux)*V';
    end
    A_hat=nearestSPD(A);
end

function A_hat = ensure_wellformed(A, Amax, Amin)
    A_hat=0.5*(A+A');
    A_hat(isnan(A_hat))=Amin;
    A_hat(isinf(A_hat))=Amax;
end

function constrained = satisfy_constraints(x_p, cov, bounds)
    num_params = length(bounds)/2;
    num_states = length(x_p) - num_params;
    con_factor=[ zeros(num_params,num_states) eye(num_params); zeros(num_params,num_states) -eye(num_params)]; % let -Vx<0 -> Vx>0
    if ~isreal(con_factor)
       fprintf('constraints factor not real \n'); 
    end
    eps = 1E-12;
    covariance_eps = cov + eps;
    inverse_covariance_eps = eye(length(x_p))/(covariance_eps);            
    if isreal(inverse_covariance_eps)
        % min 0.5*x'*H*x + f'*x   subj to:  A*x <= b
        H = inverse_covariance_eps+inverse_covariance_eps';
        f = -(inverse_covariance_eps+inverse_covariance_eps')*x_p;
        A = con_factor;
        b = bounds;
        opts = optimoptions('quadprog','Algorithm','interior-point-convex ','Display','final');
        constrained = quadprog(H, f, A, b, [],[],[],[],[],opts);
    else
        fprintf('P_aux is not real here! k= %d. \n', k)
        constrained = x_p;
    end 
end
