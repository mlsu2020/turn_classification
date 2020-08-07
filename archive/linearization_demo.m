t_interval = [0,1.2];
initial_state = [2 2]';

% analytical
[t,x] = ode45(@odefun, t_interval, initial_state);

plot(t,x)
hold on
% descretization
y = zeros(2,1200);
t = linspace(0,1.2,1201);
y(:,1) = initial_state;
for ii = [1:1200]
    y(:,ii+1) = discrete_update(0.001, y(:,ii));
end
y = transpose(y);
%plot(t,y, '--')

next_state(initial_state)

% our linearization
delta = 0.000001;%0.000001;
tic
[A, D] = linearize(delta, initial_state)
time2 = toc

% rollout states
[ts, xs] = rollout_states(A,D,5,initial_state)
plot(ts, xs', 'o')

%LTV
[ts, xs] = LTV(12, initial_state)
%legend('x1 analytical', 'x2 analytical', 'x1 LTI', 'x2 LTI', 'x1 LTV 1', 'x2 LTV 1', 'x1 LTV 2', 'x2 LTV 2', 'x1 LTV 3', 'x2 LTV 3', 'x1 LTV 4', 'x2 LTV 4')

function [ts, xs] = LTV(N, x0)
    ts = [0.1:0.1:0.1*N];
    dx = 0.000001;
    %xs = zeros(2,N);
    %LTI first
    [A,D] = linearize(dx,x0);
    [~,xs] = rollout_states(A, D, 12, x0)
    %LTV
    As = zeros(2,2,N);
    Ds = zeros(2,N);
    for k = 1:N
        [As(:,:,k), Ds(:,k)] = linearize(dx, xs(:,k));
    end
    [~,xs] = rollout_states2(As, Ds, 12, xs(:,1));
    plot(ts(2:6), xs(:,1:5)', '.')
    for k = 1:N
        [As(:,:,k), Ds(:,k)] = linearize(dx, xs(:,k));
    end
    [~,xs] = rollout_states2(As, Ds, 12, xs(:,1));
    plot(ts(3:7), xs(:,1:5)', '^')
    for k = 1:N
        [As(:,:,k), Ds(:,k)] = linearize(dx, xs(:,k));
    end
    [~,xs] = rollout_states2(As, Ds, 12, xs(:,1));
    plot(ts(4:9), xs(:,1:6)', '*')
    for k = 1:N
        [As(:,:,k), Ds(:,k)] = linearize(dx, xs(:,k));
    end
    [~,xs] = rollout_states2(As, Ds, 12, xs(:,1));
    plot(ts(5:10), xs(:,1:6)', 'x')
    
    
    
end

function [ts, xs] = rollout_states2(As, Ds, N, x0)
    ts = [0.1:0.1:0.1*N];
    xs = zeros(2,N);
    x1 = As(:,:,1) * x0 + Ds(:,1);
    xs(:,1) = x1;
    for k = 2:N
        x1 = As(:,:,k)*x1 + Ds(:,k);
        xs(:,k) = x1;
    end
end

function [ts, xs] = rollout_states(A, D, N, x0)
    ts = [0.1:0.1:0.1*N];
    xs = zeros(2,N);
    x1 = A * x0 + D;
    xs(:,1) = x1;
    for k = 2:N
        x1 = A*x1 + D;
        xs(:,k) = x1;
    end
end

function [A, D] = linearize(delta, init_x)
    x0 = init_x;
    f_minus = zeros(2,2);
    x0(1) = x0(1) - delta;
    f_minus(:,1) = next_state(x0);
    x0 = init_x;
    x0(2) = x0(2) - delta;
    f_minus(:,2) = next_state(x0);
    x0 = init_x;
    x0(1) = x0(1) + delta;
    f_plus(:,1) = next_state(x0);
    x0 = init_x;
    x0(2) = x0(2) + delta;
    f_plus(:,2) = next_state(x0);
    A = (f_plus - f_minus)/(2*delta);
    D = next_state(init_x) - A * init_x;
end

function x1 = next_state(x0)
    y = zeros(2,100);
    y(:,1) = x0;
    for ii = [1:100]
        y(:,ii+1) = discrete_update(0.001, y(:,ii));
    end
    x1 = y(:,end);
end

function x1 = discrete_update(dt, x0)
    x1 = zeros(2,1);
    x1(1) = x0(1) + dt*(-x0(1)+x0(2)*(x0(1))^2);
    x1(2) = x0(2) + dt*(x0(2)-x0(1)*x0(2));
end

function dydt = odefun(t,y)
    dydt = zeros(2,1);
    dydt(1) = -y(1)+y(2)*(y(1))^2;
    dydt(2) = y(2)-y(1)*y(2);
end
