% Produced by CVXGEN, 2019-07-09 09:48:03 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: cvxsolve.m.
% Description: Solution file, via cvx, for use with sample.m.
function [vars, status] = cvxsolve(params, settings)
A_0 = params.A_0;
if isfield(params, 'A_1')
  A_1 = params.A_1;
elseif isfield(params, 'A')
  A_1 = params.A{1};
else
  error 'could not find A_1'
end
if isfield(params, 'A_2')
  A_2 = params.A_2;
elseif isfield(params, 'A')
  A_2 = params.A{2};
else
  error 'could not find A_2'
end
if isfield(params, 'A_3')
  A_3 = params.A_3;
elseif isfield(params, 'A')
  A_3 = params.A{3};
else
  error 'could not find A_3'
end
if isfield(params, 'A_4')
  A_4 = params.A_4;
elseif isfield(params, 'A')
  A_4 = params.A{4};
else
  error 'could not find A_4'
end
if isfield(params, 'A_5')
  A_5 = params.A_5;
elseif isfield(params, 'A')
  A_5 = params.A{5};
else
  error 'could not find A_5'
end
if isfield(params, 'A_6')
  A_6 = params.A_6;
elseif isfield(params, 'A')
  A_6 = params.A{6};
else
  error 'could not find A_6'
end
if isfield(params, 'A_7')
  A_7 = params.A_7;
elseif isfield(params, 'A')
  A_7 = params.A{7};
else
  error 'could not find A_7'
end
if isfield(params, 'A_8')
  A_8 = params.A_8;
elseif isfield(params, 'A')
  A_8 = params.A{8};
else
  error 'could not find A_8'
end
if isfield(params, 'A_9')
  A_9 = params.A_9;
elseif isfield(params, 'A')
  A_9 = params.A{9};
else
  error 'could not find A_9'
end
if isfield(params, 'A_10')
  A_10 = params.A_10;
elseif isfield(params, 'A')
  A_10 = params.A{10};
else
  error 'could not find A_10'
end
if isfield(params, 'A_11')
  A_11 = params.A_11;
elseif isfield(params, 'A')
  A_11 = params.A{11};
else
  error 'could not find A_11'
end
if isfield(params, 'A_12')
  A_12 = params.A_12;
elseif isfield(params, 'A')
  A_12 = params.A{12};
else
  error 'could not find A_12'
end
B_0 = params.B_0;
if isfield(params, 'B_1')
  B_1 = params.B_1;
elseif isfield(params, 'B')
  B_1 = params.B{1};
else
  error 'could not find B_1'
end
if isfield(params, 'B_2')
  B_2 = params.B_2;
elseif isfield(params, 'B')
  B_2 = params.B{2};
else
  error 'could not find B_2'
end
if isfield(params, 'B_3')
  B_3 = params.B_3;
elseif isfield(params, 'B')
  B_3 = params.B{3};
else
  error 'could not find B_3'
end
if isfield(params, 'B_4')
  B_4 = params.B_4;
elseif isfield(params, 'B')
  B_4 = params.B{4};
else
  error 'could not find B_4'
end
if isfield(params, 'B_5')
  B_5 = params.B_5;
elseif isfield(params, 'B')
  B_5 = params.B{5};
else
  error 'could not find B_5'
end
if isfield(params, 'B_6')
  B_6 = params.B_6;
elseif isfield(params, 'B')
  B_6 = params.B{6};
else
  error 'could not find B_6'
end
if isfield(params, 'B_7')
  B_7 = params.B_7;
elseif isfield(params, 'B')
  B_7 = params.B{7};
else
  error 'could not find B_7'
end
if isfield(params, 'B_8')
  B_8 = params.B_8;
elseif isfield(params, 'B')
  B_8 = params.B{8};
else
  error 'could not find B_8'
end
if isfield(params, 'B_9')
  B_9 = params.B_9;
elseif isfield(params, 'B')
  B_9 = params.B{9};
else
  error 'could not find B_9'
end
if isfield(params, 'B_10')
  B_10 = params.B_10;
elseif isfield(params, 'B')
  B_10 = params.B{10};
else
  error 'could not find B_10'
end
if isfield(params, 'B_11')
  B_11 = params.B_11;
elseif isfield(params, 'B')
  B_11 = params.B{11};
else
  error 'could not find B_11'
end
if isfield(params, 'B_12')
  B_12 = params.B_12;
elseif isfield(params, 'B')
  B_12 = params.B{12};
else
  error 'could not find B_12'
end
Q = params.Q;
QT = params.QT;
R = params.R;
d_0 = params.d_0;
if isfield(params, 'd_1')
  d_1 = params.d_1;
elseif isfield(params, 'd')
  d_1 = params.d{1};
else
  error 'could not find d_1'
end
if isfield(params, 'd_2')
  d_2 = params.d_2;
elseif isfield(params, 'd')
  d_2 = params.d{2};
else
  error 'could not find d_2'
end
if isfield(params, 'd_3')
  d_3 = params.d_3;
elseif isfield(params, 'd')
  d_3 = params.d{3};
else
  error 'could not find d_3'
end
if isfield(params, 'd_4')
  d_4 = params.d_4;
elseif isfield(params, 'd')
  d_4 = params.d{4};
else
  error 'could not find d_4'
end
if isfield(params, 'd_5')
  d_5 = params.d_5;
elseif isfield(params, 'd')
  d_5 = params.d{5};
else
  error 'could not find d_5'
end
if isfield(params, 'd_6')
  d_6 = params.d_6;
elseif isfield(params, 'd')
  d_6 = params.d{6};
else
  error 'could not find d_6'
end
if isfield(params, 'd_7')
  d_7 = params.d_7;
elseif isfield(params, 'd')
  d_7 = params.d{7};
else
  error 'could not find d_7'
end
if isfield(params, 'd_8')
  d_8 = params.d_8;
elseif isfield(params, 'd')
  d_8 = params.d{8};
else
  error 'could not find d_8'
end
if isfield(params, 'd_9')
  d_9 = params.d_9;
elseif isfield(params, 'd')
  d_9 = params.d{9};
else
  error 'could not find d_9'
end
if isfield(params, 'd_10')
  d_10 = params.d_10;
elseif isfield(params, 'd')
  d_10 = params.d{10};
else
  error 'could not find d_10'
end
if isfield(params, 'd_11')
  d_11 = params.d_11;
elseif isfield(params, 'd')
  d_11 = params.d{11};
else
  error 'could not find d_11'
end
if isfield(params, 'd_12')
  d_12 = params.d_12;
elseif isfield(params, 'd')
  d_12 = params.d{12};
else
  error 'could not find d_12'
end
half_road_width = params.half_road_width;
target = params.target;
umax = params.umax;
x_0 = params.x_0;
cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable u_0(2, 1);
  variable x_1(8, 1);
  variable u_1(2, 1);
  variable x_2(8, 1);
  variable u_2(2, 1);
  variable x_3(8, 1);
  variable u_3(2, 1);
  variable x_4(8, 1);
  variable u_4(2, 1);
  variable x_5(8, 1);
  variable u_5(2, 1);
  variable x_6(8, 1);
  variable u_6(2, 1);
  variable x_7(8, 1);
  variable u_7(2, 1);
  variable x_8(8, 1);
  variable u_8(2, 1);
  variable x_9(8, 1);
  variable u_9(2, 1);
  variable x_10(8, 1);
  variable u_10(2, 1);
  variable x_11(8, 1);
  variable u_11(2, 1);
  variable x_12(8, 1);
  variable u_12(2, 1);
  variable x_13(8, 1);

  minimize(quad_form(x_0 - target, Q) + quad_form(u_0, R) + quad_form(x_1 - target, Q) + quad_form(u_1, R) + quad_form(x_2 - target, Q) + quad_form(u_2, R) + quad_form(x_3 - target, Q) + quad_form(u_3, R) + quad_form(x_4 - target, Q) + quad_form(u_4, R) + quad_form(x_5 - target, Q) + quad_form(u_5, R) + quad_form(x_6 - target, Q) + quad_form(u_6, R) + quad_form(x_7 - target, Q) + quad_form(u_7, R) + quad_form(x_8 - target, Q) + quad_form(u_8, R) + quad_form(x_9 - target, Q) + quad_form(u_9, R) + quad_form(x_10 - target, Q) + quad_form(u_10, R) + quad_form(x_11 - target, Q) + quad_form(u_11, R) + quad_form(x_12 - target, Q) + quad_form(u_12, R) + quad_form(x_13 - target, QT));
  subject to
    x_1 == A_0*x_0 + B_0*u_0 + d_0;
    x_2 == A_1*x_1 + B_1*u_1 + d_1;
    x_3 == A_2*x_2 + B_2*u_2 + d_2;
    x_4 == A_3*x_3 + B_3*u_3 + d_3;
    x_5 == A_4*x_4 + B_4*u_4 + d_4;
    x_6 == A_5*x_5 + B_5*u_5 + d_5;
    x_7 == A_6*x_6 + B_6*u_6 + d_6;
    x_8 == A_7*x_7 + B_7*u_7 + d_7;
    x_9 == A_8*x_8 + B_8*u_8 + d_8;
    x_10 == A_9*x_9 + B_9*u_9 + d_9;
    x_11 == A_10*x_10 + B_10*u_10 + d_10;
    x_12 == A_11*x_11 + B_11*u_11 + d_11;
    x_13 == A_12*x_12 + B_12*u_12 + d_12;
    -half_road_width <= x_1(7);
    -half_road_width <= x_2(7);
    -half_road_width <= x_3(7);
    -half_road_width <= x_4(7);
    -half_road_width <= x_5(7);
    -half_road_width <= x_6(7);
    -half_road_width <= x_7(7);
    -half_road_width <= x_8(7);
    -half_road_width <= x_9(7);
    -half_road_width <= x_10(7);
    -half_road_width <= x_11(7);
    -half_road_width <= x_12(7);
    -half_road_width <= x_13(7);
    x_1(7) <= half_road_width;
    x_2(7) <= half_road_width;
    x_3(7) <= half_road_width;
    x_4(7) <= half_road_width;
    x_5(7) <= half_road_width;
    x_6(7) <= half_road_width;
    x_7(7) <= half_road_width;
    x_8(7) <= half_road_width;
    x_9(7) <= half_road_width;
    x_10(7) <= half_road_width;
    x_11(7) <= half_road_width;
    x_12(7) <= half_road_width;
    x_13(7) <= half_road_width;
    abs(u_0) <= umax;
    abs(u_1) <= umax;
    abs(u_2) <= umax;
    abs(u_3) <= umax;
    abs(u_4) <= umax;
    abs(u_5) <= umax;
    abs(u_6) <= umax;
    abs(u_7) <= umax;
    abs(u_8) <= umax;
    abs(u_9) <= umax;
    abs(u_10) <= umax;
    abs(u_11) <= umax;
    abs(u_12) <= umax;
cvx_end
vars.u_0 = u_0;
vars.u_1 = u_1;
vars.u{1} = u_1;
vars.u_2 = u_2;
vars.u{2} = u_2;
vars.u_3 = u_3;
vars.u{3} = u_3;
vars.u_4 = u_4;
vars.u{4} = u_4;
vars.u_5 = u_5;
vars.u{5} = u_5;
vars.u_6 = u_6;
vars.u{6} = u_6;
vars.u_7 = u_7;
vars.u{7} = u_7;
vars.u_8 = u_8;
vars.u{8} = u_8;
vars.u_9 = u_9;
vars.u{9} = u_9;
vars.u_10 = u_10;
vars.u{10} = u_10;
vars.u_11 = u_11;
vars.u{11} = u_11;
vars.u_12 = u_12;
vars.u{12} = u_12;
vars.x_1 = x_1;
vars.x{1} = x_1;
vars.x_2 = x_2;
vars.x{2} = x_2;
vars.x_3 = x_3;
vars.x{3} = x_3;
vars.x_4 = x_4;
vars.x{4} = x_4;
vars.x_5 = x_5;
vars.x{5} = x_5;
vars.x_6 = x_6;
vars.x{6} = x_6;
vars.x_7 = x_7;
vars.x{7} = x_7;
vars.x_8 = x_8;
vars.x{8} = x_8;
vars.x_9 = x_9;
vars.x{9} = x_9;
vars.x_10 = x_10;
vars.x{10} = x_10;
vars.x_11 = x_11;
vars.x{11} = x_11;
vars.x_12 = x_12;
vars.x{12} = x_12;
vars.x_13 = x_13;
vars.x{13} = x_13;
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
