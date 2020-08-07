% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x_0 - target, Q) + quad_form(u_0, R) + quad_form(x_1 - target, Q) + quad_form(u_1, R) + quad_form(x_2 - target, Q) + quad_form(u_2, R) + quad_form(x_3 - target, Q) + quad_form(u_3, R) + quad_form(x_4 - target, Q) + quad_form(u_4, R) + quad_form(x_5 - target, Q) + quad_form(u_5, R) + quad_form(x_6 - target, Q) + quad_form(u_6, R) + quad_form(x_7 - target, Q) + quad_form(u_7, R) + quad_form(x_8 - target, Q) + quad_form(u_8, R) + quad_form(x_9 - target, Q) + quad_form(u_9, R) + quad_form(x_10 - target, Q) + quad_form(u_10, R) + quad_form(x_11 - target, Q) + quad_form(u_11, R) + quad_form(x_12 - target, Q) + quad_form(u_12, R) + quad_form(x_13 - target, QT))
%   subject to
%     x_1 == A_0*x_0 + B_0*u_0 + d_0
%     x_2 == A_1*x_1 + B_1*u_1 + d_1
%     x_3 == A_2*x_2 + B_2*u_2 + d_2
%     x_4 == A_3*x_3 + B_3*u_3 + d_3
%     x_5 == A_4*x_4 + B_4*u_4 + d_4
%     x_6 == A_5*x_5 + B_5*u_5 + d_5
%     x_7 == A_6*x_6 + B_6*u_6 + d_6
%     x_8 == A_7*x_7 + B_7*u_7 + d_7
%     x_9 == A_8*x_8 + B_8*u_8 + d_8
%     x_10 == A_9*x_9 + B_9*u_9 + d_9
%     x_11 == A_10*x_10 + B_10*u_10 + d_10
%     x_12 == A_11*x_11 + B_11*u_11 + d_11
%     x_13 == A_12*x_12 + B_12*u_12 + d_12
%     -half_road_width <= x_1(7)
%     -half_road_width <= x_2(7)
%     -half_road_width <= x_3(7)
%     -half_road_width <= x_4(7)
%     -half_road_width <= x_5(7)
%     -half_road_width <= x_6(7)
%     -half_road_width <= x_7(7)
%     -half_road_width <= x_8(7)
%     -half_road_width <= x_9(7)
%     -half_road_width <= x_10(7)
%     -half_road_width <= x_11(7)
%     -half_road_width <= x_12(7)
%     -half_road_width <= x_13(7)
%     x_1(7) <= half_road_width
%     x_2(7) <= half_road_width
%     x_3(7) <= half_road_width
%     x_4(7) <= half_road_width
%     x_5(7) <= half_road_width
%     x_6(7) <= half_road_width
%     x_7(7) <= half_road_width
%     x_8(7) <= half_road_width
%     x_9(7) <= half_road_width
%     x_10(7) <= half_road_width
%     x_11(7) <= half_road_width
%     x_12(7) <= half_road_width
%     x_13(7) <= half_road_width
%     abs(u_0) <= umax
%     abs(u_1) <= umax
%     abs(u_2) <= umax
%     abs(u_3) <= umax
%     abs(u_4) <= umax
%     abs(u_5) <= umax
%     abs(u_6) <= umax
%     abs(u_7) <= umax
%     abs(u_8) <= umax
%     abs(u_9) <= umax
%     abs(u_10) <= umax
%     abs(u_11) <= umax
%     abs(u_12) <= umax
%
% with variables
%      u_0   2 x 1
%      u_1   2 x 1
%      u_2   2 x 1
%      u_3   2 x 1
%      u_4   2 x 1
%      u_5   2 x 1
%      u_6   2 x 1
%      u_7   2 x 1
%      u_8   2 x 1
%      u_9   2 x 1
%     u_10   2 x 1
%     u_11   2 x 1
%     u_12   2 x 1
%      x_1   8 x 1
%      x_2   8 x 1
%      x_3   8 x 1
%      x_4   8 x 1
%      x_5   8 x 1
%      x_6   8 x 1
%      x_7   8 x 1
%      x_8   8 x 1
%      x_9   8 x 1
%     x_10   8 x 1
%     x_11   8 x 1
%     x_12   8 x 1
%     x_13   8 x 1
%
% and parameters
%      A_0   8 x 8
%      A_1   8 x 8
%      A_2   8 x 8
%      A_3   8 x 8
%      A_4   8 x 8
%      A_5   8 x 8
%      A_6   8 x 8
%      A_7   8 x 8
%      A_8   8 x 8
%      A_9   8 x 8
%     A_10   8 x 8
%     A_11   8 x 8
%     A_12   8 x 8
%      B_0   8 x 2
%      B_1   8 x 2
%      B_2   8 x 2
%      B_3   8 x 2
%      B_4   8 x 2
%      B_5   8 x 2
%      B_6   8 x 2
%      B_7   8 x 2
%      B_8   8 x 2
%      B_9   8 x 2
%     B_10   8 x 2
%     B_11   8 x 2
%     B_12   8 x 2
%        Q   8 x 8    PSD
%       QT   8 x 8    PSD
%        R   2 x 2    PSD
%      d_0   8 x 1
%      d_1   8 x 1
%      d_2   8 x 1
%      d_3   8 x 1
%      d_4   8 x 1
%      d_5   8 x 1
%      d_6   8 x 1
%      d_7   8 x 1
%      d_8   8 x 1
%      d_9   8 x 1
%     d_10   8 x 1
%     d_11   8 x 1
%     d_12   8 x 1
% half_road_width   1 x 1    positive
%   target   8 x 1
%     umax   2 x 1    positive
%      x_0   8 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A_0, ..., params.x_0, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2019-07-09 09:48:03 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
