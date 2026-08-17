%% sim_loop.m  —  Faithful discrete simulation of the ROS node itself
%  Replicates straight_line_assistant_node.py exactly:
%  20 Hz timer, parallel PID, |angular.z| <= 2.84 saturation,
%  integral clamp (integral_limit), command->yaw plant with delay+lag.
%  No toolboxes required.

K = 1.0; tau = 0.17; L = 0.10;      % plant (from fit_plant.m)
Ts = 0.05;                          % control_rate = 20 Hz
MAXW = 2.84;                        % BURGER_MAX_ANG_VEL

% ---- candidate gains (edit these) ----
gains = struct( ...
  'current',  struct('kp',1.2,'ki',0.01,'kd',0.4,'ilim',1.0), ...
  'proposed', struct('kp',1.4,'ki',0.30,'kd',0.1,'ilim',1.0) );

Tsim = 12;                          % seconds
E0   = 0.2;                         % initial heading error at engage [rad]
DIST = 0.05;                        % constant wheel-imbalance [rad/s]

names = fieldnames(gains);
figure; hold on; grid on;
for g = 1:numel(names)
    p = gains.(names{g});
    N  = round(Tsim/Ts);
    nL = max(1, round(L/Ts));
    buf = zeros(1,nL);  xw = 0;  yaw = 0;  integ = 0;  prev_e = 0;
    err_log = zeros(1,N);
    for k = 1:N
        % target is 0; robot "engages" hold with an initial error E0
        e = (0 + (k==1)*0) - yaw + (k==1)*E0;
        e = atan2(sin(e), cos(e));                 % normalize_angle
        integ = min(max(integ + e*Ts, -p.ilim), p.ilim);
        der   = (e - prev_e)/Ts;  prev_e = e;
        u = min(max(p.kp*e + p.ki*integ + p.kd*der, -MAXW), MAXW);
        % plant: tau*w' + w = K*u(t-L); yaw' = w + disturbance
        buf = [buf(2:end) u];
        xw  = xw + (Ts/tau)*(-xw + K*buf(1));
        yaw = yaw + (xw + DIST)*Ts;
        err_log(k) = e;
    end
    plot((1:N)*Ts, err_log, 'DisplayName', names{g});
end
xlabel('t [s]'); ylabel('yaw error [rad]');
title(sprintf('heading-hold: %.2f rad step + %.2f rad/s disturbance', E0, DIST));
legend show;

%% Simulink equivalent (5 minutes to build):
%   Constant(0) --> Sum(+- ) --> PID Controller (parallel, P I D,
%        |                            limit output ON [-2.84 2.84],
%        |                            anti-windup = clamping, N = 100)
%        |                            |
%        |                     Transport Delay (0.10 s)
%        |                            |
%        |                     Transfer Fcn  1.0/(0.17 s + 1)
%        |                            |
%        |                     (add Constant DIST at this node via Sum)
%        |                            |
%        +---------- Integrator  1/s <-+
%  Solver: fixed-step 0.05 (or discrete PID at Ts=0.05) to mimic the node.
