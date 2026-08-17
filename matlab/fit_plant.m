%% fit_plant.m  —  Identify the heading-hold plant from a chirp ident run
%  Works on the CSV exported by pid_dashboard during an ident_mode run
%  (columns: timestamp_sec, ..., output_omega_rad_s, ..., current_yaw_rad).
%
%  Plant being identified (what the PID "sees"):
%
%       filtered_yaw(s)        K * exp(-L s)
%  P(s) = ---------------  =  -----------------
%          omega_cmd(s)        s (tau s + 1)
%
%  The 1/s is kinematics (yaw = integral of omega). K ~ 1, tau and L lump
%  together motor/driver dynamics + OpenCR odometry + EKF filtering lag.
%
%  Requires: System Identification Toolbox (procest/compare).
%  If you don't have it, skip to the bottom — the fallback values from the
%  shipped ident_data.csv are already filled in.

csv_file = '../ident_data.csv';      % adjust path if running elsewhere

d   = readtable(csv_file);
t   = d.timestamp_sec;      t   = t - t(1);
u   = d.output_omega_rad_s;                 % plant input  [rad/s]
yaw = unwrap(d.current_yaw_rad);            % plant output [rad]
Ts  = median(diff(t));
fprintf('Data: %d samples, Ts = %.3f s (%.1f Hz)\n', numel(t), Ts, 1/Ts);

%% Fit A (preferred): fit  omega_meas / omega_cmd  as K exp(-Ls)/(tau s + 1)
%  Differentiating yaw avoids integrator-windup issues in the estimator.
w_meas = gradient(yaw, t);
z_w    = iddata(w_meas, u, Ts);
G_w    = procest(z_w, 'P1D');        % Kp/(1+Tp1 s) * exp(-Td s)
fprintf('\n--- Fit A: w_meas/u = K exp(-Ls)/(tau s+1) ---\n');
present(G_w);

%% Fit B (cross-check): fit yaw/u directly as integrating process
z_y = iddata(yaw - yaw(1), u, Ts);
G_y = procest(z_y, 'P1DI');          % Kp/(s(1+Tp1 s)) * exp(-Td s)
fprintf('\n--- Fit B: yaw/u = K exp(-Ls)/(s(tau s+1)) ---\n');
present(G_y);

%% Validation — model output vs measured data
figure;
subplot(2,1,1); compare(z_w, G_w); title('Fit A validation: \omega_{meas}');
subplot(2,1,2); compare(z_y, G_y); title('Fit B validation: yaw');

%% Assemble the plant for tuning (use Fit A numbers + the known integrator)
K   = G_w.Kp;
tau = G_w.Tp1;
L   = G_w.Td + Ts/2;                 % add control-loop zero-order-hold delay

s   = tf('s');
P   = K / (s * (tau*s + 1));
P.InputDelay = L;

fprintf('\nPlant for tuning:  P(s) = %.2f exp(-%.3fs) / (s (%.3f s + 1))\n', K, L, tau);

%% Fallback (from the shipped ident_data.csv, if you have no SysID Toolbox):
%  K = 1.0;  tau = 0.17;  L = 0.10;   ->  R^2 = 0.95 vs measured yaw
%  P = 1.0/(s*(0.17*s+1));  P.InputDelay = 0.10;

save('plant.mat', 'P', 'K', 'tau', 'L', 'Ts');
