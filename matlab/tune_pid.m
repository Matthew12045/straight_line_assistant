%% tune_pid.m  —  Design + compare PID gains for the heading-hold loop
%  Requires: Control System Toolbox. Run fit_plant.m first, or use the
%  fallback plant below.

if exist('plant.mat','file')
    load('plant.mat');           % P, K, tau, L, Ts
else
    s = tf('s');
    K = 1.0; tau = 0.17; L = 0.10; Ts = 0.05;
    P = K/(s*(tau*s+1));  P.InputDelay = L;
end
s = tf('s');

%% 1) Margin table for candidate gains (node uses PARALLEL form:
%%    u = kp*e + ki*∫e + kd*de/dt)
cands = { ...                       %  kp     ki     kd
    'current (params.yaml)'          1.2    0.01   0.40 ;
    'node defaults'                  1.5    0.01   0.20 ;
    'stage 1 (safe)'                 0.8    0.10   0.00 ;
    'stage 2 (target)'               1.4    0.30   0.10 ;
    'stage 3 (crisp)'                1.7    0.50   0.20 ;
    'ceiling (do not exceed)'        2.2    0.80   0.30 };

fprintf('\n%-26s %6s %6s %6s | %7s %8s %8s\n', 'gains', 'kp','ki','kd', 'wc r/s','PM deg','GM dB');
for i = 1:size(cands,1)
    C = pid(cands{i,2}, cands{i,3}, cands{i,4});
    [gm,pm,~,~,wc] = margin(C*P);
    fprintf('%-26s %6.2f %6.2f %6.2f | %7.2f %8.1f %8.1f\n', ...
        cands{i,1}, cands{i,2}, cands{i,3}, cands{i,4}, wc, pm, 20*log10(gm));
end
% Target: PM >= 50 deg, GM >= 8 dB. PM < 45 deg will wag visibly.

%% 2) Manual pole placement (the "formula" route)
%  Plant K/(s(tau s+1)) with PD control  =>  closed-loop char poly
%      tau s^2 + (1 + K*kd) s + K*kp  =  tau (s^2 + 2*zeta*wn*s + wn^2)
%  =>  kp = tau*wn^2 / K ,   kd = (2*zeta*wn*tau - 1) / K
%  Keep wn <= ~3.5 rad/s so the 0.10 s delay keeps PM > ~60 deg.
wn = 3.0; zeta = 1.0;
kp_pp = tau*wn^2/K;
kd_pp = max(0, (2*zeta*wn*tau - 1)/K);
ki_pp = 0.3;                          % from Ki/PM sweep (see report)
fprintf('\nPole placement (wn=%.1f, zeta=%.1f): kp=%.2f ki=%.2f kd=%.2f\n', ...
    wn, zeta, kp_pp, ki_pp, kd_pp);

%% 3) pidtune cross-check (automatic)
opts = pidtuneOptions('PhaseMargin', 60);
C_auto = pidtune(P, 'PID', 1.5, opts);   % aim crossover ~1.5 rad/s
disp('pidtune result (parallel gains):'); disp(C_auto);
% Interactive alternative:  pidTuner(P, 'PID')   and drag the bandwidth
% slider to ~1.5 rad/s with 60 deg phase margin, then read off gains.

%% 4) Compare closed-loop responses
C0 = pid(1.2, 0.01, 0.4);            % current
C1 = pid(kp_pp, ki_pp, kd_pp);       % proposed
figure;
subplot(1,2,1);
step(feedback(C0*P,1), feedback(C1*P,1), 8);
title('0.2 rad heading step'); grid on; legend('current','proposed');
subplot(1,2,2);
step(feedback(P,C0), feedback(P,C1), 8);   % input disturbance -> yaw
title('disturbance rejection (e.g. wheel imbalance)'); grid on;

%% 5) Reference numbers: ultimate gain (P-only oscillation point)
%  Ku ~ 10, Tu ~ 0.9 s for this plant. If the robot ever wags at ~1 Hz,
%  kp is far too high. Classic Z-N table gains (kp~6) are NOT suitable
%  for this integrating plant — use pole placement above instead.
