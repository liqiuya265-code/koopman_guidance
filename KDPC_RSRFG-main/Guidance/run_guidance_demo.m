%% Koopman data-driven predictive control for 2-D impact-angle guidance
% This demo is self-contained and uses only built-in MATLAB toolboxes.
clear; clc; close all;
rng(41, 'twister');
set(groot, 'defaultFigureVisible', 'off');

resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% Plant and experiment parameters
p.Vnom = 300;             % nominal vehicle speed [m/s]
p.Vactual = 0.95*p.Vnom; % validation plant speed [m/s]
p.Lscale = 4000;          % lateral displacement scale [m]
p.amax = 60;              % acceleration bound [m/s^2]
p.Ts = 0.1;               % sample time [s]
p.inputEffectiveness = 0.92;

N = 45;                   % prediction horizon (covers the turn recovery)
nTrainTraj = 220;
nTestTraj = 50;
nSteps = 100;
ridge = 1e-6;

fprintf('Generating guidance data...\n');
train = generate_data(nTrainTraj, nSteps, N, p, 0);
test = generate_data(nTestTraj, nSteps, N, p, 1000);

%% Identify one-step and direct multi-step Koopman predictors
nz = size(train.Z0, 1);
nx = 2;
C = zeros(nx, nz);
C(1,1) = 1;
C(2,2) = 1;

Omega1 = [train.Zminus; train.Uone];
AB = ridge_regression(train.Zplus, Omega1, ridge);
A = AB(:,1:nz);
B = AB(:,nz+1:end);

OmegaN = [train.Z0; train.Useq];
Theta = ridge_regression(train.Zfuture, OmegaN, ridge);
ThetaZ = Theta(:,1:nz);
ThetaU = Theta(:,nz+1:end);

%% Prediction validation
oneStepHat = AB*[test.Zminus; test.Uone];
oneStepTrueX = C*test.Zplus;
oneStepHatX = C*oneStepHat;
oneStepNRMSE = normalized_rmse(oneStepTrueX, oneStepHatX);

multiHat = Theta*[test.Z0; test.Useq];
Cbar = kron(eye(N), C);
multiTrueX = Cbar*test.Zfuture;
multiHatX = Cbar*multiHat;
multiNRMSE = normalized_rmse(reshape(multiTrueX,nx,[]), ...
    reshape(multiHatX,nx,[]));

fprintf('One-step state NRMSE: y/L = %.4f, gamma = %.4f\n', ...
    oneStepNRMSE(1), oneStepNRMSE(2));
fprintf('Direct %d-step NRMSE: y/L = %.4f, gamma = %.4f\n', ...
    N, multiNRMSE(1), multiNRMSE(2));

%% Physical-state stage and terminal weights
Qx = diag([220, 45]);
Qz = C'*Qx*C + 1e-5*eye(nz);
R = 0.10;
Qterminal = diag([900, 320]);
Pz = C'*Qterminal*C + 1e-4*eye(nz);

%% Closed-loop KDPC simulation
ctrl.N = N;
ctrl.nz = nz;
ctrl.C = C;
ctrl.Cbar = Cbar;
ctrl.ThetaZ = ThetaZ;
ctrl.ThetaU = ThetaU;
ctrl.Qzbar = kron(eye(N), Qz);
ctrl.Qzbar(end-nz+1:end,end-nz+1:end) = Pz;
ctrl.Rbar = R*eye(N);
ctrl.Q0 = 0.05*eye(nz);
ctrl.lambda = 1e5;
ctrl.gammaMax = deg2rad(75);
ctrl.options = optimoptions('quadprog', ...
    'Display', 'off', 'Algorithm', 'interior-point-convex');

Tsim = 30;
Ksim = round(Tsim/p.Ts);
x = zeros(nx, Ksim+1);
x(:,1) = [1200/p.Lscale; deg2rad(24)];
u = zeros(1, Ksim);
xi = zeros(1, Ksim);
predictionError = zeros(1, Ksim);
qpExit = zeros(1, Ksim);
zPredPrevious = lift_state(x(:,1));
zFirstPrediction = zeros(nz, Ksim);

for k = 1:Ksim
    t = (k-1)*p.Ts;
    zMeasured = lift_state(x(:,k));
    predictionError(k) = norm(zPredPrevious-zMeasured, 2);

    [uSequence, xi(k), Zpred, qpExit(k)] = kdpc_control( ...
        zMeasured, zPredPrevious, ctrl);
    u(k) = uSequence(1);
    zFirstPrediction(:,k) = Zpred(1:nz);
    zPredPrevious = zFirstPrediction(:,k);

    disturbance.gamma = 0.0035*sin(0.45*t) + 0.0015*cos(0.13*t);
    disturbance.lateral = 0.002*sin(0.2*t);
    x(:,k+1) = guidance_step(x(:,k), u(k), p, true, disturbance);
end

t = (0:Ksim)*p.Ts;
terminalLateralError = abs(x(1,end))*p.Lscale;
terminalAngleError = abs(rad2deg(x(2,end)));
maxAcceleration = max(abs(u))*p.amax;
constraintViolation = max(0, max(abs(x(2,:)))-ctrl.gammaMax);
qpFailures = sum(qpExit <= 0);

fprintf('Closed-loop final lateral error: %.3f m\n', terminalLateralError);
fprintf('Closed-loop final angle error: %.4f deg\n', terminalAngleError);
fprintf('Maximum commanded acceleration: %.3f m/s^2\n', maxAcceleration);
fprintf('QP failures: %d\n', qpFailures);
fprintf('Mean interpolation xi: %.4f\n', mean(xi));

%% Prediction plot
sample = min(50, size(test.Z0,2));
trueTrajectory = reshape(test.Zfuture(:,sample), nz, N);
predTrajectory = reshape(multiHat(:,sample), nz, N);

fig1 = figure('Color','w','Position',[100 100 920 560]);
subplot(2,1,1);
plot(1:N, p.Lscale*(C(1,:)*trueTrajectory), 'k-', 'LineWidth', 1.7); hold on;
plot(1:N, p.Lscale*(C(1,:)*predTrajectory), 'b--', 'LineWidth', 1.7);
ylabel('lateral displacement [m]'); grid on; legend('nonlinear model','Koopman');
subplot(2,1,2);
plot(1:N, rad2deg(C(2,:)*trueTrajectory), 'k-', 'LineWidth', 1.7); hold on;
plot(1:N, rad2deg(C(2,:)*predTrajectory), 'b--', 'LineWidth', 1.7);
xlabel('prediction step'); ylabel('angle error [deg]'); grid on;
exportgraphics(fig1, fullfile(resultsDir, 'prediction_validation.png'), 'Resolution', 180);

%% Closed-loop plot
fig2 = figure('Color','w','Position',[100 100 920 780]);
subplot(4,1,1);
plot(t, p.Lscale*x(1,:), 'LineWidth', 1.7); yline(0,'k:');
ylabel('y_r [m]'); grid on;
subplot(4,1,2);
plot(t, rad2deg(x(2,:)), 'LineWidth', 1.7); yline(0,'k:');
ylabel('angle [deg]'); grid on;
subplot(4,1,3);
stairs(t(1:end-1), p.amax*u, 'LineWidth', 1.7); hold on;
yline(p.amax,'r--'); yline(-p.amax,'r--');
ylabel('a_M [m/s^2]'); grid on;
subplot(4,1,4);
plot(t(1:end-1), xi, 'LineWidth', 1.5); hold on;
plot(t(1:end-1), predictionError, 'LineWidth', 1.2);
xlabel('time [s]'); ylabel('\xi / error'); grid on;
legend('\xi','||z_{pred}-z_{meas}||');
exportgraphics(fig2, fullfile(resultsDir, 'closed_loop.png'), 'Resolution', 180);

%% Save reproducible outputs
metrics = struct();
metrics.oneStepNRMSE = oneStepNRMSE;
metrics.multiStepNRMSE = multiNRMSE;
metrics.terminalLateralError_m = terminalLateralError;
metrics.terminalAngleError_deg = terminalAngleError;
metrics.maxAcceleration_mps2 = maxAcceleration;
metrics.maxAngleConstraintViolation_rad = constraintViolation;
metrics.qpFailures = qpFailures;
metrics.meanXi = mean(xi);
metrics.maxPredictionError = max(predictionError);

save(fullfile(resultsDir, 'guidance_results.mat'), ...
    'A','B','C','Theta','metrics','t','x','u','xi','predictionError','p','ctrl');

fid = fopen(fullfile(resultsDir, 'metrics.txt'), 'w');
fprintf(fid, 'Two-dimensional guidance KDPC validation\n');
fprintf(fid, 'one_step_nrmse_scaled_y=%.8f\n', oneStepNRMSE(1));
fprintf(fid, 'one_step_nrmse_gamma=%.8f\n', oneStepNRMSE(2));
fprintf(fid, 'direct_multistep_nrmse_scaled_y=%.8f\n', multiNRMSE(1));
fprintf(fid, 'direct_multistep_nrmse_gamma=%.8f\n', multiNRMSE(2));
fprintf(fid, 'terminal_lateral_error_m=%.8f\n', terminalLateralError);
fprintf(fid, 'terminal_angle_error_deg=%.8f\n', terminalAngleError);
fprintf(fid, 'max_acceleration_mps2=%.8f\n', maxAcceleration);
fprintf(fid, 'max_angle_constraint_violation_rad=%.8f\n', constraintViolation);
fprintf(fid, 'qp_failures=%d\n', qpFailures);
fprintf(fid, 'mean_xi=%.8f\n', mean(xi));
fprintf(fid, 'max_prediction_error=%.8f\n', max(predictionError));
fclose(fid);

fprintf('Results saved in %s\n', resultsDir);

%% Local functions
function data = generate_data(nTraj, nSteps, N, p, seedOffset)
    rng(41+seedOffset, 'twister');
    nz = numel(lift_state([0;0]));
    nWindows = nTraj*(nSteps-N);
    nSnapshots = nTraj*nSteps;
    data.Zminus = zeros(nz, nSnapshots);
    data.Zplus = zeros(nz, nSnapshots);
    data.Uone = zeros(1, nSnapshots);
    data.Z0 = zeros(nz, nWindows);
    data.Useq = zeros(N, nWindows);
    data.Zfuture = zeros(nz*N, nWindows);

    s = 0;
    w = 0;
    for j = 1:nTraj
        x = zeros(2,nSteps+1);
        x(:,1) = [1.6*rand-0.8; 1.2*rand-0.6];
        raw = repelem(2*rand(1,ceil(nSteps/5))-1,5);
        raw = raw(1:nSteps);
        input = filter(0.3, [1 -0.7], raw);
        input = max(-1,min(1,input));
        noDist.gamma = 0;
        noDist.lateral = 0;
        for k = 1:nSteps
            x(:,k+1) = guidance_step(x(:,k), input(k), p, false, noDist);
            s = s+1;
            data.Zminus(:,s) = lift_state(x(:,k));
            data.Zplus(:,s) = lift_state(x(:,k+1));
            data.Uone(:,s) = input(k);
        end
        for k = 1:(nSteps-N)
            w = w+1;
            data.Z0(:,w) = lift_state(x(:,k));
            data.Useq(:,w) = input(k:k+N-1).';
            zf = zeros(nz,N);
            for h = 1:N
                zf(:,h) = lift_state(x(:,k+h));
            end
            data.Zfuture(:,w) = zf(:);
        end
    end
end

function z = lift_state(x)
    y = x(1);
    g = x(2);
    z = [y; g; sin(g); cos(g)-1; y*g; y*sin(g); g^2];
end

function xnext = guidance_step(x, uScaled, p, actualPlant, disturbance)
    uScaled = max(-1,min(1,uScaled));
    if actualPlant
        V = p.Vactual;
        effectiveness = p.inputEffectiveness;
    else
        V = p.Vnom;
        effectiveness = 1;
    end
    f = @(state) [V/p.Lscale*sin(state(2)) + disturbance.lateral; ...
        effectiveness*p.amax/V*uScaled + disturbance.gamma];
    k1 = f(x);
    k2 = f(x+0.5*p.Ts*k1);
    k3 = f(x+0.5*p.Ts*k2);
    k4 = f(x+p.Ts*k3);
    xnext = x + p.Ts*(k1+2*k2+2*k3+k4)/6;
end

function W = ridge_regression(Y, X, lambda)
    W = (Y*X')/(X*X' + lambda*eye(size(X,1)));
end

function value = normalized_rmse(truth, prediction)
    error = truth-prediction;
    scale = std(truth,0,2) + 1e-12;
    value = sqrt(mean(error.^2,2))./scale;
end

function [U, xi, Zpred, exitflag] = kdpc_control(zMeasured, zPrevious, c)
    delta = zPrevious-zMeasured;
    offsetZ = c.ThetaZ*zMeasured;
    mapZ = [c.ThetaU, c.ThetaZ*delta];
    offsetX = c.Cbar*offsetZ;
    mapX = c.Cbar*mapZ;

    H = 2*(mapZ'*c.Qzbar*mapZ);
    f = 2*(mapZ'*c.Qzbar*offsetZ);
    H(1:c.N,1:c.N) = H(1:c.N,1:c.N) + 2*c.Rbar;
    H(end,end) = H(end,end) + 2*(delta'*c.Q0*delta + c.lambda*(delta'*delta));
    f(end) = f(end) + 2*delta'*c.Q0*zMeasured;
    H = 0.5*(H+H') + 1e-9*eye(c.N+1);

    gammaSelector = kron(eye(c.N), [0 1]);
    Aineq = [gammaSelector*mapX; -gammaSelector*mapX];
    bineq = [c.gammaMax*ones(c.N,1)-gammaSelector*offsetX; ...
        c.gammaMax*ones(c.N,1)+gammaSelector*offsetX];
    lb = [-ones(c.N,1);0];
    ub = [ones(c.N,1);1];

    [solution,~,exitflag] = quadprog(H,f,Aineq,bineq,[],[],lb,ub,[],c.options);
    if exitflag <= 0 || isempty(solution)
        solution = [zeros(c.N,1);0];
    end
    U = solution(1:c.N);
    xi = solution(end);
    z0 = zMeasured+xi*delta;
    Zpred = c.ThetaZ*z0+c.ThetaU*U;
end
