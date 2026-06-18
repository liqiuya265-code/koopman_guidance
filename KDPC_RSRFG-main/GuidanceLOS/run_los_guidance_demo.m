%% KDPC validation on a conventional planar LOS relative-motion model
clear; clc; close all;
rng(73, 'twister');
set(groot, 'defaultFigureVisible', 'off');

baseDir = fileparts(mfilename('fullpath'));
resultsDir = fullfile(baseDir, 'results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

%% Scaling and plant parameters
p.Rscale = 6000;           % [m]
p.Vscale = 300;            % [m/s]
p.omegaScale = 0.05;       % [rad/s]
p.amax = 55;               % [m/s^2]
p.tau = 0.22;              % actuator time constant [s]
p.Ts = 0.05;               % [s]
p.Rcapture = 300;          % [m], terminal capture set before LOS singularity
p.inputEffectiveness = 0.90;

N = 15;                   % 0.75 s horizon, shorter than terminal time-to-go
nSteps = 90;
nTrainTraj = 260;
nTestTraj = 60;
ridge = 2e-6;

fprintf('Generating LOS relative-motion data...\n');
train = generate_los_data(nTrainTraj,nSteps,N,p,0);
test = generate_los_data(nTestTraj,nSteps,N,p,1000);

%% Koopman lifting and direct multi-step predictor
nz = size(train.Z0,1);
nx = 5;
C = [eye(nx), zeros(nx,nz-nx)];
Omega1 = [train.Zminus;train.Uone];
AB = ridge_regression(train.Zplus,Omega1,ridge);
A = AB(:,1:nz);
B = AB(:,nz+1);

OmegaN = [train.Z0;train.Useq];
Theta = ridge_regression(train.Zfuture,OmegaN,ridge);
ThetaZ = Theta(:,1:nz);
ThetaU = Theta(:,nz+1:end);
Cbar = kron(eye(N),C);

oneHat = AB*[test.Zminus;test.Uone];
oneNRMSE = normalized_rmse(C*test.Zplus,C*oneHat);
multiHat = Theta*[test.Z0;test.Useq];
multiTruthX = Cbar*test.Zfuture;
multiHatX = Cbar*multiHat;
multiNRMSE = normalized_rmse(reshape(multiTruthX,nx,[]), ...
    reshape(multiHatX,nx,[]));

fprintf('One-step NRMSE [R,Rdot,q,qdot,aM]:\n'); disp(oneNRMSE.');
fprintf('Direct %d-step NRMSE [R,Rdot,q,qdot,aM]:\n',N); disp(multiNRMSE.');

%% KDPC cost and constraints
% Range and closing speed are scheduling states. The controller regulates LOS
% angle/rate and actuator state while range closes naturally.
Qx = diag([0.02,0.02,160,130,1.5]);
Qterminal = diag([0.02,0.02,520,460,3]);
Qz = C'*Qx*C + 1e-6*eye(nz);
Pz = C'*Qterminal*C + 1e-5*eye(nz);

ctrl.N = N;
ctrl.nz = nz;
ctrl.nx = nx;
ctrl.C = C;
ctrl.Cbar = Cbar;
ctrl.ThetaZ = ThetaZ;
ctrl.ThetaU = ThetaU;
ctrl.Qzbar = kron(eye(N),Qz);
ctrl.Qzbar(end-nz+1:end,end-nz+1:end) = Pz;
ctrl.Rbar = 0.12*eye(N);
ctrl.Q0 = 0.03*eye(nz);
ctrl.lambda = 2e5;
ctrl.qMax = deg2rad(55);
ctrl.omegaMaxScaled = 1.5;
ctrl.options = optimoptions('quadprog','Display','off', ...
    'Algorithm','interior-point-convex');

%% Mismatched closed-loop plant
Kmax = round(32/p.Ts);
x = zeros(nx,Kmax+1);
x(:,1) = [6000/p.Rscale;-250/p.Vscale;deg2rad(12);0.012/p.omegaScale;0];
u = zeros(1,Kmax);
xi = zeros(1,Kmax);
predError = zeros(1,Kmax);
exitflags = zeros(1,Kmax);
zPrevious = lift_los(x(:,1));
captureIndex = Kmax+1;

for k = 1:Kmax
    zMeasured = lift_los(x(:,k));
    predError(k) = norm(zPrevious-zMeasured);
    [useq,xi(k),Zpred,exitflags(k)] = kdpc_los(zMeasured,zPrevious,ctrl);
    u(k) = useq(1);
    zPrevious = Zpred(1:nz);

    time = (k-1)*p.Ts;
    disturbance.radial = 0.8*sin(0.21*time);
    disturbance.transverse = 2.0+2.5*sin(0.37*time);
    x(:,k+1) = los_step(x(:,k),u(k),p,true,disturbance);

    if x(1,k+1)*p.Rscale <= p.Rcapture
        captureIndex = k+1;
        break;
    end
end

last = min(captureIndex,Kmax+1);
x = x(:,1:last);
u = u(1:last-1);
xi = xi(1:last-1);
predError = predError(1:last-1);
exitflags = exitflags(1:last-1);
t = (0:last-1)*p.Ts;

range_m = p.Rscale*x(1,:);
closingSpeed = p.Vscale*x(2,:);
losAngle_deg = rad2deg(x(3,:));
losRate = p.omegaScale*x(4,:);
accel = p.amax*x(5,:);
command = p.amax*u;

metrics.captureAchieved = range_m(end)<=p.Rcapture;
metrics.finalRange_m = range_m(end);
metrics.finalLOSAngle_deg = losAngle_deg(end);
metrics.finalLOSRate_rads = losRate(end);
metrics.finalActuatorAcceleration_mps2 = accel(end);
metrics.maxCommand_mps2 = max(abs(command));
metrics.qpFailures = sum(exitflags<=0);
metrics.oneStepNRMSE = oneNRMSE;
metrics.multiStepNRMSE = multiNRMSE;
metrics.meanXi = mean(xi);
metrics.maxPredictionError = max(predError);

fprintf('Capture achieved: %d, final range %.3f m\n', ...
    metrics.captureAchieved,metrics.finalRange_m);
fprintf('Final LOS angle %.4f deg, LOS rate %.6f rad/s\n', ...
    metrics.finalLOSAngle_deg,metrics.finalLOSRate_rads);
fprintf('Maximum command %.3f m/s^2, QP failures %d\n', ...
    metrics.maxCommand_mps2,metrics.qpFailures);

%% Figures and outputs
fig = figure('Color','w','Position',[100 80 960 900]);
subplot(5,1,1); plot(t,range_m,'LineWidth',1.6); yline(p.Rcapture,'r--');
ylabel('R [m]'); grid on;
subplot(5,1,2); plot(t,closingSpeed,'LineWidth',1.6);
ylabel('dR/dt [m/s]'); grid on;
subplot(5,1,3); plot(t,losAngle_deg,'LineWidth',1.6); hold on;
yyaxis right; plot(t,losRate,'LineWidth',1.2); ylabel('dq/dt [rad/s]');
yyaxis left; ylabel('q [deg]'); grid on;
subplot(5,1,4); stairs(t(1:end-1),command,'LineWidth',1.5); hold on;
plot(t,accel,'LineWidth',1.2); yline(p.amax,'r--'); yline(-p.amax,'r--');
ylabel('a [m/s^2]'); legend('command','actuator'); grid on;
subplot(5,1,5); plot(t(1:end-1),xi,'LineWidth',1.4); hold on;
plot(t(1:end-1),predError,'LineWidth',1.1);
xlabel('time [s]'); ylabel('xi / error'); legend('xi','prediction error'); grid on;
exportgraphics(fig,fullfile(resultsDir,'los_closed_loop.png'),'Resolution',180);

sample = min(80,size(test.Z0,2));
zt = reshape(test.Zfuture(:,sample),nz,N);
zh = reshape(multiHat(:,sample),nz,N);
fig2 = figure('Color','w','Position',[100 100 900 620]);
subplot(2,1,1); plot(1:N,p.Rscale*C(1,:)*zt,'k','LineWidth',1.6); hold on;
plot(1:N,p.Rscale*C(1,:)*zh,'b--','LineWidth',1.6);
ylabel('R [m]'); legend('nonlinear','Koopman'); grid on;
subplot(2,1,2); plot(1:N,rad2deg(C(3,:)*zt),'k','LineWidth',1.6); hold on;
plot(1:N,rad2deg(C(3,:)*zh),'b--','LineWidth',1.6);
xlabel('prediction step'); ylabel('q [deg]'); grid on;
exportgraphics(fig2,fullfile(resultsDir,'los_prediction.png'),'Resolution',180);

save(fullfile(resultsDir,'los_guidance_results.mat'), ...
    'A','B','C','Theta','metrics','t','x','u','xi','predError','p','ctrl');
fid=fopen(fullfile(resultsDir,'metrics.txt'),'w');
fprintf(fid,'Conventional LOS guidance KDPC validation\n');
fprintf(fid,'capture_achieved=%d\n',metrics.captureAchieved);
fprintf(fid,'final_range_m=%.8f\n',metrics.finalRange_m);
fprintf(fid,'final_los_angle_deg=%.8f\n',metrics.finalLOSAngle_deg);
fprintf(fid,'final_los_rate_rads=%.10f\n',metrics.finalLOSRate_rads);
fprintf(fid,'max_command_mps2=%.8f\n',metrics.maxCommand_mps2);
fprintf(fid,'qp_failures=%d\n',metrics.qpFailures);
fprintf(fid,'mean_xi=%.8f\n',metrics.meanXi);
fprintf(fid,'max_prediction_error=%.8f\n',metrics.maxPredictionError);
fprintf(fid,'one_step_nrmse='); fprintf(fid,' %.8f',oneNRMSE); fprintf(fid,'\n');
fprintf(fid,'multi_step_nrmse='); fprintf(fid,' %.8f',multiNRMSE); fprintf(fid,'\n');
fclose(fid);
fprintf('Results saved in %s\n',resultsDir);

%% Local functions
function data=generate_los_data(nTraj,nSteps,N,p,offset)
    rng(73+offset,'twister');
    nz=numel(lift_los([1;-0.8;0;0;0]));
    data.Zminus=zeros(nz,nTraj*nSteps);
    data.Zplus=zeros(nz,nTraj*nSteps);
    data.Uone=zeros(1,nTraj*nSteps);
    data.Z0=zeros(nz,nTraj*(nSteps-N));
    data.Useq=zeros(N,nTraj*(nSteps-N));
    data.Zfuture=zeros(nz*N,nTraj*(nSteps-N));
    s=0; w=0;
    noDist.radial=0; noDist.transverse=0;
    for j=1:nTraj
        state=zeros(5,nSteps+1);
        state(:,1)=[0.45+0.75*rand;-0.95+0.35*rand; ...
            -0.45+0.9*rand;-0.55+1.1*rand;-0.6+1.2*rand];
        raw=repelem(2*rand(1,ceil(nSteps/6))-1,6);
        input=filter(0.35,[1 -0.65],raw(1:nSteps));
        input=max(-1,min(1,input));
        for k=1:nSteps
            state(:,k+1)=los_step(state(:,k),input(k),p,false,noDist);
            state(1,k+1)=max(state(1,k+1),0.08);
            s=s+1;
            data.Zminus(:,s)=lift_los(state(:,k));
            data.Zplus(:,s)=lift_los(state(:,k+1));
            data.Uone(s)=input(k);
        end
        for k=1:nSteps-N
            w=w+1;
            data.Z0(:,w)=lift_los(state(:,k));
            data.Useq(:,w)=input(k:k+N-1).';
            future=zeros(nz,N);
            for h=1:N, future(:,h)=lift_los(state(:,k+h)); end
            data.Zfuture(:,w)=future(:);
        end
    end
end

function z=lift_los(x)
    r=max(x(1),0.05); vr=x(2); q=x(3); w=x(4); a=x(5);
    z=[x;sin(q);cos(q)-1;1/r;vr/r;w/r;r*w^2;vr*w/r;q*w;a/r];
end

function xn=los_step(x,u,p,actual,dist)
    if actual, eta=p.inputEffectiveness; else, eta=1; end
    f=@(s) los_rhs(s,u,p,eta,dist);
    k1=f(x); k2=f(x+0.5*p.Ts*k1); k3=f(x+0.5*p.Ts*k2); k4=f(x+p.Ts*k3);
    xn=x+p.Ts*(k1+2*k2+2*k3+k4)/6;
    xn(1)=max(xn(1),p.Rcapture/p.Rscale*0.5);
end

function dx=los_rhs(x,u,p,eta,dist)
    R=max(x(1)*p.Rscale,p.Rcapture*0.5);
    Vr=x(2)*p.Vscale;
    omega=x(4)*p.omegaScale;
    aM=x(5)*p.amax;
    dx=zeros(5,1);
    dx(1)=Vr/p.Rscale;
    dx(2)=(R*omega^2+dist.radial)/p.Vscale;
    dx(3)=omega;
    dx(4)=((dist.transverse-eta*aM-2*Vr*omega)/R)/p.omegaScale;
    dx(5)=(u-x(5))/p.tau;
end

function W=ridge_regression(Y,X,lambda)
    W=(Y*X')/(X*X'+lambda*eye(size(X,1)));
end


function v=normalized_rmse(truth,prediction)
    v=sqrt(mean((truth-prediction).^2,2))./(std(truth,0,2)+1e-12);
end

function [U,xi,Zpred,exitflag]=kdpc_los(zMeasured,zPrevious,c)
    delta=zPrevious-zMeasured;
    offsetZ=c.ThetaZ*zMeasured;
    mapZ=[c.ThetaU,c.ThetaZ*delta];
    offsetX=c.Cbar*offsetZ;
    mapX=c.Cbar*mapZ;
    H=2*(mapZ'*c.Qzbar*mapZ);
    f=2*(mapZ'*c.Qzbar*offsetZ);
    H(1:c.N,1:c.N)=H(1:c.N,1:c.N)+2*c.Rbar;
    H(end,end)=H(end,end)+2*(delta'*c.Q0*delta+c.lambda*(delta'*delta));
    f(end)=f(end)+2*delta'*c.Q0*zMeasured;
    H=0.5*(H+H')+1e-9*eye(c.N+1);

    qSel=kron(eye(c.N),[0 0 1 0 0]);
    wSel=kron(eye(c.N),[0 0 0 1 0]);
    Aineq=[qSel*mapX;-qSel*mapX;wSel*mapX;-wSel*mapX];
    bineq=[c.qMax*ones(c.N,1)-qSel*offsetX; ...
        c.qMax*ones(c.N,1)+qSel*offsetX; ...
        c.omegaMaxScaled*ones(c.N,1)-wSel*offsetX; ...
        c.omegaMaxScaled*ones(c.N,1)+wSel*offsetX];
    lb=[-ones(c.N,1);0]; ub=[ones(c.N,1);1];
    [sol,~,exitflag]=quadprog(H,f,Aineq,bineq,[],[],lb,ub,[],c.options);
    if exitflag<=0 || isempty(sol), sol=[zeros(c.N,1);0]; end
    U=sol(1:c.N); xi=sol(end);
    z0=zMeasured+xi*delta;
    Zpred=c.ThetaZ*z0+c.ThetaU*U;
end
