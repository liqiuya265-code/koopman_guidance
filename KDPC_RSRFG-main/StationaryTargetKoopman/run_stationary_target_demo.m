%% Stationary-target Koopman guidance: theory-consistent numerical validation
clear; clc; close all;
rng(97,'twister');
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

%% Parameters
p.Vnom=300;               % [m/s]
p.Vactual=0.97*p.Vnom;   % stress-test speed [m/s]
p.Rscale=6000;            % position scale [m]
p.amax=60;                % acceleration limit [m/s^2]
p.Ts=0.1;                 % sample time [s]
p.inputEffectiveness=0.94;
p.captureRadius=25;       % Cartesian coordinates permit a small capture set

N=35;                    % 3.5 s direct prediction horizon
nSteps=90;
nTrainTraj=260;
nTestTraj=60;
ridge=2e-6;

fprintf('Generating stationary-target data...\n');
train=generate_data(nTrainTraj,nSteps,N,p,0);
test=generate_data(nTestTraj,nSteps,N,p,1000);

%% Lift and identify one-step/direct multi-step predictors
nz=size(train.Z0,1);
nx=3;
C=[zeros(3,1),eye(3),zeros(3,nz-4)]; % z=[1;x;nonlinear features]

AB=ridge_regression(train.Zplus,[train.Zminus;train.Uone],ridge);
A=AB(:,1:nz);
B=AB(:,nz+1);
Theta=ridge_regression(train.Zfuture,[train.Z0;train.Useq],ridge);
ThetaZ=Theta(:,1:nz);
ThetaU=Theta(:,nz+1:end);
Cbar=kron(eye(N),C);

oneHat=AB*[test.Zminus;test.Uone];
oneNRMSE=normalized_rmse(C*test.Zplus,C*oneHat);
multiHat=Theta*[test.Z0;test.Useq];
multiTruth=Cbar*test.Zfuture;
multiPrediction=Cbar*multiHat;
multiNRMSE=normalized_rmse(reshape(multiTruth,nx,[]), ...
    reshape(multiPrediction,nx,[]));

%% Exact bilinear lifting consistency check
bilinearErrors=zeros(1,300);
x=[0.8;0.22;deg2rad(18)];
for k=1:numel(bilinearErrors)
    u=0.8*sin(0.09*k)+0.15*cos(0.031*k);
    u=max(-1,min(1,u));
    xNext=plant_step(x,u,p,false);
    zExact=exact_bilinear_lift(x);
    zNext=bilinear_lift_step(zExact,u,p);
    xFromLift=[zNext(2);zNext(3);zNext(4)];
    bilinearErrors(k)=norm(xNext-xFromLift,2);
    x=xNext;
end
maxBilinearError=max(bilinearErrors);

fprintf('Exact bilinear-lift RK4 consistency error: %.3e\n',maxBilinearError);
fprintf('One-step EDMD NRMSE [rx,ry,gamma]:\n'); disp(oneNRMSE.');
fprintf('Direct %d-step EDMD NRMSE [rx,ry,gamma]:\n',N); disp(multiNRMSE.');

%% KDPC controller
Qx=diag([55,75,1.5]);
Qterminal=diag([600,800,4]);
Qz=C'*Qx*C+1e-7*eye(nz);
Pz=C'*Qterminal*C+1e-6*eye(nz);

ctrl.N=N; ctrl.nz=nz; ctrl.nx=nx;
ctrl.C=C; ctrl.Cbar=Cbar;
ctrl.ThetaZ=ThetaZ; ctrl.ThetaU=ThetaU;
ctrl.Qzbar=kron(eye(N),Qz);
ctrl.Qzbar(end-nz+1:end,end-nz+1:end)=Pz;
ctrl.Rbar=0.06*eye(N);
ctrl.Q0=0.02*eye(nz);
ctrl.lambda=8e4;
ctrl.gammaMax=deg2rad(85);
ctrl.options=optimoptions('quadprog','Display','off', ...
    'Algorithm','interior-point-convex');

initial=[6000/p.Rscale;1500/p.Rscale;deg2rad(25)];
nominal=run_closed_loop(initial,p,ctrl,false);
stress=run_closed_loop(initial,p,ctrl,true);

fprintf('Nominal: captured=%d, miss=%.3f m, time=%.2f s, QP failures=%d\n', ...
    nominal.captured,nominal.finalRange_m,nominal.time_s(end),nominal.qpFailures);
fprintf('Stress:  captured=%d, miss=%.3f m, time=%.2f s, QP failures=%d\n', ...
    stress.captured,stress.finalRange_m,stress.time_s(end),stress.qpFailures);

%% Prediction figure
sample=min(100,size(test.Z0,2));
zt=reshape(test.Zfuture(:,sample),nz,N);
zh=reshape(multiHat(:,sample),nz,N);
fig1=figure('Color','w','Position',[100 100 900 680]);
subplot(3,1,1); plot(1:N,p.Rscale*C(1,:)*zt,'k','LineWidth',1.6); hold on;
plot(1:N,p.Rscale*C(1,:)*zh,'b--','LineWidth',1.6); ylabel('r_x [m]'); grid on;
legend('nonlinear','linear EDMD');
subplot(3,1,2); plot(1:N,p.Rscale*C(2,:)*zt,'k','LineWidth',1.6); hold on;
plot(1:N,p.Rscale*C(2,:)*zh,'b--','LineWidth',1.6); ylabel('r_y [m]'); grid on;
subplot(3,1,3); plot(1:N,rad2deg(C(3,:)*zt),'k','LineWidth',1.6); hold on;
plot(1:N,rad2deg(C(3,:)*zh),'b--','LineWidth',1.6);
xlabel('prediction step'); ylabel('gamma [deg]'); grid on;
exportgraphics(fig1,fullfile(resultsDir,'prediction_validation.png'),'Resolution',180);

%% Closed-loop figures
fig2=figure('Color','w','Position',[100 80 980 820]);
subplot(4,1,1); plot(p.Rscale*nominal.x(1,:),p.Rscale*nominal.x(2,:), ...
    'b','LineWidth',1.7); hold on;
plot(p.Rscale*stress.x(1,:),p.Rscale*stress.x(2,:),'r--','LineWidth',1.5);
plot(0,0,'ko','MarkerFaceColor','k'); axis equal; grid on;
xlabel('r_x [m]'); ylabel('r_y [m]'); legend('nominal','stress','target');
subplot(4,1,2); plot(nominal.time_s,nominal.range_m,'b','LineWidth',1.6); hold on;
plot(stress.time_s,stress.range_m,'r--','LineWidth',1.5);
yline(p.captureRadius,'k:'); ylabel('range [m]'); grid on;
subplot(4,1,3); plot(nominal.time_s,rad2deg(nominal.x(3,:)),'b','LineWidth',1.5); hold on;
plot(stress.time_s,rad2deg(stress.x(3,:)),'r--','LineWidth',1.4);
ylabel('gamma [deg]'); grid on;
subplot(4,1,4); stairs(nominal.time_s(1:end-1),p.amax*nominal.u,'b','LineWidth',1.4); hold on;
stairs(stress.time_s(1:end-1),p.amax*stress.u,'r--','LineWidth',1.3);
yline(p.amax,'k:'); yline(-p.amax,'k:'); xlabel('time [s]');
ylabel('A [m/s^2]'); grid on;
exportgraphics(fig2,fullfile(resultsDir,'closed_loop.png'),'Resolution',180);

fig3=figure('Color','w','Position',[100 100 920 500]);
subplot(2,1,1); plot(nominal.time_s(1:end-1),nominal.xi,'LineWidth',1.5); hold on;
plot(stress.time_s(1:end-1),stress.xi,'--','LineWidth',1.4);
ylabel('xi'); legend('nominal','stress'); grid on;
subplot(2,1,2); semilogy(nominal.time_s(1:end-1),nominal.predictionError+1e-12,'LineWidth',1.4); hold on;
semilogy(stress.time_s(1:end-1),stress.predictionError+1e-12,'--','LineWidth',1.4);
xlabel('time [s]'); ylabel('prediction error'); grid on;
exportgraphics(fig3,fullfile(resultsDir,'interpolation_error.png'),'Resolution',180);

%% Save outputs
metrics.maxBilinearConsistencyError=maxBilinearError;
metrics.oneStepNRMSE=oneNRMSE;
metrics.multiStepNRMSE=multiNRMSE;
metrics.nominal=nominal.metrics;
metrics.stress=stress.metrics;
save(fullfile(resultsDir,'stationary_target_results.mat'), ...
    'A','B','C','Theta','metrics','nominal','stress','p','ctrl');

fid=fopen(fullfile(resultsDir,'metrics.txt'),'w');
fprintf(fid,'Stationary-target Koopman guidance validation\n');
fprintf(fid,'max_bilinear_consistency_error=%.12e\n',maxBilinearError);
fprintf(fid,'one_step_nrmse='); fprintf(fid,' %.8e',oneNRMSE); fprintf(fid,'\n');
fprintf(fid,'multi_step_nrmse='); fprintf(fid,' %.8e',multiNRMSE); fprintf(fid,'\n');
write_case(fid,'nominal',nominal.metrics);
write_case(fid,'stress',stress.metrics);
fclose(fid);
fprintf('Results saved in %s\n',resultsDir);

%% Local functions
function data=generate_data(nTraj,nSteps,N,p,offset)
    rng(97+offset,'twister');
    nz=numel(lift_state([0;0;0]));
    data.Zminus=zeros(nz,nTraj*nSteps);
    data.Zplus=zeros(nz,nTraj*nSteps);
    data.Uone=zeros(1,nTraj*nSteps);
    data.Z0=zeros(nz,nTraj*(nSteps-N));
    data.Useq=zeros(N,nTraj*(nSteps-N));
    data.Zfuture=zeros(nz*N,nTraj*(nSteps-N));
    s=0; w=0;
    for j=1:nTraj
        state=zeros(3,nSteps+1);
        rx=0.35+0.95*rand;
        ry=-0.45+0.9*rand;
        lambda=atan2(ry,rx);
        state(:,1)=[rx;ry;lambda-0.55+1.1*rand];
        raw=repelem(2*rand(1,ceil(nSteps/7))-1,7);
        input=filter(0.3,[1 -0.7],raw(1:nSteps));
        input=max(-1,min(1,input));
        for k=1:nSteps
            state(:,k+1)=plant_step(state(:,k),input(k),p,false);
            s=s+1;
            data.Zminus(:,s)=lift_state(state(:,k));
            data.Zplus(:,s)=lift_state(state(:,k+1));
            data.Uone(s)=input(k);
        end
        for k=1:nSteps-N
            w=w+1;
            data.Z0(:,w)=lift_state(state(:,k));
            data.Useq(:,w)=input(k:k+N-1).';
            future=zeros(nz,N);
            for h=1:N, future(:,h)=lift_state(state(:,k+h)); end
            data.Zfuture(:,w)=future(:);
        end
    end
end

function z=lift_state(x)
    rx=x(1); ry=x(2); g=x(3);
    z=[1;rx;ry;g;cos(g);sin(g);rx*cos(g);rx*sin(g); ...
        ry*cos(g);ry*sin(g);g^2];
end

function z=exact_bilinear_lift(x)
    z=[1;x(1);x(2);x(3);cos(x(3));sin(x(3))];
end

function xn=plant_step(x,u,p,stress)
    if stress, V=p.Vactual; eta=p.inputEffectiveness; else, V=p.Vnom; eta=1; end
    f=@(s)[-V/p.Rscale*cos(s(3));-V/p.Rscale*sin(s(3));eta*p.amax/V*u];
    k1=f(x); k2=f(x+0.5*p.Ts*k1); k3=f(x+0.5*p.Ts*k2); k4=f(x+p.Ts*k3);
    xn=x+p.Ts*(k1+2*k2+2*k3+k4)/6;
end

function zn=bilinear_lift_step(z,u,p)
    A0=zeros(6); B1=zeros(6);
    A0(2,5)=-p.Vnom/p.Rscale;
    A0(3,6)=-p.Vnom/p.Rscale;
    B1(4,1)=p.amax/p.Vnom;
    B1(5,6)=-p.amax/p.Vnom;
    B1(6,5)=p.amax/p.Vnom;
    f=@(s)(A0+u*B1)*s;
    k1=f(z); k2=f(z+0.5*p.Ts*k1); k3=f(z+0.5*p.Ts*k2); k4=f(z+p.Ts*k3);
    zn=z+p.Ts*(k1+2*k2+2*k3+k4)/6;
end

function out=run_closed_loop(initial,p,c,stress)
    Kmax=round(32/p.Ts);
    x=zeros(3,Kmax+1); x(:,1)=initial;
    u=zeros(1,Kmax); xi=zeros(1,Kmax); err=zeros(1,Kmax); flags=zeros(1,Kmax);
    zPrevious=lift_state(x(:,1)); last=Kmax+1; captured=false;
    for k=1:Kmax
        zMeasured=lift_state(x(:,k));
        err(k)=norm(zPrevious-zMeasured);
        [useq,xi(k),Zpred,flags(k)]=kdpc(zMeasured,zPrevious,c);
        u(k)=useq(1); zPrevious=Zpred(1:c.nz);
        x(:,k+1)=plant_step(x(:,k),u(k),p,stress);
        range=p.Rscale*hypot(x(1,k+1),x(2,k+1));
        if range<=p.captureRadius
            last=k+1; captured=true; break;
        end
    end
    out.x=x(:,1:last); out.u=u(1:last-1); out.xi=xi(1:last-1);
    out.predictionError=err(1:last-1); out.exitflags=flags(1:last-1);
    out.time_s=(0:last-1)*p.Ts;
    out.range_m=p.Rscale*hypot(out.x(1,:),out.x(2,:));
    out.captured=captured; out.finalRange_m=out.range_m(end);
    out.qpFailures=sum(out.exitflags<=0);
    out.metrics.captured=captured;
    out.metrics.finalRange_m=out.finalRange_m;
    out.metrics.captureTime_s=out.time_s(end);
    out.metrics.finalHeading_deg=rad2deg(out.x(3,end));
    out.metrics.maxAcceleration_mps2=max(abs(out.u))*p.amax;
    out.metrics.qpFailures=out.qpFailures;
    out.metrics.meanXi=mean(out.xi);
    out.metrics.maxPredictionError=max(out.predictionError);
end

function [U,xi,Zpred,exitflag]=kdpc(zMeasured,zPrevious,c)
    delta=zPrevious-zMeasured;
    offsetZ=c.ThetaZ*zMeasured;
    mapZ=[c.ThetaU,c.ThetaZ*delta];
    offsetX=c.Cbar*offsetZ; mapX=c.Cbar*mapZ;
    H=2*(mapZ'*c.Qzbar*mapZ); f=2*(mapZ'*c.Qzbar*offsetZ);
    H(1:c.N,1:c.N)=H(1:c.N,1:c.N)+2*c.Rbar;
    H(end,end)=H(end,end)+2*(delta'*c.Q0*delta+c.lambda*(delta'*delta));
    f(end)=f(end)+2*delta'*c.Q0*zMeasured;
    H=0.5*(H+H')+1e-9*eye(c.N+1);
    gammaSel=kron(eye(c.N),[0 0 1]);
    Aineq=[gammaSel*mapX;-gammaSel*mapX];
    bineq=[c.gammaMax*ones(c.N,1)-gammaSel*offsetX; ...
        c.gammaMax*ones(c.N,1)+gammaSel*offsetX];
    lb=[-ones(c.N,1);0]; ub=[ones(c.N,1);1];
    [sol,~,exitflag]=quadprog(H,f,Aineq,bineq,[],[],lb,ub,[],c.options);
    if exitflag<=0 || isempty(sol), sol=[zeros(c.N,1);0]; end
    U=sol(1:c.N); xi=sol(end);
    Zpred=c.ThetaZ*(zMeasured+xi*delta)+c.ThetaU*U;
end

function W=ridge_regression(Y,X,lambda)
    W=(Y*X')/(X*X'+lambda*eye(size(X,1)));
end

function v=normalized_rmse(truth,prediction)
    v=sqrt(mean((truth-prediction).^2,2))./(std(truth,0,2)+1e-12);
end

function write_case(fid,name,m)
    fprintf(fid,'%s_captured=%d\n',name,m.captured);
    fprintf(fid,'%s_final_range_m=%.8f\n',name,m.finalRange_m);
    fprintf(fid,'%s_capture_time_s=%.8f\n',name,m.captureTime_s);
    fprintf(fid,'%s_final_heading_deg=%.8f\n',name,m.finalHeading_deg);
    fprintf(fid,'%s_max_acceleration_mps2=%.8f\n',name,m.maxAcceleration_mps2);
    fprintf(fid,'%s_qp_failures=%d\n',name,m.qpFailures);
    fprintf(fid,'%s_mean_xi=%.8f\n',name,m.meanXi);
    fprintf(fid,'%s_max_prediction_error=%.8f\n',name,m.maxPredictionError);
end

