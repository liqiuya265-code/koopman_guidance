%% Stationary-target Koopman guidance: theory-consistent numerical validation
clearvars -except impactTimeOverride impactGammaDegOverride angleOnlyModeOverride ...
    nStepsOverride nTrainTrajOverride nTestTrajOverride;
clc; close all;
rng(97,'twister');
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

%% Parameters
p.Vnom=300;               % [m/s]
p.Vactual=0.97*p.Vnom;   % stress-test speed [m/s]
p.Rscale=6000;            % position scale [m]
p.amax=100;               % acceleration limit [m/s^2]
p.tauA=0.10;              % first-order autopilot time constant [s]
p.Ts=0.1;                 % sample time [s]
p.inputEffectiveness=0.94;
p.captureRadius=10;       % Cartesian coordinates permit a small capture set
p.impactTime=21.7;        % prescribed impact time [s]
p.impactGamma=deg2rad(-2);% prescribed impact angle / heading [rad]
p.impactTimeTolerance=p.Ts/2;
p.impactAngleTolerance=deg2rad(3);
p.postImpactWindow=2.0;   % keep simulating briefly after scheduled impact
if exist('impactTimeOverride','var'), p.impactTime=impactTimeOverride; end
if exist('impactGammaDegOverride','var'), p.impactGamma=deg2rad(impactGammaDegOverride); end
angleOnlyMode=false;
if exist('angleOnlyModeOverride','var'), angleOnlyMode=angleOnlyModeOverride; end

N=35;                    % 3.5 s direct prediction horizon
nSteps=90;
nTrainTraj=260;
nTestTraj=60;
if exist('nStepsOverride','var'), nSteps=nStepsOverride; end
if exist('nTrainTrajOverride','var'), nTrainTraj=nTrainTrajOverride; end
if exist('nTestTrajOverride','var'), nTestTraj=nTestTrajOverride; end
ridge=2e-6;

fprintf('Generating stationary-target data...\n');
train=generate_data(nTrainTraj,nSteps,N,p,0);
test=generate_data(nTestTraj,nSteps,N,p,1000);

%% Lift and identify one-step/direct multi-step predictors
nz=size(train.Z0,1);
nx=3;
C=[zeros(3,1),eye(3),zeros(3,nz-4)]; % z=[1;tau/tf;y/Rscale;theta;nonlinear features]

AB=ridge_regression(train.Zplus,[train.Zminus;train.Uone],ridge);
A=AB(:,1:nz);
B=AB(:,nz+1);
Bilinear=ridge_regression(train.Zplus, ...
    [train.Zminus;train.Uone;train.Zminus.*train.Uone],ridge);
Ablin=Bilinear(:,1:nz);
B0=Bilinear(:,nz+1);
B1=Bilinear(:,nz+2:end);
Theta=ridge_regression(train.Zfuture,[train.Z0;train.Useq],ridge);
ThetaZ=Theta(:,1:nz);
ThetaU=Theta(:,nz+1:end);
Cbar=kron(eye(N),C);

oneHat=AB*[test.Zminus;test.Uone];
oneNRMSE=normalized_rmse(C*test.Zplus,C*oneHat);
oneResidual95=prctile(abs((C*test.Zplus-C*oneHat).'),95).';
multiHat=Theta*[test.Z0;test.Useq];
multiTruth=Cbar*test.Zfuture;
multiPrediction=Cbar*multiHat;
multiNRMSE=normalized_rmse(reshape(multiTruth,nx,[]), ...
    reshape(multiPrediction,nx,[]));
bilinearHat=bilinear_predict_dataset(test.Z0,test.Useq,Ablin,B0,B1,N,nz);
bilinearPrediction=Cbar*bilinearHat;
bilinearNRMSE=normalized_rmse(reshape(multiTruth,nx,[]), ...
    reshape(bilinearPrediction,nx,[]));
tubeBound99=1.2*multi_step_residual_bound(multiTruth,bilinearPrediction,nx,N,0.99);

%% Exact bilinear lifting consistency check
bilinearErrors=zeros(1,300);
x=[0.8;0.22;deg2rad(18)];
for k=1:numel(bilinearErrors)
    u=0.8*sin(0.09*k)+0.15*cos(0.031*k);
    u=max(-1,min(1,u));
    xNext=plant_step_direct(x,u,p,false);
    zExact=exact_bilinear_lift(x);
    zNext=bilinear_lift_step(zExact,u,p);
    xFromLift=[zNext(2);zNext(3);zNext(4)];
    bilinearErrors(k)=norm(xNext-xFromLift,2);
    x=xNext;
end
maxBilinearError=max(bilinearErrors);

fprintf('Exact bilinear-lift RK4 consistency error: %.3e\n',maxBilinearError);
fprintf('One-step EDMD NRMSE [tau,y,theta]:\n'); disp(oneNRMSE.');
fprintf('Direct %d-step EDMD NRMSE [tau,y,theta]:\n',N); disp(multiNRMSE.');
fprintf('Bilinear rolling %d-step NRMSE [tau,y,theta]:\n',N); disp(bilinearNRMSE.');
fprintf('95%% one-step residual [tau/tf,y/Rs,theta]:\n'); disp(oneResidual95.');
fprintf('99%% tube terminal bound [tau/tf,y/Rs,theta]:\n'); disp(tubeBound99(:,end).');

%% KDPC controller
Qx=diag([600,900,35]);
Qterminal=diag([70000,90000,1200]);
if angleOnlyMode
    Qx=diag([80,900,35]);
    Qterminal=diag([2000,90000,1200]);
end
Qxbar=kron(eye(N),Qx);
Qxbar(end-nx+1:end,end-nx+1:end)=Qterminal;

ctrl.N=N; ctrl.nz=nz; ctrl.nx=nx;
ctrl.C=C; ctrl.Cbar=Cbar;
ctrl.ThetaZ=ThetaZ; ctrl.ThetaU=ThetaU;
ctrl.Ablin=Ablin; ctrl.B0=B0; ctrl.B1=B1;
ctrl.seqIterations=3;
ctrl.Qxbar=Qxbar;
ctrl.Rbar=0.06*eye(N);
ctrl.Rd=1.5;
ctrl.Q0=0.02*eye(nz);
ctrl.lambda=8e4;
ctrl.gammaMax=deg2rad(85);
ctrl.Ts=p.Ts;
ctrl.Vref=p.Vnom;
ctrl.Rscale=p.Rscale;
ctrl.amax=p.amax;
ctrl.impactTime=p.impactTime;
ctrl.impactGamma=p.impactGamma;
ctrl.angleOnlyMode=angleOnlyMode;
ctrl.captureRadius=p.captureRadius;
ctrl.impactTimeTolerance=p.impactTimeTolerance;
ctrl.impactAngleTolerance=p.impactAngleTolerance;
ctrl.terminalTol=[p.captureRadius/(p.Vnom*p.impactTime); ...
    p.captureRadius/p.Rscale; p.impactAngleTolerance];
ctrl.residualBound95=oneResidual95;
ctrl.tubeQuantile=0.99;
ctrl.tubeInflation=1.2;
ctrl.tubeBound=tubeBound99;
ctrl.maxTighteningFraction=0.8;
ctrl.terminalTightening=min(ctrl.maxTighteningFraction*ctrl.terminalTol, ...
    ctrl.tubeBound(:,end));
ctrl.terminalTolTight=ctrl.terminalTol-ctrl.terminalTightening;
ctrl.enableAdaptive=true;
ctrl.alphaHat=1;
ctrl.betaHat=1;
ctrl.alphaBounds=[0.90,1.10];
ctrl.betaBounds=[0.85,1.10];
ctrl.alphaGain=0.18;
ctrl.betaGain=0.12;
ctrl.adaptiveDeadzone=1e-5;
ctrl.slackPenalty=diag([2e6,2e6,5e5]);
ctrl.options=optimoptions('quadprog','Display','off', ...
    'Algorithm','interior-point-convex');

initial=[6000/p.Rscale;1500/p.Rscale;deg2rad(25);0];
nominal=run_closed_loop(initial,p,ctrl,false,angleOnlyMode);
stress=run_closed_loop(initial,p,ctrl,true,angleOnlyMode);

fprintf('Nominal: impactOK=%d, scheduled miss=%.3f m, angle error=%.2f deg, QP failures=%d\n', ...
    nominal.impactSatisfied,nominal.impactRange_m, ...
    nominal.impactHeadingError_deg,nominal.qpFailures);
fprintf('Stress:  impactOK=%d, scheduled miss=%.3f m, angle error=%.2f deg, QP failures=%d\n', ...
    stress.impactSatisfied,stress.impactRange_m, ...
    stress.impactHeadingError_deg,stress.qpFailures);

%% Prediction figure
sample=min(100,size(test.Z0,2));
zt=reshape(test.Zfuture(:,sample),nz,N);
zhLin=reshape(multiHat(:,sample),nz,N);
zhBil=reshape(bilinearHat(:,sample),nz,N);
fig1=figure('Color','w','Position',[100 100 900 680]);
subplot(3,1,1); plot(1:N,p.impactTime*C(1,:)*zt,'k','LineWidth',1.6); hold on;
plot(1:N,p.impactTime*C(1,:)*zhLin,'b--','LineWidth',1.4);
plot(1:N,p.impactTime*C(1,:)*zhBil,'r-.','LineWidth',1.4);
ylabel('tau [s]'); grid on;
legend('nonlinear','linear multi-step','bilinear rolling');
subplot(3,1,2); plot(1:N,p.Rscale*C(2,:)*zt,'k','LineWidth',1.6); hold on;
plot(1:N,p.Rscale*C(2,:)*zhLin,'b--','LineWidth',1.4);
plot(1:N,p.Rscale*C(2,:)*zhBil,'r-.','LineWidth',1.4);
ylabel('y [m]'); grid on;
subplot(3,1,3); plot(1:N,rad2deg(C(3,:)*zt),'k','LineWidth',1.6); hold on;
plot(1:N,rad2deg(C(3,:)*zhLin),'b--','LineWidth',1.4);
plot(1:N,rad2deg(C(3,:)*zhBil),'r-.','LineWidth',1.4);
xlabel('prediction step'); ylabel('theta [deg]'); grid on;
exportgraphics(fig1,fullfile(resultsDir,'prediction_validation.png'),'Resolution',180);

%% Closed-loop figures
fig2=figure('Color','w','Position',[100 80 980 820]);
subplot(4,1,1); plot(p.Rscale*nominal.x(1,:),p.Rscale*nominal.x(2,:), ...
    'b','LineWidth',1.7); hold on;
plot(p.Rscale*stress.x(1,:),p.Rscale*stress.x(2,:),'r--','LineWidth',1.5);
refLine=impact_reference_path(p);
plot(refLine(1,:),refLine(2,:),'k:','LineWidth',1.1);
plot(0,0,'ko','MarkerFaceColor','k'); axis equal; grid on;
xlabel('r_x [m]'); ylabel('r_y [m]'); legend('nominal','stress','time-angle reference','target');
subplot(4,1,2); plot(nominal.time_s,nominal.range_m,'b','LineWidth',1.6); hold on;
plot(stress.time_s,stress.range_m,'r--','LineWidth',1.5);
yline(p.captureRadius,'k:'); xline(p.impactTime,'k-.'); ylabel('range [m]'); grid on;
subplot(4,1,3); plot(nominal.time_s,rad2deg(nominal.x(3,:)),'b','LineWidth',1.5); hold on;
plot(stress.time_s,rad2deg(stress.x(3,:)),'r--','LineWidth',1.4);
yline(rad2deg(p.impactGamma),'k:'); xline(p.impactTime,'k-.');
ylabel('gamma [deg]'); grid on;
subplot(4,1,4); plot(nominal.time_s(1:end-1),p.amax*nominal.uActual, ...
    'b','LineWidth',1.5); hold on;
plot(stress.time_s(1:end-1),p.amax*stress.uActual,'r--','LineWidth',1.3);
stairs(nominal.time_s(1:end-1),p.amax*nominal.u,'Color',[0.2 0.55 1], ...
    'LineStyle',':','LineWidth',0.9);
yline(p.amax,'k:'); yline(-p.amax,'k:'); xlabel('time [s]');
ylabel('A [m/s^2]'); legend('nominal actual','stress actual','nominal command');
grid on;
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
metrics.bilinearRollingNRMSE=bilinearNRMSE;
metrics.oneStepResidual95=oneResidual95;
metrics.tubeBound99=tubeBound99;
metrics.terminalTol=ctrl.terminalTol;
metrics.terminalTightening=ctrl.terminalTightening;
metrics.terminalTolTight=ctrl.terminalTolTight;
metrics.nominal=nominal.metrics;
metrics.stress=stress.metrics;
save(fullfile(resultsDir,'stationary_target_results.mat'), ...
    'A','B','Ablin','B0','B1','C','Theta','metrics','nominal','stress','p','ctrl');

fid=fopen(fullfile(resultsDir,'metrics.txt'),'w');
fprintf(fid,'Stationary-target Koopman guidance validation\n');
fprintf(fid,'max_bilinear_consistency_error=%.12e\n',maxBilinearError);
fprintf(fid,'one_step_nrmse='); fprintf(fid,' %.8e',oneNRMSE); fprintf(fid,'\n');
fprintf(fid,'multi_step_nrmse='); fprintf(fid,' %.8e',multiNRMSE); fprintf(fid,'\n');
fprintf(fid,'bilinear_rolling_nrmse='); fprintf(fid,' %.8e',bilinearNRMSE); fprintf(fid,'\n');
fprintf(fid,'one_step_residual95='); fprintf(fid,' %.8e',oneResidual95); fprintf(fid,'\n');
fprintf(fid,'tube_bound99_terminal='); fprintf(fid,' %.8e',tubeBound99(:,end)); fprintf(fid,'\n');
fprintf(fid,'terminal_tol='); fprintf(fid,' %.8e',ctrl.terminalTol); fprintf(fid,'\n');
fprintf(fid,'terminal_tightening='); fprintf(fid,' %.8e',ctrl.terminalTightening); fprintf(fid,'\n');
fprintf(fid,'terminal_tol_tight='); fprintf(fid,' %.8e',ctrl.terminalTolTight); fprintf(fid,'\n');
write_case(fid,'nominal',nominal.metrics);
write_case(fid,'stress',stress.metrics);
fclose(fid);
fprintf('Results saved in %s\n',resultsDir);

%% Local functions
function data=generate_data(nTraj,nSteps,N,p,offset)
    rng(97+offset,'twister');
    nz=numel(lift_state([0;0;0;0],p));
    data.Zminus=zeros(nz,nTraj*nSteps);
    data.Zplus=zeros(nz,nTraj*nSteps);
    data.Uone=zeros(1,nTraj*nSteps);
    data.Z0=zeros(nz,nTraj*(nSteps-N));
    data.Useq=zeros(N,nTraj*(nSteps-N));
    data.Zfuture=zeros(nz*N,nTraj*(nSteps-N));
    s=0; w=0;
    for j=1:nTraj
        state=zeros(4,nSteps+1);
        rx=0.35+0.95*rand;
        ry=-0.45+0.9*rand;
        lambda=atan2(ry,rx);
        state(:,1)=[rx;ry;lambda-0.55+1.1*rand;0];
        raw=repelem(2*rand(1,ceil(nSteps/7))-1,7);
        input=filter(0.3,[1 -0.7],raw(1:nSteps));
        input=max(-1,min(1,input));
        for k=1:nSteps
            state(:,k+1)=plant_step(state(:,k),input(k),p,false);
            s=s+1;
            data.Zminus(:,s)=lift_state(state(:,k),p);
            data.Zplus(:,s)=lift_state(state(:,k+1),p);
            data.Uone(s)=input(k);
        end
        for k=1:nSteps-N
            w=w+1;
            data.Z0(:,w)=lift_state(state(:,k),p);
            data.Useq(:,w)=input(k:k+N-1).';
            future=zeros(nz,N);
            for h=1:N, future(:,h)=lift_state(state(:,k+h),p); end
            data.Zfuture(:,w)=future(:);
        end
    end
end

function z=lift_state(x,p)
    xt=time_to_go_state(x,p);
    tau=xt(1); y=xt(2); th=xt(3);
    ua=x(4);
    z=[1;tau;y;th;ua;cos(th);sin(th);tau*cos(th);tau*sin(th); ...
        y*cos(th);y*sin(th);th^2;ua*cos(th);ua*sin(th)];
end

function z=exact_bilinear_lift(x)
    z=[1;x(1);x(2);x(3);cos(x(3));sin(x(3))];
end

function xn=plant_step(x,u,p,stress)
    if stress, V=p.Vactual; eta=p.inputEffectiveness; else, V=p.Vnom; eta=1; end
    f=@(s)[-V/p.Rscale*cos(s(3)); ...
        -V/p.Rscale*sin(s(3)); ...
        eta*p.amax/V*s(4); ...
        (u-s(4))/p.tauA];
    k1=f(x); k2=f(x+0.5*p.Ts*k1); k3=f(x+0.5*p.Ts*k2); k4=f(x+p.Ts*k3);
    xn=x+p.Ts*(k1+2*k2+2*k3+k4)/6;
end

function xn=plant_step_direct(x,u,p,stress)
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

function Zfuture=bilinear_predict_dataset(Z0,Useq,A,B0,B1,N,nz)
    nCases=size(Z0,2);
    Zfuture=zeros(nz*N,nCases);
    for cidx=1:nCases
        z=Z0(:,cidx);
        for h=1:N
            u=Useq(h,cidx);
            z=A*z+B0*u+u*(B1*z);
            Zfuture((h-1)*nz+(1:nz),cidx)=z;
        end
    end
end

function out=run_closed_loop(initial,p,c,stress,angleOnlyMode)
    if angleOnlyMode
        Kmax=round(32/p.Ts);
    else
        Kmax=round((p.impactTime+p.postImpactWindow)/p.Ts);
    end
    x=zeros(numel(initial),Kmax+1); x(:,1)=initial;
    u=zeros(1,Kmax); xi=zeros(1,Kmax); err=zeros(1,Kmax); flags=zeros(1,Kmax);
    terminalSlack=zeros(3,Kmax);
    zPrevious=lift_state(x(:,1),p);
    uPrev=0;
    last=Kmax+1;
    alphaHist=zeros(1,Kmax+1); betaHist=zeros(1,Kmax+1);
    alphaHist(1)=c.alphaHat; betaHist(1)=c.betaHat;
    for k=1:Kmax
        zMeasured=lift_state(x(:,k),p);
        err(k)=norm(zPrevious-zMeasured);
        [useq,xi(k),Zpred,flags(k),terminalSlack(:,k)]= ...
            kdpc(zMeasured,zPrevious,c,k,uPrev);
        u(k)=useq(1); uPrev=u(k); zPrevious=Zpred(1:c.nz);
        x(:,k+1)=plant_step(x(:,k),u(k),p,stress);
        zNextMeasured=lift_state(x(:,k+1),p);
        c=update_adaptive_estimate(c,zMeasured,zNextMeasured,u(k));
        alphaHist(k+1)=c.alphaHat; betaHist(k+1)=c.betaHat;
        if angleOnlyMode
            range=p.Rscale*hypot(x(1,k+1),x(2,k+1));
            if range<=p.captureRadius
                last=k+1;
                break;
            end
        end
    end
    out.x=x(:,1:last); out.u=u(1:last-1); out.xi=xi(1:last-1);
    out.uActual=out.x(4,1:end-1);
    out.predictionError=err(1:last-1); out.exitflags=flags(1:last-1);
    out.terminalSlack=terminalSlack(:,1:last-1);
    out.alphaHat=alphaHist(1:last); out.betaHat=betaHist(1:last);
    out.time_s=(0:last-1)*p.Ts;
    out.range_m=p.Rscale*hypot(out.x(1,:),out.x(2,:));
    if angleOnlyMode
        captureIdx=find(out.range_m<=p.captureRadius,1,'first');
        if isempty(captureIdx)
            [~,impactIndex]=min(out.range_m);
        else
            impactIndex=captureIdx;
        end
    else
        [~,impactIndex]=min(abs(out.time_s-p.impactTime));
    end
    out.impactIndex=impactIndex;
    out.impactTime_s=out.time_s(impactIndex);
    out.impactTimeError_s=out.impactTime_s-p.impactTime;
    out.impactRange_m=out.range_m(impactIndex);
    out.impactHeading_deg=rad2deg(out.x(3,impactIndex));
    out.impactHeadingError_deg=rad2deg(wrap_angle( ...
        out.x(3,impactIndex)-p.impactGamma));
    if angleOnlyMode
        out.impactSatisfied=out.impactRange_m<=p.captureRadius && ...
            abs(deg2rad(out.impactHeadingError_deg))<=p.impactAngleTolerance;
    else
        out.impactSatisfied= ...
            abs(out.impactTimeError_s)<=p.impactTimeTolerance && ...
            out.impactRange_m<=p.captureRadius && ...
            abs(deg2rad(out.impactHeadingError_deg))<=p.impactAngleTolerance;
    end
    out.captured=out.impactSatisfied;
    out.finalRange_m=out.range_m(end);
    out.qpFailures=sum(out.exitflags<=0);
    out.metrics.impactSatisfied=out.impactSatisfied;
    out.metrics.captured=out.impactSatisfied;
    out.metrics.finalRange_m=out.finalRange_m;
    out.metrics.impactTime_s=out.impactTime_s;
    out.metrics.impactTimeError_s=out.impactTimeError_s;
    out.metrics.impactRange_m=out.impactRange_m;
    out.metrics.impactHeading_deg=out.impactHeading_deg;
    out.metrics.impactHeadingError_deg=out.impactHeadingError_deg;
    out.metrics.finalHeading_deg=rad2deg(out.x(3,end));
    out.metrics.maxCommand_mps2=max(abs(out.u))*p.amax;
    out.metrics.maxAcceleration_mps2=max(abs(out.uActual))*p.amax;
    out.metrics.qpFailures=out.qpFailures;
    out.metrics.meanXi=mean(out.xi);
    out.metrics.maxPredictionError=max(out.predictionError);
    out.metrics.maxTerminalSlack=max(out.terminalSlack,[],2);
    out.metrics.finalAlphaHat=out.alphaHat(end);
    out.metrics.finalBetaHat=out.betaHat(end);
end

function [U,xi,Zpred,exitflag,terminalSlack]=kdpc(zMeasured,zPrevious,c,k,uPrev)
    delta=zPrevious-zMeasured;
    U=zeros(c.N,1); xi=0; terminalSlack=zeros(3,1); exitflag=1;
    for iter=1:c.seqIterations
        z0=zMeasured+xi*delta;
        zbar=bilinear_rollout_states(z0,U,c);
        [offsetZ,mapZ]=ltv_prediction_maps(zMeasured,delta,zbar,c);
        [Unew,xinew,slacknew,flag]=solve_kdpc_qp(offsetZ,mapZ, ...
            zMeasured,delta,c,k,uPrev);
        if flag<=0
            exitflag=flag;
            break;
        end
        U=0.6*Unew+0.4*U;
        xi=xinew;
        terminalSlack=slacknew;
        exitflag=flag;
    end
    z0=zMeasured+xi*delta;
    Zpred=bilinear_rollout_stack(z0,U,c);
end

function zbar=bilinear_rollout_states(z0,U,c)
    zbar=zeros(c.nz,c.N);
    z=z0;
    for i=1:c.N
        zbar(:,i)=z;
        z=adaptive_bilinear_step(z,U(i),c);
    end
end

function Z=bilinear_rollout_stack(z0,U,c)
    Z=zeros(c.nz*c.N,1);
    z=z0;
    for i=1:c.N
        z=adaptive_bilinear_step(z,U(i),c);
        Z((i-1)*c.nz+(1:c.nz))=z;
    end
end

function [offsetZ,mapZ]=ltv_prediction_maps(zMeasured,delta,zbar,c)
    offsetZ=zeros(c.nz*c.N,1);
    mapZ=zeros(c.nz*c.N,c.N+1);
    zOff=zMeasured;
    uMap=zeros(c.nz,c.N);
    xiMap=delta;
    Aeff=eye(c.nz)+c.alphaHat*(c.Ablin-eye(c.nz));
    for i=1:c.N
        Beff=c.betaHat*(c.B0+c.B1*zbar(:,i));
        zOffNext=Aeff*zOff;
        uMapNext=Aeff*uMap;
        xiMapNext=Aeff*xiMap;
        uMapNext(:,i)=uMapNext(:,i)+Beff;
        rows=(i-1)*c.nz+(1:c.nz);
        offsetZ(rows)=zOffNext;
        mapZ(rows,:)=[uMapNext,xiMapNext];
        zOff=zOffNext;
        uMap=uMapNext;
        xiMap=xiMapNext;
    end
end

function zn=adaptive_bilinear_step(z,u,c)
    drift=c.Ablin*z-z;
    control=c.B0*u+u*(c.B1*z);
    zn=z+c.alphaHat*drift+c.betaHat*control;
    zn(1)=1;
end

function c=update_adaptive_estimate(c,z,zNext,u)
    if ~c.enableAdaptive
        return;
    end
    x0=c.C*z;
    x1=c.C*zNext;
    dx=[x1(1)-x0(1);x1(2)-x0(2);wrap_angle(x1(3)-x0(3))];
    theta=x0(3);
    phiAlpha=[-c.Ts*cos(theta)/c.impactTime; ...
        -c.Ts*c.Vref*sin(theta)/c.Rscale];
    errAlpha=dx(1:2)-phiAlpha*c.alphaHat;
    denomAlpha=phiAlpha.'*phiAlpha+c.adaptiveDeadzone;
    c.alphaHat=c.alphaHat+c.alphaGain*(phiAlpha.'*errAlpha)/denomAlpha;
    phiBeta=c.Ts*c.amax/c.Vref*u;
    if abs(phiBeta)>c.adaptiveDeadzone
        errBeta=dx(3)-phiBeta*c.betaHat;
        c.betaHat=c.betaHat+c.betaGain*phiBeta*errBeta/ ...
            (phiBeta^2+c.adaptiveDeadzone);
    end
    c.alphaHat=project_scalar(c.alphaHat,c.alphaBounds);
    c.betaHat=project_scalar(c.betaHat,c.betaBounds);
end

function value=project_scalar(value,bounds)
    value=min(max(value,bounds(1)),bounds(2));
end

function tol=terminal_tolerance_at_step(c,h)
    tube=c.tubeBound(:,h);
    tightening=min(c.maxTighteningFraction*c.terminalTol,tube);
    tol=c.terminalTol-tightening;
end

function [U,xi,terminalSlack,exitflag]=solve_kdpc_qp(offsetZ,mapZ, ...
    zMeasured,delta,c,k,uPrev)
    offsetX=c.Cbar*offsetZ; mapX=c.Cbar*mapZ;
    xRef=impact_reference_stack(k,c,zMeasured);
    nBase=c.N+1; nSlack=3; nVar=nBase+nSlack;
    H=zeros(nVar); f=zeros(nVar,1);
    H(1:nBase,1:nBase)=2*(mapX'*c.Qxbar*mapX);
    f(1:nBase)=2*(mapX'*c.Qxbar*(offsetX-xRef));
    H(1:c.N,1:c.N)=H(1:c.N,1:c.N)+2*c.Rbar;
    [Hd,fd]=delta_u_penalty(c.N,c.Rd,uPrev);
    H(1:c.N,1:c.N)=H(1:c.N,1:c.N)+2*Hd;
    f(1:c.N)=f(1:c.N)+2*fd;
    H(nBase,nBase)=H(nBase,nBase)+2*(delta'*c.Q0*delta+c.lambda*(delta'*delta));
    f(nBase)=f(nBase)+2*delta'*c.Q0*zMeasured;
    H(nBase+1:end,nBase+1:end)=H(nBase+1:end,nBase+1:end)+ ...
        2*c.slackPenalty;
    H=0.5*(H+H')+1e-9*eye(nVar);
    thetaSel=kron(eye(c.N),[0 0 1]);
    Aineq=[thetaSel*mapX,zeros(c.N,nSlack); ...
        -thetaSel*mapX,zeros(c.N,nSlack)];
    bineq=[(c.gammaMax-c.impactGamma)*ones(c.N,1)-thetaSel*offsetX; ...
        (c.gammaMax+c.impactGamma)*ones(c.N,1)+thetaSel*offsetX];
    if c.angleOnlyMode
        hImpact=c.N;
    else
        currentTime=(k-1)*c.Ts;
        hImpact=round((c.impactTime-currentTime)/c.Ts);
    end
    if hImpact>=1 && hImpact<=c.N
        idx=(hImpact-1)*c.nx+(1:c.nx);
        tol=terminal_tolerance_at_step(c,hImpact);
        slackCols=[1 0 0;0 1 0;0 0 1];
        if c.angleOnlyMode
            terminalDims=2:c.nx;
        else
            terminalDims=1:c.nx;
        end
        for j=terminalDims
            row=zeros(1,nVar);
            row(1:nBase)=mapX(idx(j),:);
            row(nBase+1:end)=-slackCols(j,:);
            Aineq=[Aineq;row];
            bineq=[bineq;tol(j)+xRef(idx(j))-offsetX(idx(j))];
            row=zeros(1,nVar);
            row(1:nBase)=-mapX(idx(j),:);
            row(nBase+1:end)=-slackCols(j,:);
            Aineq=[Aineq;row];
            bineq=[bineq;tol(j)-xRef(idx(j))+offsetX(idx(j))];
        end
        yRow=mapX(idx(2),:)/tol(2);
        thRow=mapX(idx(3),:)/tol(3);
        yOff=(offsetX(idx(2))-xRef(idx(2)))/tol(2);
        thOff=(offsetX(idx(3))-xRef(idx(3)))/tol(3);
        for sy=[-1,1]
            for st=[-1,1]
                row=zeros(1,nVar);
                row(1:nBase)=sy*yRow+st*thRow;
                row(nBase+2)=-1/tol(2);
                row(nBase+3)=-1/tol(3);
                Aineq=[Aineq;row];
                bineq=[bineq;1-sy*yOff-st*thOff];
            end
        end
    end
    lb=[-ones(c.N,1);0;zeros(nSlack,1)];
    ub=[ones(c.N,1);1;inf(nSlack,1)];
    [sol,~,exitflag]=quadprog(H,f,Aineq,bineq,[],[],lb,ub,[],c.options);
    if exitflag<=0 || isempty(sol), sol=[zeros(c.N,1);0;zeros(nSlack,1)]; end
    U=sol(1:c.N);
    xi=sol(nBase);
    terminalSlack=sol(nBase+1:end);
end

function [Hdu,fdu]=delta_u_penalty(N,Rd,uPrev)
    D=eye(N);
    for i=2:N
        D(i,i-1)=-1;
    end
    d=zeros(N,1);
    d(1)=-uPrev;
    Hdu=Rd*(D'*D);
    fdu=Rd*(D'*d);
end

function xRef=impact_reference_stack(k,c,zMeasured)
    xRef=zeros(c.nx*c.N,1);
    xNow=c.C*zMeasured;
    for h=1:c.N
        t=(k-1+h)*c.Ts;
        idx=(h-1)*c.nx+(1:c.nx);
        if c.angleOnlyMode
            tauRef=max(xNow(1)-h*c.Ts/c.impactTime,0);
            xRef(idx)=[tauRef;0;0];
        else
            tgo=max(c.impactTime-t,0);
            xRef(idx)=[tgo/c.impactTime;0;0];
        end
    end
end

function ref=impact_reference_path(p)
    t=linspace(0,p.impactTime,120);
    ef=[cos(p.impactGamma);sin(p.impactGamma)];
    ref=p.Vnom*(p.impactTime-t).*ef;
end

function a=wrap_angle(a)
    a=mod(a+pi,2*pi)-pi;
end

function xt=time_to_go_state(x,p)
    ef=[cos(p.impactGamma);sin(p.impactGamma)];
    nf=[-sin(p.impactGamma);cos(p.impactGamma)];
    r=p.Rscale*x(1:2);
    s=ef.'*r;
    y=nf.'*r;
    tau=s/p.Vnom;
    theta=wrap_angle(x(3)-p.impactGamma);
    xt=[tau/p.impactTime;y/p.Rscale;theta];
end

function W=ridge_regression(Y,X,lambda)
    W=(Y*X')/(X*X'+lambda*eye(size(X,1)));
end

function v=normalized_rmse(truth,prediction)
    v=sqrt(mean((truth-prediction).^2,2))./(std(truth,0,2)+1e-12);
end

function bound=multi_step_residual_bound(truth,prediction,nx,N,quantileLevel)
    residual=abs(reshape(truth-prediction,nx,N,[]));
    bound=zeros(nx,N);
    pct=100*quantileLevel;
    for h=1:N
        bound(:,h)=prctile(squeeze(residual(:,h,:)).',pct).';
    end
end

function write_case(fid,name,m)
    fprintf(fid,'%s_impact_satisfied=%d\n',name,m.impactSatisfied);
    fprintf(fid,'%s_captured=%d\n',name,m.captured);
    fprintf(fid,'%s_final_range_m=%.8f\n',name,m.finalRange_m);
    fprintf(fid,'%s_impact_time_s=%.8f\n',name,m.impactTime_s);
    fprintf(fid,'%s_impact_time_error_s=%.8f\n',name,m.impactTimeError_s);
    fprintf(fid,'%s_impact_range_m=%.8f\n',name,m.impactRange_m);
    fprintf(fid,'%s_impact_heading_deg=%.8f\n',name,m.impactHeading_deg);
    fprintf(fid,'%s_impact_heading_error_deg=%.8f\n',name,m.impactHeadingError_deg);
    fprintf(fid,'%s_final_heading_deg=%.8f\n',name,m.finalHeading_deg);
    fprintf(fid,'%s_max_command_mps2=%.8f\n',name,m.maxCommand_mps2);
    fprintf(fid,'%s_max_acceleration_mps2=%.8f\n',name,m.maxAcceleration_mps2);
    fprintf(fid,'%s_qp_failures=%d\n',name,m.qpFailures);
    fprintf(fid,'%s_mean_xi=%.8f\n',name,m.meanXi);
    fprintf(fid,'%s_max_prediction_error=%.8f\n',name,m.maxPredictionError);
    fprintf(fid,'%s_max_terminal_slack=',name);
    fprintf(fid,' %.8e',m.maxTerminalSlack);
    fprintf(fid,'\n');
    fprintf(fid,'%s_final_alpha_hat=%.8f\n',name,m.finalAlphaHat);
    fprintf(fid,'%s_final_beta_hat=%.8f\n',name,m.finalBetaHat);
end
