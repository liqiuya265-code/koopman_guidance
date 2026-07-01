%% Stationary-target Koopman guidance: theory-consistent numerical validation
if ~exist('skipClearOverride','var') || ~skipClearOverride
    clearvars -except impactTimeOverride impactGammaDegOverride ...
        angleOnlyModeOverride initialGammaDegOverride ...
        initialPositionMetersOverride initialPositionOverride ...
        initialAutopilotOverride resultSuffixOverride ...
        nStepsOverride nTrainTrajOverride nTestTrajOverride ...
        horizonOverride qxOverride qterminalOverride rbarScaleOverride ...
        rdOverride duMaxOverride deltaAccelMaxOverride accelRateMaxOverride ...
        moveBlockSizeOverride ...
        q0ScaleOverride xiPenaltyOverride slackPenaltyOverride ...
        seqIterationsOverride maxTighteningFractionOverride ...
        enableAdaptiveOverride alphaBoundsOverride betaBoundsOverride ...
        enableTwoStageCostOverride twoStageWindowOverride ...
        qprogressOverride qterminalStageOverride ...
        enableTerminalRefinementOverride terminalRefinementWindowOverride ...
        nmpcMoveBlocksOverride terminalNmpcMaxIterationsOverride ...
        enableImpactStageCostOverride ...
        angleOnlyMaxTimeOverride gammaMaxDegOverride ...
        trainingDataModeOverride angleStateModeOverride ...
        angleOnlyTerminalTauOverride angleOnlyZeroTauRefOverride ...
        enableYReferenceShapeOverride yRefAmplitudeOverride ...
        yRefSignOverride yRefTauScaleOverride enableJointReferenceOverride ...
        fovMaxDegOverride fovMinRangeOverride disableFovOverride ...
        skipControllerComparisonOverride;
end
clc; close all;
rng(97,'twister');
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

%% Parameters
p.Vnom=300;               % [m/s]
p.Vactual=0.97*p.Vnom;   % stress-test speed [m/s]
p.Rscale=10000;            % position scale [m]
p.amax=100;               % acceleration limit [m/s^2]
p.tauA=0.10;              % first-order autopilot time constant [s]
p.Ts=0.1;                 % sample time [s]
p.inputEffectiveness=0.94;
p.captureRadius=5;       % Cartesian coordinates permit a small capture set
p.impactTime=40;        % prescribed impact time [s]
p.impactGamma=deg2rad(30);% prescribed impact angle / heading [rad]
p.impactTimeTolerance=p.Ts/2;
p.impactAngleTolerance=deg2rad(3);
p.postImpactWindow=2.0;   % keep simulating briefly after scheduled impact
p.angleOnlyMaxTime=45.0;
p.trainingDataMode='local';
p.angleStateMode='theta';
if exist('impactTimeOverride','var'), p.impactTime=impactTimeOverride; end
if exist('impactGammaDegOverride','var'), p.impactGamma=deg2rad(impactGammaDegOverride); end
if exist('angleOnlyMaxTimeOverride','var')
    p.angleOnlyMaxTime=angleOnlyMaxTimeOverride;
end
if exist('trainingDataModeOverride','var')
    p.trainingDataMode=trainingDataModeOverride;
end
if exist('angleStateModeOverride','var')
    p.angleStateMode=angleStateModeOverride;
end
angleOnlyMode=false;
if exist('angleOnlyModeOverride','var'), angleOnlyMode=angleOnlyModeOverride; end
resultSuffix='';
if exist('resultSuffixOverride','var'), resultSuffix=resultSuffixOverride; end

N=35;                    % 3.5 s direct prediction horizon
nSteps=90;
nTrainTraj=260;
nTestTraj=60;
if exist('horizonOverride','var'), N=horizonOverride; end
if exist('nStepsOverride','var'), nSteps=nStepsOverride; end
if exist('nTrainTrajOverride','var'), nTrainTraj=nTrainTrajOverride; end
if exist('nTestTrajOverride','var'), nTestTraj=nTestTrajOverride; end
ridge=2e-6;

fprintf('Generating stationary-target data...\n');
train=generate_data(nTrainTraj,nSteps,N,p,0);
test=generate_data(nTestTraj,nSteps,N,p,1000);

%% Lift and identify one-step/direct multi-step predictors
nz=size(train.Z0,1);
nx=numel(guidance_error_state([0;0;0;0],p));
if strcmp(p.angleStateMode,'sincos')
    C=[zeros(4,1),eye(4),zeros(4,nz-5)];
else
    C=[zeros(3,1),eye(3),zeros(3,nz-4)]; % z=[1;tau/tf;y/Rscale;theta;nonlinear features]
end

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
oneRollingHat=linear_predict_dataset(test.Z0,test.Useq,A,B,N,nz);
oneRollingPrediction=Cbar*oneRollingHat;
oneRollingNRMSE=normalized_rmse(reshape(multiTruth,nx,[]), ...
    reshape(oneRollingPrediction,nx,[]));
tubeBound99=1.2*multi_step_residual_bound(multiTruth,bilinearPrediction,nx,N,0.99);
linearTubeBound99=1.2*multi_step_residual_bound(multiTruth,multiPrediction,nx,N,0.99);

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
fprintf('Rolling one-step EDMD %d-step NRMSE [tau,y,theta]:\n',N); disp(oneRollingNRMSE.');
fprintf('Direct %d-step EDMD NRMSE [tau,y,theta]:\n',N); disp(multiNRMSE.');
fprintf('Bilinear rolling %d-step NRMSE [tau,y,theta]:\n',N); disp(bilinearNRMSE.');
fprintf('95%% one-step residual [tau/tf,y/Rs,theta]:\n'); disp(oneResidual95.');
fprintf('99%% tube terminal bound [tau/tf,y/Rs,theta]:\n'); disp(tubeBound99(:,end).');

%% KDPC controller
if strcmp(p.angleStateMode,'sincos')
    Qx=diag([600,900,100,60]);
    Qterminal=diag([70000,90000,20000,12000]);
elseif angleOnlyMode
    Qx=diag([80,900,35]);
    Qterminal=diag([2000,90000,1200]);
else
    Qx=diag([600,900,35]);
    Qterminal=diag([70000,90000,1200]);
end
if exist('qxOverride','var'), Qx=diag(qxOverride); end
if exist('qterminalOverride','var'), Qterminal=diag(qterminalOverride); end
Qxbar=kron(eye(N),Qx);
Qxbar(end-nx+1:end,end-nx+1:end)=Qterminal;

ctrl.N=N; ctrl.nz=nz; ctrl.nx=nx;
ctrl.C=C; ctrl.Cbar=Cbar;
ctrl.ThetaZ=ThetaZ; ctrl.ThetaU=ThetaU;
ctrl.Ablin=Ablin; ctrl.B0=B0; ctrl.B1=B1;
ctrl.seqIterations=3;
ctrl.Qxbar=Qxbar;
ctrl.enableTwoStageCost=false;
ctrl.enableImpactStageCost=false;
ctrl.Qprogress=Qx;
ctrl.QterminalStage=Qx;
ctrl.QterminalCost=Qterminal;
ctrl.twoStageWindow=5.0;
ctrl.Rbar=0.06*eye(N);
ctrl.Rd=1.5;
ctrl.duMax=inf;
ctrl.deltaAccelMax=inf;
ctrl.amax=p.amax;
ctrl.moveBlockSize=1;
ctrl.Q0=0.02*eye(nz);
ctrl.lambda=8e4;
if exist('seqIterationsOverride','var'), ctrl.seqIterations=seqIterationsOverride; end
if exist('enableTwoStageCostOverride','var')
    ctrl.enableTwoStageCost=enableTwoStageCostOverride;
end
if exist('enableImpactStageCostOverride','var')
    ctrl.enableImpactStageCost=enableImpactStageCostOverride;
end
if exist('qprogressOverride','var'), ctrl.Qprogress=diag(qprogressOverride); end
if exist('qterminalStageOverride','var')
    ctrl.QterminalStage=diag(qterminalStageOverride);
end
if exist('qterminalOverride','var'), ctrl.QterminalCost=diag(qterminalOverride); end
if exist('twoStageWindowOverride','var'), ctrl.twoStageWindow=twoStageWindowOverride; end
if exist('rbarScaleOverride','var'), ctrl.Rbar=rbarScaleOverride*eye(N); end
if exist('rdOverride','var'), ctrl.Rd=rdOverride; end
if exist('duMaxOverride','var'), ctrl.duMax=duMaxOverride; end
if exist('deltaAccelMaxOverride','var')
    ctrl.deltaAccelMax=deltaAccelMaxOverride;
end
if exist('accelRateMaxOverride','var')
    ctrl.deltaAccelMax=accelRateMaxOverride;
end
if exist('moveBlockSizeOverride','var')
    ctrl.moveBlockSize=moveBlockSizeOverride;
end
if exist('q0ScaleOverride','var'), ctrl.Q0=q0ScaleOverride*eye(nz); end
if exist('xiPenaltyOverride','var'), ctrl.lambda=xiPenaltyOverride; end
ctrl.gammaMax=deg2rad(85);
if exist('gammaMaxDegOverride','var')
    ctrl.gammaMax=deg2rad(gammaMaxDegOverride);
end
ctrl.enableFov=false;
ctrl.fovMax=deg2rad(20);
ctrl.fovMinRange=300;
ctrl.Ts=p.Ts;
ctrl.angleStateMode=p.angleStateMode;
ctrl.angleOnlyTerminalTau=false;
ctrl.angleOnlyZeroTauRef=false;
ctrl.enableYReferenceShape=false;
ctrl.enableJointReference=false;
ctrl.yRefAmplitude=0;
ctrl.yRefSign=1;
ctrl.yRefTauScale=0.6;
if exist('angleOnlyTerminalTauOverride','var')
    ctrl.angleOnlyTerminalTau=angleOnlyTerminalTauOverride;
end
if exist('angleOnlyZeroTauRefOverride','var')
    ctrl.angleOnlyZeroTauRef=angleOnlyZeroTauRefOverride;
end
if exist('enableYReferenceShapeOverride','var')
    ctrl.enableYReferenceShape=enableYReferenceShapeOverride;
end
if exist('enableJointReferenceOverride','var')
    ctrl.enableJointReference=enableJointReferenceOverride;
end
if exist('yRefAmplitudeOverride','var')
    ctrl.yRefAmplitude=yRefAmplitudeOverride;
end
if exist('yRefSignOverride','var')
    ctrl.yRefSign=yRefSignOverride;
end
if exist('yRefTauScaleOverride','var')
    ctrl.yRefTauScale=yRefTauScaleOverride;
end
ctrl.Vref=p.Vnom;
ctrl.Rscale=p.Rscale;
ctrl.amax=p.amax;
ctrl.impactTime=p.impactTime;
ctrl.impactGamma=p.impactGamma;
ctrl.angleOnlyMode=angleOnlyMode;
ctrl.captureRadius=p.captureRadius;
ctrl.impactTimeTolerance=p.impactTimeTolerance;
ctrl.impactAngleTolerance=p.impactAngleTolerance;
if strcmp(p.angleStateMode,'sincos')
    ctrl.terminalTol=[p.captureRadius/(p.Vnom*p.impactTime); ...
        p.captureRadius/p.Rscale; sin(p.impactAngleTolerance); ...
        1-cos(p.impactAngleTolerance)];
else
    ctrl.terminalTol=[p.captureRadius/(p.Vnom*p.impactTime); ...
        p.captureRadius/p.Rscale; p.impactAngleTolerance];
end
ctrl.nSlack=nx;
ctrl.residualBound95=oneResidual95;
ctrl.tubeQuantile=0.99;
ctrl.tubeInflation=1.2;
ctrl.tubeBound=tubeBound99;
ctrl.maxTighteningFraction=0.8;
if exist('maxTighteningFractionOverride','var')
    ctrl.maxTighteningFraction=maxTighteningFractionOverride;
end
ctrl.terminalTightening=min(ctrl.maxTighteningFraction*ctrl.terminalTol, ...
    ctrl.tubeBound(:,end));
ctrl.terminalTolTight=ctrl.terminalTol-ctrl.terminalTightening;
ctrl.enableAdaptive=true;
ctrl.alphaHat=1;
ctrl.betaHat=1;
ctrl.alphaBounds=[0.90,1.10];
ctrl.betaBounds=[0.85,1.10];
if exist('enableAdaptiveOverride','var'), ctrl.enableAdaptive=enableAdaptiveOverride; end
if exist('alphaBoundsOverride','var'), ctrl.alphaBounds=alphaBoundsOverride; end
if exist('betaBoundsOverride','var'), ctrl.betaBounds=betaBoundsOverride; end
ctrl.alphaGain=0.18;
ctrl.betaGain=0.12;
ctrl.adaptiveDeadzone=1e-5;
if strcmp(p.angleStateMode,'sincos')
    ctrl.slackPenalty=diag([2e6,2e6,5e5,5e5]);
else
    ctrl.slackPenalty=diag([2e6,2e6,5e5]);
end
if exist('slackPenaltyOverride','var'), ctrl.slackPenalty=diag(slackPenaltyOverride); end
ctrl.options=optimoptions('quadprog','Display','off', ...
    'Algorithm','interior-point-convex');
ctrl.controllerType='bilinear';
ctrl.fminconOptions=optimoptions('fmincon','Display','off', ...
    'Algorithm','sqp','MaxIterations',22,'MaxFunctionEvaluations',1400, ...
    'OptimalityTolerance',2e-4,'StepTolerance',2e-5);
ctrl.enableTerminalRefinement=false;
ctrl.terminalRefinementWindow=4.0;
ctrl.nmpcMoveBlocks=N;
if exist('enableTerminalRefinementOverride','var')
    ctrl.enableTerminalRefinement=enableTerminalRefinementOverride;
end
if exist('terminalRefinementWindowOverride','var')
    ctrl.terminalRefinementWindow=terminalRefinementWindowOverride;
end
if exist('nmpcMoveBlocksOverride','var')
    ctrl.nmpcMoveBlocks=nmpcMoveBlocksOverride;
end
if exist('terminalNmpcMaxIterationsOverride','var')
    ctrl.fminconOptions.MaxIterations=terminalNmpcMaxIterationsOverride;
end
if exist('fovMaxDegOverride','var')
    ctrl.enableFov=true;
    ctrl.fovMax=deg2rad(fovMaxDegOverride);
end
if exist('disableFovOverride','var') && disableFovOverride
    ctrl.enableFov=false;
end
if exist('fovMinRangeOverride','var')
    ctrl.fovMinRange=fovMinRangeOverride;
end
runControllerComparison=true;
if exist('skipControllerComparisonOverride','var')
    runControllerComparison=~skipControllerComparisonOverride;
end

initialGammaDeg=-30;
if exist('initialGammaDegOverride','var'), initialGammaDeg=initialGammaDegOverride; end
initialPosition_m=[10000;0];
if exist('initialPositionMetersOverride','var')
    initialPosition_m=initialPositionMetersOverride(:);
elseif exist('initialPositionOverride','var')
    initialPosition_m=p.Rscale*initialPositionOverride(:);
end
if numel(initialPosition_m)~=2
    error('Initial position override must be a 2-element vector [rx; ry].');
end
initialAutopilot=0;
if exist('initialAutopilotOverride','var')
    initialAutopilot=initialAutopilotOverride;
end
p.initialPosition_m=initialPosition_m;
p.initialGammaDeg=initialGammaDeg;
p.initialAutopilot=initialAutopilot;
initial=[initialPosition_m/p.Rscale;deg2rad(initialGammaDeg);initialAutopilot];
nominal=run_closed_loop(initial,p,ctrl,false,angleOnlyMode);
stress=run_closed_loop(initial,p,ctrl,true,angleOnlyMode);

comparison=struct();
comparison.bilinear=nominal;
if runControllerComparison
    linearCtrl=ctrl;
    linearCtrl.controllerType='linear';
    linearCtrl.enableAdaptive=false;
    linearCtrl.tubeBound=linearTubeBound99;
    linearCtrl.terminalTightening=min(linearCtrl.maxTighteningFraction*linearCtrl.terminalTol, ...
        linearCtrl.tubeBound(:,end));
    linearCtrl.terminalTolTight=linearCtrl.terminalTol-linearCtrl.terminalTightening;

    nmpcCtrl=ctrl;
    nmpcCtrl.controllerType='nmpc';
    nmpcCtrl.enableAdaptive=false;
    nmpcCtrl.tubeBound=zeros(size(ctrl.tubeBound));
    nmpcCtrl.terminalTightening=zeros(size(ctrl.terminalTightening));
    nmpcCtrl.terminalTolTight=nmpcCtrl.terminalTol;
    nmpcCtrl.nmpcMoveBlocks=8;

    comparison.linear=run_closed_loop(initial,p,linearCtrl,false,angleOnlyMode);
    comparison.nmpc=run_closed_loop(initial,p,nmpcCtrl,false,angleOnlyMode);
end

fprintf('Nominal: impactOK=%d, scheduled miss=%.3f m, angle error=%.2f deg, QP failures=%d\n', ...
    nominal.impactSatisfied,nominal.impactRange_m, ...
    nominal.impactHeadingError_deg,nominal.qpFailures);
fprintf('Stress:  impactOK=%d, scheduled miss=%.3f m, angle error=%.2f deg, QP failures=%d\n', ...
    stress.impactSatisfied,stress.impactRange_m, ...
    stress.impactHeadingError_deg,stress.qpFailures);
if runControllerComparison
    fprintf('Closed-loop comparison completed: Linear Koopman-MPC, Bilinear Koopman-MPC, NMPC\n');
end

%% Prediction figure
sample=min(100,size(test.Z0,2));
zt=reshape(test.Zfuture(:,sample),nz,N);
zhOne=reshape(oneRollingHat(:,sample),nz,N);
zhLin=reshape(multiHat(:,sample),nz,N);
zhBil=reshape(bilinearHat(:,sample),nz,N);
fig1=figure('Color','w','Position',[100 100 900 680]);
subplot(3,1,1); plot(1:N,p.impactTime*C(1,:)*zt,'k','LineWidth',1.6); hold on;
plot(1:N,p.impactTime*C(1,:)*zhOne,'Color',[0.25 0.55 0.95], ...
    'LineStyle',':','LineWidth',1.5);
plot(1:N,p.impactTime*C(1,:)*zhLin,'b--','LineWidth',1.4);
plot(1:N,p.impactTime*C(1,:)*zhBil,'r-.','LineWidth',1.4);
ylabel('tau [s]'); grid on;
legend('nonlinear','one-step EDMD rolling','linear multi-step','bilinear rolling');
subplot(3,1,2); plot(1:N,p.Rscale*C(2,:)*zt,'k','LineWidth',1.6); hold on;
plot(1:N,p.Rscale*C(2,:)*zhOne,'Color',[0.25 0.55 0.95], ...
    'LineStyle',':','LineWidth',1.5);
plot(1:N,p.Rscale*C(2,:)*zhLin,'b--','LineWidth',1.4);
plot(1:N,p.Rscale*C(2,:)*zhBil,'r-.','LineWidth',1.4);
ylabel('y [m]'); grid on;
thetaTrue=prediction_theta(C,zt,p);
thetaOne=prediction_theta(C,zhOne,p);
thetaLin=prediction_theta(C,zhLin,p);
thetaBil=prediction_theta(C,zhBil,p);
subplot(3,1,3); plot(1:N,rad2deg(thetaTrue),'k','LineWidth',1.6); hold on;
plot(1:N,rad2deg(thetaOne),'Color',[0.25 0.55 0.95], ...
    'LineStyle',':','LineWidth',1.5);
plot(1:N,rad2deg(thetaLin),'b--','LineWidth',1.4);
plot(1:N,rad2deg(thetaBil),'r-.','LineWidth',1.4);
xlabel('prediction step'); ylabel('theta [deg]'); grid on;
exportgraphics(fig1,result_file(resultsDir,'prediction_validation',resultSuffix,'png'),'Resolution',180);

%% Closed-loop figures
nominalPlot=truncate_to_impact(nominal);
stressPlot=truncate_to_impact(stress);
fig2=figure('Color','w','Position',[100 80 980 820]);
subplot(4,1,1); plot(p.Rscale*nominalPlot.x(1,:),p.Rscale*nominalPlot.x(2,:), ...
    'b','LineWidth',1.7); hold on;
plot(p.Rscale*stressPlot.x(1,:),p.Rscale*stressPlot.x(2,:),'r--','LineWidth',1.5);
refLine=impact_reference_path(p);
plot(refLine(1,:),refLine(2,:),'k:','LineWidth',1.1);
plot(0,0,'ko','MarkerFaceColor','k'); axis equal; grid on;
xlabel('r_x [m]'); ylabel('r_y [m]'); legend('nominal','stress','time-angle reference','target');
subplot(4,1,2); plot(nominalPlot.time_s,nominalPlot.range_m,'b','LineWidth',1.6); hold on;
plot(stressPlot.time_s,stressPlot.range_m,'r--','LineWidth',1.5);
yline(p.captureRadius,'k:'); xline(p.impactTime,'k-.'); ylabel('range [m]'); grid on;
subplot(4,1,3); plot(nominalPlot.time_s,rad2deg(nominalPlot.x(3,:)),'b','LineWidth',1.5); hold on;
plot(stressPlot.time_s,rad2deg(stressPlot.x(3,:)),'r--','LineWidth',1.4);
yline(rad2deg(p.impactGamma),'k:'); xline(p.impactTime,'k-.');
ylabel('gamma [deg]'); grid on;
subplot(4,1,4); plot(nominalPlot.time_s(1:end-1),p.amax*nominalPlot.uActual, ...
    'b','LineWidth',1.5); hold on;
plot(stressPlot.time_s(1:end-1),p.amax*stressPlot.uActual,'r--','LineWidth',1.3);
stairs(nominalPlot.time_s(1:end-1),p.amax*nominalPlot.u,'Color',[0.2 0.55 1], ...
    'LineStyle',':','LineWidth',0.9);
yline(p.amax,'k:'); yline(-p.amax,'k:'); xlabel('time [s]');
ylabel('A [m/s^2]'); legend('nominal actual','stress actual','nominal command');
grid on;
exportgraphics(fig2,result_file(resultsDir,'closed_loop',resultSuffix,'png'),'Resolution',180);

if runControllerComparison
    figCmp=plot_guidance_comparison(comparison,p);
    exportgraphics(figCmp,result_file(resultsDir,'closed_loop_guidance_comparison',resultSuffix,'png'), ...
        'Resolution',180);
    write_guidance_comparison_csv(comparison,resultsDir,resultSuffix);
end

fig3=figure('Color','w','Position',[100 100 920 500]);
subplot(2,1,1); plot(nominal.time_s(1:end-1),nominal.xi,'LineWidth',1.5); hold on;
plot(stress.time_s(1:end-1),stress.xi,'--','LineWidth',1.4);
ylabel('xi'); legend('nominal','stress'); grid on;
subplot(2,1,2); semilogy(nominal.time_s(1:end-1),nominal.predictionError+1e-12,'LineWidth',1.4); hold on;
semilogy(stress.time_s(1:end-1),stress.predictionError+1e-12,'--','LineWidth',1.4);
xlabel('time [s]'); ylabel('prediction error'); grid on;
exportgraphics(fig3,result_file(resultsDir,'interpolation_error',resultSuffix,'png'),'Resolution',180);

%% Save outputs
metrics.maxBilinearConsistencyError=maxBilinearError;
metrics.oneStepNRMSE=oneNRMSE;
metrics.oneStepRollingNRMSE=oneRollingNRMSE;
metrics.multiStepNRMSE=multiNRMSE;
metrics.bilinearRollingNRMSE=bilinearNRMSE;
metrics.oneStepResidual95=oneResidual95;
metrics.tubeBound99=tubeBound99;
metrics.linearTubeBound99=linearTubeBound99;
metrics.terminalTol=ctrl.terminalTol;
metrics.terminalTightening=ctrl.terminalTightening;
metrics.terminalTolTight=ctrl.terminalTolTight;
metrics.nominal=nominal.metrics;
metrics.stress=stress.metrics;
metrics.comparison.bilinear=comparison.bilinear.metrics;
if runControllerComparison
    metrics.comparison.linear=comparison.linear.metrics;
    metrics.comparison.nmpc=comparison.nmpc.metrics;
end
if isempty(resultSuffix)
    save(fullfile(resultsDir,'stationary_target_results.mat'), ...
        'A','B','Ablin','B0','B1','C','Theta','metrics','nominal','stress', ...
        'comparison','p','ctrl');
else
    save(result_file(resultsDir,'stationary_target_results',resultSuffix,'mat'), ...
        'A','B','Ablin','B0','B1','C','Theta','metrics','nominal','stress', ...
        'comparison','p','ctrl');
end

fid=fopen(result_file(resultsDir,'metrics',resultSuffix,'txt'),'w');
fprintf(fid,'Stationary-target Koopman guidance validation\n');
fprintf(fid,'max_bilinear_consistency_error=%.12e\n',maxBilinearError);
fprintf(fid,'one_step_nrmse='); fprintf(fid,' %.8e',oneNRMSE); fprintf(fid,'\n');
fprintf(fid,'one_step_rolling_nrmse='); fprintf(fid,' %.8e',oneRollingNRMSE); fprintf(fid,'\n');
fprintf(fid,'multi_step_nrmse='); fprintf(fid,' %.8e',multiNRMSE); fprintf(fid,'\n');
fprintf(fid,'bilinear_rolling_nrmse='); fprintf(fid,' %.8e',bilinearNRMSE); fprintf(fid,'\n');
fprintf(fid,'one_step_residual95='); fprintf(fid,' %.8e',oneResidual95); fprintf(fid,'\n');
fprintf(fid,'tube_bound99_terminal='); fprintf(fid,' %.8e',tubeBound99(:,end)); fprintf(fid,'\n');
fprintf(fid,'terminal_tol='); fprintf(fid,' %.8e',ctrl.terminalTol); fprintf(fid,'\n');
fprintf(fid,'terminal_tightening='); fprintf(fid,' %.8e',ctrl.terminalTightening); fprintf(fid,'\n');
fprintf(fid,'terminal_tol_tight='); fprintf(fid,' %.8e',ctrl.terminalTolTight); fprintf(fid,'\n');
write_case(fid,'nominal',nominal.metrics);
write_case(fid,'stress',stress.metrics);
write_case(fid,'comparison_bilinear_koopman_mpc',comparison.bilinear.metrics);
if runControllerComparison
    write_case(fid,'comparison_linear_koopman_mpc',comparison.linear.metrics);
    write_case(fid,'comparison_nmpc',comparison.nmpc.metrics);
end
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
        if isfield(p,'trainingDataMode') && strcmp(p.trainingDataMode,'largeAngle')
            ef=[cos(p.impactGamma);sin(p.impactGamma)];
            nf=[-sin(p.impactGamma);cos(p.impactGamma)];
            sAlong=(0.08+1.25*rand)*p.Rscale;
            yCross=(-0.85+1.70*rand)*p.Rscale;
            r=(sAlong*ef+yCross*nf)/p.Rscale;
            theta=deg2rad(-170+340*rand);
            gamma=wrap_angle(p.impactGamma+theta);
            ua=-0.7+1.4*rand;
            state(:,1)=[r(1);r(2);gamma;ua];
            raw=repelem(2*rand(1,ceil(nSteps/5))-1,5);
            harmonic=0.55*sin(0.045*(1:nSteps)+2*pi*rand)+ ...
                0.30*cos(0.11*(1:nSteps)+2*pi*rand);
            input=filter(0.45,[1 -0.55],raw(1:nSteps))+harmonic;
        else
            rx=0.35+0.95*rand;
            ry=-0.45+0.9*rand;
            lambda=atan2(ry,rx);
            state(:,1)=[rx;ry;lambda-0.55+1.1*rand;0];
            raw=repelem(2*rand(1,ceil(nSteps/7))-1,7);
            input=filter(0.3,[1 -0.7],raw(1:nSteps));
        end
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
    xtRaw=time_to_go_state(x,p);
    tau=xtRaw(1); y=xtRaw(2); th=xtRaw(3);
    ua=x(4);
    if isfield(p,'angleStateMode') && strcmp(p.angleStateMode,'sincos')
        cth=cos(th); sth=sin(th);
        z=[1;tau;y;sth;cth;ua;tau*cth;tau*sth; ...
            y*cth;y*sth;ua*cth;ua*sth;sth^2;cth^2];
    else
        z=[1;tau;y;th;ua;cos(th);sin(th);tau*cos(th);tau*sin(th); ...
            y*cos(th);y*sin(th);th^2;ua*cos(th);ua*sin(th)];
    end
end

function xt=guidance_error_state(x,p)
    xtRaw=time_to_go_state(x,p);
    if isfield(p,'angleStateMode') && strcmp(p.angleStateMode,'sincos')
        th=xtRaw(3);
        xt=[xtRaw(1);xtRaw(2);sin(th);cos(th)];
    else
        xt=xtRaw;
    end
end

function theta=prediction_theta(C,Z,p)
    X=C*Z;
    if isfield(p,'angleStateMode') && strcmp(p.angleStateMode,'sincos')
        theta=atan2(X(3,:),X(4,:));
    else
        theta=X(3,:);
    end
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

function Zfuture=linear_predict_dataset(Z0,Useq,A,B,N,nz)
    nCases=size(Z0,2);
    Zfuture=zeros(nz*N,nCases);
    for cidx=1:nCases
        z=Z0(:,cidx);
        for h=1:N
            z=A*z+B*Useq(h,cidx);
            z(1)=1;
            Zfuture((h-1)*nz+(1:nz),cidx)=z;
        end
    end
end

function out=run_closed_loop(initial,p,c,stress,angleOnlyMode)
    if angleOnlyMode
        Kmax=round(p.angleOnlyMaxTime/p.Ts);
    else
        Kmax=round((p.impactTime+p.postImpactWindow)/p.Ts);
    end
    x=zeros(numel(initial),Kmax+1); x(:,1)=initial;
    u=zeros(1,Kmax); xi=zeros(1,Kmax); err=zeros(1,Kmax); flags=zeros(1,Kmax);
    terminalSlack=zeros(c.nSlack,Kmax);
    zPrevious=lift_state(x(:,1),p);
    uPrev=0;
    last=Kmax+1;
    alphaHist=zeros(1,Kmax+1); betaHist=zeros(1,Kmax+1);
    alphaHist(1)=c.alphaHat; betaHist(1)=c.betaHat;
    warmU=zeros(c.N,1);
    solveTime=zeros(1,Kmax);
    terminalRefinementUsed=false(1,Kmax);
    for k=1:Kmax
        zMeasured=lift_state(x(:,k),p);
        err(k)=norm(zPrevious-zMeasured);
        tic;
        useTerminalRefinement=terminal_refinement_active(c,k,angleOnlyMode);
        if useTerminalRefinement
            [useq,xi(k),Zpred,flags(k),terminalSlack(:,k)]= ...
                nmpc_guidance(x(:,k),p,c,k,uPrev,warmU);
            terminalRefinementUsed(k)=true;
        elseif isfield(c,'controllerType') && strcmp(c.controllerType,'linear')
            [useq,xi(k),Zpred,flags(k),terminalSlack(:,k)]= ...
                linear_kdpc(zMeasured,zPrevious,c,k,uPrev);
        elseif isfield(c,'controllerType') && strcmp(c.controllerType,'nmpc')
            [useq,xi(k),Zpred,flags(k),terminalSlack(:,k)]= ...
                nmpc_guidance(x(:,k),p,c,k,uPrev,warmU);
        else
            [useq,xi(k),Zpred,flags(k),terminalSlack(:,k)]= ...
                kdpc(zMeasured,zPrevious,c,k,uPrev);
        end
        solveTime(k)=toc;
        u(k)=useq(1); uPrev=u(k); zPrevious=Zpred(1:c.nz);
        warmU=[useq(2:end);useq(end)];
        x(:,k+1)=plant_step(x(:,k),u(k),p,stress);
        zNextMeasured=lift_state(x(:,k+1),p);
        if ~isfield(c,'controllerType') || strcmp(c.controllerType,'bilinear')
            c=update_adaptive_estimate(c,zMeasured,zNextMeasured,u(k));
        end
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
    out.solveTime_s=solveTime(1:last-1);
    out.terminalRefinementUsed=terminalRefinementUsed(1:last-1);
    out.alphaHat=alphaHist(1:last); out.betaHat=betaHist(1:last);
    out.time_s=(0:last-1)*p.Ts;
    out.range_m=p.Rscale*hypot(out.x(1,:),out.x(2,:));
    out.lookAngle_rad=wrap_angle(out.x(3,:)-atan2(out.x(2,:),out.x(1,:)));
    out.lookAngle_deg=rad2deg(out.lookAngle_rad);
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
    out.impactSatisfied=out.impactRange_m<=p.captureRadius;
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
    out.metrics.maxCommandDelta_mps2=max(abs(diff([0,out.u])))*p.amax;
    out.metrics.maxAccelerationDelta_mps2=max(abs(diff([0,out.uActual])))*p.amax;
    out.metrics.qpFailures=out.qpFailures;
    out.metrics.terminalRefinementSteps=sum(out.terminalRefinementUsed);
    out.metrics.meanSolveTime_ms=1000*mean(out.solveTime_s);
    out.metrics.maxSolveTime_ms=1000*max(out.solveTime_s);
    if isfield(c,'fovMinRange')
        lookMask=out.time_s<=p.impactTime & out.range_m>=c.fovMinRange;
    else
        lookMask=out.time_s<=p.impactTime;
    end
    if ~any(lookMask)
        lookMask=1:out.impactIndex;
    end
    out.metrics.maxLookAngle_deg=max(abs(out.lookAngle_deg(lookMask)));
    if isfield(c,'enableFov') && c.enableFov
        out.metrics.fovLimit_deg=rad2deg(c.fovMax);
        out.metrics.fovViolation_deg=max(0,out.metrics.maxLookAngle_deg-rad2deg(c.fovMax));
    else
        out.metrics.fovLimit_deg=inf;
        out.metrics.fovViolation_deg=0;
    end
    out.metrics.meanXi=mean(out.xi);
    out.metrics.maxPredictionError=max(out.predictionError);
    out.metrics.maxTerminalSlack=max(out.terminalSlack,[],2);
    out.metrics.finalAlphaHat=out.alphaHat(end);
    out.metrics.finalBetaHat=out.betaHat(end);
end

function out=truncate_to_impact(out)
    idx=max(1,min(out.impactIndex,numel(out.time_s)));
    out.x=out.x(:,1:idx);
    out.time_s=out.time_s(1:idx);
    out.range_m=out.range_m(1:idx);
    out.lookAngle_rad=out.lookAngle_rad(1:idx);
    out.lookAngle_deg=out.lookAngle_deg(1:idx);
    uIdx=max(0,idx-1);
    out.u=out.u(1:uIdx);
    out.uActual=out.uActual(1:uIdx);
    out.xi=out.xi(1:uIdx);
    out.predictionError=out.predictionError(1:uIdx);
    out.exitflags=out.exitflags(1:uIdx);
    out.solveTime_s=out.solveTime_s(1:uIdx);
    out.terminalRefinementUsed=out.terminalRefinementUsed(1:uIdx);
    out.terminalSlack=out.terminalSlack(:,1:uIdx);
    out.alphaHat=out.alphaHat(1:idx);
    out.betaHat=out.betaHat(1:idx);
    out.impactIndex=idx;
end

function active=terminal_refinement_active(c,k,angleOnlyMode)
    active=false;
    if angleOnlyMode
        return;
    end
    if ~isfield(c,'enableTerminalRefinement') || ~c.enableTerminalRefinement
        return;
    end
    timeNow=(k-1)*c.Ts;
    timeToImpact=c.impactTime-timeNow;
    active=timeToImpact>0 && timeToImpact<=c.terminalRefinementWindow;
end

function [U,xi,Zpred,exitflag,terminalSlack]=linear_kdpc(zMeasured,zPrevious,c,k,uPrev)
    delta=zPrevious-zMeasured;
    offsetZ=c.ThetaZ*zMeasured;
    mapZ=[c.ThetaU,c.ThetaZ*delta];
    [U,xi,terminalSlack,exitflag]=solve_kdpc_qp(offsetZ,mapZ, ...
        zMeasured,delta,c,k,uPrev);
    Zpred=offsetZ+mapZ*[U;xi];
end

function [U,xi,Zpred,exitflag,terminalSlack]=kdpc(zMeasured,zPrevious,c,k,uPrev)
    delta=zPrevious-zMeasured;
    U=zeros(c.N,1); xi=0; terminalSlack=zeros(c.nSlack,1); exitflag=1;
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

function [U,xi,Zpred,exitflag,terminalSlack]=nmpc_guidance(xMeasured,p,c,k,uPrev,warmU)
    xi=0;
    nSlack=c.nSlack;
    nMove=nmpc_num_move_vars(c);
    blockIdx=round(linspace(1,c.N,nMove));
    x0=[warmU(blockIdx);zeros(nSlack,1)];
    lb=[-ones(nMove,1);zeros(nSlack,1)];
    ub=[ones(nMove,1);inf(nSlack,1)];
    obj=@(v)nmpc_objective(v,xMeasured,p,c,k,uPrev);
    nonlcon=@(v)nmpc_constraints(v,xMeasured,p,c,k);
    [sol,~,exitflag]=fmincon(obj,x0,[],[],[],[],lb,ub,nonlcon,c.fminconOptions);
    if isempty(sol)
        sol=x0;
        exitflag=-99;
    end
    U=nmpc_controls_from_vars(sol,c);
    terminalSlack=sol(nMove+1:end);
    Zpred=nmpc_lifted_rollout(xMeasured,U,p,c);
end

function cost=nmpc_objective(v,xMeasured,p,c,k,uPrev)
    nMove=nmpc_num_move_vars(c);
    U=nmpc_controls_from_vars(v,c);
    slack=v(nMove+1:end);
    [~,Xerr]=nmpc_rollout(xMeasured,U,p,c);
    zMeasured=lift_state(xMeasured,p);
    xRef=impact_reference_stack(k,c,zMeasured);
    e=Xerr(:)-xRef;
    [Hd,fd]=delta_u_penalty(c.N,c.Rd,uPrev);
    Qcost=prediction_cost_matrix(c,k);
    cost=e.'*Qcost*e+U.'*c.Rbar*U+U.'*Hd*U+2*fd.'*U+ ...
        slack.'*c.slackPenalty*slack;
end

function [cineq,ceq]=nmpc_constraints(v,xMeasured,p,c,k)
    nMove=nmpc_num_move_vars(c);
    U=nmpc_controls_from_vars(v,c);
    slack=v(nMove+1:end);
    [~,Xerr]=nmpc_rollout(xMeasured,U,p,c);
    zMeasured=lift_state(xMeasured,p);
    xRef=reshape(impact_reference_stack(k,c,zMeasured),c.nx,c.N);
    theta=Xerr(3,:);
    cineq=[theta.'-(c.gammaMax-c.impactGamma); ...
        -theta.'-(c.gammaMax+c.impactGamma)];
    if isfield(c,'enableFov') && c.enableFov
        for h=1:c.N
            tauRef=max(abs(xRef(1,h)),c.fovMinRange/(c.Vref*c.impactTime));
            yToLos=c.Rscale/(c.Vref*c.impactTime*tauRef);
            if strcmp(c.angleStateMode,'sincos')
                thetaApprox=atan2(Xerr(3,h),Xerr(4,h));
            else
                thetaApprox=Xerr(3,h);
            end
            sigmaApprox=thetaApprox-yToLos*Xerr(2,h);
            cineq=[cineq;sigmaApprox-c.fovMax;-sigmaApprox-c.fovMax];
        end
    end
    if c.angleOnlyMode
        hImpact=c.N;
    else
        currentTime=(k-1)*c.Ts;
        hImpact=round((c.impactTime-currentTime)/c.Ts);
    end
    if hImpact>=1 && hImpact<=c.N
        tol=terminal_tolerance_at_step(c,hImpact);
        if c.angleOnlyMode
            if isfield(c,'angleOnlyTerminalTau') && c.angleOnlyTerminalTau
                terminalDims=1:c.nx;
            else
                terminalDims=2:c.nx;
            end
        else
            terminalDims=1:c.nx;
        end
        errTerm=Xerr(:,hImpact)-xRef(:,hImpact);
        for j=terminalDims
            cineq=[cineq;errTerm(j)-tol(j)-slack(j); ...
                -errTerm(j)-tol(j)-slack(j)];
        end
        yNorm=errTerm(2)/tol(2);
        thNorm=errTerm(3)/tol(3);
        cineq=[cineq; ...
            yNorm+thNorm-1-slack(2)/tol(2)-slack(3)/tol(3); ...
            yNorm-thNorm-1-slack(2)/tol(2)-slack(3)/tol(3); ...
            -yNorm+thNorm-1-slack(2)/tol(2)-slack(3)/tol(3); ...
            -yNorm-thNorm-1-slack(2)/tol(2)-slack(3)/tol(3)];
    end
    ceq=[];
end

function nMove=nmpc_num_move_vars(c)
    if isfield(c,'nmpcMoveBlocks')
        nMove=min(c.N,c.nmpcMoveBlocks);
    else
        nMove=c.N;
    end
end

function U=nmpc_controls_from_vars(v,c)
    nMove=nmpc_num_move_vars(c);
    uBlocks=v(1:nMove);
    if nMove==c.N
        U=uBlocks;
        return;
    end
    blockId=ceil((1:c.N)'*nMove/c.N);
    blockId=max(1,min(nMove,blockId));
    U=uBlocks(blockId);
end

function [Xstate,Xerr]=nmpc_rollout(xMeasured,U,p,c)
    Xstate=zeros(numel(xMeasured),c.N);
    Xerr=zeros(c.nx,c.N);
    x=xMeasured;
    for i=1:c.N
        x=plant_step(x,U(i),p,false);
        Xstate(:,i)=x;
        Xerr(:,i)=guidance_error_state(x,p);
    end
end

function Zpred=nmpc_lifted_rollout(xMeasured,U,p,c)
    Zpred=zeros(c.nz*c.N,1);
    x=xMeasured;
    for i=1:c.N
        x=plant_step(x,U(i),p,false);
        Zpred((i-1)*c.nz+(1:c.nz))=lift_state(x,p);
    end
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
    if isfield(c,'angleStateMode') && strcmp(c.angleStateMode,'sincos')
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

function Qbar=prediction_cost_matrix(c,k)
    if ~isfield(c,'enableTwoStageCost') || ~c.enableTwoStageCost
        Qbar=c.Qxbar;
        if isfield(c,'enableImpactStageCost') && c.enableImpactStageCost
            hImpact=impact_horizon_index(c,k);
            if hImpact>=1 && hImpact<=c.N
                idx=(hImpact-1)*c.nx+(1:c.nx);
                Qbar(idx,idx)=c.QterminalCost;
            end
        end
        return;
    end
    Qbar=zeros(c.nx*c.N);
    hImpact=impact_horizon_index(c,k);
    for h=1:c.N
        t=(k-1+h)*c.Ts;
        timeToImpact=c.impactTime-t;
        if h==hImpact
            Qh=c.QterminalCost;
        elseif timeToImpact<=c.twoStageWindow
            Qh=c.QterminalStage;
        else
            Qh=c.Qprogress;
        end
        idx=(h-1)*c.nx+(1:c.nx);
        Qbar(idx,idx)=Qh;
    end
end

function hImpact=impact_horizon_index(c,k)
    if c.angleOnlyMode
        hImpact=c.N;
    else
        currentTime=(k-1)*c.Ts;
        hImpact=round((c.impactTime-currentTime)/c.Ts);
    end
end

function [U,xi,terminalSlack,exitflag]=solve_kdpc_qp(offsetZ,mapZ, ...
    zMeasured,delta,c,k,uPrev)
    [Umap,nControl]=control_block_map(c);
    baseMap=blkdiag(Umap,1);
    offsetX=c.Cbar*offsetZ; mapX=c.Cbar*mapZ*baseMap;
    xRef=impact_reference_stack(k,c,zMeasured);
    Qcost=prediction_cost_matrix(c,k);
    nBase=nControl+1; nSlack=c.nSlack; nVar=nBase+nSlack;
    H=zeros(nVar); f=zeros(nVar,1);
    H(1:nBase,1:nBase)=2*(mapX'*Qcost*mapX);
    f(1:nBase)=2*(mapX'*Qcost*(offsetX-xRef));
    H(1:nControl,1:nControl)=H(1:nControl,1:nControl)+ ...
        2*(Umap'*c.Rbar*Umap);
    [Hd,fd]=delta_u_penalty(c.N,c.Rd,uPrev);
    H(1:nControl,1:nControl)=H(1:nControl,1:nControl)+ ...
        2*(Umap'*Hd*Umap);
    f(1:nControl)=f(1:nControl)+2*(Umap'*fd);
    H(nBase,nBase)=H(nBase,nBase)+2*(delta'*c.Q0*delta+c.lambda*(delta'*delta));
    f(nBase)=f(nBase)+2*delta'*c.Q0*zMeasured;
    H(nBase+1:end,nBase+1:end)=H(nBase+1:end,nBase+1:end)+ ...
        2*c.slackPenalty;
    H=0.5*(H+H')+1e-9*eye(nVar);
    if strcmp(c.angleStateMode,'sincos')
        Aineq=zeros(0,nVar);
        bineq=zeros(0,1);
    else
        thetaSel=kron(eye(c.N),[0 0 1]);
        Aineq=[thetaSel*mapX,zeros(c.N,nSlack); ...
            -thetaSel*mapX,zeros(c.N,nSlack)];
        bineq=[(c.gammaMax-c.impactGamma)*ones(c.N,1)-thetaSel*offsetX; ...
            (c.gammaMax+c.impactGamma)*ones(c.N,1)+thetaSel*offsetX];
    end
    if isfield(c,'enableFov') && c.enableFov
        xRefMat=reshape(xRef,c.nx,c.N);
        for h=1:c.N
            tauRef=max(abs(xRefMat(1,h)),c.fovMinRange/(c.Vref*c.impactTime));
            yToLos=c.Rscale/(c.Vref*c.impactTime*tauRef);
            idx=(h-1)*c.nx+(1:c.nx);
            if strcmp(c.angleStateMode,'sincos')
                sin0=offsetX(idx(3));
                cos0=offsetX(idx(4));
                normSq=max(sin0^2+cos0^2,1e-6);
                theta0=atan2(sin0,cos0);
                dThetaDsin=cos0/normSq;
                dThetaDcos=-sin0/normSq;
                fovRow=dThetaDsin*mapX(idx(3),:)+ ...
                    dThetaDcos*mapX(idx(4),:)-yToLos*mapX(idx(2),:);
                fovOff=theta0-yToLos*offsetX(idx(2));
            else
                fovRow=mapX(idx(3),:)-yToLos*mapX(idx(2),:);
                fovOff=offsetX(idx(3))-yToLos*offsetX(idx(2));
            end
            Aineq=[Aineq;fovRow,zeros(1,nSlack); ...
                -fovRow,zeros(1,nSlack)];
            bineq=[bineq;c.fovMax-fovOff;c.fovMax+fovOff];
        end
    end
    if isfield(c,'duMax') && isfinite(c.duMax)
        [Adu,bdu]=delta_u_hard_constraints( ...
            c.N,c.duMax,uPrev,nVar,Umap,nControl);
        Aineq=[Aineq;Adu];
        bineq=[bineq;bdu];
    end
    if isfield(c,'deltaAccelMax') && isfinite(c.deltaAccelMax)
        [Aacc,bacc]=delta_accel_hard_constraints( ...
            c.N,c.deltaAccelMax,uPrev,c.amax,nVar,Umap,nControl);
        Aineq=[Aineq;Aacc];
        bineq=[bineq;bacc];
    end
    if c.angleOnlyMode
        hImpact=c.N;
    else
        currentTime=(k-1)*c.Ts;
        hImpact=round((c.impactTime-currentTime)/c.Ts);
    end
    if hImpact>=1 && hImpact<=c.N
        idx=(hImpact-1)*c.nx+(1:c.nx);
        tol=terminal_tolerance_at_step(c,hImpact);
        slackCols=eye(nSlack);
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
        angleRow=mapX(idx(3),:)/tol(3);
        yOff=(offsetX(idx(2))-xRef(idx(2)))/tol(2);
        angleOff=(offsetX(idx(3))-xRef(idx(3)))/tol(3);
        for sy=[-1,1]
            for st=[-1,1]
                row=zeros(1,nVar);
                row(1:nBase)=sy*yRow+st*angleRow;
                row(nBase+2)=-1/tol(2);
                row(nBase+3)=-1/tol(3);
                Aineq=[Aineq;row];
                bineq=[bineq;1-sy*yOff-st*angleOff];
            end
        end
    end
    lb=[-ones(nControl,1);0;zeros(nSlack,1)];
    ub=[ones(nControl,1);1;inf(nSlack,1)];
    [sol,~,exitflag]=quadprog(H,f,Aineq,bineq,[],[],lb,ub,[],c.options);
    if exitflag<=0 || isempty(sol), sol=zeros(nVar,1); end
    U=Umap*sol(1:nControl);
    xi=sol(nBase);
    terminalSlack=sol(nBase+1:end);
end

function [Umap,nControl]=control_block_map(c)
    blockSize=1;
    if isfield(c,'moveBlockSize')
        blockSize=max(1,round(c.moveBlockSize));
    end
    nControl=ceil(c.N/blockSize);
    Umap=zeros(c.N,nControl);
    for j=1:nControl
        first=(j-1)*blockSize+1;
        last=min(j*blockSize,c.N);
        Umap(first:last,j)=1;
    end
end

function [Adu,bdu]=delta_u_hard_constraints( ...
    N,duMax,uPrev,nVar,Umap,nControl)
    D=eye(N);
    for i=2:N
        D(i,i-1)=-1;
    end
    offset=zeros(N,1);
    offset(1)=uPrev;
    Dblock=D*Umap;
    Adu=[Dblock,zeros(N,nVar-nControl);-Dblock,zeros(N,nVar-nControl)];
    bdu=[duMax+offset;duMax-offset];
end

function [Aacc,bacc]=delta_accel_hard_constraints( ...
    N,deltaAccelMax,uPrev,amax,nVar,Umap,nControl)
    duMax=max(deltaAccelMax,0)/amax;
    [Aacc,bacc]=delta_u_hard_constraints( ...
        N,duMax,uPrev,nVar,Umap,nControl);
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
            tauProgress=max(xNow(1)-h*c.Ts/c.impactTime,0);
            if isfield(c,'angleOnlyZeroTauRef') && c.angleOnlyZeroTauRef
                tauRef=0;
            else
                tauRef=tauProgress;
            end
            [yRef,thetaRef]=angle_only_joint_reference(tauProgress,c);
            if strcmp(c.angleStateMode,'sincos')
                xRef(idx)=[tauRef;yRef;sin(thetaRef);cos(thetaRef)];
            else
                xRef(idx)=[tauRef;yRef;thetaRef];
            end
        else
            tgo=max(c.impactTime-t,0);
            if strcmp(c.angleStateMode,'sincos')
                xRef(idx)=[tgo/c.impactTime;0;0;1];
            else
                xRef(idx)=[tgo/c.impactTime;0;0];
            end
        end
    end
end

function [yRef,thetaRef]=angle_only_joint_reference(tauRef,c)
    yRef=0;
    thetaRef=0;
    if ~isfield(c,'enableYReferenceShape') || ~c.enableYReferenceShape
        return;
    end
    tauScale=max(c.yRefTauScale,1e-6);
    phase=min(max(tauRef/tauScale,0),1);
    yRef=c.yRefSign*c.yRefAmplitude*sin(pi*phase);
    if isfield(c,'enableJointReference') && c.enableJointReference
        if tauRef/tauScale>=0 && tauRef/tauScale<=1
            dyDtau=c.yRefSign*c.yRefAmplitude*pi/tauScale*cos(pi*phase);
        else
            dyDtau=0;
        end
        dyDs=(c.Rscale/(c.Vref*c.impactTime))*dyDtau;
        thetaRef=atan(dyDs);
    end
end

function ref=impact_reference_path(p)
    t=linspace(0,p.impactTime,120);
    ef=[cos(p.impactGamma);sin(p.impactGamma)];
    ref=p.Vnom*(p.impactTime-t).*ef;
end

function fig=plot_guidance_comparison(comparison,p)
    methods={'linear','bilinear','nmpc'};
    labels={'Linear Koopman-MPC','Bilinear Koopman-MPC','NMPC'};
    colors=[0.05 0.35 0.75;0.82 0.18 0.16;0.12 0.52 0.25];
    styles={'--','-','-.'};
    fig=figure('Color','w','Position',[90 60 1060 820]);
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

    nexttile; hold on; grid on; axis equal;
    for i=1:numel(methods)
        out=comparison.(methods{i});
        plot(p.Rscale*out.x(1,:),p.Rscale*out.x(2,:), ...
            'Color',colors(i,:),'LineStyle',styles{i},'LineWidth',1.7, ...
            'DisplayName',labels{i});
        plot(p.Rscale*out.x(1,out.impactIndex), ...
            p.Rscale*out.x(2,out.impactIndex),'o', ...
            'Color',colors(i,:),'MarkerFaceColor',colors(i,:), ...
            'HandleVisibility','off');
    end
    refLine=impact_reference_path(p);
    plot(refLine(1,:),refLine(2,:),'k:','LineWidth',1.0, ...
        'DisplayName','reference');
    plot(0,0,'ko','MarkerFaceColor','k','HandleVisibility','off');
    xlabel('r_x [m]'); ylabel('r_y [m]'); title('Trajectory');
    legend('Location','southoutside','NumColumns',2);

    nexttile; hold on; grid on;
    for i=1:numel(methods)
        out=comparison.(methods{i});
        plot(out.time_s,out.range_m,'Color',colors(i,:), ...
            'LineStyle',styles{i},'LineWidth',1.5);
        plot(out.impactTime_s,out.impactRange_m,'o', ...
            'Color',colors(i,:),'MarkerFaceColor',colors(i,:));
    end
    yline(p.captureRadius,'k--'); xline(p.impactTime,'k-.');
    xlabel('time [s]'); ylabel('range [m]'); title('Range');

    nexttile; hold on; grid on;
    for i=1:numel(methods)
        out=comparison.(methods{i});
        plot(out.time_s,rad2deg(out.x(3,:)),'Color',colors(i,:), ...
            'LineStyle',styles{i},'LineWidth',1.5);
    end
    yline(rad2deg(p.impactGamma),'k:'); xline(p.impactTime,'k-.');
    xlabel('time [s]'); ylabel('gamma [deg]'); title('Flight-path angle');

    nexttile; hold on; grid on;
    for i=1:numel(methods)
        out=comparison.(methods{i});
        plot(out.time_s(1:end-1),p.amax*out.uActual, ...
            'Color',colors(i,:),'LineStyle',styles{i},'LineWidth',1.4);
    end
    yline(p.amax,'k--'); yline(-p.amax,'k--');
    xlabel('time [s]'); ylabel('A [m/s^2]');
    title('Actual lateral acceleration');
    sgtitle('Closed-loop guidance comparison');
end

function write_guidance_comparison_csv(comparison,resultsDir,suffix)
    path=result_file(resultsDir,'closed_loop_guidance_comparison_summary',suffix,'csv');
    fid=fopen(path,'w');
    fprintf(fid,['method,satisfied,impact_time_s,impact_time_error_s,miss_m,', ...
        'angle_error_deg,max_command_mps2,max_accel_mps2,solver_failures,', ...
        'mean_solve_ms,max_solve_ms\n']);
    write_comparison_row(fid,'Linear Koopman-MPC',comparison.linear.metrics);
    write_comparison_row(fid,'Bilinear Koopman-MPC',comparison.bilinear.metrics);
    write_comparison_row(fid,'NMPC',comparison.nmpc.metrics);
    fclose(fid);
end

function write_comparison_row(fid,name,m)
    fprintf(fid,'"%s",%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%d,%.8f,%.8f\n', ...
        name,m.impactSatisfied,m.impactTime_s,m.impactTimeError_s, ...
        m.impactRange_m,m.impactHeadingError_deg,m.maxCommand_mps2, ...
        m.maxAcceleration_mps2,m.qpFailures,m.meanSolveTime_ms, ...
        m.maxSolveTime_ms);
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
    fprintf(fid,'%s_max_command_delta_mps2=%.8f\n',name,m.maxCommandDelta_mps2);
    fprintf(fid,'%s_max_acceleration_delta_mps2=%.8f\n',name,m.maxAccelerationDelta_mps2);
    fprintf(fid,'%s_qp_failures=%d\n',name,m.qpFailures);
    fprintf(fid,'%s_mean_solve_time_ms=%.8f\n',name,m.meanSolveTime_ms);
    fprintf(fid,'%s_max_solve_time_ms=%.8f\n',name,m.maxSolveTime_ms);
    fprintf(fid,'%s_max_look_angle_deg=%.8f\n',name,m.maxLookAngle_deg);
    fprintf(fid,'%s_fov_limit_deg=%.8f\n',name,m.fovLimit_deg);
    fprintf(fid,'%s_fov_violation_deg=%.8f\n',name,m.fovViolation_deg);
    fprintf(fid,'%s_mean_xi=%.8f\n',name,m.meanXi);
    fprintf(fid,'%s_max_prediction_error=%.8f\n',name,m.maxPredictionError);
    fprintf(fid,'%s_max_terminal_slack=',name);
    fprintf(fid,' %.8e',m.maxTerminalSlack);
    fprintf(fid,'\n');
    fprintf(fid,'%s_final_alpha_hat=%.8f\n',name,m.finalAlphaHat);
    fprintf(fid,'%s_final_beta_hat=%.8f\n',name,m.finalBetaHat);
end

function path=result_file(resultsDir,baseName,suffix,ext)
    if isempty(suffix)
        fileName=sprintf('%s.%s',baseName,ext);
    else
        fileName=sprintf('%s_%s.%s',baseName,suffix,ext);
    end
    path=fullfile(resultsDir,fileName);
end
