%% Quick gamma_f=60 deg move-blocking check with balanced QP penalties
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

skipClearOverride=true;
skipControllerComparisonOverride=true;
angleOnlyModeOverride=true;
impactGammaDegOverride=120;
initialGammaDegOverride=-30;
initialPositionMetersOverride=[10000;0];
angleOnlyMaxTimeOverride=45;
gammaMaxDegOverride=150;
trainingDataModeOverride='largeAngle';
angleStateModeOverride='sincos';
enableAdaptiveOverride=false;
horizonOverride=50;
moveBlockSizeOverride=5;
seqIterationsOverride=2;
nStepsOverride=180;
nTrainTrajOverride=500;
nTestTrajOverride=100;
qxOverride=[800,4000,100,80];
qterminalOverride=[500000,2000000,0,0];
angleOnlyTerminalTauOverride=true;
angleOnlyZeroTauRefOverride=true;
disableFovOverride = true;
fovMaxDegOverride =50;
fovMinRangeOverride = 500;
rbarScaleOverride=0.01;
deltaAccelMaxOverride=100;%15.5

% Balanced penalty setting:
% 1) keep position as the dominant terminal objective
% 2) keep angle as a weaker secondary objective
% 3) keep modest control and model-consistency penalties
rdOverride=0.1;
q0ScaleOverride=0.002;
xiPenaltyOverride=1e3;
slackPenaltyOverride=[5e8,1e9,1e5,1e5];

resultSuffixOverride='large60_move_block5_balanced_penalty_quick';
run(fullfile(baseDir,'run_stationary_target_demo.m'));

summaryPath=fullfile(resultsDir,'large_angle_60_move_blocking_balanced_penalty_quick_summary.csv');
fid=fopen(summaryPath,'w');
fprintf(fid,['case,horizon_s,move_block_size,seq_iterations,', ...
    'delta_accel_max,rd,q0_scale,xi_penalty,slack_penalty_sum,', ...
    'nominal_ok,nominal_time_s,nominal_miss_m,nominal_angle_error_deg,', ...
    'nominal_max_a,nominal_max_delta_a,stress_ok,stress_time_s,', ...
    'stress_miss_m,stress_angle_error_deg,stress_max_a,', ...
    'stress_max_delta_a,nominal_qp_failures,stress_qp_failures\n']);
fprintf(fid,['"large-angle-60deg-move-blocking-balanced-penalty-quick",', ...
    '%.8f,%d,%d,%.8f,%.8f,%.8f,%.8f,%.8f,', ...
    '%d,%.8f,%.8f,%.8f,%.8f,%.8f,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%d,%d\n'], ...
    horizonOverride*0.1,moveBlockSizeOverride,seqIterationsOverride, ...
    deltaAccelMaxOverride,rdOverride,q0ScaleOverride,xiPenaltyOverride, ...
    sum(slackPenaltyOverride), ...
    nominal.metrics.impactSatisfied,nominal.metrics.impactTime_s, ...
    nominal.metrics.impactRange_m,nominal.metrics.impactHeadingError_deg, ...
    nominal.metrics.maxAcceleration_mps2, ...
    nominal.metrics.maxAccelerationDelta_mps2, ...
    stress.metrics.impactSatisfied,stress.metrics.impactTime_s, ...
    stress.metrics.impactRange_m,stress.metrics.impactHeadingError_deg, ...
    stress.metrics.maxAcceleration_mps2, ...
    stress.metrics.maxAccelerationDelta_mps2, ...
    nominal.metrics.qpFailures,stress.metrics.qpFailures);
fclose(fid);

closedLoopPath=fullfile(resultsDir, ...
    sprintf('closed_loop_%s.png',resultSuffixOverride));
quickOutputPath=fullfile(resultsDir, ...
    'large_angle_60_move_blocking_balanced_penalty_quick_output.png');
if exist(closedLoopPath,'file')
    copyfile(closedLoopPath,quickOutputPath);
    fprintf('Balanced-penalty quick output image saved to %s\n',quickOutputPath);
else
    warning('Closed-loop image was not found: %s',closedLoopPath);
end

fprintf('\nBalanced-penalty quick summary saved to %s\n',summaryPath);
