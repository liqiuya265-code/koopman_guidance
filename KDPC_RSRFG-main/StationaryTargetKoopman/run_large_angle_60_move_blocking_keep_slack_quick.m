%% Quick gamma_f=60 deg move-blocking check without Rd/xi penalties but with slack penalty
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

skipClearOverride=true;
skipControllerComparisonOverride=true;
angleOnlyModeOverride=true;
impactGammaDegOverride=60;
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
qxOverride=[800,4000,200,120];
qterminalOverride=[250000,1000000,35000,25000];
slackPenaltyOverride=[1e8,5e8,1e6,1e6];
angleOnlyTerminalTauOverride=true;
angleOnlyZeroTauRefOverride=true;
rbarScaleOverride=0.03;
deltaAccelMaxOverride=10;

rdOverride=0;
q0ScaleOverride=0;
xiPenaltyOverride=0;

resultSuffixOverride='large60_move_block5_keep_slack_quick';
run(fullfile(baseDir,'run_stationary_target_demo.m'));

summaryPath=fullfile(resultsDir,'large_angle_60_move_blocking_keep_slack_quick_summary.csv');
fid=fopen(summaryPath,'w');
fprintf(fid,['case,horizon_s,move_block_size,seq_iterations,', ...
    'delta_accel_max,rd,q0_scale,xi_penalty,slack_penalty_sum,', ...
    'nominal_ok,nominal_time_s,nominal_miss_m,nominal_angle_error_deg,', ...
    'nominal_max_a,nominal_max_delta_a,stress_ok,stress_time_s,', ...
    'stress_miss_m,stress_angle_error_deg,stress_max_a,', ...
    'stress_max_delta_a,nominal_qp_failures,stress_qp_failures\n']);
fprintf(fid,['"large-angle-60deg-move-blocking-keep-slack-quick",', ...
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
    'large_angle_60_move_blocking_keep_slack_quick_output.png');
if exist(closedLoopPath,'file')
    copyfile(closedLoopPath,quickOutputPath);
    fprintf('Keep-slack quick output image saved to %s\n',quickOutputPath);
end

fprintf('\nKeep-slack quick summary saved to %s\n',summaryPath);
