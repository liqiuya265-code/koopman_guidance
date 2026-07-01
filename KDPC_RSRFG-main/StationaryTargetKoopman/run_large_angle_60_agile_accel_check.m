%% Gamma_f=60 deg agile check with a looser acceleration-vibration constraint
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
angleOnlyMaxTimeOverride=50;
gammaMaxDegOverride=150;
trainingDataModeOverride='largeAngle';
angleStateModeOverride='sincos';
enableAdaptiveOverride=false;
horizonOverride=100;
nStepsOverride=260;
nTrainTrajOverride=900;
nTestTrajOverride=180;
qxOverride=[500,2500,300,180];
qterminalOverride=[120000,500000,60000,40000];
slackPenaltyOverride=[5e7,2e8,2e6,2e6];
angleOnlyTerminalTauOverride=true;
angleOnlyZeroTauRefOverride=true;
rbarScaleOverride=0.02;
rdOverride=0.2;
deltaAccelMaxOverride=30;
resultSuffixOverride='large60_agile_dA_30_rd_0p2_h100';
run(fullfile(baseDir,'run_stationary_target_demo.m'));

summaryPath=fullfile(resultsDir,'large_angle_60_agile_accel_check_summary.csv');
fid=fopen(summaryPath,'w');
fprintf(fid,['case,delta_accel_max,rd,nominal_ok,nominal_time_s,', ...
    'nominal_miss_m,nominal_angle_error_deg,nominal_max_a,', ...
    'nominal_max_delta_a,stress_ok,stress_time_s,stress_miss_m,', ...
    'stress_angle_error_deg,stress_max_a,stress_max_delta_a,', ...
    'nominal_qp_failures,stress_qp_failures\n']);
fprintf(fid,['"large-angle-60deg-agile",%.8f,%.8f,%d,%.8f,%.8f,%.8f,%.8f,%.8f,', ...
    '%d,%.8f,%.8f,%.8f,%.8f,%.8f,%d,%d\n'], ...
    deltaAccelMaxOverride,rdOverride, ...
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

fprintf('\nGamma_f=60 agile acceleration-vibration summary saved to %s\n',summaryPath);
