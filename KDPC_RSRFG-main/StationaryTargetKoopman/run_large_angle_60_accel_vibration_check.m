%% Gamma_f=60 deg check with hard acceleration-vibration constraint
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
initialPositionMetersOverride=[10000;2000];
angleOnlyMaxTimeOverride=60;
gammaMaxDegOverride=130;
trainingDataModeOverride='largeAngle';
angleStateModeOverride='sincos';
enableAdaptiveOverride=false;
horizonOverride=100;
nStepsOverride=260;
nTrainTrajOverride=900;
nTestTrajOverride=180;
qxOverride=[1200,2500,100,60];
qterminalOverride=[300000,450000,20000,12000];
slackPenaltyOverride=[2e8,2e8,5e5,5e5];
angleOnlyTerminalTauOverride=true;
angleOnlyZeroTauRefOverride=true;
rdOverride=1.5;
deltaAccelMaxOverride=5;
resultSuffixOverride='large60_zero_tau_dA_15_h100';
run(fullfile(baseDir,'run_stationary_target_demo.m'));

summaryPath=fullfile(resultsDir,'large_angle_60_accel_vibration_check_summary.csv');
fid=fopen(summaryPath,'w');
fprintf(fid,['case,delta_accel_max,nominal_ok,nominal_time_s,', ...
    'nominal_miss_m,nominal_angle_error_deg,nominal_max_a,', ...
    'nominal_max_delta_a,stress_ok,stress_time_s,stress_miss_m,', ...
    'stress_angle_error_deg,stress_max_a,stress_max_delta_a,', ...
    'nominal_qp_failures,stress_qp_failures\n']);
fprintf(fid,['"large-angle-60deg",%.8f,%d,%.8f,%.8f,%.8f,%.8f,%.8f,', ...
    '%d,%.8f,%.8f,%.8f,%.8f,%.8f,%d,%d\n'], ...
    deltaAccelMaxOverride, ...
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

fprintf('\nGamma_f=60 acceleration-vibration summary saved to %s\n',summaryPath);
