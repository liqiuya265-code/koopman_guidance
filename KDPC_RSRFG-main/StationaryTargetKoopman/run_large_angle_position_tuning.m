%% Position-hit tuning for large-impact-angle sin/cos Koopman model
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

cases(1)=make_case('baseline-sincos','large_pos_baseline',60, ...
    [600,900,100,60],[70000,90000,20000,12000], ...
    [2e6,2e6,5e5,5e5],false,false,50,1.5);
cases(2)=make_case('strong-y-terminal','large_pos_strong_y',60, ...
    [600,2500,100,60],[70000,450000,20000,12000], ...
    [2e6,2e8,5e5,5e5],false,false,50,1.5);
cases(3)=make_case('zero-tau-terminal','large_pos_zero_tau',60, ...
    [1200,2500,100,60],[300000,450000,20000,12000], ...
    [2e8,2e8,5e5,5e5],true,true,50,1.5);
cases(4)=make_case('zero-tau-h70','large_pos_zero_tau_h70',60, ...
    [1200,2500,100,60],[300000,550000,20000,12000], ...
    [2e8,2e8,5e5,5e5],true,true,70,1.2);
cases(5)=make_case('zero-tau-90deg','large_pos_zero_tau_90',90, ...
    [1200,2500,100,60],[300000,550000,25000,15000], ...
    [2e8,2e8,5e5,5e5],true,true,70,1.2);

summary=repmat(struct('case','','angle_deg',nan,'satisfied',false, ...
    'miss_m',nan,'angle_error_deg',nan,'stress_satisfied',false, ...
    'stress_miss_m',nan,'stress_angle_error_deg',nan, ...
    'qp_failures',nan,'stress_qp_failures',nan),1,numel(cases));

for i=1:numel(cases)
    fprintf('\nRunning position tuning case %d/%d: %s\n', ...
        i,numel(cases),cases(i).name);
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    angleOnlyModeOverride=true;
    impactGammaDegOverride=cases(i).Angle;
    initialGammaDegOverride=-30;
    angleOnlyMaxTimeOverride=75;
    gammaMaxDegOverride=130;
    trainingDataModeOverride='largeAngle';
    angleStateModeOverride='sincos';
    enableAdaptiveOverride=false;
    horizonOverride=cases(i).Horizon;
    nStepsOverride=180;
    nTrainTrajOverride=900;
    nTestTrajOverride=180;
    qxOverride=cases(i).Qx;
    qterminalOverride=cases(i).Qterminal;
    slackPenaltyOverride=cases(i).Slack;
    angleOnlyTerminalTauOverride=cases(i).TerminalTau;
    angleOnlyZeroTauRefOverride=cases(i).ZeroTauRef;
    rdOverride=cases(i).Rd;
    resultSuffixOverride=cases(i).suffix;
    run(fullfile(baseDir,'run_stationary_target_demo.m'));

    summary(i).case=cases(i).name;
    summary(i).angle_deg=cases(i).Angle;
    summary(i).satisfied=nominal.metrics.impactSatisfied;
    summary(i).miss_m=nominal.metrics.impactRange_m;
    summary(i).angle_error_deg=nominal.metrics.impactHeadingError_deg;
    summary(i).stress_satisfied=stress.metrics.impactSatisfied;
    summary(i).stress_miss_m=stress.metrics.impactRange_m;
    summary(i).stress_angle_error_deg=stress.metrics.impactHeadingError_deg;
    summary(i).qp_failures=nominal.metrics.qpFailures;
    summary(i).stress_qp_failures=stress.metrics.qpFailures;
end

summaryPath=fullfile(resultsDir,'large_angle_position_tuning_summary.csv');
write_position_summary(summary,summaryPath);
plot_position_summary(summary,fullfile(resultsDir, ...
    'large_angle_position_tuning_summary.png'));
fprintf('\nPosition tuning summary saved to %s\n',summaryPath);

function c=make_case(name,suffix,angle,Qx,Qterminal,Slack,terminalTau, ...
    zeroTauRef,horizon,rd)
    c=struct('name',name,'suffix',suffix,'Angle',angle,'Qx',Qx, ...
        'Qterminal',Qterminal,'Slack',Slack, ...
        'TerminalTau',terminalTau,'ZeroTauRef',zeroTauRef, ...
        'Horizon',horizon,'Rd',rd);
end

function write_position_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['case,angle_deg,satisfied,miss_m,angle_error_deg,', ...
        'stress_satisfied,stress_miss_m,stress_angle_error_deg,', ...
        'qp_failures,stress_qp_failures\n']);
    for i=1:numel(summary)
        fprintf(fid,'"%s",%.8f,%d,%.8f,%.8f,%d,%.8f,%.8f,%d,%d\n', ...
            summary(i).case,summary(i).angle_deg,summary(i).satisfied, ...
            summary(i).miss_m,summary(i).angle_error_deg, ...
            summary(i).stress_satisfied,summary(i).stress_miss_m, ...
            summary(i).stress_angle_error_deg,summary(i).qp_failures, ...
            summary(i).stress_qp_failures);
    end
    fclose(fid);
end

function plot_position_summary(summary,path)
    labels={summary.case};
    x=1:numel(summary);
    fig=figure('Visible','off','Position',[100 100 1150 720]);
    tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
    nexttile;
    bar(x,[[summary.miss_m].',[summary.stress_miss_m].']);
    yline(5,'k--','5 m'); ylabel('miss [m]'); grid on;
    title('Position-hit tuning for large impact angles');
    legend('nominal','stress','Location','best');
    xticks(x); xticklabels(labels); xtickangle(15);
    nexttile;
    bar(x,[[summary.angle_error_deg].',[summary.stress_angle_error_deg].']);
    yline(3,'k--','+/-3 deg'); yline(-3,'k--');
    ylabel('angle error [deg]'); grid on;
    xticks(x); xticklabels(labels); xtickangle(15);
    exportgraphics(fig,path,'Resolution',180);
end
