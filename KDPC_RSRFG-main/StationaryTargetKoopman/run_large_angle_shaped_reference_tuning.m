%% Shaped lateral-reference tuning for large-impact-angle guidance
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

cases(1)=make_case('zero-tau-baseline','large_shape_baseline',0,1,0.6);
cases(2)=make_case('shape-pos-0p10','large_shape_pos_0p10',0.10,1,0.6);
cases(3)=make_case('shape-pos-0p20','large_shape_pos_0p20',0.20,1,0.6);
cases(4)=make_case('shape-pos-0p35','large_shape_pos_0p35',0.35,1,0.6);
cases(5)=make_case('shape-neg-0p10','large_shape_neg_0p10',0.10,-1,0.6);
cases(6)=make_case('shape-neg-0p20','large_shape_neg_0p20',0.20,-1,0.6);
cases(7)=make_case('shape-neg-0p35','large_shape_neg_0p35',0.35,-1,0.6);
cases(8)=make_case('shape-pos-0p20-wide','large_shape_pos_0p20_wide',0.20,1,0.9);

summary=repmat(struct('case','','amplitude',nan,'sign',nan, ...
    'tau_scale',nan,'satisfied',false,'miss_m',nan, ...
    'angle_error_deg',nan,'stress_satisfied',false,'stress_miss_m',nan, ...
    'stress_angle_error_deg',nan,'qp_failures',nan, ...
    'stress_qp_failures',nan),1,numel(cases));

for i=1:numel(cases)
    fprintf('\nRunning shaped-reference case %d/%d: %s\n', ...
        i,numel(cases),cases(i).name);
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    angleOnlyModeOverride=true;
    impactGammaDegOverride=60;
    initialGammaDegOverride=-30;
    angleOnlyMaxTimeOverride=75;
    gammaMaxDegOverride=130;
    trainingDataModeOverride='largeAngle';
    angleStateModeOverride='sincos';
    enableAdaptiveOverride=false;
    horizonOverride=50;
    nStepsOverride=180;
    nTrainTrajOverride=900;
    nTestTrajOverride=180;
    qxOverride=[1200,2500,100,60];
    qterminalOverride=[300000,450000,20000,12000];
    slackPenaltyOverride=[2e8,2e8,5e5,5e5];
    angleOnlyTerminalTauOverride=true;
    angleOnlyZeroTauRefOverride=true;
    rdOverride=1.5;
    enableYReferenceShapeOverride=cases(i).Amplitude>0;
    yRefAmplitudeOverride=cases(i).Amplitude;
    yRefSignOverride=cases(i).Sign;
    yRefTauScaleOverride=cases(i).TauScale;
    resultSuffixOverride=cases(i).suffix;
    run(fullfile(baseDir,'run_stationary_target_demo.m'));

    summary(i).case=cases(i).name;
    summary(i).amplitude=cases(i).Amplitude;
    summary(i).sign=cases(i).Sign;
    summary(i).tau_scale=cases(i).TauScale;
    summary(i).satisfied=nominal.metrics.impactSatisfied;
    summary(i).miss_m=nominal.metrics.impactRange_m;
    summary(i).angle_error_deg=nominal.metrics.impactHeadingError_deg;
    summary(i).stress_satisfied=stress.metrics.impactSatisfied;
    summary(i).stress_miss_m=stress.metrics.impactRange_m;
    summary(i).stress_angle_error_deg=stress.metrics.impactHeadingError_deg;
    summary(i).qp_failures=nominal.metrics.qpFailures;
    summary(i).stress_qp_failures=stress.metrics.qpFailures;
end

summaryPath=fullfile(resultsDir,'large_angle_shaped_reference_summary.csv');
write_shape_summary(summary,summaryPath);
plot_shape_summary(summary,fullfile(resultsDir, ...
    'large_angle_shaped_reference_summary.png'));
plot_shape_trajectories(cases,resultsDir,fullfile(resultsDir, ...
    'large_angle_shaped_reference_trajectories.png'));
fprintf('\nShaped-reference summary saved to %s\n',summaryPath);

function c=make_case(name,suffix,amplitude,signValue,tauScale)
    c=struct('name',name,'suffix',suffix,'Amplitude',amplitude, ...
        'Sign',signValue,'TauScale',tauScale);
end

function write_shape_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['case,amplitude,sign,tau_scale,satisfied,miss_m,', ...
        'angle_error_deg,stress_satisfied,stress_miss_m,', ...
        'stress_angle_error_deg,qp_failures,stress_qp_failures\n']);
    for i=1:numel(summary)
        fprintf(fid,'"%s",%.8f,%.8f,%.8f,%d,%.8f,%.8f,%d,%.8f,%.8f,%d,%d\n', ...
            summary(i).case,summary(i).amplitude,summary(i).sign, ...
            summary(i).tau_scale,summary(i).satisfied,summary(i).miss_m, ...
            summary(i).angle_error_deg,summary(i).stress_satisfied, ...
            summary(i).stress_miss_m,summary(i).stress_angle_error_deg, ...
            summary(i).qp_failures,summary(i).stress_qp_failures);
    end
    fclose(fid);
end

function plot_shape_summary(summary,path)
    labels={summary.case};
    x=1:numel(summary);
    fig=figure('Visible','off','Position',[80 80 1250 760]);
    tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
    nexttile;
    bar(x,[[summary.miss_m].',[summary.stress_miss_m].']);
    yline(5,'k--','5 m'); ylabel('miss [m]'); grid on;
    title('Shaped lateral-reference tuning, gamma_f=60 deg');
    legend('nominal','stress','Location','best');
    xticks(x); xticklabels(labels); xtickangle(18);
    nexttile;
    bar(x,[[summary.angle_error_deg].',[summary.stress_angle_error_deg].']);
    yline(3,'k--','+/-3 deg'); yline(-3,'k--');
    ylabel('angle error [deg]'); grid on;
    xticks(x); xticklabels(labels); xtickangle(18);
    exportgraphics(fig,path,'Resolution',180);
end

function plot_shape_trajectories(cases,resultsDir,path)
    fig=figure('Visible','off','Position',[80 80 1250 900]);
    tiledlayout(2,4,'Padding','compact','TileSpacing','compact');
    for i=1:numel(cases)
        S=load(fullfile(resultsDir, ...
            sprintf('stationary_target_results_%s.mat',cases(i).suffix)), ...
            'nominal','stress','p');
        nexttile;
        plot(S.p.Rscale*S.nominal.x(1,:),S.p.Rscale*S.nominal.x(2,:), ...
            'b','LineWidth',1.1); hold on;
        plot(S.p.Rscale*S.stress.x(1,:),S.p.Rscale*S.stress.x(2,:), ...
            'r--','LineWidth',1.0);
        plot(0,0,'kx','LineWidth',1.4);
        axis equal; grid on;
        title(cases(i).name,'Interpreter','none');
        xlabel('x [m]'); ylabel('y [m]');
        if i==1
            legend('nominal','stress','target','Location','best');
        end
    end
    exportgraphics(fig,path,'Resolution',180);
end
