%% Large-impact-angle validation with broad maneuver training data
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

angles=[60,90];
initialGamma=-30;
summary=repmat(struct('case','','impact_angle_deg',nan, ...
    'initial_gamma_deg',initialGamma,'nominal_ok',false, ...
    'nominal_time_s',nan,'nominal_miss_m',nan, ...
    'nominal_angle_error_deg',nan,'stress_ok',false, ...
    'stress_time_s',nan,'stress_miss_m',nan, ...
    'stress_angle_error_deg',nan,'nominal_qp_failures',nan, ...
    'stress_qp_failures',nan),1,numel(angles));

for i=1:numel(angles)
    a=angles(i);
    fprintf('\nRunning broad-data large-angle case %d/%d: gamma_f=%d deg, gamma_0=%d deg\n', ...
        i,numel(angles),a,initialGamma);
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    angleOnlyModeOverride=true;
    impactGammaDegOverride=a;
    initialGammaDegOverride=initialGamma;
    angleOnlyMaxTimeOverride=75;
    gammaMaxDegOverride=130;
    trainingDataModeOverride='largeAngle';
    angleStateModeOverride='sincos';
    enableAdaptiveOverride=false;
    horizonOverride=50;
    nStepsOverride=180;
    nTrainTrajOverride=900;
    nTestTrajOverride=180;
    rdOverride=1.5;
    resultSuffixOverride=sprintf('angle_only_g%d_i_m30_broad',a);
    run(fullfile(baseDir,'run_stationary_target_demo.m'));

    summary(i).case=sprintf('gamma_f_%d_init_%d_broad',a,initialGamma);
    summary(i).impact_angle_deg=a;
    summary(i).nominal_ok=nominal.metrics.impactSatisfied;
    summary(i).nominal_time_s=nominal.metrics.impactTime_s;
    summary(i).nominal_miss_m=nominal.metrics.impactRange_m;
    summary(i).nominal_angle_error_deg= ...
        nominal.metrics.impactHeadingError_deg;
    summary(i).stress_ok=stress.metrics.impactSatisfied;
    summary(i).stress_time_s=stress.metrics.impactTime_s;
    summary(i).stress_miss_m=stress.metrics.impactRange_m;
    summary(i).stress_angle_error_deg= ...
        stress.metrics.impactHeadingError_deg;
    summary(i).nominal_qp_failures=nominal.metrics.qpFailures;
    summary(i).stress_qp_failures=stress.metrics.qpFailures;
end

summaryPath=fullfile(resultsDir, ...
    'angle_only_large_angle_broad_data_summary.csv');
write_broad_summary(summary,summaryPath);
plot_broad_summary(summary,fullfile(resultsDir, ...
    'angle_only_large_angle_broad_data_summary.png'));
plot_broad_trajectories(angles,resultsDir,fullfile(resultsDir, ...
    'angle_only_large_angle_broad_data_trajectories.png'));
fprintf('\nBroad-data large-angle summary saved to %s\n',summaryPath);

function write_broad_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['case,impact_angle_deg,initial_gamma_deg,nominal_ok,', ...
        'nominal_time_s,nominal_miss_m,nominal_angle_error_deg,', ...
        'stress_ok,stress_time_s,stress_miss_m,stress_angle_error_deg,', ...
        'nominal_qp_failures,stress_qp_failures\n']);
    for i=1:numel(summary)
        fprintf(fid,['"%s",%.8f,%.8f,%d,%.8f,%.8f,%.8f,%d,', ...
            '%.8f,%.8f,%.8f,%d,%d\n'], ...
            summary(i).case,summary(i).impact_angle_deg, ...
            summary(i).initial_gamma_deg,summary(i).nominal_ok, ...
            summary(i).nominal_time_s,summary(i).nominal_miss_m, ...
            summary(i).nominal_angle_error_deg,summary(i).stress_ok, ...
            summary(i).stress_time_s,summary(i).stress_miss_m, ...
            summary(i).stress_angle_error_deg, ...
            summary(i).nominal_qp_failures,summary(i).stress_qp_failures);
    end
    fclose(fid);
end

function plot_broad_summary(summary,path)
    labels=arrayfun(@(s)sprintf('%d deg',s.impact_angle_deg),summary, ...
        'UniformOutput',false);
    x=1:numel(summary);
    fig=figure('Visible','off','Position',[100 100 900 680]);
    tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
    nexttile;
    bar(x,[[summary.nominal_miss_m].',[summary.stress_miss_m].']);
    yline(5,'k--','5 m'); ylabel('miss [m]'); grid on;
    title('Large-angle validation with broad training data');
    legend('nominal','stress','Location','best');
    xticks(x); xticklabels(labels);
    nexttile;
    bar(x,[[summary.nominal_angle_error_deg].', ...
        [summary.stress_angle_error_deg].']);
    yline(3,'k--','+/-3 deg'); yline(-3,'k--');
    ylabel('angle error [deg]'); grid on;
    xticks(x); xticklabels(labels);
    exportgraphics(fig,path,'Resolution',180);
end

function plot_broad_trajectories(angles,resultsDir,path)
    fig=figure('Visible','off','Position',[100 100 1000 520]);
    tiledlayout(1,numel(angles),'Padding','compact','TileSpacing','compact');
    for i=1:numel(angles)
        a=angles(i);
        suffix=sprintf('angle_only_g%d_i_m30_broad',a);
        S=load(fullfile(resultsDir, ...
            sprintf('stationary_target_results_%s.mat',suffix)), ...
            'nominal','stress','p');
        nexttile;
        plot(S.p.Rscale*S.nominal.x(1,:),S.p.Rscale*S.nominal.x(2,:), ...
            'b','LineWidth',1.2); hold on;
        plot(S.p.Rscale*S.stress.x(1,:),S.p.Rscale*S.stress.x(2,:), ...
            'r--','LineWidth',1.1);
        plot(0,0,'kx','LineWidth',1.4);
        axis equal; grid on;
        title(sprintf('\\gamma_f=%d deg, \\gamma_0=-30 deg',a));
        xlabel('x [m]'); ylabel('y [m]');
        if i==1
            legend('nominal','stress','target','Location','best');
        end
    end
    exportgraphics(fig,path,'Resolution',180);
end
