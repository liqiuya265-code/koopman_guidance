%% Hard acceleration-vibration constraint scan for angle-only guidance
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

deltaAValues=[inf,15,10,8,5,3];
summary=repmat(struct('delta_accel_max',nan,'satisfied',false, ...
    'miss_m',nan,'angle_error_deg',nan,'stress_satisfied',false, ...
    'stress_miss_m',nan,'stress_angle_error_deg',nan, ...
    'max_accel_mps2',nan,'max_delta_accel_mps2',nan, ...
    'max_command_delta_mps2',nan,'stress_max_delta_accel_mps2',nan, ...
    'qp_failures',nan,'stress_qp_failures',nan),1,numel(deltaAValues));

for i=1:numel(deltaAValues)
    deltaA=deltaAValues(i);
    if isfinite(deltaA)
        label=sprintf('%g',deltaA);
        suffix=sprintf('angle_no_time_dA_%g',deltaA);
    else
        label='inf';
        suffix='angle_no_time_dA_inf';
    end
    fprintf('\nRunning hard acceleration-vibration scan %d/%d: deltaA=%s m/s^2\n', ...
        i,numel(deltaAValues),label);
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    angleOnlyModeOverride=true;
    impactGammaDegOverride=30;
    angleOnlyMaxTimeOverride=50;
    resultSuffixOverride=suffix;
    rdOverride=1.5;
    if isfinite(deltaA)
        deltaAccelMaxOverride=deltaA;
    elseif exist('deltaAccelMaxOverride','var')
        clear deltaAccelMaxOverride;
    end
    if exist('duMaxOverride','var'), clear duMaxOverride; end
    run(fullfile(baseDir,'run_stationary_target_demo.m'));

    summary(i).delta_accel_max=deltaA;
    summary(i).satisfied=nominal.metrics.impactSatisfied;
    summary(i).miss_m=nominal.metrics.impactRange_m;
    summary(i).angle_error_deg=nominal.metrics.impactHeadingError_deg;
    summary(i).stress_satisfied=stress.metrics.impactSatisfied;
    summary(i).stress_miss_m=stress.metrics.impactRange_m;
    summary(i).stress_angle_error_deg=stress.metrics.impactHeadingError_deg;
    summary(i).max_accel_mps2=nominal.metrics.maxAcceleration_mps2;
    summary(i).max_delta_accel_mps2=nominal.metrics.maxAccelerationDelta_mps2;
    summary(i).max_command_delta_mps2=nominal.metrics.maxCommandDelta_mps2;
    summary(i).stress_max_delta_accel_mps2=stress.metrics.maxAccelerationDelta_mps2;
    summary(i).qp_failures=nominal.metrics.qpFailures;
    summary(i).stress_qp_failures=stress.metrics.qpFailures;
end

summaryPath=fullfile(resultsDir,'angle_only_accel_vibration_constraint_scan_summary.csv');
write_accel_scan_summary(summary,summaryPath);
plot_accel_scan_summary(summary, ...
    fullfile(resultsDir,'angle_only_accel_vibration_constraint_scan_summary.png'));
plot_acceleration_vibration_comparison(deltaAValues,resultsDir, ...
    fullfile(resultsDir,'angle_only_accel_vibration_comparison.png'));
fprintf('\nHard acceleration-vibration scan summary saved to %s\n',summaryPath);

function write_accel_scan_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['delta_accel_max,satisfied,miss_m,angle_error_deg,', ...
        'stress_satisfied,stress_miss_m,stress_angle_error_deg,', ...
        'max_accel_mps2,max_delta_accel_mps2,max_command_delta_mps2,', ...
        'stress_max_delta_accel_mps2,qp_failures,stress_qp_failures\n']);
    for i=1:numel(summary)
        fprintf(fid,'%g,%d,%.8f,%.8f,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%d,%d\n', ...
            summary(i).delta_accel_max,summary(i).satisfied, ...
            summary(i).miss_m,summary(i).angle_error_deg, ...
            summary(i).stress_satisfied,summary(i).stress_miss_m, ...
            summary(i).stress_angle_error_deg,summary(i).max_accel_mps2, ...
            summary(i).max_delta_accel_mps2,summary(i).max_command_delta_mps2, ...
            summary(i).stress_max_delta_accel_mps2,summary(i).qp_failures, ...
            summary(i).stress_qp_failures);
    end
    fclose(fid);
end

function plot_accel_scan_summary(summary,path)
    x=1:numel(summary);
    labels=arrayfun(@accel_label,[summary.delta_accel_max],'UniformOutput',false);
    fig=figure('Visible','off','Position',[100 100 1060 820]);
    tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
    nexttile;
    plot(x,[summary.miss_m],'o-','LineWidth',1.6); hold on;
    plot(x,[summary.stress_miss_m],'s--','LineWidth',1.4);
    yline(5,'k--','5 m'); ylabel('miss [m]'); grid on;
    title('Hard acceleration-vibration constraint scan');
    legend('nominal','stress','Location','best');
    xticks(x); xticklabels(labels);
    nexttile;
    plot(x,[summary.angle_error_deg],'o-','LineWidth',1.6); hold on;
    plot(x,[summary.stress_angle_error_deg],'s--','LineWidth',1.4);
    yline(3,'k--','+/-3 deg'); yline(-3,'k--');
    ylabel('angle error [deg]'); grid on; xticks(x); xticklabels(labels);
    nexttile;
    plot(x,[summary.max_delta_accel_mps2],'o-','LineWidth',1.6); hold on;
    plot(x,[summary.stress_max_delta_accel_mps2],'s--','LineWidth',1.4);
    xlabel('Delta A max [m/s^2]'); ylabel('max Delta A actual [m/s^2]');
    grid on; xticks(x); xticklabels(labels);
    exportgraphics(fig,path,'Resolution',180);
end

function plot_acceleration_vibration_comparison(deltaAValues,resultsDir,path)
    fig=figure('Visible','off','Position',[80 80 1250 780]);
    tiledlayout(2,3,'Padding','compact','TileSpacing','compact');
    for i=1:numel(deltaAValues)
        deltaA=deltaAValues(i);
        if isfinite(deltaA)
            suffix=sprintf('angle_no_time_dA_%g',deltaA);
            label=sprintf('\\Delta A_{cmd} <= %g m/s^2',deltaA);
        else
            suffix='angle_no_time_dA_inf';
            label='\Delta A_{cmd} unconstrained';
        end
        S=load(fullfile(resultsDir, ...
            sprintf('stationary_target_results_%s.mat',suffix)), ...
            'nominal','stress','p');
        nexttile;
        plot(S.nominal.time_s(1:end-1),S.p.amax*S.nominal.uActual, ...
            'b','LineWidth',1.1); hold on;
        plot(S.stress.time_s(1:end-1),S.p.amax*S.stress.uActual, ...
            'r--','LineWidth',1.0);
        stairs(S.nominal.time_s(1:end-1),S.p.amax*S.nominal.u, ...
            'Color',[0.1 0.45 0.95],'LineStyle',':','LineWidth',0.9);
        yline(S.p.amax,'k:'); yline(-S.p.amax,'k:');
        grid on; title(label); xlabel('time [s]'); ylabel('A [m/s^2]');
        if i==1
            legend('nominal actual','stress actual','nominal command', ...
                'Location','best');
        end
    end
    exportgraphics(fig,path,'Resolution',180);
end

function label=accel_label(value)
    if isfinite(value)
        label=sprintf('%g',value);
    else
        label='inf';
    end
end
