%% Hard input-rate constraint scan for angle-only guidance
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

duValues=[inf,0.5,0.3,0.2,0.1,0.05];
summary=repmat(struct('du_max',nan,'satisfied',false,'miss_m',nan, ...
    'angle_error_deg',nan,'stress_satisfied',false,'stress_miss_m',nan, ...
    'stress_angle_error_deg',nan,'max_accel_mps2',nan, ...
    'rms_delta_accel_mps2',nan,'max_delta_accel_mps2',nan, ...
    'qp_failures',nan),1,numel(duValues));

for i=1:numel(duValues)
    duMax=duValues(i);
    if isfinite(duMax)
        label=sprintf('%g',duMax);
        suffix=sprintf('angle_no_time_du_%g',duMax);
    else
        label='inf';
        suffix='angle_no_time_du_inf';
    end
    fprintf('\nRunning angle-only hard-du scan %d/%d: duMax=%s\n', ...
        i,numel(duValues),label);
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    angleOnlyModeOverride=true;
    impactGammaDegOverride=30;
    angleOnlyMaxTimeOverride=50;
    resultSuffixOverride=suffix;
    rdOverride=1.5;
    if isfinite(duMax)
        duMaxOverride=duMax;
    elseif exist('duMaxOverride','var')
        clear duMaxOverride;
    end
    run(fullfile(baseDir,'run_stationary_target_demo.m'));

    accel=100*nominal.uActual;
    dAccel=diff([0,accel]);
    summary(i).du_max=duMax;
    summary(i).satisfied=nominal.metrics.impactSatisfied;
    summary(i).miss_m=nominal.metrics.impactRange_m;
    summary(i).angle_error_deg=nominal.metrics.impactHeadingError_deg;
    summary(i).stress_satisfied=stress.metrics.impactSatisfied;
    summary(i).stress_miss_m=stress.metrics.impactRange_m;
    summary(i).stress_angle_error_deg=stress.metrics.impactHeadingError_deg;
    summary(i).max_accel_mps2=max(abs(accel));
    summary(i).rms_delta_accel_mps2=sqrt(mean(dAccel.^2));
    summary(i).max_delta_accel_mps2=max(abs(dAccel));
    summary(i).qp_failures=nominal.metrics.qpFailures;
end

summaryPath=fullfile(resultsDir,'angle_only_du_constraint_scan_summary.csv');
write_du_scan_summary(summary,summaryPath);
plot_du_scan_summary(summary, ...
    fullfile(resultsDir,'angle_only_du_constraint_scan_summary.png'));
plot_du_acceleration_comparison(duValues,resultsDir, ...
    fullfile(resultsDir,'angle_only_du_acceleration_comparison.png'));
fprintf('\nHard-du scan summary saved to %s\n',summaryPath);

function write_du_scan_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['du_max,satisfied,miss_m,angle_error_deg,stress_satisfied,', ...
        'stress_miss_m,stress_angle_error_deg,max_accel_mps2,', ...
        'rms_delta_accel_mps2,max_delta_accel_mps2,qp_failures\n']);
    for i=1:numel(summary)
        fprintf(fid,'%g,%d,%.8f,%.8f,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%d\n', ...
            summary(i).du_max,summary(i).satisfied,summary(i).miss_m, ...
            summary(i).angle_error_deg,summary(i).stress_satisfied, ...
            summary(i).stress_miss_m,summary(i).stress_angle_error_deg, ...
            summary(i).max_accel_mps2,summary(i).rms_delta_accel_mps2, ...
            summary(i).max_delta_accel_mps2,summary(i).qp_failures);
    end
    fclose(fid);
end

function plot_du_scan_summary(summary,path)
    du=[summary.du_max];
    x=1:numel(du);
    labels=arrayfun(@du_label,du,'UniformOutput',false);
    miss=[summary.miss_m];
    stressMiss=[summary.stress_miss_m];
    rmsDelta=[summary.rms_delta_accel_mps2];
    fig=figure('Visible','off','Position',[100 100 1000 780]);
    tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
    nexttile;
    plot(x,miss,'o-','LineWidth',1.6); hold on;
    plot(x,stressMiss,'s--','LineWidth',1.4);
    yline(5,'k--','5 m'); ylabel('miss [m]'); grid on;
    title('Hard input-rate constraint scan');
    legend('nominal','stress','Location','best');
    xticks(x); xticklabels(labels);
    nexttile;
    plot(x,[summary.angle_error_deg],'o-','LineWidth',1.6); hold on;
    plot(x,[summary.stress_angle_error_deg],'s--','LineWidth',1.4);
    yline(3,'k--','+/-3 deg'); yline(-3,'k--');
    ylabel('angle error [deg]'); grid on; xticks(x); xticklabels(labels);
    nexttile;
    plot(x,rmsDelta,'o-','LineWidth',1.6);
    xlabel('duMax'); ylabel('RMS Delta A [m/s^2]'); grid on;
    xticks(x); xticklabels(labels);
    exportgraphics(fig,path,'Resolution',180);
end

function plot_du_acceleration_comparison(duValues,resultsDir,path)
    fig=figure('Visible','off','Position',[80 80 1250 780]);
    tiledlayout(2,3,'Padding','compact','TileSpacing','compact');
    for i=1:numel(duValues)
        duMax=duValues(i);
        if isfinite(duMax)
            suffix=sprintf('angle_no_time_du_%g',duMax);
            label=sprintf('duMax = %g',duMax);
        else
            suffix='angle_no_time_du_inf';
            label='duMax = inf';
        end
        S=load(fullfile(resultsDir, ...
            sprintf('stationary_target_results_%s.mat',suffix)), ...
            'nominal','stress','p');
        nexttile;
        plot(S.nominal.time_s(1:end-1),S.p.amax*S.nominal.uActual, ...
            'b','LineWidth',1.1); hold on;
        plot(S.stress.time_s(1:end-1),S.p.amax*S.stress.uActual, ...
            'r--','LineWidth',1.0);
        yline(S.p.amax,'k:'); yline(-S.p.amax,'k:');
        grid on; title(label); xlabel('time [s]'); ylabel('A [m/s^2]');
        if i==1
            legend('nominal','stress','Location','best');
        end
    end
    exportgraphics(fig,path,'Resolution',180);
end

function label=du_label(value)
    if isfinite(value)
        label=sprintf('%g',value);
    else
        label='inf';
    end
end
