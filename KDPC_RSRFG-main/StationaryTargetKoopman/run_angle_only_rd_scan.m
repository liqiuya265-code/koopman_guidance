%% Control-smoothing scan for angle-only guidance
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

rdValues=[0.9,1.5,3.0,6.0,10.0,15.0];
summary=repmat(struct('rd',nan,'satisfied',false,'miss_m',nan, ...
    'angle_error_deg',nan,'max_accel_mps2',nan,'rms_delta_accel_mps2',nan, ...
    'max_delta_accel_mps2',nan,'qp_failures',nan),1,numel(rdValues));

for i=1:numel(rdValues)
    fprintf('\nRunning angle-only Rd scan %d/%d: Rd=%.2f\n', ...
        i,numel(rdValues),rdValues(i));
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    angleOnlyModeOverride=true;
    impactGammaDegOverride=30;
    angleOnlyMaxTimeOverride=50;
    resultSuffixOverride=sprintf('angle_no_time_rd_%g',rdValues(i));
    rdOverride=rdValues(i);
    run(fullfile(baseDir,'run_stationary_target_demo.m'));

    accel=100*nominal.uActual;
    dAccel=diff([0,accel]);
    summary(i).rd=rdValues(i);
    summary(i).satisfied=nominal.metrics.impactSatisfied;
    summary(i).miss_m=nominal.metrics.impactRange_m;
    summary(i).angle_error_deg=nominal.metrics.impactHeadingError_deg;
    summary(i).max_accel_mps2=max(abs(accel));
    summary(i).rms_delta_accel_mps2=sqrt(mean(dAccel.^2));
    summary(i).max_delta_accel_mps2=max(abs(dAccel));
    summary(i).qp_failures=nominal.metrics.qpFailures;
end

summaryPath=fullfile(resultsDir,'angle_only_rd_scan_summary.csv');
write_rd_scan_summary(summary,summaryPath);
plot_rd_scan_summary(summary,fullfile(resultsDir,'angle_only_rd_scan_summary.png'));
fprintf('\nRd scan summary saved to %s\n',summaryPath);

function write_rd_scan_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['rd,satisfied,miss_m,angle_error_deg,max_accel_mps2,', ...
        'rms_delta_accel_mps2,max_delta_accel_mps2,qp_failures\n']);
    for i=1:numel(summary)
        fprintf(fid,'%g,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%d\n', ...
            summary(i).rd,summary(i).satisfied,summary(i).miss_m, ...
            summary(i).angle_error_deg,summary(i).max_accel_mps2, ...
            summary(i).rms_delta_accel_mps2,summary(i).max_delta_accel_mps2, ...
            summary(i).qp_failures);
    end
    fclose(fid);
end

function plot_rd_scan_summary(summary,path)
    rd=[summary.rd];
    miss=[summary.miss_m];
    angle=[summary.angle_error_deg];
    rmsDelta=[summary.rms_delta_accel_mps2];
    fig=figure('Visible','off','Position',[100 100 980 760]);
    tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
    nexttile;
    plot(rd,miss,'o-','LineWidth',1.6); yline(5,'k--','5 m');
    ylabel('miss [m]'); grid on; title('Effect of increasing Rd');
    nexttile;
    plot(rd,angle,'o-','LineWidth',1.6); yline(3,'k--','+/-3 deg');
    yline(-3,'k--'); ylabel('angle error [deg]'); grid on;
    nexttile;
    plot(rd,rmsDelta,'o-','LineWidth',1.6);
    xlabel('Rd'); ylabel('RMS Delta A [m/s^2]'); grid on;
    exportgraphics(fig,path,'Resolution',180);
end
