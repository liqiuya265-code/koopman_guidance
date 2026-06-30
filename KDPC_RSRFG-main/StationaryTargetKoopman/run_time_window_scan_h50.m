%% Impact-time window scan using the best 5 s tuning configuration
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

impactTimes=[39.70,39.85,40.00,40.15,40.30,40.45];
summary=repmat(struct('impactTime',nan,'suffix','','impactSatisfied',false, ...
    'miss_m',nan,'angle_error_deg',nan,'qp_failures',nan, ...
    'tau_slack',nan,'y_slack',nan,'theta_slack',nan),1,numel(impactTimes));

for i=1:numel(impactTimes)
    tf=impactTimes(i);
    suffix=sprintf('scan_h50_tf%05.2f',tf);
    suffix=strrep(suffix,'.','p');
    fprintf('\nRunning time-window scan %d/%d: tf=%.2f s\n', ...
        i,numel(impactTimes),tf);

    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    resultSuffixOverride=suffix;
    impactTimeOverride=tf;
    horizonOverride=50;
    nStepsOverride=120;
    qxOverride=[3500,900,35];
    qterminalOverride=[250000,90000,1200];
    rbarScaleOverride=0.045;
    rdOverride=0.9;
    slackPenaltyOverride=[8e7,2e6,5e5];
    seqIterationsOverride=3;
    run(fullfile(baseDir,'run_stationary_target_demo.m'));

    summary(i).impactTime=tf;
    summary(i).suffix=suffix;
    summary(i).impactSatisfied=nominal.metrics.impactSatisfied;
    summary(i).miss_m=nominal.metrics.impactRange_m;
    summary(i).angle_error_deg=nominal.metrics.impactHeadingError_deg;
    summary(i).qp_failures=nominal.metrics.qpFailures;
    summary(i).tau_slack=nominal.metrics.maxTerminalSlack(1);
    summary(i).y_slack=nominal.metrics.maxTerminalSlack(2);
    summary(i).theta_slack=nominal.metrics.maxTerminalSlack(3);
end

write_time_window_summary(summary,fullfile(resultsDir,'time_window_scan_h50_summary.csv'));
fprintf('\nTime-window scan summary saved to %s\n', ...
    fullfile(resultsDir,'time_window_scan_h50_summary.csv'));

function write_time_window_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,'impact_time_cmd_s,suffix,satisfied,miss_m,angle_error_deg,qp_failures,tau_slack,y_slack,theta_slack\n');
    for i=1:numel(summary)
        fprintf(fid,'%.8f,"%s",%d,%.8f,%.8f,%d,%.8e,%.8e,%.8e\n', ...
            summary(i).impactTime,summary(i).suffix,summary(i).impactSatisfied, ...
            summary(i).miss_m,summary(i).angle_error_deg,summary(i).qp_failures, ...
            summary(i).tau_slack,summary(i).y_slack,summary(i).theta_slack);
    end
    fclose(fid);
end
