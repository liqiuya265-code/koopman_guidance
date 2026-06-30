%% Position-priority tuning near the best 5 s configuration
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

cases(1)=make_case('h50-terminal-y2','tune_h50_terminal_y2', ...
    [3500,900,25],[250000,200000,900],0.04,0.65,[8e7,1e7,5e5],3);
cases(2)=make_case('h50-terminal-y4','tune_h50_terminal_y4', ...
    [3500,900,20],[250000,400000,800],0.04,0.65,[8e7,5e7,5e5],3);
cases(3)=make_case('h50-low-angle','tune_h50_low_angle', ...
    [3500,900,8],[250000,200000,300],0.04,0.65,[8e7,1e7,2e5],3);
cases(4)=make_case('h50-aggressive-input','tune_h50_aggressive_input', ...
    [3500,900,20],[250000,250000,800],0.02,0.20,[8e7,2e7,5e5],4);

summary=repmat(struct('name','','suffix','','impactSatisfied',false, ...
    'miss_m',nan,'angle_error_deg',nan,'max_command_mps2',nan, ...
    'qp_failures',nan,'tau_slack',nan,'y_slack',nan,'theta_slack',nan), ...
    1,numel(cases));

for i=1:numel(cases)
    fprintf('\nRunning position-priority case %d/%d: %s\n', ...
        i,numel(cases),cases(i).name);
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    resultSuffixOverride=cases(i).suffix;
    horizonOverride=50;
    nStepsOverride=120;
    qxOverride=cases(i).Qx;
    qterminalOverride=cases(i).Qterminal;
    rbarScaleOverride=cases(i).Rbar;
    rdOverride=cases(i).Rd;
    slackPenaltyOverride=cases(i).Slack;
    seqIterationsOverride=cases(i).SeqIter;
    run(fullfile(baseDir,'run_stationary_target_demo.m'));

    summary(i).name=cases(i).name;
    summary(i).suffix=cases(i).suffix;
    summary(i).impactSatisfied=nominal.metrics.impactSatisfied;
    summary(i).miss_m=nominal.metrics.impactRange_m;
    summary(i).angle_error_deg=nominal.metrics.impactHeadingError_deg;
    summary(i).max_command_mps2=nominal.metrics.maxCommand_mps2;
    summary(i).qp_failures=nominal.metrics.qpFailures;
    summary(i).tau_slack=nominal.metrics.maxTerminalSlack(1);
    summary(i).y_slack=nominal.metrics.maxTerminalSlack(2);
    summary(i).theta_slack=nominal.metrics.maxTerminalSlack(3);
end

write_position_summary(summary,fullfile(resultsDir,'time_constraint_tuning_position_priority_summary.csv'));
fprintf('\nPosition-priority tuning summary saved to %s\n', ...
    fullfile(resultsDir,'time_constraint_tuning_position_priority_summary.csv'));

function c=make_case(name,suffix,Qx,Qterminal,Rbar,Rd,Slack,SeqIter)
    c=struct('name',name,'suffix',suffix,'Qx',Qx,'Qterminal',Qterminal, ...
        'Rbar',Rbar,'Rd',Rd,'Slack',Slack,'SeqIter',SeqIter);
end

function write_position_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['case,suffix,satisfied,miss_m,angle_error_deg,max_command_mps2,', ...
        'qp_failures,tau_slack,y_slack,theta_slack\n']);
    for i=1:numel(summary)
        fprintf(fid,'"%s","%s",%d,%.8f,%.8f,%.8f,%d,%.8e,%.8e,%.8e\n', ...
            summary(i).name,summary(i).suffix,summary(i).impactSatisfied, ...
            summary(i).miss_m,summary(i).angle_error_deg, ...
            summary(i).max_command_mps2,summary(i).qp_failures, ...
            summary(i).tau_slack,summary(i).y_slack,summary(i).theta_slack);
    end
    fclose(fid);
end
