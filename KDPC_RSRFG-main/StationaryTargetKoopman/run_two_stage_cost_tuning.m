%% Two-stage cost tuning for impact-time constrained Koopman guidance
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

cases(1)=make_case('two-stage-4s','tune_two_stage_4s',4.0, ...
    [3500,700,25],[800,8000,90],[250000,120000,1500], ...
    0.045,0.9,[8e7,2e6,5e5],3);
cases(2)=make_case('two-stage-5s','tune_two_stage_5s',5.0, ...
    [3500,700,25],[900,10000,100],[250000,150000,1800], ...
    0.045,0.9,[8e7,2e6,5e5],3);
cases(3)=make_case('two-stage-6s','tune_two_stage_6s',6.0, ...
    [3500,700,25],[1000,12000,120],[250000,180000,2000], ...
    0.045,0.9,[8e7,2e6,5e5],3);
cases(4)=make_case('two-stage-5s-soft-tau','tune_two_stage_5s_soft_tau',5.0, ...
    [3500,700,25],[300,12000,100],[180000,180000,1800], ...
    0.045,0.9,[5e7,3e6,5e5],3);
cases(5)=make_case('two-stage-5s-aggressive','tune_two_stage_5s_aggressive',5.0, ...
    [3500,700,25],[500,15000,120],[220000,220000,2000], ...
    0.03,0.45,[8e7,5e6,5e5],4);

summary=repmat(struct('name','','suffix','','satisfied',false, ...
    'miss_m',nan,'angle_error_deg',nan,'mean_solve_ms',nan, ...
    'max_solve_ms',nan,'qp_failures',nan,'tau_slack',nan, ...
    'y_slack',nan,'theta_slack',nan,'max_command_mps2',nan),1,numel(cases));

for i=1:numel(cases)
    fprintf('\nRunning two-stage cost case %d/%d: %s\n', ...
        i,numel(cases),cases(i).name);
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    resultSuffixOverride=cases(i).suffix;
    horizonOverride=50;
    nStepsOverride=120;
    enableTwoStageCostOverride=true;
    twoStageWindowOverride=cases(i).Window;
    qprogressOverride=cases(i).Qprogress;
    qterminalStageOverride=cases(i).QterminalStage;
    qterminalOverride=cases(i).Qterminal;
    qxOverride=cases(i).Qprogress;
    rbarScaleOverride=cases(i).Rbar;
    rdOverride=cases(i).Rd;
    slackPenaltyOverride=cases(i).Slack;
    seqIterationsOverride=cases(i).SeqIter;
    run(fullfile(baseDir,'run_stationary_target_demo.m'));

    summary(i).name=cases(i).name;
    summary(i).suffix=cases(i).suffix;
    summary(i).satisfied=nominal.metrics.impactSatisfied;
    summary(i).miss_m=nominal.metrics.impactRange_m;
    summary(i).angle_error_deg=nominal.metrics.impactHeadingError_deg;
    summary(i).mean_solve_ms=nominal.metrics.meanSolveTime_ms;
    summary(i).max_solve_ms=nominal.metrics.maxSolveTime_ms;
    summary(i).qp_failures=nominal.metrics.qpFailures;
    summary(i).tau_slack=nominal.metrics.maxTerminalSlack(1);
    summary(i).y_slack=nominal.metrics.maxTerminalSlack(2);
    summary(i).theta_slack=nominal.metrics.maxTerminalSlack(3);
    summary(i).max_command_mps2=nominal.metrics.maxCommand_mps2;
end

write_two_stage_summary(summary,fullfile(resultsDir,'two_stage_cost_tuning_summary.csv'));
fprintf('\nTwo-stage tuning summary saved to %s\n', ...
    fullfile(resultsDir,'two_stage_cost_tuning_summary.csv'));

function c=make_case(name,suffix,window,Qprogress,QterminalStage,Qterminal, ...
    Rbar,Rd,Slack,SeqIter)
    c=struct('name',name,'suffix',suffix,'Window',window, ...
        'Qprogress',Qprogress,'QterminalStage',QterminalStage, ...
        'Qterminal',Qterminal,'Rbar',Rbar,'Rd',Rd, ...
        'Slack',Slack,'SeqIter',SeqIter);
end

function write_two_stage_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['case,suffix,satisfied,miss_m,angle_error_deg,mean_solve_ms,', ...
        'max_solve_ms,qp_failures,tau_slack,y_slack,theta_slack,max_command_mps2\n']);
    for i=1:numel(summary)
        fprintf(fid,'"%s","%s",%d,%.8f,%.8f,%.8f,%.8f,%d,%.8e,%.8e,%.8e,%.8f\n', ...
            summary(i).name,summary(i).suffix,summary(i).satisfied, ...
            summary(i).miss_m,summary(i).angle_error_deg, ...
            summary(i).mean_solve_ms,summary(i).max_solve_ms, ...
            summary(i).qp_failures,summary(i).tau_slack,summary(i).y_slack, ...
            summary(i).theta_slack,summary(i).max_command_mps2);
    end
    fclose(fid);
end
