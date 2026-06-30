%% Refined tuning around the best 5 s time-constrained case
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

cases(1)=make_case('h50-y-strong','tune_h50_y_strong',50,120, ...
    [3500,3000,50],[250000,650000,3000],0.04,0.65, ...
    [1e8,2e8,2e6],4,true);
cases(2)=make_case('h50-y-strong-noadapt','tune_h50_y_strong_noadapt',50,120, ...
    [3500,3000,50],[250000,650000,3000],0.04,0.65, ...
    [1e8,2e8,2e6],4,false);
cases(3)=make_case('h55-balanced','tune_h55_balanced',55,130, ...
    [5000,3500,60],[350000,800000,4000],0.035,0.55, ...
    [1.5e8,3e8,3e6],4,true);
cases(4)=make_case('h55-balanced-noadapt','tune_h55_balanced_noadapt',55,130, ...
    [5000,3500,60],[350000,800000,4000],0.035,0.55, ...
    [1.5e8,3e8,3e6],4,false);
cases(5)=make_case('h60-balanced','tune_h60_balanced',60,140, ...
    [5500,3500,60],[400000,800000,4000],0.035,0.55, ...
    [1.5e8,3e8,3e6],4,true);

summary=repmat(struct('name','','suffix','','impactSatisfied',false, ...
    'miss_m',nan,'angle_error_deg',nan,'mean_solve_ms',nan, ...
    'max_solve_ms',nan,'qp_failures',nan,'tau_slack',nan, ...
    'y_slack',nan,'theta_slack',nan,'final_beta',nan),1,numel(cases));

for i=1:numel(cases)
    fprintf('\nRunning refined tuning case %d/%d: %s\n', ...
        i,numel(cases),cases(i).name);
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    resultSuffixOverride=cases(i).suffix;
    horizonOverride=cases(i).N;
    nStepsOverride=cases(i).nSteps;
    qxOverride=cases(i).Qx;
    qterminalOverride=cases(i).Qterminal;
    rbarScaleOverride=cases(i).Rbar;
    rdOverride=cases(i).Rd;
    slackPenaltyOverride=cases(i).Slack;
    seqIterationsOverride=cases(i).SeqIter;
    enableAdaptiveOverride=cases(i).EnableAdaptive;
    run(fullfile(baseDir,'run_stationary_target_demo.m'));

    summary(i).name=cases(i).name;
    summary(i).suffix=cases(i).suffix;
    summary(i).impactSatisfied=nominal.metrics.impactSatisfied;
    summary(i).miss_m=nominal.metrics.impactRange_m;
    summary(i).angle_error_deg=nominal.metrics.impactHeadingError_deg;
    summary(i).mean_solve_ms=nominal.metrics.meanSolveTime_ms;
    summary(i).max_solve_ms=nominal.metrics.maxSolveTime_ms;
    summary(i).qp_failures=nominal.metrics.qpFailures;
    summary(i).tau_slack=nominal.metrics.maxTerminalSlack(1);
    summary(i).y_slack=nominal.metrics.maxTerminalSlack(2);
    summary(i).theta_slack=nominal.metrics.maxTerminalSlack(3);
    summary(i).final_beta=nominal.metrics.finalBetaHat;
end

write_refined_summary(summary,fullfile(resultsDir,'time_constraint_tuning_refined_summary.csv'));
fprintf('\nRefined tuning summary saved to %s\n', ...
    fullfile(resultsDir,'time_constraint_tuning_refined_summary.csv'));

function c=make_case(name,suffix,N,nSteps,Qx,Qterminal,Rbar,Rd,Slack,SeqIter,EnableAdaptive)
    c=struct('name',name,'suffix',suffix,'N',N,'nSteps',nSteps, ...
        'Qx',Qx,'Qterminal',Qterminal,'Rbar',Rbar,'Rd',Rd, ...
        'Slack',Slack,'SeqIter',SeqIter,'EnableAdaptive',EnableAdaptive);
end

function write_refined_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['case,suffix,satisfied,miss_m,angle_error_deg,mean_solve_ms,', ...
        'max_solve_ms,qp_failures,tau_slack,y_slack,theta_slack,final_beta\n']);
    for i=1:numel(summary)
        fprintf(fid,'"%s","%s",%d,%.8f,%.8f,%.8f,%.8f,%d,%.8e,%.8e,%.8e,%.8f\n', ...
            summary(i).name,summary(i).suffix,summary(i).impactSatisfied, ...
            summary(i).miss_m,summary(i).angle_error_deg, ...
            summary(i).mean_solve_ms,summary(i).max_solve_ms, ...
            summary(i).qp_failures,summary(i).tau_slack,summary(i).y_slack, ...
            summary(i).theta_slack,summary(i).final_beta);
    end
    fclose(fid);
end
