%% Minimal terminal-refinement check using the successful impact-stage cost
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

cases(1)=make_case('impact-cost-only','tune_terminal_min_impact_cost_only', ...
    false,0,10);
cases(2)=make_case('terminal-3s-same-cost','tune_terminal_min_3s_same_cost', ...
    true,3.0,10);
cases(3)=make_case('terminal-4s-same-cost','tune_terminal_min_4s_same_cost', ...
    true,4.0,10);
cases(4)=make_case('terminal-5s-same-cost','tune_terminal_min_5s_same_cost', ...
    true,5.0,10);

summary=repmat(struct('name','','suffix','','satisfied',false, ...
    'miss_m',nan,'angle_error_deg',nan,'mean_solve_ms',nan, ...
    'max_solve_ms',nan,'qp_failures',nan,'terminal_steps',nan, ...
    'tau_slack',nan,'y_slack',nan,'theta_slack',nan),1,numel(cases));

for i=1:numel(cases)
    fprintf('\nRunning minimal terminal case %d/%d: %s\n', ...
        i,numel(cases),cases(i).name);
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    resultSuffixOverride=cases(i).suffix;
    horizonOverride=50;
    nStepsOverride=120;
    qxOverride=[3500,900,35];
    qterminalOverride=[250000,90000,1200];
    rbarScaleOverride=0.045;
    rdOverride=0.9;
    slackPenaltyOverride=[8e7,2e6,5e5];
    seqIterationsOverride=3;
    enableImpactStageCostOverride=true;
    enableTerminalRefinementOverride=cases(i).EnableTerminal;
    terminalRefinementWindowOverride=cases(i).Window;
    nmpcMoveBlocksOverride=cases(i).MoveBlocks;
    terminalNmpcMaxIterationsOverride=26;
    run(fullfile(baseDir,'run_stationary_target_demo.m'));

    summary(i).name=cases(i).name;
    summary(i).suffix=cases(i).suffix;
    summary(i).satisfied=nominal.metrics.impactSatisfied;
    summary(i).miss_m=nominal.metrics.impactRange_m;
    summary(i).angle_error_deg=nominal.metrics.impactHeadingError_deg;
    summary(i).mean_solve_ms=nominal.metrics.meanSolveTime_ms;
    summary(i).max_solve_ms=nominal.metrics.maxSolveTime_ms;
    summary(i).qp_failures=nominal.metrics.qpFailures;
    summary(i).terminal_steps=nominal.metrics.terminalRefinementSteps;
    summary(i).tau_slack=nominal.metrics.maxTerminalSlack(1);
    summary(i).y_slack=nominal.metrics.maxTerminalSlack(2);
    summary(i).theta_slack=nominal.metrics.maxTerminalSlack(3);
end

write_minimal_summary(summary, ...
    fullfile(resultsDir,'terminal_refinement_minimal_summary.csv'));
fprintf('\nMinimal terminal summary saved to %s\n', ...
    fullfile(resultsDir,'terminal_refinement_minimal_summary.csv'));

function c=make_case(name,suffix,enableTerminal,window,moveBlocks)
    c=struct('name',name,'suffix',suffix, ...
        'EnableTerminal',enableTerminal,'Window',window, ...
        'MoveBlocks',moveBlocks);
end

function write_minimal_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['case,suffix,satisfied,miss_m,angle_error_deg,mean_solve_ms,', ...
        'max_solve_ms,qp_failures,terminal_steps,tau_slack,y_slack,theta_slack\n']);
    for i=1:numel(summary)
        fprintf(fid,['"%s","%s",%d,%.8f,%.8f,%.8f,%.8f,%d,%d,', ...
            '%.8e,%.8e,%.8e\n'], ...
            summary(i).name,summary(i).suffix,summary(i).satisfied, ...
            summary(i).miss_m,summary(i).angle_error_deg, ...
            summary(i).mean_solve_ms,summary(i).max_solve_ms, ...
            summary(i).qp_failures,summary(i).terminal_steps, ...
            summary(i).tau_slack,summary(i).y_slack,summary(i).theta_slack);
    end
    fclose(fid);
end
