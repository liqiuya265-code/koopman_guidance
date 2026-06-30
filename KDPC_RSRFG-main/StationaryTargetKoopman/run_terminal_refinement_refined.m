%% Refined terminal NMPC scan with impact-stage cost alignment
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

cases(1)=make_case('impact-cost-only','tune_terminal_ref_impact_cost_only', ...
    false,0,10,26,[250000,90000,1200],[8e7,2e6,5e5],0.045,0.9);
cases(2)=make_case('terminal-4s-impact-cost','tune_terminal_ref_4s_impact_cost', ...
    true,4.0,10,30,[350000,180000,2500],[8e7,8e7,2e6],0.045,0.9);
cases(3)=make_case('terminal-5s-impact-cost','tune_terminal_ref_5s_impact_cost', ...
    true,5.0,10,30,[350000,180000,2500],[8e7,8e7,2e6],0.045,0.9);
cases(4)=make_case('terminal-4s-14b-impact-cost','tune_terminal_ref_4s_14b_impact_cost', ...
    true,4.0,14,34,[450000,220000,3500],[8e7,1.5e8,4e6],0.04,0.7);
cases(5)=make_case('terminal-5s-14b-impact-cost','tune_terminal_ref_5s_14b_impact_cost', ...
    true,5.0,14,34,[450000,220000,3500],[8e7,1.5e8,4e6],0.04,0.7);
cases(6)=make_case('terminal-4s-18b-impact-cost','tune_terminal_ref_4s_18b_impact_cost', ...
    true,4.0,18,38,[600000,260000,4500],[1e8,2e8,6e6],0.04,0.55);

summary=repmat(struct('name','','suffix','','satisfied',false, ...
    'miss_m',nan,'angle_error_deg',nan,'mean_solve_ms',nan, ...
    'max_solve_ms',nan,'qp_failures',nan,'terminal_steps',nan, ...
    'tau_slack',nan,'y_slack',nan,'theta_slack',nan, ...
    'max_command_mps2',nan),1,numel(cases));

for i=1:numel(cases)
    fprintf('\nRunning refined terminal case %d/%d: %s\n', ...
        i,numel(cases),cases(i).name);
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    resultSuffixOverride=cases(i).suffix;
    horizonOverride=50;
    nStepsOverride=120;
    qxOverride=[3500,900,35];
    qterminalOverride=cases(i).Qterminal;
    rbarScaleOverride=cases(i).Rbar;
    rdOverride=cases(i).Rd;
    slackPenaltyOverride=cases(i).Slack;
    seqIterationsOverride=3;
    enableImpactStageCostOverride=true;
    enableTerminalRefinementOverride=cases(i).EnableTerminal;
    terminalRefinementWindowOverride=cases(i).Window;
    nmpcMoveBlocksOverride=cases(i).MoveBlocks;
    terminalNmpcMaxIterationsOverride=cases(i).MaxIterations;
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
    summary(i).max_command_mps2=nominal.metrics.maxCommand_mps2;
end

write_refined_summary(summary, ...
    fullfile(resultsDir,'terminal_refinement_refined_summary.csv'));
fprintf('\nRefined terminal summary saved to %s\n', ...
    fullfile(resultsDir,'terminal_refinement_refined_summary.csv'));

function c=make_case(name,suffix,enableTerminal,window,moveBlocks, ...
    maxIterations,Qterminal,Slack,Rbar,Rd)
    c=struct('name',name,'suffix',suffix, ...
        'EnableTerminal',enableTerminal,'Window',window, ...
        'MoveBlocks',moveBlocks,'MaxIterations',maxIterations, ...
        'Qterminal',Qterminal,'Slack',Slack,'Rbar',Rbar,'Rd',Rd);
end

function write_refined_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['case,suffix,satisfied,miss_m,angle_error_deg,mean_solve_ms,', ...
        'max_solve_ms,qp_failures,terminal_steps,tau_slack,y_slack,', ...
        'theta_slack,max_command_mps2\n']);
    for i=1:numel(summary)
        fprintf(fid,['"%s","%s",%d,%.8f,%.8f,%.8f,%.8f,%d,%d,', ...
            '%.8e,%.8e,%.8e,%.8f\n'], ...
            summary(i).name,summary(i).suffix,summary(i).satisfied, ...
            summary(i).miss_m,summary(i).angle_error_deg, ...
            summary(i).mean_solve_ms,summary(i).max_solve_ms, ...
            summary(i).qp_failures,summary(i).terminal_steps, ...
            summary(i).tau_slack,summary(i).y_slack,summary(i).theta_slack, ...
            summary(i).max_command_mps2);
    end
    fclose(fid);
end
