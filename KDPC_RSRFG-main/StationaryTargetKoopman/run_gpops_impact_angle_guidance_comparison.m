%% GPOPS benchmark for the impact-angle-constrained Koopman-MPC case
% This script compares the current angle-only Koopman-MPC guidance result
% against an offline fixed-final-time GPOPS-II solution with terminal
% position and impact-angle constraints.  GPOPS-II is not bundled with this
% repository; add it to the MATLAB path before running this script.
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

statusPath=fullfile(resultsDir,'gpops_impact_angle_guidance_status.txt');
if exist('gpops2','file')~=2
    write_missing_gpops_status(statusPath);
    fprintf(['GPOPS-II was not found on the MATLAB path. ', ...
        'Status written to %s\n'],statusPath);
    return;
end

koopmanPath=fullfile(resultsDir, ...
    'stationary_target_results_angle_no_time_dA_5_main.mat');
if ~exist(koopmanPath,'file')
    fprintf('Running impact-angle Koopman-MPC case first...\n');
    run(fullfile(baseDir,'run_angle_only_no_time_check.m'));
end
koopmanData=load(koopmanPath,'nominal','p');
koopman=koopmanData.nominal;
p=koopmanData.p;

problem=impact_problem_from_koopman_result(koopman,p);
gpopsOut=solve_impact_gpops(problem);
bench=postprocess_impact_gpops(gpopsOut,problem);

summaryPath=fullfile(resultsDir, ...
    'gpops_impact_angle_guidance_comparison_summary.csv');
write_impact_summary(summaryPath,koopman,bench,problem);
plotPath=fullfile(resultsDir,'gpops_impact_angle_guidance_comparison.png');
plot_impact_comparison(plotPath,koopman,bench,problem);
save(fullfile(resultsDir,'gpops_impact_angle_guidance_comparison.mat'), ...
    'koopman','bench','problem','gpopsOut');

fprintf('\nImpact-angle guidance GPOPS comparison complete.\n');
fprintf('  Koopman-MPC miss = %.4f m, angle error = %.4f deg\n', ...
    koopman.metrics.impactRange_m,koopman.metrics.impactHeadingError_deg);
fprintf('  GPOPS miss       = %.4f m, angle error = %.4f deg\n', ...
    bench.miss_m,bench.impactAngleError_deg);
fprintf('Summary saved to %s\n',summaryPath);

function problem=impact_problem_from_koopman_result(koopman,p)
    problem.name='impact_angle_fixed_time_energy_optimal_guidance';
    problem.V=p.Vnom;
    problem.amax=p.amax;
    problem.tauA=p.tauA;
    problem.Rscale=p.Rscale;
    problem.r0=p.Rscale*koopman.x(1:2,1);
    problem.gamma0=koopman.x(3,1);
    problem.ua0=0;
    problem.gammaF=p.impactGamma;
    problem.tf=koopman.metrics.impactTime_s;
    problem.captureRadius=p.captureRadius;
    problem.angleTolerance_deg=3;
end

function output=solve_impact_gpops(problem)
    bounds.phase.initialtime.lower=0;
    bounds.phase.initialtime.upper=0;
    bounds.phase.finaltime.lower=problem.tf;
    bounds.phase.finaltime.upper=problem.tf;
    bounds.phase.initialstate.lower=[problem.r0(:).',problem.gamma0,problem.ua0];
    bounds.phase.initialstate.upper=[problem.r0(:).',problem.gamma0,problem.ua0];
    bounds.phase.state.lower=[-2e4,-2e4,-pi,-1];
    bounds.phase.state.upper=[ 2e4, 2e4, pi, 1];
    bounds.phase.finalstate.lower=[0,0,problem.gammaF,-1];
    bounds.phase.finalstate.upper=[0,0,problem.gammaF, 1];
    bounds.phase.control.lower=-1;
    bounds.phase.control.upper=1;
    bounds.phase.integral.lower=0;
    bounds.phase.integral.upper=1e8;

    guess.phase.time=[0;problem.tf];
    guess.phase.state=[
        problem.r0(1),problem.r0(2),problem.gamma0,problem.ua0
        0,0,problem.gammaF,0];
    guess.phase.control=[0;0];
    guess.phase.integral=0;

    setup.name=problem.name;
    setup.functions.continuous=@impact_continuous;
    setup.functions.endpoint=@impact_endpoint;
    setup.auxdata=problem;
    setup.bounds=bounds;
    setup.guess=guess;
    setup.nlp.solver='snopt';
    setup.derivatives.supplier='sparseCD';
    setup.derivatives.derivativelevel='second';
    setup.mesh.method='hp-PattersonRao';
    setup.mesh.tolerance=1e-6;
    setup.mesh.maxiterations=10;
    setup.scales.method='automatic-bounds';
    output=gpops2(setup);
end

function phaseout=impact_continuous(input)
    p=input.auxdata;
    x=input.phase.state;
    uc=input.phase.control(:,1);
    ua=x(:,4);
    phaseout.dynamics=[-p.V*cos(x(:,3)), ...
        -p.V*sin(x(:,3)), ...
        (p.amax/p.V)*ua, ...
        (-ua+uc)/p.tauA];
    phaseout.integrand=(p.amax*uc).^2;
end

function output=impact_endpoint(input)
    output.objective=input.phase.integral;
end

function bench=postprocess_impact_gpops(output,problem)
    sol=output.result.solution.phase;
    bench.time_s=sol.time(:).';
    bench.x=sol.state.';
    bench.uCommand=sol.control(:,1).';
    bench.uActual=bench.x(4,:);
    bench.range_m=sqrt(sum(bench.x(1:2,:).^2,1));
    [bench.miss_m,bench.impactIndex]=min(bench.range_m);
    bench.impactTime_s=bench.time_s(bench.impactIndex);
    bench.captured=bench.miss_m<=problem.captureRadius;
    bench.impactHeading_deg=rad2deg(bench.x(3,bench.impactIndex));
    bench.impactAngleError_deg=rad2deg(wrap_angle( ...
        bench.x(3,bench.impactIndex)-problem.gammaF));
    bench.maxCommand_mps2=problem.amax*max(abs(bench.uCommand));
    bench.maxAcceleration_mps2=problem.amax*max(abs(bench.uActual));
    bench.controlEnergy=trapz(bench.time_s,(problem.amax*bench.uCommand).^2);
    bench.rmsCommand_mps2=sqrt(mean((problem.amax*bench.uCommand).^2));
end

function write_impact_summary(path,koopman,bench,problem)
    kt=koopman.time_s(1:end-1);
    ku=koopman.u(:).';
    kEnergy=trapz(kt,(problem.amax*ku).^2);
    kRms=sqrt(mean((problem.amax*ku).^2));
    fid=fopen(path,'w');
    fprintf(fid,['method,captured,impact_time_s,miss_m,angle_error_deg,', ...
        'max_command_mps2,max_accel_mps2,rms_command_mps2,', ...
        'control_energy,solver_failures\n']);
    fprintf(fid,'Koopman-MPC,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.12e,%d\n', ...
        koopman.metrics.impactSatisfied,koopman.metrics.impactTime_s, ...
        koopman.metrics.impactRange_m, ...
        koopman.metrics.impactHeadingError_deg, ...
        koopman.metrics.maxCommand_mps2, ...
        koopman.metrics.maxAcceleration_mps2,kRms,kEnergy, ...
        koopman.metrics.qpFailures);
    fprintf(fid,'GPOPS,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.12e,%d\n', ...
        bench.captured,bench.impactTime_s,bench.miss_m, ...
        bench.impactAngleError_deg,bench.maxCommand_mps2, ...
        bench.maxAcceleration_mps2,bench.rmsCommand_mps2, ...
        bench.controlEnergy,0);
    fclose(fid);
end

function plot_impact_comparison(path,koopman,bench,problem)
    fig=figure('Color','w','Position',[120 80 980 780]);
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    nexttile; hold on; grid on; axis equal;
    plot(problem.Rscale*koopman.x(1,:),problem.Rscale*koopman.x(2,:), ...
        'b','LineWidth',1.6);
    plot(bench.x(1,:),bench.x(2,:),'r--','LineWidth',1.6);
    plot(0,0,'ko','MarkerFaceColor','k');
    xlabel('r_x [m]'); ylabel('r_y [m]');
    legend('Koopman-MPC','GPOPS','Target','Location','best');
    title('Trajectory');

    nexttile; hold on; grid on;
    plot(koopman.time_s,koopman.range_m,'b','LineWidth',1.5);
    plot(bench.time_s,bench.range_m,'r--','LineWidth',1.5);
    yline(problem.captureRadius,'k:');
    xlabel('time [s]'); ylabel('range [m]');
    title('Range');

    nexttile; hold on; grid on;
    plot(koopman.time_s,rad2deg(koopman.x(3,:)),'b','LineWidth',1.5);
    plot(bench.time_s,rad2deg(bench.x(3,:)),'r--','LineWidth',1.5);
    yline(rad2deg(problem.gammaF),'k:');
    xlabel('time [s]'); ylabel('\gamma [deg]');
    title('Flight-path angle');

    nexttile; hold on; grid on;
    stairs(koopman.time_s(1:end-1),problem.amax*koopman.u, ...
        'b','LineWidth',1.4);
    plot(bench.time_s,problem.amax*bench.uCommand,'r--','LineWidth',1.4);
    yline(problem.amax,'k:'); yline(-problem.amax,'k:');
    xlabel('time [s]'); ylabel('A_c [m/s^2]');
    title('Command');
    exportgraphics(fig,path,'Resolution',220);
    close(fig);
end

function a=wrap_angle(a)
    a=mod(a+pi,2*pi)-pi;
end

function write_missing_gpops_status(path)
    fid=fopen(path,'w');
    fprintf(fid,'GPOPS-II was not found on the MATLAB path.\n');
    fprintf(fid,['Add GPOPS-II and rerun ', ...
        'run_gpops_impact_angle_guidance_comparison.m.\n']);
    fclose(fid);
end
