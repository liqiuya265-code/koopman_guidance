%% GPOPS benchmark for the basic exact-bilinear Koopman-MPC guidance case
% This script compares the basic exact-bilinear Koopman-MPC interception
% result against an offline fixed-final-time optimal-control solution from
% GPOPS-II.  GPOPS-II is not bundled with this repository; add it to the
% MATLAB path before running this script.
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

statusPath=fullfile(resultsDir,'gpops_basic_guidance_status.txt');
if exist('gpops2','file')~=2
    write_missing_gpops_status(statusPath);
    fprintf(['GPOPS-II was not found on the MATLAB path. ', ...
        'Status written to %s\n'],statusPath);
    return;
end

koopmanPath=fullfile(resultsDir, ...
    'exact_bilinear_koopman_mpc_precision_guidance.mat');
if ~exist(koopmanPath,'file')
    fprintf('Running basic exact-bilinear Koopman-MPC case first...\n');
    run(fullfile(baseDir,'run_exact_bilinear_koopman_mpc_precision_guidance.m'));
end
koopmanData=load(koopmanPath,'out','p');
koopman=koopmanData.out;
p=koopmanData.p;

problem=basic_problem_from_koopman_result(koopman,p);
gpopsOut=solve_basic_gpops(problem);
bench=postprocess_basic_gpops(gpopsOut,problem);

summaryPath=fullfile(resultsDir,'gpops_basic_guidance_comparison_summary.csv');
write_basic_summary(summaryPath,koopman,bench,problem);
plotPath=fullfile(resultsDir,'gpops_basic_guidance_comparison.png');
plot_basic_comparison(plotPath,koopman,bench,problem);
save(fullfile(resultsDir,'gpops_basic_guidance_comparison.mat'), ...
    'koopman','bench','problem','gpopsOut');

fprintf('\nBasic guidance GPOPS comparison complete.\n');
fprintf('  Koopman-MPC miss = %.4f m, energy = %.4e\n', ...
    koopman.minRange_m,trapz(koopman.time_s(1:end-1), ...
    (problem.amax*koopman.u).^2));
fprintf('  GPOPS miss       = %.4f m, energy = %.4e\n', ...
    bench.miss_m,bench.controlEnergy);
fprintf('Summary saved to %s\n',summaryPath);

function problem=basic_problem_from_koopman_result(koopman,p)
    problem.name='basic_fixed_time_energy_optimal_guidance';
    problem.V=p.V;
    problem.amax=p.amax;
    problem.r0=p.r0(:);
    problem.gamma0=p.gamma0;
    problem.tf=koopman.impactTime_s;
    problem.captureRadius=p.captureRadius;
    problem.stateScale=[max(norm(p.r0),1),max(norm(p.r0),1),1];
end

function output=solve_basic_gpops(problem)
    bounds.phase.initialtime.lower=0;
    bounds.phase.initialtime.upper=0;
    bounds.phase.finaltime.lower=problem.tf;
    bounds.phase.finaltime.upper=problem.tf;
    bounds.phase.initialstate.lower=[problem.r0(:).',problem.gamma0];
    bounds.phase.initialstate.upper=[problem.r0(:).',problem.gamma0];
    bounds.phase.state.lower=[-2e4,-2e4,-pi];
    bounds.phase.state.upper=[ 2e4, 2e4, pi];
    bounds.phase.finalstate.lower=[0,0,-pi];
    bounds.phase.finalstate.upper=[0,0, pi];
    bounds.phase.control.lower=-1;
    bounds.phase.control.upper=1;
    bounds.phase.integral.lower=0;
    bounds.phase.integral.upper=1e8;

    guess.phase.time=[0;problem.tf];
    guess.phase.state=[
        problem.r0(1),problem.r0(2),problem.gamma0
        0,0,problem.gamma0];
    guess.phase.control=[0;0];
    guess.phase.integral=0;

    setup.name=problem.name;
    setup.functions.continuous=@basic_continuous;
    setup.functions.endpoint=@basic_endpoint;
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

function phaseout=basic_continuous(input)
    p=input.auxdata;
    x=input.phase.state;
    u=input.phase.control(:,1);
    phaseout.dynamics=[-p.V*cos(x(:,3)), ...
        -p.V*sin(x(:,3)), ...
        (p.amax/p.V)*u];
    phaseout.integrand=(p.amax*u).^2;
end

function output=basic_endpoint(input)
    output.objective=input.phase.integral;
end

function bench=postprocess_basic_gpops(output,problem)
    sol=output.result.solution.phase;
    bench.time_s=sol.time(:).';
    bench.x=sol.state.';
    bench.u=sol.control(:,1).';
    bench.range_m=sqrt(sum(bench.x(1:2,:).^2,1));
    [bench.miss_m,bench.impactIndex]=min(bench.range_m);
    bench.impactTime_s=bench.time_s(bench.impactIndex);
    bench.captured=bench.miss_m<=problem.captureRadius;
    bench.maxCommand_mps2=problem.amax*max(abs(bench.u));
    bench.controlEnergy=trapz(bench.time_s,(problem.amax*bench.u).^2);
    bench.rmsCommand_mps2=sqrt(mean((problem.amax*bench.u).^2));
end

function write_basic_summary(path,koopman,bench,problem)
    koopmanEnergy=trapz(koopman.time_s(1:end-1), ...
        (problem.amax*koopman.u).^2);
    koopmanRms=sqrt(mean((problem.amax*koopman.u).^2));
    fid=fopen(path,'w');
    fprintf(fid,['method,captured,impact_time_s,miss_m,max_command_mps2,', ...
        'rms_command_mps2,control_energy,solver_failures\n']);
    fprintf(fid,'Koopman-MPC,%d,%.8f,%.8f,%.8f,%.8f,%.12e,%d\n', ...
        koopman.captured,koopman.impactTime_s,koopman.minRange_m, ...
        problem.amax*max(abs(koopman.u)),koopmanRms,koopmanEnergy, ...
        koopman.optimizerFailures);
    fprintf(fid,'GPOPS,%d,%.8f,%.8f,%.8f,%.8f,%.12e,%d\n', ...
        bench.captured,bench.impactTime_s,bench.miss_m, ...
        bench.maxCommand_mps2,bench.rmsCommand_mps2, ...
        bench.controlEnergy,0);
    fclose(fid);
end

function plot_basic_comparison(path,koopman,bench,problem)
    fig=figure('Color','w','Position',[120 80 980 760]);
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    nexttile; hold on; grid on; axis equal;
    plot(koopman.x(1,:),koopman.x(2,:),'b','LineWidth',1.6);
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
    xlabel('time [s]'); ylabel('\gamma [deg]');
    title('Flight-path angle');

    nexttile; hold on; grid on;
    stairs(koopman.time_s(1:end-1),problem.amax*koopman.u, ...
        'b','LineWidth',1.4);
    plot(bench.time_s,problem.amax*bench.u,'r--','LineWidth',1.4);
    yline(problem.amax,'k:'); yline(-problem.amax,'k:');
    xlabel('time [s]'); ylabel('A [m/s^2]');
    title('Command');
    exportgraphics(fig,path,'Resolution',220);
    close(fig);
end

function write_missing_gpops_status(path)
    fid=fopen(path,'w');
    fprintf(fid,'GPOPS-II was not found on the MATLAB path.\n');
    fprintf(fid,['Add GPOPS-II and rerun ', ...
        'run_gpops_basic_guidance_comparison.m.\n']);
    fclose(fid);
end
