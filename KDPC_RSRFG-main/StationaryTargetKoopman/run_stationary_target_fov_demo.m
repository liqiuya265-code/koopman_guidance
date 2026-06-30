%% FOV-constrained stationary-target Koopman guidance extension
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

cases=struct( ...
    'name',{'No FOV constraint','FOV <= 12 deg','FOV <= 20 deg', ...
        'FOV <= 30 deg','FOV <= 45 deg'}, ...
    'suffix',{'fov_free','fov_12deg','fov_20deg','fov_30deg','fov_45deg'}, ...
    'enableFov',{false,true,true,true,true}, ...
    'fovMaxDeg',{inf,12,20,30,45});

outputs=repmat(struct('name','','suffix','','nominal',[],'stress',[], ...
    'p',[],'ctrl',[],'metrics',[]),1,numel(cases));

for i=1:numel(cases)
    fprintf('\nRunning FOV extension case %d/%d: %s\n', ...
        i,numel(cases),cases(i).name);
    skipClearOverride=true;
    skipControllerComparisonOverride=true;
    resultSuffixOverride=cases(i).suffix;
    angleOnlyModeOverride=false;
    initialGammaDegOverride=25;
    fovMinRangeOverride=300;
    if cases(i).enableFov
        fovMaxDegOverride=cases(i).fovMaxDeg;
        disableFovOverride=false;
    else
        disableFovOverride=true;
        if exist('fovMaxDegOverride','var')
            clear fovMaxDegOverride;
        end
    end
    run(fullfile(baseDir,'run_stationary_target_demo.m'));
    outputs(i).name=cases(i).name;
    outputs(i).suffix=cases(i).suffix;
    outputs(i).nominal=nominal;
    outputs(i).stress=stress;
    outputs(i).p=p;
    outputs(i).ctrl=ctrl;
    outputs(i).metrics=metrics.nominal;
end

fig=plot_fov_comparison(outputs);
exportgraphics(fig,fullfile(resultsDir,'fov_guidance_comparison.png'), ...
    'Resolution',220);
write_fov_summary(outputs,fullfile(resultsDir,'fov_guidance_summary.csv'));
save(fullfile(resultsDir,'fov_guidance_results.mat'),'outputs');

fprintf('\nFOV comparison saved in %s\n',resultsDir);

function fig=plot_fov_comparison(outputs)
    colors=lines(numel(outputs));
    styles={'--','-',':','-.','-'};
    p=outputs(1).p;
    fig=figure('Color','w','Position',[80 60 1080 840]);
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

    nexttile; hold on; grid on; axis equal;
    for i=1:numel(outputs)
        out=outputs(i).nominal;
        plot(p.Rscale*out.x(1,:),p.Rscale*out.x(2,:), ...
            'Color',colors(i,:),'LineStyle',styles{i},'LineWidth',1.7, ...
            'DisplayName',outputs(i).name);
        plot(p.Rscale*out.x(1,out.impactIndex), ...
            p.Rscale*out.x(2,out.impactIndex),'o', ...
            'Color',colors(i,:),'MarkerFaceColor',colors(i,:), ...
            'HandleVisibility','off');
    end
    plot(0,0,'ko','MarkerFaceColor','k','HandleVisibility','off');
    xlabel('r_x [m]'); ylabel('r_y [m]');
    title('Trajectory'); legend('Location','southoutside');

    nexttile; hold on; grid on;
    for i=1:numel(outputs)
        out=outputs(i).nominal;
        plot(out.time_s,out.range_m,'Color',colors(i,:), ...
            'LineStyle',styles{i},'LineWidth',1.5);
    end
    yline(p.captureRadius,'k--');
    xline(p.impactTime,'k-.');
    xlabel('time [s]'); ylabel('range [m]');
    title('Range');

    nexttile; hold on; grid on;
    for i=1:numel(outputs)
        out=outputs(i).nominal;
        plot(out.time_s,abs(out.lookAngle_deg),'Color',colors(i,:), ...
            'LineStyle',styles{i},'LineWidth',1.5);
    end
    for i=1:numel(outputs)
        c=outputs(i).ctrl;
        if isfield(c,'enableFov') && c.enableFov
            yline(rad2deg(c.fovMax),':','Color',colors(i,:), ...
                'LineWidth',1.0);
        end
    end
    xlabel('time [s]'); ylabel('|\sigma| [deg]');
    title('Look angle');

    nexttile; hold on; grid on;
    for i=1:numel(outputs)
        out=outputs(i).nominal;
        plot(out.time_s(1:end-1),p.amax*out.uActual, ...
            'Color',colors(i,:),'LineStyle',styles{i},'LineWidth',1.4);
    end
    yline(p.amax,'k--'); yline(-p.amax,'k--');
    xlabel('time [s]'); ylabel('A [m/s^2]');
    title('Actual lateral acceleration');
    sgtitle('FOV-constrained Koopman-MPC guidance extension');
end

function write_fov_summary(outputs,path)
    fid=fopen(path,'w');
    fprintf(fid,['case,fov_enabled,fov_limit_deg,satisfied,impact_time_s,', ...
        'miss_m,angle_error_deg,max_look_angle_deg,fov_violation_deg,', ...
        'max_accel_mps2,qp_failures,mean_solve_ms\n']);
    for i=1:numel(outputs)
        m=outputs(i).metrics;
        c=outputs(i).ctrl;
        if isfield(c,'enableFov') && c.enableFov
            fovEnabled=1;
            fovLimit=rad2deg(c.fovMax);
        else
            fovEnabled=0;
            fovLimit=inf;
        end
        fprintf(fid,'"%s",%d,%.8f,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%d,%.8f\n', ...
            outputs(i).name,fovEnabled,fovLimit,m.impactSatisfied, ...
            m.impactTime_s,m.impactRange_m,m.impactHeadingError_deg, ...
            m.maxLookAngle_deg,m.fovViolation_deg,m.maxAcceleration_mps2, ...
            m.qpFailures,m.meanSolveTime_ms);
    end
    fclose(fid);
end
