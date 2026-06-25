%% Multi-scenario Koopman guidance comparisons
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

fixedInitialCases=struct( ...
    'name',{'tf=21.70 s, gamma_f=-2 deg', ...
        'tf=22.10 s, gamma_f=30 deg', ...
        'tf=22.90 s, gamma_f=45 deg'}, ...
    'suffix',{'fixed_init_tf21p70_gm2', ...
        'fixed_init_tf22p10_g30', ...
        'fixed_init_tf22p90_g45'}, ...
    'impactTime',{21.70,22.10,22.90}, ...
    'impactGammaDeg',{-2,30,45}, ...
    'initialGammaDeg',{25,25,25});

varyInitialCases=struct( ...
    'name',{'gamma_0=20 deg, tf=21.30 s', ...
        'gamma_0=25 deg, tf=21.70 s', ...
        'gamma_0=30 deg, tf=22.10 s'}, ...
    'suffix',{'vary_init_g20_tf21p30', ...
        'vary_init_g25_tf21p70', ...
        'vary_init_g30_tf22p10'}, ...
    'impactTime',{21.30,21.70,22.10}, ...
    'impactGammaDeg',{-2,-2,-2}, ...
    'initialGammaDeg',{20,25,30});

fixedInitialResults=run_case_group(fixedInitialCases,baseDir);
plot_case_group(fixedInitialCases,fixedInitialResults,resultsDir, ...
    'Fixed initial condition; varying impact-time and impact-angle constraints', ...
    'comparison_fixed_initial_vary_constraints');
write_case_group_csv(fixedInitialCases,fixedInitialResults,resultsDir, ...
    'comparison_fixed_initial_vary_constraints_summary.csv');

varyInitialResults=run_case_group(varyInitialCases,baseDir);
plot_case_group(varyInitialCases,varyInitialResults,resultsDir, ...
    'Varying initial heading; fixed impact angle and varying impact time', ...
    'comparison_vary_initial_time_fixed_angle');
write_case_group_csv(varyInitialCases,varyInitialResults,resultsDir, ...
    'comparison_vary_initial_time_fixed_angle_summary.csv');

fprintf('Comparison figures and summaries saved in %s\n',resultsDir);

function results=run_case_group(cases,baseDir)
    results=repmat(struct('nominal',[],'p',[],'metrics',[]),1,numel(cases));
    for i=1:numel(cases)
        resultPath=fullfile(baseDir,'results', ...
            ['stationary_target_results_',cases(i).suffix,'.mat']);
        if exist(resultPath,'file')
            fprintf('\nLoading cached comparison case %d/%d: %s\n', ...
                i,numel(cases),cases(i).name);
        else
            fprintf('\nRunning comparison case %d/%d: %s\n', ...
                i,numel(cases),cases(i).name);
            skipClearOverride=true;
            impactTimeOverride=cases(i).impactTime;
            impactGammaDegOverride=cases(i).impactGammaDeg;
            initialGammaDegOverride=cases(i).initialGammaDeg;
            angleOnlyModeOverride=false;
            resultSuffixOverride=cases(i).suffix;
            run(fullfile(baseDir,'run_stationary_target_demo.m'));
        end
        caseData=load(resultPath);
        results(i).nominal=caseData.nominal;
        results(i).p=caseData.p;
        results(i).metrics=caseData.metrics.nominal;
    end
end

function plot_case_group(cases,results,resultsDir,plotTitle,fileBase)
    colors=[0.00 0.24 0.72;
        0.82 0.20 0.18;
        0.10 0.55 0.25];
    fig=figure('Color','w','Position',[80 60 1060 820]);
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

    axTraj=nexttile;
    hold on; grid on; axis equal;
    legendLabels=case_labels(cases);
    for i=1:numel(cases)
        out=results(i).nominal;
        p=results(i).p;
        plot(p.Rscale*out.x(1,:),p.Rscale*out.x(2,:), ...
            'Color',colors(i,:),'LineWidth',1.7,'DisplayName',legendLabels{i});
        if out.impactSatisfied
            marker='o';
        else
            marker='x';
        end
        plot(p.Rscale*out.x(1,out.impactIndex), ...
            p.Rscale*out.x(2,out.impactIndex),marker, ...
            'Color',colors(i,:),'MarkerFaceColor',colors(i,:), ...
            'LineWidth',1.5,'HandleVisibility','off');
    end
    plot(0,0,'ko','MarkerFaceColor','k','HandleVisibility','off');
    xlabel('r_x [m]'); ylabel('r_y [m]');
    title('Trajectory');
    legend(axTraj,'Location','southoutside','NumColumns',1);

    nexttile;
    hold on; grid on;
    for i=1:numel(cases)
        out=results(i).nominal;
        p=results(i).p;
        plot(out.time_s,out.range_m,'Color',colors(i,:),'LineWidth',1.5);
        plot(out.impactTime_s,out.impactRange_m,'o', ...
            'Color',colors(i,:),'MarkerFaceColor',colors(i,:));
        xline(p.impactTime,':','Color',colors(i,:));
    end
    yline(results(1).p.captureRadius,'k--');
    xlabel('time [s]'); ylabel('range [m]');
    title('Range');

    nexttile;
    hold on; grid on;
    for i=1:numel(cases)
        out=results(i).nominal;
        p=results(i).p;
        plot(out.time_s,rad2deg(out.x(3,:)), ...
            'Color',colors(i,:),'LineWidth',1.5);
        yline(rad2deg(p.impactGamma),':','Color',colors(i,:));
    end
    xlabel('time [s]'); ylabel('gamma [deg]');
    title('Flight-path angle');

    nexttile;
    hold on; grid on;
    for i=1:numel(cases)
        out=results(i).nominal;
        p=results(i).p;
        plot(out.time_s(1:end-1),p.amax*out.uActual, ...
            'Color',colors(i,:),'LineWidth',1.4);
    end
    yline(results(1).p.amax,'k--');
    yline(-results(1).p.amax,'k--');
    xlabel('time [s]'); ylabel('A [m/s^2]');
    title('Actual lateral acceleration');

    sgtitle(plotTitle);
    exportgraphics(fig,fullfile(resultsDir,[fileBase,'.png']),'Resolution',220);
end

function labels=case_labels(cases)
    labels=cell(1,numel(cases));
    initialValues=[cases.initialGammaDeg];
    impactAngles=[cases.impactGammaDeg];
    sameInitial=max(abs(initialValues-initialValues(1)))<1e-9;
    sameImpactAngle=max(abs(impactAngles-impactAngles(1)))<1e-9;
    for i=1:numel(cases)
        if sameInitial && ~sameImpactAngle
            labels{i}=sprintf('t_f=%.2f s, gamma_f=%g deg', ...
                cases(i).impactTime,cases(i).impactGammaDeg);
        elseif sameImpactAngle
            labels{i}=sprintf('gamma_0=%g deg, t_f=%.2f s', ...
                cases(i).initialGammaDeg,cases(i).impactTime);
        else
            labels{i}=cases(i).name;
        end
    end
end

function write_case_group_csv(cases,results,resultsDir,fileName)
    fid=fopen(fullfile(resultsDir,fileName),'w');
    fprintf(fid,['scenario,initial_gamma_deg,impact_time_cmd_s,', ...
        'impact_gamma_cmd_deg,satisfied,impact_time_s,miss_m,', ...
        'angle_error_deg,max_accel_mps2,qp_failures\n']);
    for i=1:numel(cases)
        m=results(i).metrics;
        fprintf(fid,'"%s",%.8f,%.8f,%.8f,%d,%.8f,%.8f,%.8f,%.8f,%d\n', ...
            cases(i).name,cases(i).initialGammaDeg,cases(i).impactTime, ...
            cases(i).impactGammaDeg,m.impactSatisfied,m.impactTime_s, ...
            m.impactRange_m,m.impactHeadingError_deg, ...
            m.maxAcceleration_mps2,m.qpFailures);
    end
    fclose(fid);
end
