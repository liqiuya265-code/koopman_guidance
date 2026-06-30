%% Large-impact-angle validation without impact-time constraint
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

angles=[60,90];
initialGammas=-30;
nCase=numel(angles)*numel(initialGammas);
summary=repmat(struct('case','','impact_angle_deg',nan, ...
    'initial_gamma_deg',nan,'nominal_ok',false,'nominal_time_s',nan, ...
    'nominal_miss_m',nan,'nominal_angle_error_deg',nan, ...
    'stress_ok',false,'stress_time_s',nan,'stress_miss_m',nan, ...
    'stress_angle_error_deg',nan,'nominal_qp_failures',nan, ...
    'stress_qp_failures',nan),1,nCase);

idx=0;
for a=angles
    for g0=initialGammas
        idx=idx+1;
        fprintf('\nRunning large-angle case %d/%d: gamma_f=%d deg, gamma_0=%d deg\n', ...
            idx,nCase,a,g0);
        skipClearOverride=true;
        skipControllerComparisonOverride=true;
        angleOnlyModeOverride=true;
        impactGammaDegOverride=a;
        initialGammaDegOverride=g0;
        angleOnlyMaxTimeOverride=70;
        gammaMaxDegOverride=120;
        rdOverride=1.5;
        duMaxOverride=0.05;
        resultSuffixOverride=sprintf('angle_only_g%d_i%s_du005', ...
            a,signed_label(g0));
        run(fullfile(baseDir,'run_stationary_target_demo.m'));

        summary(idx).case=sprintf('gamma_f_%d_init_%d',a,g0);
        summary(idx).impact_angle_deg=a;
        summary(idx).initial_gamma_deg=g0;
        summary(idx).nominal_ok=nominal.metrics.impactSatisfied;
        summary(idx).nominal_time_s=nominal.metrics.impactTime_s;
        summary(idx).nominal_miss_m=nominal.metrics.impactRange_m;
        summary(idx).nominal_angle_error_deg= ...
            nominal.metrics.impactHeadingError_deg;
        summary(idx).stress_ok=stress.metrics.impactSatisfied;
        summary(idx).stress_time_s=stress.metrics.impactTime_s;
        summary(idx).stress_miss_m=stress.metrics.impactRange_m;
        summary(idx).stress_angle_error_deg= ...
            stress.metrics.impactHeadingError_deg;
        summary(idx).nominal_qp_failures=nominal.metrics.qpFailures;
        summary(idx).stress_qp_failures=stress.metrics.qpFailures;
    end
end

summaryPath=fullfile(resultsDir,'angle_only_large_angle_scenarios_summary.csv');
write_large_angle_summary(summary,summaryPath);
plot_large_angle_summary(summary, ...
    fullfile(resultsDir,'angle_only_large_angle_scenarios_summary.png'));
plot_large_angle_trajectories(angles,initialGammas,resultsDir, ...
    fullfile(resultsDir,'angle_only_large_angle_trajectories.png'));
fprintf('\nLarge-angle scenario summary saved to %s\n',summaryPath);

function label=signed_label(value)
    if value<0
        label=sprintf('m%d',abs(value));
    elseif value>0
        label=sprintf('p%d',value);
    else
        label='0';
    end
end

function write_large_angle_summary(summary,path)
    fid=fopen(path,'w');
    fprintf(fid,['case,impact_angle_deg,initial_gamma_deg,nominal_ok,', ...
        'nominal_time_s,nominal_miss_m,nominal_angle_error_deg,', ...
        'stress_ok,stress_time_s,stress_miss_m,stress_angle_error_deg,', ...
        'nominal_qp_failures,stress_qp_failures\n']);
    for i=1:numel(summary)
        fprintf(fid,['"%s",%.8f,%.8f,%d,%.8f,%.8f,%.8f,%d,', ...
            '%.8f,%.8f,%.8f,%d,%d\n'], ...
            summary(i).case,summary(i).impact_angle_deg, ...
            summary(i).initial_gamma_deg,summary(i).nominal_ok, ...
            summary(i).nominal_time_s,summary(i).nominal_miss_m, ...
            summary(i).nominal_angle_error_deg,summary(i).stress_ok, ...
            summary(i).stress_time_s,summary(i).stress_miss_m, ...
            summary(i).stress_angle_error_deg, ...
            summary(i).nominal_qp_failures,summary(i).stress_qp_failures);
    end
    fclose(fid);
end

function plot_large_angle_summary(summary,path)
    labels=arrayfun(@(s)sprintf('%d deg, g0=%d', ...
        s.impact_angle_deg,s.initial_gamma_deg),summary,'UniformOutput',false);
    x=1:numel(summary);
    nominalMiss=[summary.nominal_miss_m];
    stressMiss=[summary.stress_miss_m];
    nominalAngle=[summary.nominal_angle_error_deg];
    stressAngle=[summary.stress_angle_error_deg];
    fig=figure('Visible','off','Position',[90 90 1200 760]);
    tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
    nexttile;
    bar(x,[nominalMiss(:),stressMiss(:)]);
    yline(5,'k--','5 m'); ylabel('miss [m]'); grid on;
    title('Large-impact-angle validation');
    legend('nominal','stress','Location','best');
    xticks(x); xticklabels(labels); xtickangle(18);
    nexttile;
    bar(x,[nominalAngle(:),stressAngle(:)]);
    yline(3,'k--','+/-3 deg'); yline(-3,'k--');
    ylabel('angle error [deg]'); grid on;
    xticks(x); xticklabels(labels); xtickangle(18);
    exportgraphics(fig,path,'Resolution',180);
end

function plot_large_angle_trajectories(angles,initialGammas,resultsDir,path)
    fig=figure('Visible','off','Position',[80 80 1250 820]);
    tiledlayout(numel(angles),numel(initialGammas), ...
        'Padding','compact','TileSpacing','compact');
    for a=angles
        for g0=initialGammas
            suffix=sprintf('angle_only_g%d_i%s_du005',a,signed_label(g0));
            S=load(fullfile(resultsDir, ...
                sprintf('stationary_target_results_%s.mat',suffix)), ...
                'nominal','stress','p');
            nexttile;
            plot(S.p.Rscale*S.nominal.x(1,:),S.p.Rscale*S.nominal.x(2,:), ...
                'b','LineWidth',1.15); hold on;
            plot(S.p.Rscale*S.stress.x(1,:),S.p.Rscale*S.stress.x(2,:), ...
                'r--','LineWidth',1.05);
            plot(0,0,'kx','LineWidth',1.4);
            axis equal; grid on;
            title(sprintf('\\gamma_f=%d deg, \\gamma_0=%d deg',a,g0));
            xlabel('x [m]'); ylabel('y [m]');
            if a==angles(1) && g0==initialGammas(1)
                legend('nominal','stress','target','Location','best');
            end
        end
    end
    exportgraphics(fig,path,'Resolution',180);
end
