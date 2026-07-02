%% Exact-bilinear Koopman basic guidance demo
% This script verifies that the analytic finite-dimensional bilinear Koopman
% structure of the ideal stationary-target guidance core is already sufficient
% to implement a basic hit-to-capture guidance law. It does not use EDMD,
% learned predictors, residual tubes, impact-time constraints, or autopilot
% augmentation.
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

%% Parameters
p.V=300;                 % speed [m/s]
p.amax=100;              % lateral acceleration bound [m/s^2]
p.Ts=0.02;               % sample time [s]
p.captureRadius=5;       % hit radius [m]
p.maxTime=35;            % maximum simulation time [s]
p.navigationGain=3.5;    % proportional-navigation gain
p.gamma0=deg2rad(25);
p.r0=[6000;1500];

%% Closed-loop simulation
out=run_closed_loop_exact_koopman_pn(p);
summaryPath=fullfile(resultsDir,'exact_bilinear_koopman_basic_guidance_summary.csv');
write_summary(summaryPath,out,p);
plotPath=fullfile(resultsDir,'exact_bilinear_koopman_basic_guidance.png');
plot_exact_bilinear_guidance(out,p,plotPath);
save(fullfile(resultsDir,'exact_bilinear_koopman_basic_guidance.mat'), ...
    'out','p');

fprintf('\nExact-bilinear Koopman basic guidance result:\n');
fprintf('  captured          = %d\n',out.captured);
fprintf('  closest range     = %.3f m\n',out.minRange_m);
fprintf('  closest time      = %.2f s\n',out.impactTime_s);
fprintf('  final range       = %.3f m\n',out.range_m(end));
fprintf('  max command       = %.3f m/s^2\n',p.amax*max(abs(out.u)));
fprintf('  consistency error = %.3e\n',out.maxLiftConsistencyError);
fprintf('Summary saved to %s\n',summaryPath);
fprintf('Figure saved to %s\n',plotPath);

%% Local functions
function out=run_closed_loop_exact_koopman_pn(p)
    Kmax=round(p.maxTime/p.Ts);
    x=zeros(3,Kmax+1);
    x(:,1)=[p.r0;p.gamma0];
    u=zeros(1,Kmax);
    liftErr=zeros(1,Kmax);
    last=Kmax+1;

    for k=1:Kmax
        z=exact_lift(x(:,k));
        u(k)=exact_koopman_pn_command(z,p);
        xNext=plant_step_exact(x(:,k),u(k),p);
        zNext=exact_bilinear_step(z,u(k),p);
        liftErr(k)=norm(exact_lift(xNext)-zNext,2);
        x(:,k+1)=xNext;

        if norm(x(1:2,k+1))<=p.captureRadius
            last=k+1;
            break;
        end
    end

    x=x(:,1:last);
    u=u(1:last-1);
    liftErr=liftErr(1:last-1);
    time=(0:last-1)*p.Ts;
    range=sqrt(sum(x(1:2,:).^2,1));
    [minRange,impactIndex]=min(range);

    out.x=x;
    out.u=u;
    out.time_s=time;
    out.range_m=range;
    out.minRange_m=minRange;
    out.impactIndex=impactIndex;
    out.impactTime_s=time(impactIndex);
    out.captured=minRange<=p.captureRadius;
    out.maxLiftConsistencyError=max(liftErr);
end

function u=exact_koopman_pn_command(z,p)
    rx=z(2);
    ry=z(3);
    cg=z(5);
    sg=z(6);
    rangeSq=max(rx^2+ry^2,1e-9);
    range=sqrt(rangeSq);

    % The exact Koopman lift gives the relative velocity components directly:
    % rdot=[-V cos(gamma); -V sin(gamma)].
    vx=-p.V*cg;
    vy=-p.V*sg;
    losRate=(rx*vy-ry*vx)/rangeSq;
    closingSpeed=-(rx*vx+ry*vy)/range;

    if closingSpeed<=0
        u=0;
        return;
    end

    accel=p.navigationGain*closingSpeed*losRate;
    u=max(-1,min(1,accel/p.amax));
end

function z=exact_lift(x)
    z=[1;x(1);x(2);x(3);cos(x(3));sin(x(3))];
end

function zn=exact_bilinear_step(z,u,p)
    A0=zeros(6);
    B1=zeros(6);
    A0(2,5)=-p.V;
    A0(3,6)=-p.V;
    B1(4,1)=p.amax/p.V;
    B1(5,6)=-p.amax/p.V;
    B1(6,5)=p.amax/p.V;
    M=A0+u*B1;
    zn=expm(p.Ts*M)*z;
    zn(1)=1;
end

function xn=plant_step_exact(x,u,p)
    f=@(s)[-p.V*cos(s(3));-p.V*sin(s(3));p.amax/p.V*u];
    k1=f(x);
    k2=f(x+0.5*p.Ts*k1);
    k3=f(x+0.5*p.Ts*k2);
    k4=f(x+p.Ts*k3);
    xn=x+p.Ts*(k1+2*k2+2*k3+k4)/6;
end

function write_summary(path,out,p)
    fid=fopen(path,'w');
    fprintf(fid,['captured,impact_time_s,min_range_m,final_range_m,', ...
        'max_command_mps2,navigation_gain,max_lift_consistency_error\n']);
    fprintf(fid,'%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.12e\n', ...
        out.captured,out.impactTime_s,out.minRange_m,out.range_m(end), ...
        p.amax*max(abs(out.u)),p.navigationGain, ...
        out.maxLiftConsistencyError);
    fclose(fid);
end

function plot_exact_bilinear_guidance(out,p,path)
    fig=figure('Color','w','Position',[120 80 980 720]);
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

    nexttile; hold on; grid on; axis equal;
    plot(out.x(1,:),out.x(2,:),'b','LineWidth',1.8);
    plot(0,0,'ko','MarkerFaceColor','k');
    th=linspace(0,2*pi,160);
    plot(p.captureRadius*cos(th),p.captureRadius*sin(th),'k--');
    plot(out.x(1,out.impactIndex),out.x(2,out.impactIndex), ...
        'ro','MarkerFaceColor','r');
    xlabel('r_x [m]'); ylabel('r_y [m]');
    title('Trajectory');

    nexttile; hold on; grid on;
    plot(out.time_s,out.range_m,'b','LineWidth',1.6);
    yline(p.captureRadius,'k--');
    plot(out.impactTime_s,out.minRange_m,'ro','MarkerFaceColor','r');
    xlabel('time [s]'); ylabel('range [m]');
    title('Range');

    nexttile; grid on;
    plot(out.time_s,rad2deg(out.x(3,:)),'LineWidth',1.5);
    xlabel('time [s]'); ylabel('\gamma [deg]');
    title('Flight-path angle');

    nexttile; hold on; grid on;
    stairs(out.time_s(1:end-1),p.amax*out.u,'LineWidth',1.5);
    yline(p.amax,'k--'); yline(-p.amax,'k--');
    xlabel('time [s]'); ylabel('A [m/s^2]');
    title('Commanded lateral acceleration');

    sgtitle(sprintf(['Exact-bilinear Koopman PN guidance: ', ...
        'captured=%d, min range=%.2f m'],out.captured,out.minRange_m));
    exportgraphics(fig,path,'Resolution',180);
    close(fig);
end
