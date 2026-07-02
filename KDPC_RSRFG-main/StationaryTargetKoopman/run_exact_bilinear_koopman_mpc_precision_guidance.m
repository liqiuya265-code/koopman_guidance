%% Exact-bilinear Koopman MPC precision guidance demo
% This script uses the analytic bilinear Koopman model as the prediction model
% inside a receding-horizon MPC. PN is used only to initialize the nonlinear
% optimizer. The plant is the same ideal stationary-target guidance core, so no
% EDMD, residual tube, impact-time constraint, or autopilot model is involved.
clear; clc; close all;
set(groot,'defaultFigureVisible','off');

baseDir=fileparts(mfilename('fullpath'));
resultsDir=fullfile(baseDir,'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

%% Parameters
p.V=300;                  % speed [m/s]
p.amax=100;               % lateral acceleration bound [m/s^2]
p.Ts=0.1;                 % sample time [s]
p.captureRadius=5;        % precision hit radius [m]
p.maxTime=35;             % maximum simulation time [s]
p.gamma0=deg2rad(25);
p.r0=[6000;1500];

% MPC settings. The horizon shrinks with the estimated closing time and is
% parameterized by piecewise-constant control blocks.
p.Nmin=45;
p.Nmax=240;
p.moveBlockSize=10;
p.updateEvery=5;
p.navigationGainWarmStart=3.5;
p.maxIterations=28;
p.qTerminal=180;
p.qStage=0.04;
p.rU=0.012;
p.rDU=0.35;

out=run_closed_loop_exact_koopman_mpc(p);
summaryPath=fullfile(resultsDir, ...
    'exact_bilinear_koopman_mpc_precision_guidance_summary.csv');
write_summary(summaryPath,out,p);
plotPath=fullfile(resultsDir, ...
    'exact_bilinear_koopman_mpc_precision_guidance.png');
plot_exact_bilinear_mpc(out,p,plotPath);
save(fullfile(resultsDir,'exact_bilinear_koopman_mpc_precision_guidance.mat'), ...
    'out','p');

fprintf('\nExact-bilinear Koopman MPC precision guidance result:\n');
fprintf('  captured          = %d\n',out.captured);
fprintf('  closest range     = %.4f m\n',out.minRange_m);
fprintf('  closest time      = %.2f s\n',out.impactTime_s);
fprintf('  final range       = %.4f m\n',out.range_m(end));
fprintf('  max command       = %.3f m/s^2\n',p.amax*max(abs(out.u)));
fprintf('  optimizer failures= %d\n',out.optimizerFailures);
fprintf('  mean solve time   = %.2f ms\n',out.meanSolveTime_ms);
fprintf('  consistency error = %.3e\n',out.maxLiftConsistencyError);
fprintf('Summary saved to %s\n',summaryPath);
fprintf('Figure saved to %s\n',plotPath);

%% Local functions
function out=run_closed_loop_exact_koopman_mpc(p)
    Kmax=round(p.maxTime/p.Ts);
    x=zeros(3,Kmax+1);
    x(:,1)=[p.r0;p.gamma0];
    u=zeros(1,Kmax);
    exitflag=zeros(1,Kmax);
    solveTime=zeros(1,Kmax);
    liftErr=zeros(1,Kmax);
    plannedU=[];
    previousBlocks=[];
    uPrev=0;
    last=Kmax+1;

    for k=1:Kmax
        planIndex=mod(k-1,p.updateEvery)+1;
        if planIndex==1 || isempty(plannedU)
            z=exact_lift(x(:,k));
            tic;
            [plannedU,previousBlocks,exitflag(k)]= ...
                solve_exact_koopman_mpc(z,p,previousBlocks,uPrev);
            solveTime(k)=toc;
        else
            exitflag(k)=exitflag(k-1);
            solveTime(k)=0;
        end

        u(k)=plannedU(min(planIndex,numel(plannedU)));
        zNow=exact_lift(x(:,k));
        xNext=plant_step_exact(x(:,k),u(k),p);
        zNext=exact_bilinear_step(zNow,u(k),p);
        liftErr(k)=norm(exact_lift(xNext)-zNext,2);
        x(:,k+1)=xNext;
        uPrev=u(k);

        if norm(x(1:2,k+1))<=p.captureRadius
            last=k+1;
            break;
        end
    end

    x=x(:,1:last);
    u=u(1:last-1);
    exitflag=exitflag(1:last-1);
    solveTime=solveTime(1:last-1);
    liftErr=liftErr(1:last-1);
    time=(0:last-1)*p.Ts;
    range=sqrt(sum(x(1:2,:).^2,1));
    [minRange,impactIndex]=min(range);
    activeSolveTimes=solveTime(solveTime>0);

    out.x=x;
    out.u=u;
    out.time_s=time;
    out.range_m=range;
    out.minRange_m=minRange;
    out.impactIndex=impactIndex;
    out.impactTime_s=time(impactIndex);
    out.captured=minRange<=p.captureRadius;
    out.exitflag=exitflag;
    out.optimizerFailures=sum(exitflag(1:p.updateEvery:end)<=0);
    out.solveTime_s=solveTime;
    out.meanSolveTime_ms=1000*mean(activeSolveTimes);
    out.maxSolveTime_ms=1000*max(activeSolveTimes);
    out.maxLiftConsistencyError=max(liftErr);
end

function [U,blocks,exitflag]=solve_exact_koopman_mpc(z0,p,previousBlocks,uPrev)
    N=select_horizon(z0,p);
    nBlocks=ceil(N/p.moveBlockSize);
    pnSeq=pn_warm_start_sequence(z0,p,N);
    pnBlocks=blocks_from_sequence(pnSeq,p.moveBlockSize,nBlocks);
    if isempty(previousBlocks)
        x0=pnBlocks;
    else
        shifted=shift_blocks(previousBlocks,nBlocks);
        x0=0.65*shifted+0.35*pnBlocks;
    end

    lb=-ones(nBlocks,1);
    ub=ones(nBlocks,1);
    obj=@(v)exact_koopman_mpc_objective(z0,v,p,N,uPrev);
    options=optimoptions('fmincon','Display','off','Algorithm','sqp', ...
        'MaxIterations',p.maxIterations,'MaxFunctionEvaluations',1800, ...
        'OptimalityTolerance',1e-6,'StepTolerance',1e-7);
    [blocks,~,exitflag]=fmincon(obj,x0,[],[],[],[],lb,ub,[],options);
    if isempty(blocks) || exitflag<=0
        blocks=x0;
    end
    U=sequence_from_blocks(blocks,p.moveBlockSize,N);
end

function J=exact_koopman_mpc_objective(z0,blocks,p,N,uPrev)
    U=sequence_from_blocks(blocks,p.moveBlockSize,N);
    Z=rollout_exact_bilinear(z0,U,p);
    range2=Z(2,:).^2+Z(3,:).^2;
    r0=max(sqrt(z0(2)^2+z0(3)^2),1);
    hTarget=target_index(z0,p,N);
    du=diff([uPrev;U(:)]);
    J=p.qTerminal*range2(hTarget)/(r0^2)+ ...
        p.qStage*mean(range2)/(r0^2)+ ...
        p.rU*mean(U.^2)+p.rDU*mean(du.^2);
end

function N=select_horizon(z,p)
    tgo=estimated_time_to_go(z,p);
    N=min(p.Nmax,max(p.Nmin,ceil(1.10*tgo/p.Ts)));
end

function h=target_index(z,p,N)
    tgo=estimated_time_to_go(z,p);
    h=min(N,max(1,round(tgo/p.Ts)));
end

function tgo=estimated_time_to_go(z,p)
    rx=z(2);
    ry=z(3);
    cg=z(5);
    sg=z(6);
    range=max(sqrt(rx^2+ry^2),1e-9);
    vx=-p.V*cg;
    vy=-p.V*sg;
    closing=-(rx*vx+ry*vy)/range;
    if closing<=1e-6
        tgo=range/p.V;
    else
        tgo=range/closing;
    end
end

function U=pn_warm_start_sequence(z0,p,N)
    U=zeros(N,1);
    z=z0;
    for i=1:N
        U(i)=exact_koopman_pn_command(z,p);
        z=exact_bilinear_step(z,U(i),p);
    end
end

function u=exact_koopman_pn_command(z,p)
    rx=z(2);
    ry=z(3);
    cg=z(5);
    sg=z(6);
    rangeSq=max(rx^2+ry^2,1e-9);
    range=sqrt(rangeSq);
    vx=-p.V*cg;
    vy=-p.V*sg;
    losRate=(rx*vy-ry*vx)/rangeSq;
    closingSpeed=-(rx*vx+ry*vy)/range;
    if closingSpeed<=0
        u=0;
        return;
    end
    accel=p.navigationGainWarmStart*closingSpeed*losRate;
    u=max(-1,min(1,accel/p.amax));
end

function Z=rollout_exact_bilinear(z0,U,p)
    Z=zeros(6,numel(U));
    z=z0;
    for i=1:numel(U)
        z=exact_bilinear_step(z,U(i),p);
        Z(:,i)=z;
    end
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
    zn=expm(p.Ts*(A0+u*B1))*z;
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

function U=sequence_from_blocks(blocks,blockSize,N)
    U=repelem(blocks(:),blockSize);
    U=U(1:N);
end

function blocks=blocks_from_sequence(U,blockSize,nBlocks)
    blocks=zeros(nBlocks,1);
    for j=1:nBlocks
        idx=(j-1)*blockSize+1:min(j*blockSize,numel(U));
        blocks(j)=mean(U(idx));
    end
end

function shifted=shift_blocks(blocks,nBlocks)
    if isempty(blocks)
        shifted=zeros(nBlocks,1);
        return;
    end
    blocks=[blocks(:);blocks(end)];
    shifted=blocks(2:min(numel(blocks),nBlocks+1));
    if numel(shifted)<nBlocks
        shifted=[shifted;repmat(shifted(end),nBlocks-numel(shifted),1)];
    end
end

function write_summary(path,out,p)
    fid=fopen(path,'w');
    fprintf(fid,['captured,impact_time_s,min_range_m,final_range_m,', ...
        'max_command_mps2,capture_radius_m,optimizer_failures,', ...
        'mean_solve_ms,max_solve_ms,max_lift_consistency_error\n']);
    fprintf(fid,'%d,%.8f,%.8f,%.8f,%.8f,%.8f,%d,%.8f,%.8f,%.12e\n', ...
        out.captured,out.impactTime_s,out.minRange_m,out.range_m(end), ...
        p.amax*max(abs(out.u)),p.captureRadius,out.optimizerFailures, ...
        out.meanSolveTime_ms,out.maxSolveTime_ms,out.maxLiftConsistencyError);
    fclose(fid);
end

function plot_exact_bilinear_mpc(out,p,path)
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
    title('MPC lateral acceleration');

    sgtitle(sprintf(['Exact-bilinear Koopman MPC: captured=%d, ', ...
        'min range=%.3f m'],out.captured,out.minRange_m));
    exportgraphics(fig,path,'Resolution',180);
    close(fig);
end
