function result = gpopsIpoptHandlerRPMD(ZG, ZL, ZU, FL, FU, name, nlpoptions, probinfo)

% NLP derivative options
if probinfo.derivativelevel == 1;
    % get Jacobian linear part, and nonlinear index
    probinfo = gpopsIpoptGrdJacSparsityRPMD(probinfo);
    
    if probinfo.scaleflag;
        % set IPOPT callback functions
        funcs.objective   = @gpopsIpoptObjScaledRPMD;
        funcs.constraints = @gpopsIpoptConScaledRPMD;
        funcs.gradient = @gpopsIpoptGrdScaledRPMD;
        funcs.jacobian = @gpopsIpoptJacScaledRPMD;
        funcs.jacobianstructure = @gpopsIpoptJacPatRPMD;
        options.ipopt.hessian_approximation = 'limited-memory';
        
        % scale NLP bounds
        ZL = probinfo.Zscale.*ZL + probinfo.Zshift;
        ZU = probinfo.Zscale.*ZU + probinfo.Zshift;
        ZG = probinfo.Zscale.*ZG + probinfo.Zshift;
        FL = probinfo.Fscale.*FL;
        FU = probinfo.Fscale.*FU;
        
        % get Jacobian scales, scale linear part
        probinfo.grdscale = probinfo.objscale./probinfo.Zscale(probinfo.grdpat);
        probinfo.jacscale = probinfo.Fscale(probinfo.jacnlpat(:,1))./probinfo.Zscale(probinfo.jacnlpat(:,2));
        % TEST ----
        probinfo.jaclinNS = probinfo.jaclin;
        % ---------
        probinfo.jaclin = diag(sparse(probinfo.Fscale))*probinfo.jaclin*diag(sparse(1./probinfo.Zscale));
    else
        % set IPOPT callback functions
        funcs.objective   = @gpopsIpoptObjRPMD;
        funcs.constraints = @gpopsIpoptConRPMD;
        funcs.gradient = @gpopsIpoptGrdRPMD;
        funcs.jacobian = @gpopsIpoptJacRPMD;
        funcs.jacobianstructure = @gpopsIpoptJacPatRPMD;
        options.ipopt.hessian_approximation = 'limited-memory';
    end
elseif probinfo.derivativelevel == 2;
    % get Jacobian linear part, and nonlinear index
    probinfo = gpopsIpoptGrdJacSparsityRPMD(probinfo);
    % get Hessian nonlinear index
    probinfo = gpopsIpoptHesSparsityRPMD(probinfo);
    if probinfo.scaleflag;
        % set IPOPT callback functions
        funcs.objective   = @gpopsIpoptObjScaledRPMD;
        funcs.constraints = @gpopsIpoptConScaledRPMD;
        funcs.gradient = @gpopsIpoptGrdScaledRPMD;
        funcs.jacobian = @gpopsIpoptJacScaledRPMD;
        funcs.jacobianstructure = @gpopsIpoptJacPatRPMD;
        funcs.hessian = @gpopsIpoptHesScaledRPMD;
        funcs.hessianstructure = @gpopsIpoptHesPatRPMD;
        
        % scale NLP bounds
        ZL = probinfo.Zscale.*ZL + probinfo.Zshift;
        ZU = probinfo.Zscale.*ZU + probinfo.Zshift;
        ZG = probinfo.Zscale.*ZG + probinfo.Zshift;
        FL = probinfo.Fscale.*FL;
        FU = probinfo.Fscale.*FU;
        
        % get Jacobian scales, scale linear part, get Hessian scales
        probinfo.grdscale = probinfo.objscale./probinfo.Zscale(probinfo.grdpat);
        probinfo.jacscale = probinfo.Fscale(probinfo.jacnlpat(:,1))./probinfo.Zscale(probinfo.jacnlpat(:,2));
        % TEST ----
        probinfo.jaclinNS = probinfo.jaclin;
        % ---------
        probinfo.jaclin = diag(sparse(probinfo.Fscale))*probinfo.jaclin*diag(sparse(1./probinfo.Zscale));
        probinfo.hesscale = 1./probinfo.Zscale(probinfo.hespat(:,1))./probinfo.Zscale(probinfo.hespat(:,2));
    else
        % set IPOPT callback functions
        funcs.objective   = @gpopsIpoptObjRPMD;
        funcs.constraints = @gpopsIpoptConRPMD;
        funcs.gradient = @gpopsIpoptGrdRPMD;
        funcs.jacobian = @gpopsIpoptJacRPMD;
        funcs.jacobianstructure = @gpopsIpoptJacPatRPMD;
        funcs.hessian = @gpopsIpoptHesRPMD;
        funcs.hessianstructure = @gpopsIpoptHesPatRPMD;
    end
end

options.lb = ZL; % Lower bound on the variables.
options.ub = ZU; % Upper bound on the variables.
options.cl = FL; % Lower bounds on the constraint functions.
options.cu = FU; % Upper bounds on the constraint functions.

clear ZL ZU FL FU

% set ipopt auxdata as probinfo
options.auxdata = probinfo;

% setup nlp options from nlpoptions (add more later)
if ~isempty(nlpoptions);
    if isfield(nlpoptions, 'tolerance');
        options.ipopt.tol = nlpoptions.tolerance;
    else
        options.ipopt.tol = 1e-7;
    end
else
    % set default
    options.ipopt.tol = 1e-7;
end

% ipopt options
% options.ipopt.derivative_test = 'first-order'; % derivative check
% options.ipopt.print_level = 5 % set print level default
%options.ipopt.nlp_scaling_method = 'none'; % turn scaling off
%options.ipopt.nlp_scaling_method = 'gradient-based'; % turn scaling on (default)
%options.ipopt.mu_init = 1e-2; % changin initial barrier parameter
%options.ipopt.bound_relax_factor = 1e-6; % equality constraint relaxation factor
options.ipopt.output_file = [name,'IPOPTinfo.txt']; % print output file
%options.ipopt.file_print_level = 12; 
options.ipopt.mu_strategy = 'adaptive';

% Run IPOPT.
tstart = tic; % record ipopt runtime
[Zsol info] = ipopt(ZG,funcs,options);
runtime = toc(tstart);

% change Fmul to what is expected
Fmul = -info.lambda;

% unscale output
if probinfo.scaleflag;
    Zsol = (Zsol - probinfo.Zshift)./probinfo.Zscale;
    Fmul = Fmul.*probinfo.Fscale;
end

% get cost
result.objective = gpopsObjRPMD(Zsol, probinfo);

% get solution
result.solution = gpopsSolutionRPMD(Zsol, Fmul, probinfo);

% get nlp output info
result.nlpinfo = info.status;

% get nlp solver time
result.nlptime = runtime;