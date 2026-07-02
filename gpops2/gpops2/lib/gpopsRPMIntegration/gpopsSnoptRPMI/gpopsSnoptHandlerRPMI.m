function result = gpopsSnoptHandlerRPMI(ZG, ZL, ZU, FL, FU, name, nlpoptions, probinfo)

% snopt requires that aux data be global
global GLOBALPROBINFO igrid
igrid = 1;

% get NLP sparsity patterns
[grdjacnonlinpat, iGrow, jGcol, probinfo] = gpopsSnoptSparsityRPMI(probinfo);

% for snopt add cost gradient as first row of jacobian
FL = [-inf; FL];
FU = [inf; FU];

% set snopt options
clear snoptcmex; %clean snopt setup

ObjAdd = 0; %add nothing to objective
ObjRow = 1; %objective row

% initiate states and multipliers as 0
xmul   = zeros(probinfo.nlpnumvar,1);
xstate = xmul;
Fmul   = zeros(probinfo.nlpnumcon+1,1);
Fstate = Fmul;

% make probinfo set to GLOBALPROBINFO
GLOBALPROBINFO = probinfo;
clear probinfo

% setup nlp options from nlpoptions (add more later)
if ~isempty(nlpoptions);
    if isfield(nlpoptions, 'tolerance');
        snsetr('Optimality Tolerance',nlpoptions.tolerance);
        snsetr('Feasibility Tolerance',2*nlpoptions.tolerance);
    else
        % default
        % snsetr('Optimality Tolerance',ProbInfo.OptTol);
        % snsetr('Feasibility Tolerance',ProbInfo.FeasTol);
    end
else
    % default
    % snsetr('Optimality Tolerance',ProbInfo.OptTol);
    % snsetr('Feasibility Tolerance',ProbInfo.FeasTol);
end

% snopt options
snprint([name,'SNOPTinfo.txt']);  % Name of SNOPT Print File
snseti('Timing level',3);         % Print Execution Time to File
snset('Hessian Limited Memory');  % Choose Hessian Type
%snset('LU Complete Pivoting');    % Choose Pivoting Method/ do not set to
% complete pivoting
snseti('Verify Level',-1);        % Derivative Verification Level
snseti('Iteration Limit',100000); % Iteration Limit
snseti('Major Iterations Limit',100000); % Major Iteration Limit
snseti('Minor Iterations Limit',100000); % Minor Iteration Limit

% NLP print to screen
snscreen on % Print snopt to terminal screen

% NLP derivative options
snseti('Derivative Option',1); % 1 derivates are supplied >>>>>>>>>

%snseti('Derivative Option',0); % 0 derivates NOT supplied
% use matlab linear indexing
%Jacindex = (jGcol-1)*GLOBALPROBINFO.nlpnumcon+1 + iGrow;
Jacindex = sub2ind([GLOBALPROBINFO.nlpnumcon+1, GLOBALPROBINFO.nlpnumvar], iGrow, jGcol);
if GLOBALPROBINFO.scaleflag;
    
    % add scale for objective
    GLOBALPROBINFO.Fscale = [GLOBALPROBINFO.objscale; GLOBALPROBINFO.Fscale];
    
    % scale NLP bounds
    ZL = GLOBALPROBINFO.Zscale.*ZL + GLOBALPROBINFO.Zshift;
    ZU = GLOBALPROBINFO.Zscale.*ZU + GLOBALPROBINFO.Zshift;
    ZG = GLOBALPROBINFO.Zscale.*ZG + GLOBALPROBINFO.Zshift;
    FL = GLOBALPROBINFO.Fscale.*FL;
    FU = GLOBALPROBINFO.Fscale.*FU;
    
    % scale AA
    GLOBALPROBINFO.Jacscale = GLOBALPROBINFO.Fscale(grdjacnonlinpat(:,1))./GLOBALPROBINFO.Zscale(grdjacnonlinpat(:,2));
    
    % scale jaclinear (used in callback)
    %JaclinMat = diag(sparse(GLOBALPROBINFO.Fscale))*JaclinMat*diag(sparse(1./GLOBALPROBINFO.Zscale));
    GLOBALPROBINFO.grdjaclinVectscaled = GLOBALPROBINFO.grdjaclinMat(Jacindex).*GLOBALPROBINFO.Fscale(iGrow)./GLOBALPROBINFO.Zscale(jGcol);
    
    % set snopt callback function
    NLPfun = 'gpopsSnoptFunJacScaledRPMI';
else
    % set snopt callback function
    NLPfun = 'gpopsSnoptFunJacRPMI';
    
    GLOBALPROBINFO.grdjaclinVect = GLOBALPROBINFO.grdjaclinMat(Jacindex);
end

% the linear parts of the snopt input will be set to blank '[]' because the
% integration scheme doesnot contain enought linear jacobian elements to
% justify linear seperation

% record snopt solution time
tstart = tic; % record ipopt runtime
[Zsol,F,xmul,Fmul,info,xstate,Fstate,ns,ninf,sinf,mincw,miniw,minrw] = snsolve(ZG,ZL,ZU,xmul,xstate,FL,FU,Fmul,Fstate,ObjAdd,ObjRow,[],[],[],iGrow,jGcol,NLPfun);
runtime = toc(tstart);

% unscale output
if GLOBALPROBINFO.scaleflag;
    Zsol = (Zsol - GLOBALPROBINFO.Zshift)./GLOBALPROBINFO.Zscale;
    Fmul = Fmul.*GLOBALPROBINFO.Fscale;
    F = F./GLOBALPROBINFO.Fscale;
end

% remove multiplier on cost
Fmul = Fmul(2:GLOBALPROBINFO.nlpnumcon+1);

% get cost
result.objective = gpopsObjRPMI(Zsol, GLOBALPROBINFO);

% get solution
result.solution = gpopsSolutionRPMI(Zsol, Fmul, GLOBALPROBINFO);

% get nlp output info
result.nlpinfo = info;

% get nlp solver time
result.nlptime = runtime;