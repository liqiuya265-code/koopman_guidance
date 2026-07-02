function result = gpopsSnoptHandlerRPMD(ZG, ZL, ZU, FL, FU, name, nlpoptions, probinfo)

% snopt requires that aux data be global
global GLOBALPROBINFO igrid
igrid = 1;

% get NLP sparsity patterns
[iArow, jAcol, AA, grdjacpat, iGrow, jGcol, JaclinMat, probinfo] = gpopsSnoptSparsityRPMD(probinfo);

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
    
    % ERASE TEST
    % DOES NOT WORK WITH SHIFT...?????? FIGURE THIS OUT LATER
    GLOBALPROBINFO.Zshift = zeros(size(ZG));
    
    % add scale for objective
    GLOBALPROBINFO.Fscale = [GLOBALPROBINFO.objscale; GLOBALPROBINFO.Fscale];
    
    % scale NLP bounds
    ZL = GLOBALPROBINFO.Zscale.*ZL + GLOBALPROBINFO.Zshift;
    ZU = GLOBALPROBINFO.Zscale.*ZU + GLOBALPROBINFO.Zshift;
    ZG = GLOBALPROBINFO.Zscale.*ZG + GLOBALPROBINFO.Zshift;
    FL = GLOBALPROBINFO.Fscale.*FL;
    FU = GLOBALPROBINFO.Fscale.*FU;
    
    % scale AA
    AA = AA.*GLOBALPROBINFO.Fscale(iArow)./GLOBALPROBINFO.Zscale(jAcol);
    GLOBALPROBINFO.Jacscale = GLOBALPROBINFO.Fscale(grdjacpat(:,1))./GLOBALPROBINFO.Zscale(grdjacpat(:,2));
    
    % scale jaclinear (used in callback)
    %JaclinMat = diag(sparse(GLOBALPROBINFO.Fscale))*JaclinMat*diag(sparse(1./GLOBALPROBINFO.Zscale));
    JaclinMat(Jacindex) = JaclinMat(Jacindex).*GLOBALPROBINFO.Fscale(iGrow)./GLOBALPROBINFO.Zscale(jGcol);
    
    % linear function shift
    Amat = sparse(iArow, jAcol, AA, GLOBALPROBINFO.nlpnumcon+1, GLOBALPROBINFO.nlpnumvar);
    %Fshift = full((Amat + JaclinMat)*(GLOBALPROBINFO.Zshift./GLOBALPROBINFO.Zscale));
    GLOBALPROBINFO.Fshift = -full((Amat)*GLOBALPROBINFO.Zshift);
    
    % set snopt callback function
    NLPfun = 'gpopsSnoptFunJacScaledRPMD';
else
    % set snopt callback function
    NLPfun = 'gpopsSnoptFunJacRPMD';
end
GLOBALPROBINFO.JaclinC = JaclinMat(Jacindex);
GLOBALPROBINFO.JaclinMat = JaclinMat;

% record snopt solution time
tstart = tic; % record ipopt runtime
[Zsol,F,xmul,Fmul,info,xstate,Fstate,ns,ninf,sinf,mincw,miniw,minrw] = snsolve(ZG,ZL,ZU,xmul,xstate,FL,FU,Fmul,Fstate,ObjAdd,ObjRow,AA,iArow,jAcol,iGrow,jGcol,NLPfun);
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
result.objective = gpopsObjRPMD(Zsol, GLOBALPROBINFO);

% get solution
result.solution = gpopsSolutionRPMD(Zsol, Fmul, GLOBALPROBINFO);

% get nlp output info
result.nlpinfo = info;

% get nlp solver time
result.nlptime = runtime;