function result = gpopsSolveRPMD(setup, probinfo)

% gpopsSolveRPMD
% this function solves the optimal control problem using the Radau
% Pseudospectral Method using the mesh, derivative method, derivative
% level, scaling method, and nonlinear program defined in the user setup

% get full collocation points, weights and diff matrix for each phase
disp(' Finding Radau Points, Integration Weights, and Differentiation Matrix');
[probinfo.numnodes, probinfo.collocation] = gpopsPointsWeightsRPMD(setup.mesh.phase);

% get NLP bounds and map
disp(' Creating Bounds For Nonlinear Program');
[ZL, ZU, FL, FU, probinfo] = gpopsBoundsRPMD(setup, probinfo);

% get NLP Guess
disp(' Creating Guess For Nonlinear Program');
ZG = gpopsGuessRPMD(setup, probinfo);

% get NLP scales
disp(' Creating Scales For Nonlinear Program');
[probinfo, ocpscales] = gpopsScaleRPMD(setup, probinfo);

% call NLP
if strcmpi(setup.nlp.solver,'snopt');
    result = gpopsSnoptHandlerRPMD(ZG, ZL, ZU, FL, FU, setup.name, setup.nlp.options, probinfo);
elseif strcmpi(setup.nlp.solver,'ipopt');
    result = gpopsIpoptHandlerRPMD(ZG, ZL, ZU, FL, FU, setup.name, setup.nlp.options, probinfo);
end

% add scales and collocation to result structure
if probinfo.scaleflag
    result.ocpscales = ocpscales;
end

% store mesh properties
numphase = probinfo.numphase;
collocation(numphase).s = [];
collocation(numphase).w = [];
collocation(numphase).D = [];
for phasecount = 1:probinfo.numphase;
    collocation(phasecount).s = probinfo.collocation(phasecount).s;
    collocation(phasecount).w = probinfo.collocation(phasecount).w;
    collocation(phasecount).D = sortrows([probinfo.collocation(phasecount).Doffdiag; probinfo.collocation(phasecount).Ddiag]);
end
result.collocation = collocation;
