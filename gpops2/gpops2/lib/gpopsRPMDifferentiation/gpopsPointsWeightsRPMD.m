function [numnodes, collocation] = gpopsPointsWeightsRPMD(currentmesh)

% gpopsPointsWeightsRPMD
% this function finds the points, quadrature weights,
% differentiation matrix and the time map derivatives for the
% Segmented Radau Pseudospectral Method, 
% the diagnal and off diagnal differentiation matrics are stored sparsely
% (row, colunm, value)

% NOTE
% s, w second column is values on [-1,1)
% D 4 column is values for [-1,1)

% get number of phases
numphase = size(currentmesh,2);

% preallocate fraction Mat, collocation points, weights and diff matrix for
% each phase
numnodes = zeros(1,numphase);
collocation(numphase).fractionMat = [];
collocation(numphase).s = [];
collocation(numphase).w = [];
collocation(numphase).Ddiag = [];
collocation(numphase).Doffdiag = [];
collocation(numphase).dtdt0 = [];
collocation(numphase).dtdtf = [];

% find LGR points, weights, and differentation matrix for each phase
for phasecount = 1:numphase;
    % get currentmesh information
    colpoints = currentmesh(phasecount).colpoints;
    fraction = currentmesh(phasecount).fraction;
    sumcolpoints = sum(colpoints);
    cumsumfraction = [0, cumsum(fraction)];
    cumsumcolpoints = [0, cumsum(colpoints)];
    cumsumcolpointssq = [0, cumsum(colpoints.^2)];
    
    % number of segments
    numseg = length(colpoints);
    
    % preallocate fraction, s, w, Ddiag, Doffdiag for each phase
    fractionvect = zeros(sumcolpoints,1);
    s = zeros(sumcolpoints,2);
    w = zeros(sumcolpoints,2);
    Ddiag = zeros(sumcolpoints,4);
    Doffdiag = zeros(sum(colpoints.^2),4);
    for segcount = 1:numseg;
        % assignment index 
        sindex = (1:colpoints(segcount)) + cumsumcolpoints(segcount);
        oDindex = (1:colpoints(segcount)^2) + cumsumcolpointssq(segcount);
        
        % get LGR points, weights, and differentation matrix for segment
        [tau, weights, Doffdiagseg, DDdiagseg] = gpopsPointsWeightsLGRD(colpoints(segcount));
        
        % fraction vector for segment
        fractionvect(sindex,1) = fraction(segcount);
        
        % points for segment
        s(sindex,1) = (tau + 1).*fraction(segcount) + 2.*cumsumfraction(segcount) - 1;
        s(sindex,2) = tau;
        
        % weights for segment
        w(sindex,1) = weights.*fraction(segcount);
        w(sindex,2) = weights;
        
        % differentation matrix for segment
        Ddiag(sindex,1:2) = DDdiagseg(:,1:2) + cumsumcolpoints(segcount);
        Ddiag(sindex,3) = DDdiagseg(:,3)./fraction(segcount);
        Ddiag(sindex,4) = DDdiagseg(:,3);
        Doffdiag(oDindex,1:2) = Doffdiagseg(:,1:2) + cumsumcolpoints(segcount);
        Doffdiag(oDindex,3) = Doffdiagseg(:,3)./fraction(segcount);
        Doffdiag(oDindex,4) = Doffdiagseg(:,3);
    end
    numnodes(phasecount) = sumcolpoints;
    collocation(phasecount).fractionMat = diag(fractionvect);
    collocation(phasecount).s = s;
    collocation(phasecount).w = w;
    collocation(phasecount).Ddiag = Ddiag;
    collocation(phasecount).Doffdiag = Doffdiag;
    collocation(phasecount).dtdt0 = (1-s(:,1))/2;
    collocation(phasecount).dtdtf = (1+s(:,1))/2;
end