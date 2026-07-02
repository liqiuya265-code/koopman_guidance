function [numnodes, collocation] = gpopsPointsWeightsRPMI(currentmesh)

% gpopsPointsWeightsRPMI
% this function finds the points, quadrature weights, integration matrix
% and intial value matrix for the Segmented Radau Pseudospectral Method
% E and F are stored as sparsely
% (row, colunm, value)

% the first column of s are the collocation points for entire domain
% the second column of s are the LGR points, where each segment is on the
% domain [-1, 1) (the second column is used for interpulation only)

% s, w second column is values on [-1,1)
% E 4 column is values for [-1,1)

% get number of phases
numphase = size(currentmesh,2);

% preallocate fractionMat, collocation points, weights and diff matrix for
% each phase
numnodes = zeros(1,numphase);
collocation(numphase).fractionMat = [];
collocation(numphase).s = [];
collocation(numphase).w = [];
collocation(numphase).E = [];
collocation(numphase).Emat = [];
collocation(numphase).F = [];

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
    
    % preallocate fractionvect, s, E, F for each phase
    fractionvect = zeros(sumcolpoints,1);
    s = zeros(sumcolpoints,2);
    w = zeros(sumcolpoints,2);
    E = zeros(sum(colpoints.^2),4);
    F = zeros(sumcolpoints,3);
    for segcount = 1:numseg;
        % assignment index 
        sindex = (1:colpoints(segcount)) + cumsumcolpoints(segcount);
        Eindex = (1:(colpoints(segcount)^2)) + cumsumcolpointssq(segcount);
        
        % get LGR points, integration matrix, and initial value matrix for segment
        [tau, weights, Eseg, Fseg] = gpopsPointsWeightsLGRI(colpoints(segcount));
        
        % fraction vector for segment
        fractionvect(sindex,1) = fraction(segcount);
        
        % points for segment
        s(sindex,1) = (tau + 1).*fraction(segcount) + 2.*cumsumfraction(segcount) - 1;
        s(sindex,2) = tau;
        
        % weights for segment
        w(sindex,1) = weights.*fraction(segcount);
        w(sindex,2) = weights;
        
        % integration matrix for segment
        E(Eindex,1:2) = Eseg(:,1:2) + cumsumcolpoints(segcount);
        E(Eindex,3) = Eseg(:,3).*fraction(segcount);
        E(Eindex,4) = Eseg(:,3);
        
        % initial value matrix for segment
        F(sindex,1:2) = Fseg(:,1:2) + cumsumcolpoints(segcount);
        F(sindex,3) = Fseg(:,3);
    end
    numnodes(phasecount) = sumcolpoints;
    collocation(phasecount).fractionMat = sparse(diag(fractionvect));
    collocation(phasecount).s = s;
    collocation(phasecount).w = w;
    collocation(phasecount).E = E;
    collocation(phasecount).Emat = sparse(E(:,1), E(:,2), E(:,4), sumcolpoints, sumcolpoints);
    collocation(phasecount).F = F;
    collocation(phasecount).dtdt0 = (1-s(:,1))/2;
    collocation(phasecount).dtdtf = (1+s(:,1))/2;
end