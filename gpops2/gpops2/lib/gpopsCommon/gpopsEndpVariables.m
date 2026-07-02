function endpvarloc = gpopsEndpVariables(probinfo)

% gpopsEndpVariables
% this function gets endpoint function variable locations
% each column reposents a variable in the endpoint function
% the first row contains the phase the variable is in 
% (parameters are listed as phase 0)
% the second row contains the variable number in that phase

% get OCP sizes
numphase = probinfo.numphase;
numstate = probinfo.numstate;
numintegral = probinfo.numintegral;
statesum = sum(numstate,2);
integrandsum = sum(numintegral,2);
numparameters = probinfo.numparameter;

% total number of endpoint variables
numOCPendpvar = 2*statesum+integrandsum+2*numphase+numparameters;

% preallocate endpvarloc
endpvarloc = zeros(2,numOCPendpvar);

refmarker = 0;
for phasecount = 1:numphase
    % number of variables in each phase
    numvarphase = 2*numstate(phasecount)+2+numintegral(phasecount);
    phaseindex = 1:numvarphase;
    % the first row contains the phase the variable is in
    % the second row contains the variable number in that phase
    endpvarloc(:,phaseindex+refmarker) = [phasecount*ones(1,numvarphase); phaseindex];
    refmarker = refmarker+numvarphase;
end
% parameters are inidcated with 0 in the first column
phaseindex = 1:numparameters;
endpvarloc(:,phaseindex+refmarker) = [zeros(1,numparameters); phaseindex];