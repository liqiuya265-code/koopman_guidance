function derivativemap = gpopsDependSparseNaN(probinfo, setup)

% gpopsDependSparseNaN
% This function gets the optimal control problem dependencies for either
% the first or second derivative levels. the functions of the optimal
% control problem are evaluated with one of the variables of the optimal
% control problem having the value 'NaN' to find the function dependencies,
% this process is repeated for all variables. The locations of the
% zeros are removed from the first derivative map, the second derivative
% map is created from a projection of the first derivative

if probinfo.derivativelevel == 1;
    % start by initiating a full first derivative level
    probinfo.derivativemap = gpopsDependFull(probinfo, 1);
    
    % find NaN dependencies of continuous and endpoint functions
    [contdependnan, endpdependnan] = gpopsEvalNaN(setup, probinfo);
    
    % remove first derivative zeros
    derivativemap = gpopsRemoveZeros1(probinfo, contdependnan, endpdependnan);
else
    % probinfo.derivativelevel == 2;
    % start by initiating a full second derivative level
    probinfo.derivativemap = gpopsDependFull(probinfo, 1);
    
    % find NaN dependencies of continuous and endpoint functions
    [contdependnan, endpdependnan] = gpopsEvalNaN(setup, probinfo);
    
    % remove first derivative zeros
    derivativemap = gpopsRemoveZeros1(probinfo, contdependnan, endpdependnan);
    
    % estimate second derivative map from first derivative map
    derivativemap = gpopsDependSecondFromFirst(derivativemap, probinfo);
end