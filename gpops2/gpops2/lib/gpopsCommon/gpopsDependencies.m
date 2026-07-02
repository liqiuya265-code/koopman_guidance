function probinfo = gpopsDependencies(setup, probinfo)

% gpopsDependencies
% This function gets the optimal control problem dependencies for either
% the first or second derivative levels

% get endpoint function variable locations
probinfo.endpvarloc = gpopsEndpVariables(probinfo);

% if analytic derivatives are given the OCP dependencies are found directly
% from the derivative functions
if strcmpi(setup.derivatives.supplier, 'analytic');
    derivativemap = gpopsDependAnalytic(probinfo, setup);
else
    % full OCPdependencies assumes every function of the optimal control
    % problem has a derivative with respect to every variable in the
    % problem
    if strcmpi(setup.derivatives.dependencies, 'full');
        derivativemap = gpopsDependFull(probinfo, probinfo.derivativelevel);
    end
    
    % the optimal control problem dependencies are found using
    % finite differencing to differentiate the optimal control problem at
    % many sample points
    if strcmpi(setup.derivatives.dependencies, 'sparse');
        derivativemap = gpopsDependSparse(probinfo, setup);
    end
    
    % the optimal control problem dependencies are found using
    % NaN as a variable value to find function dependencies, second
    % derivatives are projected from the first derivatives
    if strcmpi(setup.derivatives.dependencies, 'sparsenan');
        derivativemap = gpopsDependSparseNaN(probinfo, setup);
    end
end

% get continuous function Hessian map
if probinfo.derivativelevel == 2
    derivativemap = gpopsContHesMap(probinfo, derivativemap);
end

% add derivative map to probinfo structure
probinfo.derivativemap = derivativemap;