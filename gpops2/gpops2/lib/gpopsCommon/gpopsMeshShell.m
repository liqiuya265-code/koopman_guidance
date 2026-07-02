function output = gpopsMeshShell(setup, probinfo)

% gpopsMeshShell
% this is the shell function where different mesh refinement algorithms are
% called

% ouput fields
output.name = setup.name;

% so that result is the second field of output
output.result = [];

% this function calls the different mesh refinement algorithms
if strcmpi(setup.mesh.method,'none');
    % no refinement algorithm
    % call Radau pseudospectral method shell
    if strcmpi(setup.method, 'RPMdifferentiation');
        % use differentiation method
        result = gpopsSolveRPMD(setup, probinfo);
    elseif strcmpi(setup.method, 'RPMintegration');
        % use integration method
        result = gpopsSolveRPMI(setup, probinfo);
    end
    
    % save setup as field of result
    result.setup = setup;
elseif strcmpi(setup.mesh.method,'hp');
    % call hp mesh refinement algorithm
    [result, meshhistory, meshiterations] = gpopsMeshHP(setup, probinfo);
    
    % ouput fields
    output.meshiterations = meshiterations;
    output.meshhistory = meshhistory;
elseif strcmpi(setup.mesh.method,'hp1');
    % call hp mesh refinement algorithm
    [result, meshhistory, meshiterations] = gpopsMeshHP1(setup, probinfo);
    
    % ouput fields
    output.meshiterations = meshiterations;
    output.meshhistory = meshhistory;
end

% interp final solution here
result = gpopsInterpResult(result);

% ouput fields
output.result = result;