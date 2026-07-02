function [contoutput, endpoutput] = gpopsEvalUserFun(result)

% gpopsEvalUserFun
% this function evaluates the continuous and endpoint functions on the
% solution

% extract setup and solution
setup = result.setup;
solution = result.solution;

% get number of phases
numphase = length(solution.phase);

% get number of parameters
if isfield(solution,'parameter');
    numparameter = length(solution.parameter);
else
    numparameter = 0;
end

% preallocate contphase and endpphase
contphase(numphase).state = [];
endpphase(numphase).initialstate = [];
endpphase(numphase).finalstate = [];
if isfield(solution.phase, 'control');
    controlswitch = true;
    contphase(numphase).control = [];
else
    controlswitch = false;
end
contphase(numphase).time = [];
endpphase(numphase).initialtime = [];
endpphase(numphase).finaltime = [];
if numparameter ~= 0;
    contphase(numphase).parameter = [];
end
if isfield(solution.phase, 'integral');
    integralswitch = true;
    endpphase(numphase).integral = [];
else
    integralswitch = false;
end

% get input values for each phase
for phasecount = 1:numphase;
    % get solution for each phase
    solphase = solution.phase(phasecount);
    
    % get input for state solution
    contphase(phasecount).state = solphase.state;
    endpphase(phasecount).initialstate = solphase.state(1,:);
    endpphase(phasecount).finalstate = solphase.state(end,:);
    
    % get input for control solution
    if controlswitch;
        contphase(phasecount).control = solphase.control;
    end
    
    % get input for time solution
    contphase(phasecount).time = solphase.time;
    endpphase(phasecount).initialtime = solphase.time(1);
    endpphase(phasecount).finaltime = solphase.time(end);
    
    % get contphase for parameter solution
    if numparameter ~= 0;
        contphase(phasecount).parameter = ones(length(solphase.time),1)*solution.parameter;
    end
    
    % get endpphase for integral solution
    if integralswitch;
        endpphase(phasecount).integral = solphase.integral;
    end
end

% adding solution for all phases to continput and endpinput
continput.phase = contphase;
endpinput.phase = endpphase;

% get endpinput for parameter solution
if numparameter ~= 0;
    endpinput.parameter = solution.parameter;
end

% add auxdata to continput and endpinput
if isfield(setup,'auxdata');
    continput.auxdata = setup.auxdata;
    endpinput.auxdata = setup.auxdata;
end

% evaluate OCP continuous function
contoutput = feval(setup.functions.continuous, continput);

% evaluate OCP endpoint function
endpoutput = feval(setup.functions.endpoint, endpinput);