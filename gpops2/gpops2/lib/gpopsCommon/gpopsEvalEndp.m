function endpoutput = gpopsEvalEndp(solution, setup)

% gpopsEvalEndp
% this function evaluates the endpoint function on the solution

% get number of phases
numphase = length(solution.phase);

% get number of parameters
if isfield(solution,'parameter');
    numparameter = length(solution.parameter);
else
    numparameter = 0;
end

% preallocate endpphase
endpphase(numphase).initialstate = [];
endpphase(numphase).finalstate = [];
endpphase(numphase).initialtime = [];
endpphase(numphase).finaltime = [];
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
    endpphase(phasecount).initialstate = solphase.state(1,:);
    endpphase(phasecount).finalstate = solphase.state(end,:);

    % get input for time solution
    endpphase(phasecount).initialtime = solphase.time(1);
    endpphase(phasecount).finaltime = solphase.time(end);
    
    % get endpphase for integral solution
    if integralswitch;
        endpphase(phasecount).integral = solphase.integral;
    end
end

% adding solution for all phases to endpinput
endpinput.phase = endpphase;

% get endpinput for parameter solution
if numparameter ~= 0;
    endpinput.parameter = solution.parameter;
end

% add auxdata to endpinput
if isfield(setup,'auxdata');
    endpinput.auxdata = setup.auxdata;
end

% evaluate OCP endpoint function
endpoutput = feval(setup.functions.endpoint, endpinput);