function [continput, endpinput] = gpopsInputFromGuess(setup)

% gpopsInputFromGuess
% this function generates the inputs for the continuous and endpoint
% functions from the guess

% get guess
guess = setup.guess;

% get number of phases
numphase = length(guess.phase);

% get number of parameters
if isfield(guess,'parameter');
    numparameter = length(guess.parameter);
else
    numparameter = 0;
end

% preallocate contphase and endpphase
contphase(numphase).state = [];
endpphase(numphase).initialstate = [];
endpphase(numphase).finalstate = [];
if isfield(guess.phase, 'control');
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
if isfield(guess.phase, 'integral');
    integralswitch = true;
    endpphase(numphase).integral = [];
else
    integralswitch = false;
end

% get input values for each phase
for phasecount = 1:numphase;
    % get guess for each phase
    guessphase = guess.phase(phasecount);
    
    % get input for state guess
    contphase(phasecount).state = guessphase.state;
    endpphase(phasecount).initialstate = guessphase.state(1,:);
    endpphase(phasecount).finalstate = guessphase.state(end,:);
    
    % get input for control guess
    if controlswitch;
        contphase(phasecount).control = guessphase.control;
    end
    
    % get input for time guess
    contphase(phasecount).time = guessphase.time;
    endpphase(phasecount).initialtime = guessphase.time(1);
    endpphase(phasecount).finaltime = guessphase.time(end);
    
    % get contphase for parameter guess
    if numparameter ~= 0;
        contphase(phasecount).parameter = ones(length(guessphase.time),1)*guess.parameter;
    end
    
    % get endpphase for integral guess
    if integralswitch;
        endpphase(phasecount).integral = guessphase.integral;
    end
end

% add guess for all phases to continput and endpinput
continput.phase = contphase;
endpinput.phase = endpphase;

% get endpinput for parameter guess
if numparameter ~= 0;
    endpinput.parameter = guess.parameter;
end

% add auxdata to continput and endpinput
if isfield(setup,'auxdata');
    continput.auxdata = setup.auxdata;
    endpinput.auxdata = setup.auxdata;
end