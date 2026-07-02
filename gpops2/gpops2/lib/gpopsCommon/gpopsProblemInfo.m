function probinfo = gpopsProblemInfo(setup)

% gpopsProblemInfo
% This function collects information about the optimal control problem

% get OCP function names
probinfo.contfunction = setup.functions.continuous;
probinfo.endpfunction = setup.functions.endpoint;

% set derivative level
if strcmpi(setup.derivatives.derivativelevel,'first');
    probinfo.derivativelevel = 1;
elseif strcmpi(setup.derivatives.derivativelevel,'second');
    probinfo.derivativelevel = 2;
else
    error('derivativelevel not defined');
end

if strcmpi(setup.derivatives.supplier, 'analytic');
    probinfo.analyticflag = true;
    % get OCP analytic first derivative Grdfunction names
    if isfield(setup.derivatives,'Grdfunctions');
        probinfo.contgrd = setup.derivatives.Grdfunctions.continuous;
        probinfo.endpgrd = setup.derivatives.Grdfunctions.endpoint;
    else
        error('first derivative function files not provided.');
    end
    % get OCP analytic second derivative Hesfunction names
    if probinfo.derivativelevel == 2;
        if isfield(setup.derivatives,'Hesfunctions');
            probinfo.conthes = setup.derivatives.Hesfunctions.continuous;
            probinfo.endphes = setup.derivatives.Hesfunctions.endpoint;
        else
            error('second derivative function files not provided.');
        end
    end
else
    % define function handles for sparse finite difference methods
    probinfo.analyticflag = false;
    if strcmpi(setup.derivatives.supplier, 'sparseCD');
        probinfo.contgrd = @gpopsContFirstCD;
        probinfo.endpgrd = @gpopsEndpFirstCD;
        probinfo.objgrd = @gpopsObjFirstCD;
        probinfo.eventgrd = @gpopsEventFirstCD;
        if probinfo.derivativelevel == 2;
            probinfo.conthes = @gpopsContSecondCD;
            probinfo.endphes = @gpopsEndpSecondCD;
        end
    elseif strcmpi(setup.derivatives.supplier, 'sparseFD');
        probinfo.contgrd = @gpopsContFirstFD;
        probinfo.endpgrd = @gpopsEndpFirstFD;
        probinfo.objgrd = @gpopsObjFirstFD;
        probinfo.eventgrd = @gpopsEventFirstFD;
        if probinfo.derivativelevel == 2;
            probinfo.conthes = @gpopsContSecondFD;
            probinfo.endphes = @gpopsEndpSecondFD;
        end
    elseif strcmpi(setup.derivatives.supplier, 'sparseBD');
        probinfo.contgrd = @gpopsContFirstBD;
        probinfo.endpgrd = @gpopsEndpFirstBD;
        probinfo.objgrd = @gpopsObjFirstBD;
        probinfo.eventgrd = @gpopsEventFirstBD;
        if probinfo.derivativelevel == 2;
            probinfo.conthes = @gpopsContSecondBD;
            probinfo.endphes = @gpopsEndpSecondBD;
        end
    end
end

% get auxdata data
if isfield(setup,'auxdata');
    probinfo.auxflag = true;
    probinfo.auxdata = setup.auxdata;
else
    probinfo.auxflag = false;
end

if strcmpi(setup.scales.method, 'none');
    probinfo.scaleflag = false;
else
    probinfo.scaleflag = true;
end

% get number of phases
numphase = size(setup.bounds.phase,2);
probinfo.numphase = numphase;

% dertermine if any controls exsist
if ~isfield(setup.bounds.phase,'control');
    setup.bounds.phase(numphase).control = [];
end

% determine if path constraints exsist
if ~isfield(setup.bounds.phase,'path');
    setup.bounds.phase(numphase).path = [];
end

% determine if integral constraints exsist
if ~isfield(setup.bounds.phase,'integral');
    setup.bounds.phase(numphase).integral = [];
end

% determine if duration constraints exsist
if ~isfield(setup.bounds.phase,'duration');
    setup.bounds.phase(numphase).duration = [];
end

% find number of states, controls, path constraints and
% integral constraints in each phase
% num_state, num_control, num_path, num_integral are all
% (1 by num_phase) integer array
% each value indicates the number for that phase
numstate    = zeros(1,numphase);
numcontrol  = zeros(1,numphase);
numpath     = zeros(1,numphase);
numintegral = zeros(1,numphase);
phaseduration = false(1,numphase);
for phasecount = 1:numphase;
    numstate(1,phasecount) = size(setup.bounds.phase(phasecount).state.lower,2);
    if ~isempty(setup.bounds.phase(phasecount).control);
        numcontrol(1,phasecount) = size(setup.bounds.phase(phasecount).control.lower,2);
    end
    if ~isempty(setup.bounds.phase(phasecount).path);
        numpath(1,phasecount) = size(setup.bounds.phase(phasecount).path.lower,2);
    end
    if ~isempty(setup.bounds.phase(phasecount).integral);
        numintegral(1,phasecount) = size(setup.bounds.phase(phasecount).integral.lower,2);
    end
    % find what phases have duration constraints
    % phase_duration is (1 by num_phase) logical array
    % false when no defined duration is in phase, true when defined duration exsist
    phaseduration(1,phasecount) = ~isempty(setup.bounds.phase(phasecount).duration);
end
probinfo.numstate      = numstate;
probinfo.numcontrol    = numcontrol;
probinfo.numpath       = numpath;
probinfo.numintegral   = numintegral;
probinfo.phaseduration = phaseduration;

% find number of parameters
% num_parameter is an integer value
if isfield(setup.bounds,'parameter');
    numparameter = size(setup.bounds.parameter.lower,2);
    probinfo.numparameter = numparameter;
else
    probinfo.numparameter = 0;
end

% find number of eventgroups and number of events per each eventgroup
% num_event is (1 by num_eventgroups) integer array
% each value indicates the number of events for that eventgroup
if isfield(setup.bounds,'eventgroup');
    numeventgroup = size(setup.bounds.eventgroup,2);
    numevent = zeros(1,numeventgroup);
    for eventgroupcount = 1:numeventgroup;
        numevent(1,eventgroupcount) = size(setup.bounds.eventgroup(eventgroupcount).lower,2);
    end
    probinfo.numeventgroup = numeventgroup;
    probinfo.numevent      = numevent;
else
    probinfo.numeventgroup = 0;
    probinfo.numevent      = 0;
end

% find if initial and final times are fixed in each phase
% fixedtime.flag is (2 by numphase) logical array
% row 1 represents intial, row 2 represents final
% 1 if time is fixed, 0 if free
%
% fixedtime.value is (1 by numphase) double array
% contains the values of the fixed initial or final time
% (only valid when the corresponding flag is 1)
fixedtimeflag  = false(2,numphase);
fixedtimevalue = zeros(2,numphase);
for phasecount = 1:numphase;
    % check initial time
    if setup.bounds.phase(phasecount).initialtime.lower == setup.bounds.phase(phasecount).initialtime.upper;
        fixedtimeflag(1,phasecount)  = true;
        fixedtimevalue(1,phasecount) = setup.bounds.phase(phasecount).initialtime.lower;
    end
    % check final time
    if setup.bounds.phase(phasecount).finaltime.lower == setup.bounds.phase(phasecount).finaltime.upper;
        fixedtimeflag(2,phasecount)  = true;
        fixedtimevalue(2,phasecount) = setup.bounds.phase(phasecount).finaltime.lower;
    end
end
probinfo.fixedtimeflag  = fixedtimeflag;
probinfo.fixedtimevalue = fixedtimevalue;

% base step size
probinfo.stepsize = setup.derivatives.stepsize;