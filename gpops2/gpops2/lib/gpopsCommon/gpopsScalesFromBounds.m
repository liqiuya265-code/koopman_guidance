function ocpscales = gpopsScalesFromBounds(setup, probinfo)

% gpopsScalesFromBounds
% this function finds variable scales and shifts, and the function scales
% of the optimal control problem
% the variable scales are based on the problem bounds
% the function scales for the dynamic constraints are equal to that of the
% state scale
% all other function scales are found by makeing the average norm of the
% the functions gradient equal to 1

% get OCP info
numphase = probinfo.numphase;
numstate = probinfo.numstate;
numcontrol = probinfo.numcontrol;
numpath = probinfo.numpath;
numintegral = probinfo.numintegral;
numparameter = probinfo.numparameter;
numeventgroup = probinfo.numeventgroup;
numevent = probinfo.numevent;

% get OCP function first derivatives for random values between bounds
[contgrdsamples, endpgrdsamples] = gpopsRandomGrd(setup, probinfo, setup.scales.numsamples);


%[contsamples, endpsamples] = gpopsRandom(setup, probinfo, setup.scales.numsamples);
%keyboard;

% preallocate scalesphase structure
scalesphase(numphase).statescale = [];
scalesphase(numphase).stateshift = [];
if sum(numcontrol,2) ~= 0;
    scalesphase(numphase).controlscale = [];
    scalesphase(numphase).controlshift = [];
end
scalesphase(numphase).t0scale = [];
scalesphase(numphase).t0shift = [];
scalesphase(numphase).tfscale = [];
scalesphase(numphase).tfshift = [];
if sum(numintegral,2) ~= 0;
    scalesphase(numphase).integralscale = [];
    scalesphase(numphase).integralshift = [];
end
scalesphase(numphase).dynamicsconscale = [];
if sum(numpath,2) ~= 0;
    scalesphase(numphase).pathconscale = [];
end
if sum(numintegral,2) ~= 0;
    scalesphase(numphase).integrandconscale = [];
end

% get cont derivative map
contmap1 = probinfo.derivativemap.contmap1;


% find scales and shifts
for phasecount = 1:numphase;
    % get bounds for each phase
    phasebounds = setup.bounds.phase(phasecount);
    
    % OCP info for phase
    numstatep = numstate(phasecount);
    numcontrolp = numcontrol(phasecount);
    numpathp = numpath(phasecount);
    numintegralp = numintegral(phasecount);
    
    % find scales and shifts for states and dynamic constraint scales
    statescale = zeros(1,numstatep);
    stateshift = zeros(1,numstatep);
    dynamicsconscale = zeros(1,numstatep);
    for statecount = 1:numstatep;
        statediff = phasebounds.state.upper(statecount) - phasebounds.state.lower(statecount);
        statesum = phasebounds.state.upper(statecount) + phasebounds.state.lower(statecount);
        % state scale and shift
        if statediff ~= 0;
            statescale(statecount) = 1/statediff;
            stateshift(statecount) = -(statesum/statediff)/2;
        else
            if phasebounds.state.upper(statecount) == 0;
                statescale(statecount) = 1;
                stateshift(statecount) = 0;
            else
                statescale(statecount) = 1/abs(phasebounds.state.upper(statecount));
                stateshift(statecount) = -sign(phasebounds.state.upper(statecount));
            end
        end
        % dynamic constraint scale
        dynamicsconscale(statecount) = statescale(statecount);
        %dynamicsconscale(statecount) = 1;
        % (alternative) get dynamic constraint scale ----------------------
        % find reference for dynamics gradient
        %refmarks = nonzeros(contmap1(phasecount).dynamicsmap1(statecount,:));
        % mean norm of the dynamics gradient
        %dynconscale(statecount) = 1/mean(sqrt(sum(contgrdsamples(phasecount).dynamicsgrd(:,refmarks).^2,2)));
    end
    % assign values in structure
    scalesphase(phasecount).statescale = statescale;
    scalesphase(phasecount).stateshift = stateshift;
    scalesphase(phasecount).dynamicsconscale = dynamicsconscale;
    
    % find scales and shifts for controls
    if numcontrolp ~= 0;
        controlscale = zeros(1,numcontrolp);
        controlshift = zeros(1,numcontrolp);
        for controlcount = 1:numcontrolp;
            controldiff = phasebounds.control.upper(controlcount) - phasebounds.control.lower(controlcount);
            controlsum = phasebounds.control.upper(controlcount) + phasebounds.control.lower(controlcount);
            if controldiff ~= 0;
                controlscale(controlcount) = 1/controldiff;
                controlshift(controlcount) = -(controlsum/controldiff)/2;
            else
                if phasebounds.control.upper(controlcount) == 0;
                    controlscale(controlcount) = 1;
                    controlshift(controlcount) = 0;
                else
                    controlscale(controlcount) = 1/abs(phasebounds.control.upper(controlcount));
                    controlshift(controlcount) = -sign(phasebounds.control.upper(controlcount));
                end
            end
        end
        % assign values in structure
        scalesphase(phasecount).controlscale = controlscale;
        scalesphase(phasecount).controlshift = controlshift;
    end
    
    % find scales and shifts for t0
    t0diff = phasebounds.initialtime.upper - phasebounds.initialtime.lower;
    t0sum = phasebounds.initialtime.upper + phasebounds.initialtime.lower;
    if t0diff ~= 0;
        scalesphase(phasecount).t0scale = 1/t0diff;
        scalesphase(phasecount).t0shift = -(t0sum/t0diff)/2;
    else
        if phasebounds.initialtime.upper == 0;
            scalesphase(phasecount).t0scale = 1;
            scalesphase(phasecount).t0shift = 0;
        else
            scalesphase(phasecount).t0scale = 1/abs(phasebounds.initialtime.upper);
            scalesphase(phasecount).t0shift = -sign(phasebounds.initialtime.upper);
        end
    end
    
    % find scales and shifts for tf
    tfdiff = phasebounds.finaltime.upper - phasebounds.finaltime.lower;
    tfsum = phasebounds.finaltime.upper + phasebounds.finaltime.lower;
    if tfdiff ~= 0;
        scalesphase(phasecount).tfscale = 1/tfdiff;
        scalesphase(phasecount).tfshift = -(tfsum/tfdiff)/2;
    else
        if phasebounds.finaltime.upper == 0;
            scalesphase(phasecount).tfscale = 1;
            scalesphase(phasecount).tfshift = 0;
        else
            scalesphase(phasecount).tfscale = 1/abs(phasebounds.finaltime.upper);
            scalesphase(phasecount).tfshift = -sign(phasebounds.finaltime.upper);
        end
    end
    
    % find pathconscale constraint scales
    if numpathp ~= 0;
        pathconscale = zeros(1,numpathp);
        for pathcount = 1:numpathp;
            % find reference for path gradient
            refmarks = nonzeros(contmap1(phasecount).pathmap1(pathcount,:));
            % mean norm of the path gradient
            pathconscale(pathcount) = 1/mean(sqrt(sum(contgrdsamples(phasecount).pathgrd(:,refmarks).^2,2)));
            %pathconscale(pathcount) = 1/300000;
            %pathconscale(pathcount) = 1;
        end
        % assign values in structure
        scalesphase(phasecount).pathconscale = pathconscale;
    end
    
    % find integral variable scales and shifts and intergrand constraint scales
    if numintegralp ~= 0;
        integralscale = zeros(1,numintegralp);
        integralshift = zeros(1,numintegralp);
        integrandconscale = zeros(1,numintegralp);
        for integralcount = 1:numintegralp;
            % scale and shift for intergal variable
            integraldiff = phasebounds.integral.upper(integralcount) - phasebounds.integral.lower(integralcount);
            integralsum = phasebounds.integral.upper(integralcount) + phasebounds.integral.lower(integralcount);
            if integraldiff ~= 0;
                integralscale(integralcount) = 1/integraldiff;
                integralshift(integralcount) = -(integralsum/integraldiff)/2;
            else
                if phasebounds.integral.upper(integralcount) == 0;
                    integralscale(integralcount) = 1;
                    integralshift(integralcount) = 0;
                else
                    integralscale(integralcount) = 1/abs(phasebounds.integral.upper(integralcount));
                    integralshift(integralcount) = -sign(phasebounds.integral.upper(integralcount));
                end
            end
            % integrandconscale constraint scale
            integrandconscale(integralcount) = integralscale(integralcount);
            %integrandconscale(integralcount) = 1/10000;
            % (alternative) get integral constraint scale -----------------
            % find reference for integrand gradient
            %refmarks = nonzeros(contmap1(phasecount).integrandmap1(integralcount,:));
            % mean norm of the integrand gradient
            %integrandconscale(integralcount) = mean(sqrt(sum(contgrdsamples(phasecount).integrandgrd(:,refmarks).^2,2)));
        end
        % assign values in structure
        scalesphase(phasecount).integralscale = integralscale;
        scalesphase(phasecount).integralshift = integralshift;
        scalesphase(phasecount).integrandconscale = integrandconscale;
    end
end

% find parameter scales and shifts
if numparameter ~= 0;
    parabounds = setup.bounds.parameter;
    parameterscale = zeros(1,numparameter);
    parametershift = zeros(1,numparameter);
    for parametercount = 1:numparameter;
        paradiff = parabounds.upper(parametercount) - parabounds.lower(parametercount);
        parasum = parabounds.upper(parametercount) + parabounds.lower(parametercount);
        if paradiff ~= 0;
            parameterscale(parametercount) = 1/paradiff;
            parametershift(parametercount) = -(parasum/paradiff)/2;
        else
            if parabounds.upper(parametercount) == 0;
                parameterscale(parametercount) = 1;
                parametershift(parametercount) = 0;
            else
                parameterscale(parametercount) = 1/abs(parabounds.upper(parametercount));
                parametershift(parametercount) = -sign(parabounds.upper(parametercount));
            end
        end
    end
end

% objective scale
objscale = 1/mean(sqrt(sum(endpgrdsamples.objectivegrd.^2,2)));

% find event constraint scales
if numeventgroup ~= 0;
    % preallocate scaleseventgroup structure
    scaleseventgroup(numeventgroup).eventconscale = [];
    for eventgroupcount = 1:numeventgroup;
        % get eventfunmap1
        eventfunmap1 = probinfo.derivativemap.eventfunmap1(eventgroupcount).first;
        % preallocate event scales
        eventconscale = zeros(1,numevent(eventgroupcount));
        for eventcount = 1:numevent(eventgroupcount);
            % find reference for event gradient
            refmarks = nonzeros(eventfunmap1(eventcount,:));
            % mean norm of the path gradient
            eventconscale(eventcount) = 1/mean(sqrt(sum(endpgrdsamples.eventgroup(eventgroupcount).eventgrd(:,refmarks).^2,2)));
        end
        % assign values in structure
        scaleseventgroup(eventgroupcount).eventconscale = eventconscale;
    end
end

% put scalesphase into setup structure
ocpscales.phase = scalesphase;

% objective scale (set to 1)
ocpscales.objscale = 1;

% put parameter scales and shifts into setup structure
if numparameter ~= 0;
    ocpscales.parameterscale = parameterscale;
    ocpscales.parametershift = parametershift;
end

% put event scales in setup structure
if numeventgroup ~= 0;
    ocpscales.eventgroup = scaleseventgroup;
end