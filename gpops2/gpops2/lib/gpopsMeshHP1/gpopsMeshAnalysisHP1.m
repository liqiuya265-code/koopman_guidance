function [meshaccepted, newmesh, maxerror] = gpopsMeshAnalysisHP1(result, setup)

% gpopsMeshAnalysisHP1
% this function interpolates the result on to a new mesh of N+1 Radau
% points in each section
% the state is interpolated using the Nth degree lagrange polynomial in
% each section
% the control is interpolated using a cubic in each segment
% the error is defined as the difference between the interpolated state and
% the integrated right hand side evaluated at the interpolated state,
% control and time

% meshaccepted is assumed true unless integration
% difference on N+1 mesh does not meet tolerance
meshaccepted = true;

% get mesh refinement options
mesh = setup.mesh;

% get number of phases
numphases = size(result.solution.phase,2);

% add 1 to the number of collocation points
currentmesh = setup.mesh.phase;
meshinterp = currentmesh;
for phasecount = 1:numphases;
    meshinterp(phasecount).colpoints = meshinterp(phasecount).colpoints + 1;
end

% get collocation points, integration matrix and intial value matrix for
% meshinterp
[numnodes, collocationInt] = gpopsPointsWeightsRPMI(meshinterp);

% preallocate interpsolution
interpphase(numphases).time = [];
interpphase(numphases).state = [];
interpphase(numphases).control = [];

% initiate max error
maxerror = 0;

% interp solution, control
for phasecount = 1:numphases;
    % get solution for each phase
    phasesol = result.solution.phase(phasecount);
    
    % get result mesh and interp for each phase
    interpcolpoints = meshinterp(phasecount).colpoints;
    resultcolpoints = currentmesh(phasecount).colpoints;
    sinterp = collocationInt(phasecount).s;
    ssol = [result.collocation(phasecount).s; [1 1]];
    
    % time on interp mesh for phase
    t0 = phasesol.time(1);
    tf = phasesol.time(end);
    interpphase(phasecount).time = (sinterp(:,1) + 1).*(tf - t0)./2 + t0;
    
    % get number of segments
    numseg = size(interpcolpoints,2);
    
    % preallocate state and control for each phase
    numstate = size(phasesol.state,2);
    stateinterp = zeros(numnodes(phasecount), numstate);
    if ~isempty(phasesol.control);
        numcontrol = size(phasesol.control,2);
        controlinterp = zeros(numnodes(phasecount), numcontrol);
    else
        controlinterp = [];
    end
    
    % initalize segment markers
    solstart = 1;
    intstart = 1;
    for segcount = 1:numseg;
        % update end marker
        solend = solstart + resultcolpoints(segcount);
        intend = intstart + interpcolpoints(segcount)-1;
        
        % get solution and interpolation index
        solindex = (solstart:solend)';
        intindex = (intstart:intend)';
        
        % get segment LGR points
        ssolseg = ssol(solindex,2);
        ssolseg(end) = 1;
        sinterpseg = sinterp(intindex,2);
        
        % interp state using lagrange polynomial
        segstate = phasesol.state(solindex,:);
        stateinterp(intindex,:) = gpopsLagrangeInterp(ssolseg, segstate, sinterpseg);
        
        % interp control
        if ~isempty(phasesol.control);
            segcontrol = phasesol.control(solindex,:);
            
            %interp using lagrange polynomial
            controlinterp(intindex,:) = gpopsLagrangeInterp(ssolseg(1:end-1,:), segcontrol(1:end-1,:), sinterpseg);
        end
        
        % update start marker
        solstart = solend;
        intstart = intend+1;
    end
    % Get full state (state at radau points including end point at 1
    statefull = [stateinterp; phasesol.state(end,:)];
    
    % save interp state and control for each phase
    interpphase(phasecount).state = stateinterp;
    interpphase(phasecount).statefull = statefull;
    interpphase(phasecount).control = controlinterp;
end

%
interpsolution.phase = interpphase;
if isfield(result.solution, 'parameter');
    interpsolution.parameter = result.solution.parameter;
end

% evaluate continuous function on interpsolution
contoutput = gpopsEvalCont(interpsolution, setup);

% find integration error
% preallocate interror
phaseerror(numphases).absdifference = [];
for phasecount = 1:numphases;
    % get interpolated state
    statefullp = interpsolution.phase(phasecount).statefull;
    
    % get initial and final time
    N = numnodes(phasecount);
    t0 = result.solution.phase(phasecount).time(1);
    tf = result.solution.phase(phasecount).time(end);
    
    % get integration and intital value matrices
    Ep = collocationInt(phasecount).Emat;
    Fp = sparse(collocationInt(phasecount).F(:,1), collocationInt(phasecount).F(:,2), collocationInt(phasecount).F(:,3), N, N+1);
    
    % scale dynamics by segement fractionages
    contoutput(phasecount).dynamics = full(collocationInt(phasecount).fractionMat*contoutput(phasecount).dynamics);
    
    % find inegrated state
    intstate = [statefullp(1,:); Fp*statefullp + (tf-t0)/2.*(Ep*contoutput(phasecount).dynamics)];
    
    % find difference
    statescalemat = diag(1./(max(abs(statefullp))+1));
    phaseerror(phasecount).absdifference = abs(statefullp - intstate)*statescalemat;
end

% preallocate newmesh = mesh
newmesh = currentmesh;

disp('                  ');
disp(' _______________________________________________________________________________________________________');
disp('|                                                                                                       |');
disp('|                              Analysis of Mesh in Each Phase of Problem                                |');
disp('|_______________________________________________________________________________________________________|');
disp('');
% find if mesh meets tolerance
% check if tolerance is meet in each phase
for phasecount = 1:numphases;
  meshString = strcat(['                                    | Analysis of Mesh in Phase ',num2str(phasecount),' |']);
  if phasecount<10
    disp('                                     _____________________________ ');
    disp('                                    |                             |');
    disp(meshString);
    disp('                                    |_____________________________|');
  else
    disp('                                     ______________________________ ');
    disp('                                    |                              |');
    disp(meshString);
    disp('                                    |______________________________|');
  end;
  % get max error
  phasemaxerror = max(max(phaseerror(phasecount).absdifference));
  
  % save max error
  if phasemaxerror > maxerror;
      maxerror = phasemaxerror;
  end
  
  % check if tolerance is meet in phase
  disp('    ');	       
  if phasemaxerror <= mesh.tolerance;
    % tolerance is meet in phase
    newcolpoints = currentmesh(phasecount).colpoints;
    newfraction = currentmesh(phasecount).fraction;
    disp(['Maximum Relative Error on Current Mesh in Phase ',num2str(phasecount),' = ',num2str(phasemaxerror)]);
    disp(['Mesh Error Tolerance IS satisfied in Phase ',num2str(phasecount)]);
  else
    % tolerance not meet in phase
    % the mesh will need to be refined and problem resolved
    disp(' ');
    disp(['Maximum Relative Error on Current Mesh in Phase ',num2str(phasecount),' = ',num2str(phasemaxerror)]);
    disp(['Mesh Error Tolerance is NOT satisfied in Phase ',num2str(phasecount)]);
    disp('     ');
    if phasecount<10,
      disp('                                  _________________________________');
      disp('                                 |                                 |');
      disp(['                                 | Modification of Mesh in Phase ',num2str(phasecount),' |']);
      disp('                                 |_________________________________|');
    else
      disp('                                  __________________________________');
      disp('                                 |                                  |');
      disp(['                                 | Modification of Mesh in Phase ',num2str(phasecount),' |']);
      disp('                                 |__________________________________|');
    end;
    disp('     ');

    meshaccepted = false;
    % get result and interpolation mesh and collocation points for each phase
    interpcolpoints = meshinterp(phasecount).colpoints;
    resultcolpoints = currentmesh(phasecount).colpoints;
    resultfraction = currentmesh(phasecount).fraction;
    
    % initialize newcolpoints and newfraction
    newcolpoints = newmesh(phasecount).colpoints;
    newfraction = newmesh(phasecount).fraction;
    
    % get number of segments
    numseg = size(interpcolpoints,2);
    
    % initalize segment markers
    solstart = 1;
    intstart = 1;
    for segcount = 1:numseg;
      % get result colication points and fraction for each segment
      segcolpoints = resultcolpoints(segcount);
      segfraction = resultfraction(segcount);
      
      % update end marker
      solend = solstart + resultcolpoints(segcount);
      intend = intstart + interpcolpoints(segcount)-1;
      
      % get solution and interpolation index
      intindex = (intstart:intend)';
      
      % get error in segment
      segerror = phaseerror(phasecount).absdifference(intindex,:);
      
      % find the state with largest error in segment
      segmaxerror = max(max(segerror));
      
      if segmaxerror > mesh.tolerance;
        % modify segment
        [segcolpoints, segfraction] = gpopsMeshModifySegmentHP1(segcount, mesh, segcolpoints, segfraction, segmaxerror);
      end
      % save new mesh
      if segcount == 1;
        % if phase only has 1 segment, replace mesh in phase
        newcolpoints = segcolpoints;
        newfraction = segfraction;
      else
        % if phase only has 1 segment, replace mesh in phase
        newcolpoints = [newcolpoints, segcolpoints];
        newfraction = [newfraction, segfraction];
      end
      
      % update start marker
      solstart = solend;
      intstart = intend+1;
    end
    newfraction(end) = 1 - sum(newfraction(1:end-1));
  end
  % save newmesh for each phase
  newmesh(phasecount).colpoints = newcolpoints;
  newmesh(phasecount).fraction = newfraction;
  
  % print line break
  disp(' ');
end
