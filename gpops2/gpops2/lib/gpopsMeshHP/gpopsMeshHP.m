function [result, meshhistory, meshiterations] = gpopsMeshHP(setup, probinfo)

% gpopsMeshHP
% HP mesh refinement

% get mesh refinement options
mesh = setup.mesh;

% perform mesh analysis
% initialize meshinteration = 0;
meshiteration = 0;

% initialize meshaccepted = false to start while loop
meshaccepted = false;

% initialize meshhistory
maxiteration = mesh.maxiteration;
meshhistory(maxiteration).result = [];

% while the mesh is not accepted
while ~meshaccepted;
  % increase meshint
  meshiteration = meshiteration + 1;
  disp(' ');
  if meshiteration<10, 
    disp(' _______________________________________________________________________________________________________');
    disp('|                                                                                                       |');
    disp(['|------------------------------------------ Mesh iteration ',num2str(meshiteration),' -------------------------------------------|']);
    disp('|_______________________________________________________________________________________________________|');
  else
    disp(' _______________________________________________________________________________________________________');
    disp('|                                                                                                       |');
    disp(['|------------------------------------------ Mesh iteration ',num2str(meshiteration),' ------------------------------------------|']);
    disp('|_______________________________________________________________________________________________________|');
  end;
  disp('  ');
  
  % get solution on current mesh
  % call Radau pseudospectral method shell
  if strcmpi(setup.method, 'RPMdifferentiation');
    % use differentiation method
    result = gpopsSolveRPMD(setup, probinfo);
  elseif strcmpi(setup.method, 'RPMintegration');
    % use integration method
    result = gpopsSolveRPMI(setup, probinfo);
  end
  
  % find integration result performed at N+1 LGR points
  [meshaccepted, newmesh, maxerror] = gpopsMeshAnalysisHP(result, setup);
  
  % save setup as field of result
  result.setup = setup;
  result.maxerror = maxerror;
  
  % store mesh history
  meshhistory(meshiteration).result = result;
  meshiterations = meshiteration;
  
  % determine if the mesh is accepted
  if ~meshaccepted;
    if meshiteration < maxiteration;
      % if mesh is not accepted and iteration limit not reached
      % resolve problem on new mesh
      setup.mesh.phase = newmesh;
      setup.guess = result.solution;
      
      % re use scales without re computing them
      if probinfo.scaleflag;
        setup.scales = result.ocpscales;
        setup.scales.method = 'defined';
      end
    else
      meshaccepted = true;
      disp('  Mesh iteration limit reached without satisfying error tolerance');
    end
  end
end

% remove empty fields from meshhistory
meshhistory = meshhistory(1:meshiterations);