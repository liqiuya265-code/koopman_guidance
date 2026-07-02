function setup = gpopsDefaults(setup)

% gpopsDefaults
% This function sets any unassigned options in the GPOPS-II
% setup to their default values

% check for Radau Method
if isfield(setup, 'method');
    if strcmpi(setup.method, 'RPMdifferentiation');
        % set as differentiation method
    elseif strcmpi(setup.method, 'RPMintegration');
        % set as integration method
    else
        error([setup.method,' is not a valid option, valid options are ''RPMdifferentiation'' (default) and '' RPMintegration''']);
    end
else
    disp('    Using Default Derivative Radau Method');
    setup.method = 'RPMdifferentiation';
end

% check user NLP setup
if isfield(setup, 'nlp');
    % set solver
    if isfield(setup.nlp, 'solver');
        % check for valid NLP solver snopt or ipopt
    else
        % default nlp solver = 'snopt'
        disp('    Using Default NLP Solver: SNOPT');
        setup.nlp.solver = 'snopt';
    end
    
    % set options
    if isfield(setup.nlp, 'options');
        % check for valid NLP options
    else
        % default nlp options = empty
        setup.nlp.options = [];
    end
else
    % default nlp solver = snopt
    disp('    Using Default NLP Solver: SNOPT');
    setup.nlp.solver = 'snopt';
    % default nlp options = empty
    setup.nlp.options = [];
end

% check user derivative setup
if isfield(setup, 'derivatives');
    % set derivative supplier
    if isfield(setup.derivatives, 'supplier');
        % check for valid derivative supplier: 'sparseCD', 'sparseFD', 'sparseBD', 'intlab', 'analytic', ('none', snopt only)
        if strcmpi(setup.derivatives.supplier, 'analytic');
            % make sure all derivative functions for the set derivative level are provided.
            % OCP sparsity is defined by analytic derivatives
            if isfield(setup.derivatives, 'dependencies');
                % check for valid OCP sparsity setting 'sparse' or 'full'
                disp('    dependencies setting is ignored while using analytic derivatives');
                setup.derivatives.dependencies = 'analytic';
            else
                % default OCP sparsity setting = sparse
                setup.derivatives.dependencies = 'analytic';
            end
        end
    else
        % default derivative supplier = 'sparseFD'
        disp('    Using Default Derivative Supplier: sparseCD');
        setup.derivatives.supplier = 'sparseCD'; % default derivative
    end
    % set derivative level
    if isfield(setup.derivatives, 'derivativelevel');
        % check for valid derivative level: first, second
        if strcmpi(setup.derivatives.derivativelevel, 'second');
            if strcmpi(setup.nlp.solver, 'snopt');
                % snopt is first derivatve method only
                disp(' ');
                disp('    SNOPT Uses First Derivatives Only:');
                disp('    Changing Derivative Level from ''second'' to ''first''');
                disp(' ');
                setup.derivatives.derivativelevel = 'first';
            end
        end
    else
        % default derivativelevel = first
        disp('    Using Default Derivative Level: first');
        setup.derivatives.derivativelevel = 'first';
        
        if strcmpi(setup.derivatives.supplier, 'analytic');
            % if derivatives are analytic, default derivativelevel is what
            % level derivatives are supplied (first, second)
        end
    end
    % set dependencies setting
    if isfield(setup.derivatives, 'dependencies');
        % check for valid dependencies setting 'sparseNaN', sparse or 'full', or
        % 'analytic': analytic is only valid when using analytic
        % derivatives
    else
        % default dependencies setting = sparseNaN
        disp('    Using Default Optimal Control Dependencies Setting: sparseNaN');
        setup.derivatives.dependencies = 'sparseNaN';
    end
    % set stepsize
    if isfield(setup.derivatives, 'stepsize');
        % check for valid spetsize
    else
        % default stepsize = 10^-6
        setup.derivatives.stepsize = 10^-6;
    end
    % set numsamples
    if isfield(setup.derivatives, 'numsamples');
        % check for valid number of samples
    else
        % default number of samples = 10
        setup.derivatives.numsamples = 10;
    end
    % check for options
    if isfield(setup.derivatives, 'options');
        %check for valid derivative options
    else
        % default derivative options = empty
        setup.derivatives.options = [];
    end
else
    % default derivative supplier sparseFD
    disp('    Using Default Derivative Supplier: sparseCD');
    setup.derivatives.supplier = 'sparseCD';
    % default derivativelevel first
    disp('    Using Default Derivative Level: first');
    setup.derivatives.derivativelevel = 'first';
    % default OCP dependencies = sparse
    disp('    Using Default Optimal Control Dependencies Setting: sparseNaN');
    setup.derivatives.dependencies = 'sparseNaN';
    % default stepsize = 10^-3
    setup.derivatives.stepsize = 10^-6;
    % default number of samples = 10
    setup.derivatives.numsamples = 10;
    setup.derivatives.options = [];
end

% check user scaling setup
if isfield(setup, 'scales');
    if isfield(setup.scales, 'method');
        %check for valid mesh method
    else
        setup.scales.method = 'none';
    end
    if isfield(setup.scales, 'numsamples');
        
    else
        setup.scales.numsamples = 100;
    end
else
    setup.scales.method = 'none';
end

% future user mesh defaults
if isfield(setup, 'mesh');
    if isfield(setup.mesh, 'method');
        %check for valid mesh method
        if strcmpi(setup.mesh.method, 'hp');
            % check options for method 'hp'
            % check if tolerance is supplied
            if isfield(setup.mesh, 'tolerance');
                %
            else
                setup.mesh.tolerance = 1e-3;
            end
            % check if colpointsmin is supplied
            if isfield(setup.mesh, 'colpointsmin');
                %
            else
                setup.mesh.colpointsmin = 3;
            end
            % check if colpointsmax is supplied
            if isfield(setup.mesh, 'colpointsmax');
                %
            else
                setup.mesh.colpointsmax = 10;
            end
            % check if splitmult is supplied
            if isfield(setup.mesh, 'splitmult');
                %
            else
                setup.mesh.splitmult = 1.2;
            end
            % check if curveratio is supplied
            if isfield(setup.mesh, 'curveratio');
                %
            else
                setup.mesh.curveratio = 2;
            end
            % check if maxiteration is supplied
            if isfield(setup.mesh, 'maxiteration');
                %
            else
                setup.mesh.maxiteration = 25;
            end
        elseif strcmpi(setup.mesh.method, 'hp1');
            % check options for method 'hp'
            
            % check if tolerance is supplied
            if isfield(setup.mesh, 'tolerance');
                %
            else
                setup.mesh.tolerance = 1e-3;
            end
            % check if colpointsmin is supplied
            if isfield(setup.mesh, 'colpointsmin');
                %
            else
                setup.mesh.colpointsmin = 3;
            end
            % check if colpointsmax is supplied
            if isfield(setup.mesh, 'colpointsmax');
                %
            else
                setup.mesh.colpointsmax = 10;
            end
            % check if maxiteration is supplied
            if isfield(setup.mesh, 'maxiteration');
                %
            else
                setup.mesh.maxiteration = 25;
            end
        else
            setup.mesh.method = 'none';
        end
    else
        setup.mesh.method = 'none';
    end
else
    setup.mesh.method = 'hp1';
    setup.mesh.tolerance = 1e-3;
    setup.mesh.colpointsmin = 3;
    setup.mesh.colpointsmax = 10;
    setup.mesh.maxiteration = 25;
end

% get number of phases
numphase = size(setup.bounds.phase,2);
% check if user entered any initial mesh info
% check for mesh info, set defaults
if isfield(setup.mesh, 'phase')
    if isfield(setup.mesh.phase, 'colpoints') && isfield(setup.mesh.phase, 'fraction')
        % colpoints and fraction must be same length in each phase
        % sum fraction must be == 1 in each phase
        % colpoints must be between 3 and 50 (proposed upper limit)
        for phasecount = 1:numphase
            colpoints = setup.mesh.phase(phasecount).colpoints;
            fraction   = setup.mesh.phase(phasecount).fraction;
            num_sections_colpoints = size(colpoints,2);
            num_sections_fraction   = size(fraction,2);
            if num_sections_colpoints ~= num_sections_fraction;
                if num_sections_colpoints == 0;
                    disp(' ');
                    disp(['    No User-Supplied Information for Number of Points Per Mesh Interval in Phase ',num2str(phasecount),]);
                    disp('    Using Default 4 Collocation Points in Each Mesh Interval');
                    disp(' ');
                    colpoints = 4.*ones(1,num_sections_fraction);
                elseif num_sections_fraction == 0;
                    disp(' ');
                    disp(['    No User-Supplied Information for Mesh Interval Length fractionages in Phase ',num2str(phasecount),]);
                    disp('    Using Default Uniformly-Spaced Mesh Intervals');
                    disp(' ');
                    fraction = 1/num_sections_colpoints.*ones(1,num_sections_colpoints);
                else
                    % error mismatched mesh inputs
                    error('setup.mesh(%d).colpoints must be the same length as setup.mesh(%d).fraction',phasecount,phasecount);
                end
            elseif num_sections_colpoints == 0;
                % default 10 equal sections, 4 colpoints
                disp(' ');
                disp(['    No User-Supplied Mesh Information provided in Phase ',num2str(phasecount),]);
                disp('    Using Default Mesh: 10 Uniformly-Spaced Mesh Intervals with 4 Collocation Points in Each Mesh Interval');
                disp(' ');
                colpoints = 4.*ones(1,10);
                fraction   = 1/10.*ones(1,10);
            end
            setup.mesh.phase(phasecount).colpoints = colpoints;
            setup.mesh.phase(phasecount).fraction   = fraction;
        end
    elseif isfield(setup.mesh.phase, 'colpoints');
        % colpoints must be between 2 and 50 (proposed upper limit)
        for phasecount = 1:numphase
            colpoints = setup.mesh.phase(phasecount).colpoints;
            % default equal sections,
            num_sections = size(colpoints,2);
            if num_sections == 0;
                %set default section to replace empty phase
                disp(' ');
                disp(['    No User-Supplied Mesh Information Provided in Phase ',num2str(phasecount),]);
                disp('    Using Default Mesh: 10 Uniformly-Spaced Mesh Interval with 4 Collocation Points in Each Mesh Interval');
                disp(' ');
                colpoints = 4.*ones(1,10);
                num_sections = 10;
            else
                disp(' ');
                disp(['    No User-Supplied Information Provided for Mesh Interval Length fractionages in Phase ',num2str(phasecount),]);
                disp('    Using Default Uniformly-Spaced Mesh Intervals');
                disp(' ');
            end
            fraction = 1/num_sections.*ones(1,num_sections);
            setup.mesh.phase(phasecount).colpoints = colpoints;
            setup.mesh.phase(phasecount).fraction   = fraction;
        end
    elseif isfield(setup.mesh.phase, 'fraction');
        % sum fraction must be == 1 in each phase
        for phasecount = 1:numphase
            fraction = setup.mesh.phase(phasecount).fraction;
            % default 4 colpoints per section
            num_sections = size(fraction,2);
            if num_sections == 0;
                %set default section to replace empty phase
                disp(' ');
                disp(['    No User-Supplied Mesh Information Provided in Phase ',num2str(phasecount),]);
                disp('    Using Default Mesh: 10 Uniformly-Spaced Mesh Intervals with 4 Collocation Points in Each Mesh Interval');
                disp(' ');
                fraction   = 1/10.*ones(1,10);
                num_sections = 10;
            else
                disp(' ');
                disp(['    No User-Supplied Information Provided for Number of Collocation Points per Mesh Interval in Phase ',num2str(phasecount),]);
                disp('    Using Default 4 Collocation Points in each Mesh Interval');
                disp(' ');
            end
            colpoints = 4.*ones(1,num_sections);
            setup.mesh.phase(phasecount).colpoints = colpoints;
            setup.mesh.phase(phasecount).fraction   = fraction;
        end
    end
else
    disp(' ');
    disp('    User Has Supplied No Mesh Information');
    disp('    Using Default Mesh: 10 Uniformly-Spaced Sections with 4 Collocation Points in Each Mesh Interval for Each Phase');
    disp(' ');
    for phasecount = 1:numphase
        % default 10 equal sections, 4 colpoints
        setup.mesh.phase(phasecount).colpoints = 4.*ones(1,10);
        setup.mesh.phase(phasecount).fraction   = 1/10.*ones(1,10);
    end
end