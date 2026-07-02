% ----------- Reusable Launch Vehicle Entry Example ------------%
% This example is taken verbatim from the following reference:  %
% Betts, J. T., Practical Methods for Optimal Control Using     %
% Nonlinear Programming, SIAM Press, Philadelphia, 2009.        %
% --------------------------------------------------------------%
%close all
clear all
clc

cft2m = 0.3048;
cft2km = cft2m/1000;
cslug2kg = 14.5939029;
%-------------------------------------%
%             Problem Setup           %
%-------------------------------------%
auxdata.Re = 20902900*cft2m;              % Equatorial Radius of Earth (m)
auxdata.S  = 2690*cft2m^2;                % Vehicle Reference Area (m^2)
auxdata.cl(1) = -0.2070;                  % Parameters for lift coefficient
auxdata.cl(2) = 1.6756;       
auxdata.cd(1) = 0.0785;       
auxdata.cd(2) = -0.3529;       
auxdata.cd(3) = 2.0400;
auxdata.b(1)  = 0.07854;      
auxdata.b(2)  = -0.061592;    
auxdata.b(3)  = 0.00621408;
auxdata.H     = 23800*cft2m;              % Density Scale Height (m)
auxdata.al(1) = -0.20704;    
auxdata.al(2) = 0.029244;
auxdata.rho0  = 0.002378*cslug2kg/cft2m^3;% Sea Level Atmospheric Density (slug/ft^3)
auxdata.mu    = 1.4076539e16*cft2m^3;     % Earth Gravitational Parameter (ft^^3/s^2) 
auxdata.mass  = 6309.433*cslug2kg;      

% inital conditions
t0 = 0;
alt0 = 260000*cft2m;
rad0 = alt0+auxdata.Re;
lon0 = 0;
lat0 = 0;
speed0 = 25600*cft2m;
fpa0   = -1*pi/180;
azi0   = 90*pi/180;

% terminal conditions
altf = 80000*cft2m;
radf = altf+auxdata.Re;
speedf = 2500*cft2m;
fpaf   = -5*pi/180;
azif   = -90*pi/180;

%----------------------------------------------------%
% Lower and Upper Limits on Time, State, and Control %
%----------------------------------------------------%
tfMin = 0;            tfMax = 3000;
radMin = auxdata.Re;  radMax = rad0;
lonMin = -pi;         lonMax = -lonMin;
latMin = -70*pi/180;  latMax = -latMin;
speedMin = 10;        speedMax = 45000;
fpaMin = -80*pi/180;  fpaMax =  80*pi/180;
aziMin = -180*pi/180; aziMax =  180*pi/180;
aoaMin = -90*pi/180;  aoaMax = -aoaMin;
bankMin = -90*pi/180; bankMax =   1*pi/180;

bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tfMin;
bounds.phase.finaltime.upper = tfMax;
bounds.phase.initialstate.lower = [rad0, lon0, lat0, speed0, fpa0, azi0];
bounds.phase.initialstate.upper = [rad0, lon0, lat0, speed0, fpa0, azi0];
bounds.phase.state.lower = [radMin, lonMin, latMin, speedMin, fpaMin, aziMin];
bounds.phase.state.upper = [radMax, lonMax, latMax, speedMax, fpaMax, aziMax];
bounds.phase.finalstate.lower = [radf, lonMin, latMin, speedf, fpaf, aziMin];
bounds.phase.finalstate.upper = [radf, lonMax, latMax, speedf, fpaf, aziMax];
bounds.phase.control.lower = [aoaMin, bankMin];
bounds.phase.control.upper = [aoaMax, bankMax];

%----------------------%
% Set up Initial Guess %
%----------------------%
tGuess = [0; 1000];
radGuess = [rad0; radf];
lonGuess = [lon0; lon0+10*pi/180];
latGuess = [lat0; lat0+10*pi/180];
speedGuess = [speed0; speedf];
fpaGuess = [fpa0; fpaf];
aziGuess = [azi0; azif];
aoaGuess = [0; 0];
bankGuess = [0; 0];

guess.phase.state   = [radGuess, lonGuess, latGuess, speedGuess, fpaGuess, aziGuess];
guess.phase.control = [aoaGuess, bankGuess];
guess.phase.time    = tGuess;

%---------------------%
% Set up Initial Mesh %
%---------------------%
meshphase.colpoints = 4*ones(1,10);
meshphase.fraction = 0.1*ones(1,10);

setup.name = 'Reusable-Launch-Vehicle-Entry-Problem';
setup.functions.continuous = @rlvEntryContinuous;
setup.functions.endpoint   = @rlvEntryEndpoint;
setup.auxdata = auxdata;
setup.mesh.phase = meshphase;
setup.bounds = bounds;
setup.guess = guess;
setup.nlp.solver = 'ipopt';
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.scales.method = 'automatic-bounds';
%setup.method = 'RPMintegration';
setup.mesh.method = 'hp1';
setup.mesh.tolerance = 1e-6; % default 1e-3
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 16;

%----------------------------------%
% Solve Problem Using OptimalPrime %
%----------------------------------%
output = gpops2(setup);
