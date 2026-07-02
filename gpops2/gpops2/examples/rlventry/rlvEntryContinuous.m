function phaseout = rlvEntryContinuous(input)

% input
% input.phase(phasenumber).state
% input.phase(phasenumber).control
% input.phase(phasenumber).time
% input.phase(phasenumber).parameter
%
% input.auxdata = auxiliary information
%
% output
% phaseout(phasenumber).dynamics
% phaseout(phasenumber).path
% phaseout(phasenumber).integrand

rad = input.phase.state(:,1);
lon = input.phase.state(:,2);
lat = input.phase.state(:,3);
speed = input.phase.state(:,4);
fpa = input.phase.state(:,5);
azimuth = input.phase.state(:,6);
aoa = input.phase.control(:,1);
bank = input.phase.control(:,2);

cd0 = input.auxdata.cd(1);
cd1 = input.auxdata.cd(2);
cd2 = input.auxdata.cd(3);
cl0 = input.auxdata.cl(1);
cl1 = input.auxdata.cl(2);
mu  = input.auxdata.mu;
rho0 = input.auxdata.rho0;
H = input.auxdata.H;
S = input.auxdata.S;
mass = input.auxdata.mass;
altitude = rad - input.auxdata.Re;

CD = cd0+cd1*aoa+cd2*aoa.^2;

rho = rho0*exp(-altitude/H);
CL = cl0+cl1*aoa;
gravity = mu./rad.^2;
dynamic_pressure = 0.5*rho.*speed.^2;
D = dynamic_pressure.*S.*CD./mass;
L = dynamic_pressure.*S.*CL./mass;
slon = sin(lon);
clon = cos(lon);
slat = sin(lat);
clat = cos(lat);
tlat = tan(lat);
sfpa = sin(fpa);
cfpa = cos(fpa);
sazi = sin(azimuth);
cazi = cos(azimuth);
cbank = cos(bank);
sbank = sin(bank);

raddot   = speed.*sfpa;
londot   = speed.*cfpa.*sazi./(rad.*clat);
latdot   = speed.*cfpa.*cazi./rad;
speeddot = -D-gravity.*sfpa;
fpadot   = (L.*cbank-cfpa.*(gravity-speed.^2./rad))./speed;
azidot   = (L.*sbank./cfpa + speed.^2.*cfpa.*sazi.*tlat./rad)./speed;

phaseout.dynamics  = [raddot, londot, latdot, speeddot, fpadot, azidot];