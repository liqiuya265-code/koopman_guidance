function output = gpops2(usersetup)

% Splash
disp(' _______________________________________________________________________________________________________ ');
disp('|                                                                                                       |');
disp('|                                 __________  ____  ____  _____       ________                          |');
disp('|                                / ____/ __ \/ __ \/ __ \/ ___/      /  _/  _/                          |');
disp('|                               / / __/ /_/ / / / / /_/ /\__ \______ / / / /                            |');
disp('|                              / /_/ / ____/ /_/ / ____/___/ /_____// /_/ /                             |');
disp('|                              \____/_/    \____/_/    /____/     /___/___/                             |');
disp('|                                                                                                       |');
disp('|_______________________________________________________________________________________________________|');
disp('|       __           __        __  ___         ___     __        __                                     |');
disp('| |__| |__) __  /\  |  \  /\  |__)  |  | \  / |__     |__)  /\  |  \  /\  |  |                          |');
disp('| |  | |       /~~\ |__/ /~~\ |     |  |  \/  |___    |  \ /~~\ |__/ /~~\ \__/                          |');
disp('|  __   __   ___       __   __   __   __   ___  __  ___  __                    ___ ___       __   __    |'); 
disp('| |__) /__` |__  |  | |  \ /  \ /__` |__) |__  /  `  |  |__)  /\  |      |\/| |__   |  |__| /  \ |  \   |'); 
disp('| |    .__/ |___ \__/ |__/ \__/ .__/ |    |___ \__,  |  |  \ /~~\ |___   |  | |___  |  |  | \__/ |__/   |'); 
disp('|_______________________________________________________________________________________________________|');
disp('|                                                                                                       |');
disp('| GPOPS-II Version 1.0 Running with the hp-Adaptive Radau Pseudospectral Method.                        |');
disp('|_______________________________________________________________________________________________________|');
disp('|                                                                                                       |');
disp('| GPOPS-II Copyright (c) 2007-2012 Michael A. Patterson and Anil V. Rao.                                |');
disp('|_______________________________________________________________________________________________________|');
disp('|                                                                                                       |');
disp('| Downloading, using, copying, or modifying the GPOPS-II code constitutes an agreement to ALL           |');
disp('| of the terms of the GPOPS-II license.                                                                 |');
disp('|_______________________________________________________________________________________________________|');
disp('                                                                                                         ');

% start clock to get total time
totaltimestart = tic;

% gpops2
% This is the main function for GPOPS-II
% test
% set default options
disp(' ');
disp(' _______________________________________________________________________________________________________ ');
disp('|                                                                                                       |');
disp('|                                     Setting GPOPS-II defaults                                         |');
disp('|_______________________________________________________________________________________________________|');
setup = gpopsDefaults(usersetup);

% setup error check -----------------------------------------------------<<
% eval user functions on guess

% get Problem info
disp(' _______________________________________________________________________________________________________ ');
disp('|                                                                                                       |');
disp('|                            Getting Optimal Control Problem Information                                |');
disp('|____________________________ __________________________________________________________________________|');
disp(' ');
probinfo = gpopsProblemInfo(setup);

% get derivative dependencies
disp(' _______________________________________________________________________________________________________ ');
disp('|                                                                                                       |');
disp('|                            Finding Optimal Control Problem Dependencies                               |');
disp('|_______________________________________________________________________________________________________|');
probinfo = gpopsDependencies(setup, probinfo);

% call mesh refinement algorithm
output = gpopsMeshShell(setup, probinfo);

% get total time
totaltime = toc(totaltimestart);
output.totaltime = totaltime;
