
% optimalprimePathSetup
% this script adds the appropriate paths for use of optimalprime to the
% MATLAB path directory

% get current directory
currdir = pwd;

% add opRPMintegration/opRPMIsnopt/
disp(['Adding path ',pwd,'/lib/gpopsRPMIntegration/gpopsSnoptRPMI/']);
addpath([pwd,'/lib/gpopsRPMIntegration/gpopsSnoptRPMI/'],'-begin');

% add opRPMintegration/opRPMIipopt/
disp(['Adding path ',pwd,'/lib/gpopsRPMIntegration/gpopsIpoptRPMI/']);
addpath([pwd,'/lib/gpopsRPMIntegration/gpopsIpoptRPMI/'],'-begin');

% add opRPMintegration/
disp(['Adding path ',pwd,'/lib/gpopsRPMIntegration/']);
addpath([pwd,'/lib/gpopsRPMIntegration/'],'-begin');

% add opRPMdifferentiation/opRPMDsnopt/
disp(['Adding path ',pwd,'/lib/gpopsRPMDifferentiation/gpopsSnoptRPMD/']);
addpath([pwd,'/lib/gpopsRPMDifferentiation/gpopsSnoptRPMD/'],'-begin');

% add opRPMdifferentiation/opRPMDipopt/
disp(['Adding path ',pwd,'/lib/gpopsRPMDifferentiation/gpopsIpoptRPMD/']);
addpath([pwd,'/lib/gpopsRPMDifferentiation/gpopsIpoptRPMD/'],'-begin');

% add opRPMdifferentiation/
disp(['Adding path ',pwd,'/lib/gpopsRPMDifferentiation/']);
addpath([pwd,'/lib/gpopsRPMDifferentiation/'],'-begin');
% add opOCPfinitediff/
disp(['Adding path ',pwd,'/lib/gpopsFiniteDifference/']);
addpath([pwd,'/lib/gpopsFiniteDifference/'],'-begin');

% add opMeshHP1/
disp(['Adding path ',pwd,'/lib/gpopsMeshHP1/']);
addpath([pwd,'/lib/gpopsMeshHP1/'],'-begin');

% add opMeshHP/
disp(['Adding path ',pwd,'/lib/gpopsMeshHP/']);
addpath([pwd,'/lib/gpopsMeshHP/'],'-begin');

% add opCommon/
disp(['Adding path ',pwd,'/lib/gpopsCommon/']);
addpath([pwd,'/lib/gpopsCommon/'],'-begin');

% add NLP directory
disp(['Adding path ',pwd,'/nlp/']);
addpath([pwd,'/nlp/'],'-begin');

% add SNOPT directory
disp(['Adding path ',pwd,'/nlp/snopt/']);
addpath([pwd,'/nlp/snopt/'],'-begin');

% add IPOPT directory
disp(['Adding path ',pwd,'/nlp/ipopt/']);
addpath([pwd,'/nlp/ipopt/'],'-begin');

% save current path
savepath;
