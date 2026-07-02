function [segcolpoints, segfraction] = gpopsMeshModifySegmentHP1(segnum, mesh, colpoints, fraction, segmaxerror)

% gpopsMeshModifySegmentHP1
% this function modifies the mesh for the current segment

% compute logdiff
logdiff = ceil(log10(segmaxerror/mesh.tolerance)/log10(colpoints));

% logdiff must be greater then 1
if logdiff < 1;
    logdiff = 1;
end


% compute numbreaks
%numbreaks = 2;

if (colpoints + logdiff) <= mesh.colpointsmax;
    % increase number of collocation points
    segcolpoints = colpoints + logdiff;
    segfraction = fraction;
    disp(['Increasing Number of Collocation Points in Mesh Interval ',num2str(segnum),' to ',num2str(segcolpoints)]);
else
    %numbreaks = colpoints + logdiff - mesh.colpointsmax;
    numbreaks = ceil((colpoints + logdiff)/mesh.colpointsmin);
    
    %numbreaks = 1+ceil((logdiff)/mesh.colpointsmin);
    if numbreaks < 2;
        numbreaks = 2;
    end
    
    % break segment into length segments
    segcolpoints = mesh.colpointsmin*ones(1,numbreaks);
    segfraction = (fraction/numbreaks)*ones(1,numbreaks);
    disp(['Dividing Mesh Interval ',num2str(segnum),' Into ',num2str(numbreaks),' Mesh Intervals']);
end
