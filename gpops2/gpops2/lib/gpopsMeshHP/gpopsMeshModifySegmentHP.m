function [segcolpoints, segfraction] = gpopsMeshModifySegmentHP(segnum, mesh, colpoints, fraction, segmaxerror, k)

% gpopsMeshModifySegmentHP
% this function modifies the mesh for the current segment

% compute logdiff
logdiff = ceil(log10(segmaxerror/mesh.tolerance));

% logdiff must be greater then 0
if logdiff < 0;
    logdiff = 0;
end

% compute numbreaks
numbreaks = ceil(mesh.splitmult*logdiff);

% % numbreaks must be greater then 2
if numbreaks < 2;
    numbreaks = 2;
end

% compute the ratio of max curvature to mean curvature
curveratio = max(k)/mean(k);

if curveratio <= mesh.curveratio;
    % ratio is less then curveratio, increase number of collocation points
    if colpoints >= mesh.colpointsmax;
        % break segment
        segcolpoints = mesh.colpointsmin*ones(1,numbreaks);
        segfraction = (fraction/numbreaks)*ones(1,numbreaks);
        disp(['Dividing Mesh Interval ',num2str(segnum),' Into ',num2str(numbreaks),' Mesh Intervals']);
    else
        segcolpoints = colpoints + logdiff;
        segfraction = fraction;
        if segcolpoints > mesh.colpointsmax;
            segcolpoints = mesh.colpointsmax;
        end
        disp(['Increasing Number of Collocation Points in Mesh Interval ',num2str(segnum),' to ',num2str(segcolpoints)]);
    end
else
    % ratio is greater then curveratio, break segment into length segments
    segcolpoints = mesh.colpointsmin*ones(1,numbreaks);
    segfraction = (fraction/numbreaks)*ones(1,numbreaks);
    disp(['Dividing Mesh Interval ',num2str(segnum),' Into ',num2str(numbreaks),' Mesh Intervals']);
end