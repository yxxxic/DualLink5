plotFixedPoseExample();

function plotFixedPoseExample()
originalPath = path;
pathCleanup = onCleanup(@()path(originalPath));
exampleDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(exampleDir);
run(fullfile(projectRoot, 'startup.m'));

geometry = duallink5.model.defaultGeometry();
pose = duallink5.kinematics.forwardFiveBar( ...
    deg2rad([85, 52.93]), geometry);
figureHandle = figure;
try
    axesHandle = axes(figureHandle);
    duallink5.viz.plotMechanism( ...
        pose, axesHandle, struct('showLabels', true));
    title(axesHandle, 'DualLink5 fixed pose');
catch exception
    if isgraphics(figureHandle, 'figure')
        close(figureHandle);
    end
    rethrow(exception);
end
clear pathCleanup
end
