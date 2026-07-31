plotFixedPoseExample();

function plotFixedPoseExample()
originalPath = path;
pathCleanup = onCleanup(@()path(originalPath));
exampleDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(exampleDir);
run(fullfile(projectRoot, 'startup.m'));

geometry = duallink5.model.defaultGeometry();
q.lower = deg2rad([85, 52.93]);
q.upper = q.lower;
assembly = duallink5.kinematics.forwardAssembly(q, geometry);
figureHandle = figure( ...
    'Name', 'DualLink5 complete assembly', ...
    'Color', 'w', ...
    'Position', [100, 100, 700, 800]);
try
    axesHandle = axes(figureHandle);
    duallink5.viz.plotAssembly( ...
        assembly, axesHandle, struct('showLabels', true));
    title(axesHandle, 'DualLink5 complete symmetric assembly');
catch exception
    if isgraphics(figureHandle, 'figure')
        close(figureHandle);
    end
    rethrow(exception);
end
clear pathCleanup
end
