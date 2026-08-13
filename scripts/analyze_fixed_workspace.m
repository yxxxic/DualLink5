analyzeFixedWorkspaceExample();

function analyzeFixedWorkspaceExample()
originalPath = path;
pathCleanup = onCleanup(@()path(originalPath));
exampleDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(exampleDir);
run(fullfile(projectRoot, 'startup.m'));

geometry = duallink5.model.defaultGeometry();
grid.theta = linspace( ...
    geometry.analysis.thetaRange(1), ...
    geometry.analysis.thetaRange(2), 121);
grid.phi = linspace( ...
    geometry.analysis.phiRange(1), ...
    geometry.analysis.phiRange(2), 121);
taskSpec = struct('kind', "pointG", 'includeOrientation', false);
workspaceOptions = struct('topologyProfile', "reference");
samples = duallink5.workspace.sampleWorkspace( ...
    grid, geometry, taskSpec, workspaceOptions);
result = duallink5.workspace.analyzeWorkspace( ...
    samples, struct('gridSize', [180, 180]));
figureHandle = figure;
try
    axesHandle = axes(figureHandle);
    duallink5.viz.plotWorkspaceResult(samples, result, axesHandle);
    title(axesHandle, sprintf( ...
        'Workspace area %.1f mm^2', 1e6 * result.area));
catch exception
    if isgraphics(figureHandle, 'figure')
        close(figureHandle);
    end
    rethrow(exception);
end
clear pathCleanup
end
