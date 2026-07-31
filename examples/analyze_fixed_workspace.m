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
samples = duallink5.workspace.sampleWorkspace( ...
    grid, geometry, taskSpec, struct());
result = duallink5.workspace.analyzeWorkspace( ...
    samples, struct('gridSize', [180, 180]));
figureHandle = figure;
axesHandle = axes(figureHandle);
duallink5.viz.plotWorkspaceResult(samples, result, axesHandle);
title(axesHandle, sprintf('Workspace area %.6f m^2', result.area));
