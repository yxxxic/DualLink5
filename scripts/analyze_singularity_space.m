analyzeSingularitySpaceExample();

function analyzeSingularitySpaceExample()
originalPath = path;
pathCleanup = onCleanup(@()path(originalPath));
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
run(fullfile(projectRoot, 'startup.m'));

geometry = duallink5.model.defaultGeometry();
grid.theta = deg2rad(0:1:180);
grid.phi = deg2rad(0:1:90);
options = struct( ...
    'exactThreshold', 1e-8, ...
    'nearMetricThreshold', 0.05, ...
    'nearConditionThreshold', 100, ...
    'collisionProfile', "centerline", ...
    'topologyProfile', "reference");

timer = tic;
samples = duallink5.singularity.sampleSpace( ...
    grid, geometry, options);
elapsed = toc(timer);

jointFigure = figure('Name', 'DualLink5 joint singularity space');
jointLayout = tiledlayout(jointFigure, 1, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
jointAxes = [nexttile(jointLayout), nexttile(jointLayout)];
duallink5.viz.plotJointSingularitySpace(samples, jointAxes);

taskFigure = figure('Name', 'DualLink5 task singularity space');
taskLayout = tiledlayout(taskFigure, 1, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
taskAxes = [nexttile(taskLayout), nexttile(taskLayout)];
duallink5.viz.plotTaskSingularitySpace(samples, taskAxes);

theoreticalCount = nnz(samples.theoreticalReachableMask);
mechanicalCount = nnz(samples.mechanicallyValidMask);
safeCount = nnz(samples.safeUsableMask);
if mechanicalCount == 0
    safeFraction = NaN;
else
    safeFraction = safeCount / mechanicalCount;
end
summary = struct( ...
    'theoreticalCount', theoreticalCount, ...
    'mechanicalCount', mechanicalCount, ...
    'safeCount', safeCount, ...
    'nearTypeICount', nnz(samples.near.typeI), ...
    'nearTypeIICount', nnz(samples.near.typeII));
jointFigure.UserData = summary;
taskFigure.UserData = summary;
fprintf('DualLink5 singularity sampling: %.3f s\n', elapsed);
fprintf('Theoretical/mechanical/safe samples: %d / %d / %d\n', ...
    theoreticalCount, mechanicalCount, safeCount);
fprintf('Exact Type I/II/III: %d / %d / %d\n', ...
    nnz(samples.exact.typeI), nnz(samples.exact.typeII), ...
    nnz(samples.exact.typeIII));
fprintf('Near Type I/II/multiple: %d / %d / %d\n', ...
    nnz(samples.near.typeI), nnz(samples.near.typeII), ...
    nnz(samples.near.typeIII));
fprintf('Safe fraction of mechanically valid samples: %.3f\n', ...
    safeFraction);
clear pathCleanup
end
