plotContinuousTrajectoryExample();

function plotContinuousTrajectoryExample()
originalPath = path;
pathCleanup = onCleanup(@()path(originalPath));
exampleDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(exampleDir);
run(fullfile(projectRoot, 'startup.m'));

geometry = duallink5.model.defaultGeometry();
theta = linspace(deg2rad(80), deg2rad(90), 80);
phi = linspace(deg2rad(48), deg2rad(58), 80);
trajectory = nan(2, numel(theta));
previousAssembly = [];
startAssembly = [];
for index = 1:numel(theta)
    if isempty(previousAssembly)
        options = struct('collisionProfile', "none");
    else
        options = struct('branchMode', "continuous", ...
            'previousAssembly', previousAssembly, ...
            'maxContinuityCost', 5e-3, ...
            'collisionProfile', "none");
    end
    q = [theta(index), phi(index)];
    jointInput = struct('lower', q, 'upper', q);
    assembly = duallink5.kinematics.forwardAssembly( ...
        jointInput, geometry, options);
    if ~assembly.quality.valid
        error('duallink5:example:InvalidTrajectory', ...
            'Trajectory failed at sample %d: %s.', ...
            index, assembly.quality.statusCode);
    end
    trajectory(:, index) = assembly.lower.points.G;
    if isempty(startAssembly)
        startAssembly = assembly;
    end
    previousAssembly = assembly;
end
endAssembly = previousAssembly;

figureHandle = figure;
try
    axesHandle = axes(figureHandle);
    hold(axesHandle, 'on');
    startHandles = duallink5.viz.plotAssembly( ...
        startAssembly, axesHandle, struct('showLabels', false));
    styleStartAssembly(startHandles);
    trajectoryMillimetres = 1e3 * trajectory;
    trajectoryHandle = plot(axesHandle, trajectoryMillimetres(1, :), ...
        trajectoryMillimetres(2, :), ...
        'Color', [0.10, 0.10, 0.10], 'LineWidth', 2.5);
    duallink5.viz.plotAssembly( ...
        endAssembly, axesHandle, struct('showLabels', true));
    startLegend = plot(axesHandle, NaN, NaN, '--', ...
        'Color', [0.65, 0.65, 0.65], 'LineWidth', 1.5);
    endLegend = plot(axesHandle, NaN, NaN, '-', ...
        'Color', [0.00, 0.45, 0.74], 'LineWidth', 2);
    legend(axesHandle, ...
        [startLegend, trajectoryHandle, endLegend], ...
        {'Start assembly', 'Point G trajectory', 'End assembly'}, ...
        'Location', 'best');
    title(axesHandle, ...
        'Continuous trajectory with complete endpoint assemblies');
catch exception
    if isgraphics(figureHandle, 'figure')
        close(figureHandle);
    end
    rethrow(exception);
end
clear pathCleanup
end

function styleStartAssembly(handles)
gray = [0.65, 0.65, 0.65];
links = [handles.lowerLinks; handles.upperLinks; ...
    handles.sharedLink; handles.alphaLinks; handles.betaLinks];
set(links, 'Color', gray, 'LineStyle', '--', 'LineWidth', 1.5);
joints = [handles.mainJoints; handles.parallelJoints; ...
    handles.sharedCenter];
set(joints, 'MarkerEdgeColor', gray, 'MarkerFaceColor', 'w');
end
