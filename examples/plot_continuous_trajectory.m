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
previousPose = [];
for index = 1:numel(theta)
    if isempty(previousPose)
        options = struct('collisionProfile', "none");
    else
        options = struct('branchMode', "continuous", ...
            'previousPose', previousPose, ...
            'maxContinuityCost', 5e-3, ...
            'collisionProfile', "none");
    end
    pose = duallink5.kinematics.forwardFiveBar( ...
        [theta(index), phi(index)], geometry, options);
    if ~pose.quality.valid
        error('duallink5:example:InvalidTrajectory', ...
            'Trajectory failed at sample %d: %s.', ...
            index, pose.quality.statusCode);
    end
    trajectory(:, index) = pose.points.G;
    previousPose = pose;
end

figureHandle = figure;
try
    axesHandle = axes(figureHandle);
    trajectoryMillimetres = 1e3 * trajectory;
    plot(axesHandle, trajectoryMillimetres(1, :), ...
        trajectoryMillimetres(2, :), ...
        'LineWidth', 1.5);
    hold(axesHandle, 'on');
    duallink5.viz.plotMechanism( ...
        previousPose, axesHandle, struct('showLabels', false));
    title(axesHandle, 'Continuous branch trajectory');
catch exception
    if isgraphics(figureHandle, 'figure')
        close(figureHandle);
    end
    rethrow(exception);
end
clear pathCleanup
end
