function handles = plotMechanism(pose, axesHandle, options)
if nargin < 3
    options = struct();
end
if ~(isscalar(axesHandle) && isgraphics(axesHandle, 'axes'))
    error('duallink5:viz:InvalidGraphicsHandle', ...
        'axesHandle must be a live scalar axes handle.');
end
[lineWidth, showLabels] = validateOptions(options);
[p, sharedCenter] = validatePose(pose);

originalNextPlot = axesHandle.NextPlot;
axesHandle.NextPlot = 'add';
cleanup = onCleanup( ...
    @()restoreNextPlot(axesHandle, originalNextPlot));
segments = {p.A, p.E; p.B, p.C; p.C, p.D; p.E, p.D; p.A, p.B};
handles.links = gobjects(5, 1);
for index = 1:5
    startPoint = segments{index, 1};
    endPoint = segments{index, 2};
    handles.links(index) = plot(axesHandle, ...
        [startPoint(1), endPoint(1)], [startPoint(2), endPoint(2)], ...
        '-o', 'LineWidth', lineWidth);
end
handles.sharedCenter = plot(axesHandle, sharedCenter(1), ...
    sharedCenter(2), 'kp', 'MarkerFaceColor', 'y');

if showLabels
    names = {'A', 'B', 'C', 'D', 'E', 'G'};
    handles.labels = gobjects(numel(names), 1);
    for index = 1:numel(names)
        point = p.(names{index});
        handles.labels(index) = text(axesHandle, point(1), point(2), ...
            ['  ', names{index}], 'Interpreter', 'none');
    end
end
axis(axesHandle, 'equal');
xlabel(axesHandle, 'x [m]');
ylabel(axesHandle, 'y [m]');
clear cleanup
end

function [lineWidth, showLabels] = validateOptions(options)
if ~isstruct(options) || ~isscalar(options)
    invalidOptions();
end
if isfield(options, 'lineWidth')
    lineWidth = options.lineWidth;
else
    lineWidth = 2;
end
if ~isnumeric(lineWidth) || ~isreal(lineWidth) || ...
        ~isscalar(lineWidth) || ~isfinite(lineWidth) || lineWidth <= 0
    invalidOptions();
end
lineWidth = full(double(lineWidth));

if isfield(options, 'showLabels')
    showLabels = options.showLabels;
else
    showLabels = true;
end
validShowLabels = islogical(showLabels) && isscalar(showLabels);
validNumericLabels = isnumeric(showLabels) && isreal(showLabels) && ...
    isscalar(showLabels) && isfinite(showLabels) && ...
    ismember(double(showLabels), [0, 1]);
if ~(validShowLabels || validNumericLabels)
    invalidOptions();
end
showLabels = logical(showLabels);
end

function [points, sharedCenter] = validatePose(pose)
if ~isstruct(pose) || ~isscalar(pose) || ...
        ~all(isfield(pose, {'quality', 'points', 'sharedLink'})) || ...
        ~isstruct(pose.quality) || ~isscalar(pose.quality) || ...
        ~isfield(pose.quality, 'valid') || ...
        ~isstruct(pose.points) || ~isscalar(pose.points) || ...
        ~isstruct(pose.sharedLink) || ~isscalar(pose.sharedLink) || ...
        ~isfield(pose.sharedLink, 'center')
    invalidPose();
end
validFlag = pose.quality.valid;
validLogicalFlag = islogical(validFlag) && isscalar(validFlag);
validNumericFlag = isnumeric(validFlag) && isreal(validFlag) && ...
    isscalar(validFlag) && isfinite(validFlag) && ...
    ismember(double(validFlag), [0, 1]);
if ~(validLogicalFlag || validNumericFlag) || ~logical(validFlag)
    invalidPose();
end

names = {'A', 'B', 'C', 'D', 'E', 'G'};
if ~all(isfield(pose.points, names))
    invalidPose();
end
points = struct();
for index = 1:numel(names)
    name = names{index};
    point = pose.points.(name);
    if ~isnumeric(point) || ~isreal(point) || ...
            ~isvector(point) || numel(point) ~= 2 || ...
            any(~isfinite(point), 'all')
        invalidPose();
    end
    points.(name) = full(double(point(:)));
end
sharedCenter = pose.sharedLink.center;
if ~isnumeric(sharedCenter) || ~isreal(sharedCenter) || ...
        ~isvector(sharedCenter) || numel(sharedCenter) ~= 2 || ...
        any(~isfinite(sharedCenter), 'all')
    invalidPose();
end
sharedCenter = full(double(sharedCenter(:)));
end

function restoreNextPlot(axesHandle, originalNextPlot)
if isscalar(axesHandle) && isgraphics(axesHandle, 'axes')
    axesHandle.NextPlot = originalNextPlot;
end
end

function invalidOptions()
error('duallink5:viz:InvalidPlotOptions', ...
    'options must contain valid lineWidth and showLabels values.');
end

function invalidPose()
error('duallink5:viz:InvalidPose', ...
    'pose must contain a valid solved mechanism pose.');
end
