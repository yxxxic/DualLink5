function handles = plotAssembly(assembly, axesHandle, options)
if nargin < 3
    options = struct();
end
if ~(isscalar(axesHandle) && isgraphics(axesHandle, 'axes'))
    error('duallink5:viz:InvalidGraphicsHandle', ...
        'axesHandle must be a live scalar axes handle.');
end
[lineWidth, showLabels] = validateOptions(options);
[lower, upper, sharedCenter] = validateAssembly(assembly);
millimetresPerMetre = 1e3;
lower = scalePoints(lower, millimetresPerMetre);
upper = scalePoints(upper, millimetresPerMetre);
sharedCenter = millimetresPerMetre * sharedCenter;

originalNextPlot = axesHandle.NextPlot;
axesHandle.NextPlot = 'add';
cleanup = onCleanup( ...
    @()restoreNextPlot(axesHandle, originalNextPlot));

colors.lower = [0.0000, 0.4470, 0.7410];
colors.upper = [0.8500, 0.3250, 0.0980];
colors.alpha = [0.4940, 0.1840, 0.5560];
colors.beta = [0.4660, 0.6740, 0.1880];
colors.shared = [0.15, 0.15, 0.15];

physicalSegments = {'A', 'B'; 'A', 'E'; 'B', 'C'; 'C', 'D'};
handles.lowerLinks = plotNamedSegments( ...
    axesHandle, lower, physicalSegments, '-', colors.lower, lineWidth);
handles.upperLinks = plotNamedSegments( ...
    axesHandle, upper, physicalSegments, '-', colors.upper, lineWidth);
handles.sharedLink = plot(axesHandle, ...
    [lower.E(1), lower.D(1)], [lower.E(2), lower.D(2)], ...
    '--', 'Color', colors.shared, 'LineWidth', lineWidth);

alphaSegments = {'Palpha1', 'Palpha2'; 'Palpha2', 'Palpha3'; ...
    'Palpha3', 'Palpha4'; 'Palpha4', 'Palpha1'};
betaSegments = {'Pbeta1', 'Pbeta2'; 'Pbeta2', 'Pbeta3'; ...
    'Pbeta3', 'Pbeta4'; 'Pbeta4', 'Pbeta1'};
handles.alphaLinks = plotNamedSegments( ...
    axesHandle, lower, alphaSegments, '-', colors.alpha, lineWidth);
handles.betaLinks = plotNamedSegments( ...
    axesHandle, lower, betaSegments, '-', colors.beta, lineWidth);

mainJointNames = {'A', 'B', 'C', 'D', 'E'};
mainJoints = pointsMatrix(lower, mainJointNames);
upperJointNames = {'A', 'B', 'C'};
mainJoints = [mainJoints, pointsMatrix(upper, upperJointNames)];
handles.mainJoints = plot(axesHandle, mainJoints(1, :), ...
    mainJoints(2, :), 'o', 'LineStyle', 'none', ...
    'MarkerEdgeColor', colors.shared, 'MarkerFaceColor', 'w');

parallelNames = {'Palpha1', 'Palpha2', 'Palpha3', 'Palpha4', ...
    'Pbeta1', 'Pbeta2', 'Pbeta3', 'Pbeta4'};
parallelJoints = pointsMatrix(lower, parallelNames);
handles.parallelJoints = plot(axesHandle, parallelJoints(1, :), ...
    parallelJoints(2, :), 'o', 'LineStyle', 'none', ...
    'MarkerSize', 5, 'MarkerEdgeColor', colors.shared, ...
    'MarkerFaceColor', 'w');
handles.sharedCenter = plot(axesHandle, sharedCenter(1), ...
    sharedCenter(2), 'kp', 'MarkerFaceColor', 'y');

if showLabels
    [labelPoints, labelText] = assemblyLabels(lower, upper, sharedCenter);
    handles.labels = gobjects(numel(labelText), 1);
    for index = 1:numel(labelText)
        handles.labels(index) = text(axesHandle, ...
            labelPoints(1, index), labelPoints(2, index), ...
            ['  ', labelText{index}], 'Interpreter', 'tex');
    end
else
    handles.labels = gobjects(0, 1);
end

axis(axesHandle, 'equal');
axis(axesHandle, 'padded');
xlabel(axesHandle, 'x [mm]');
ylabel(axesHandle, 'y [mm]');
clear cleanup
end

function handles = plotNamedSegments( ...
        axesHandle, points, segments, lineStyle, color, lineWidth)
handles = gobjects(size(segments, 1), 1);
for index = 1:size(segments, 1)
    startPoint = points.(segments{index, 1});
    endPoint = points.(segments{index, 2});
    handles(index) = plot(axesHandle, ...
        [startPoint(1), endPoint(1)], ...
        [startPoint(2), endPoint(2)], lineStyle, ...
        'Color', color, 'LineWidth', lineWidth);
end
end

function matrix = pointsMatrix(points, names)
matrix = zeros(2, numel(names));
for index = 1:numel(names)
    matrix(:, index) = points.(names{index});
end
end

function points = scalePoints(points, scale)
names = fieldnames(points);
for index = 1:numel(names)
    name = names{index};
    points.(name) = scale * points.(name);
end
end

function [points, labels] = assemblyLabels(lower, upper, sharedCenter)
mainLowerNames = {'A', 'B', 'C', 'D', 'E'};
mainUpperNames = {'A', 'B', 'C'};
parallelNames = {'Palpha1', 'Palpha2', 'Palpha3', 'Palpha4', ...
    'Pbeta1', 'Pbeta2', 'Pbeta3', 'Pbeta4'};
points = [pointsMatrix(lower, mainLowerNames), ...
    pointsMatrix(upper, mainUpperNames), ...
    pointsMatrix(lower, parallelNames), sharedCenter];
labels = {'A_k', 'B_k', 'C_k', 'D_k (E_{k+1})', ...
    'E_k (D_{k+1})', 'A_{k+1}', 'B_{k+1}', 'C_{k+1}', ...
    'P_{\alpha}^{1}', 'P_{\alpha}^{2}', 'P_{\alpha}^{3}', ...
    'P_{\alpha}^{4}', 'P_{\beta}^{1}', 'P_{\beta}^{2}', ...
    'P_{\beta}^{3}', 'P_{\beta}^{4}', 'P_k'};
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
validLogical = islogical(showLabels) && isscalar(showLabels);
validNumeric = isnumeric(showLabels) && isreal(showLabels) && ...
    isscalar(showLabels) && isfinite(showLabels) && ...
    ismember(double(showLabels), [0, 1]);
if ~(validLogical || validNumeric)
    invalidOptions();
end
showLabels = logical(showLabels);
end

function [lower, upper, sharedCenter] = validateAssembly(assembly)
requiredAssemblyFields = {'quality', 'lower', 'upper', 'sharedLink'};
if ~isstruct(assembly) || ~isscalar(assembly) || ...
        ~all(isfield(assembly, requiredAssemblyFields)) || ...
        ~isstruct(assembly.quality) || ~isscalar(assembly.quality) || ...
        ~isfield(assembly.quality, 'valid')
    invalidAssembly();
end
validFlag = assembly.quality.valid;
validLogical = islogical(validFlag) && isscalar(validFlag);
validNumeric = isnumeric(validFlag) && isreal(validFlag) && ...
    isscalar(validFlag) && isfinite(validFlag) && ...
    ismember(double(validFlag), [0, 1]);
if ~(validLogical || validNumeric) || ~logical(validFlag)
    invalidAssembly();
end

mainNames = {'A', 'B', 'C', 'D', 'E'};
parallelNames = {'Palpha1', 'Palpha2', 'Palpha3', 'Palpha4', ...
    'Pbeta1', 'Pbeta2', 'Pbeta3', 'Pbeta4'};
lower = validatePoints(assembly.lower, [mainNames, parallelNames]);
upper = validatePoints(assembly.upper, mainNames);
if ~isstruct(assembly.sharedLink) || ...
        ~isscalar(assembly.sharedLink) || ...
        ~isfield(assembly.sharedLink, 'center')
    invalidAssembly();
end
sharedCenter = validatePoint(assembly.sharedLink.center);
end

function points = validatePoints(pose, names)
if ~isstruct(pose) || ~isscalar(pose) || ...
        ~isfield(pose, 'points') || ...
        ~isstruct(pose.points) || ~isscalar(pose.points) || ...
        ~all(isfield(pose.points, names))
    invalidAssembly();
end
points = struct();
for index = 1:numel(names)
    name = names{index};
    points.(name) = validatePoint(pose.points.(name));
end
end

function point = validatePoint(point)
if ~isnumeric(point) || ~isreal(point) || ...
        ~isvector(point) || numel(point) ~= 2 || ...
        any(~isfinite(point), 'all')
    invalidAssembly();
end
point = full(double(point(:)));
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

function invalidAssembly()
error('duallink5:viz:InvalidAssembly', ...
    'assembly must contain a valid ideal symmetric mechanism pose.');
end
