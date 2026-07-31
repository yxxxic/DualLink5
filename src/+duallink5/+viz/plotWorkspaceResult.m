function handles = plotWorkspaceResult(samples, result, axesHandle)
if ~(isscalar(axesHandle) && isgraphics(axesHandle, 'axes'))
    error('duallink5:viz:InvalidGraphicsHandle', ...
        'axesHandle must be a live scalar axes handle.');
end
[validX, validY] = validateSamples(samples);
[boundaryShape, rectangleInfo] = validateResult(result);
millimetresPerMetre = 1e3;
validX = millimetresPerMetre * validX;
validY = millimetresPerMetre * validY;

originalNextPlot = axesHandle.NextPlot;
axesHandle.NextPlot = 'add';
cleanup = onCleanup( ...
    @()restoreNextPlot(axesHandle, originalNextPlot));
handles.samples = scatter(axesHandle, validX, validY, 8, '.');
[triangles, shapePoints] = alphaTriangulation(boundaryShape);
shapePoints = millimetresPerMetre * shapePoints;
boundaryColor = [0, 0.45, 0.74];
handles.boundary = patch(axesHandle, ...
    'Faces', triangles, 'Vertices', shapePoints, ...
    'FaceColor', boundaryColor, 'FaceAlpha', 0.08, ...
    'EdgeColor', 'none');
[boundaryEdges, boundaryPoints] = boundaryFacets(boundaryShape);
boundaryPoints = millimetresPerMetre * boundaryPoints;
edgeCount = size(boundaryEdges, 1);
outlineX = [boundaryPoints(boundaryEdges(:, 1), 1), ...
    boundaryPoints(boundaryEdges(:, 2), 1), nan(edgeCount, 1)].';
outlineY = [boundaryPoints(boundaryEdges(:, 1), 2), ...
    boundaryPoints(boundaryEdges(:, 2), 2), nan(edgeCount, 1)].';
handles.boundaryOutline = plot(axesHandle, ...
    outlineX(:), outlineY(:), '-', ...
    'Color', boundaryColor, 'LineWidth', 1.2);
handles.rectangle = gobjects(0);
if rectangleInfo.areaCells > 0
    bounds = millimetresPerMetre * rectangleInfo.bounds;
    handles.rectangle = rectangle(axesHandle, 'Position', ...
        [bounds(1), bounds(3), bounds(2) - bounds(1), ...
        bounds(4) - bounds(3)], 'EdgeColor', 'r', 'LineWidth', 2);
end
axis(axesHandle, 'equal');
xlabel(axesHandle, 'x [mm]');
ylabel(axesHandle, 'y [mm]');
clear cleanup
end

function [validX, validY] = validateSamples(samples)
requiredFields = {'x', 'y', 'validMask'};
if ~isstruct(samples) || ~isscalar(samples) || ...
        ~all(isfield(samples, requiredFields)) || ...
        ~isnumeric(samples.x) || ~isreal(samples.x) || ...
        ~ismatrix(samples.x) || ...
        ~isnumeric(samples.y) || ~isreal(samples.y) || ...
        ~ismatrix(samples.y) || ...
        ~isequal(size(samples.x), size(samples.y))
    invalidWorkspaceInput();
end

mask = samples.validMask;
logicalMask = islogical(mask) && ismatrix(mask);
numericMask = isnumeric(mask) && isreal(mask) && ismatrix(mask) && ...
    all(isfinite(mask), 'all') && all(mask == 0 | mask == 1, 'all');
if ~(logicalMask || numericMask) || ...
        ~isequal(size(mask), size(samples.x))
    invalidWorkspaceInput();
end
mask = logical(mask);
if any(~isfinite(samples.x(mask)), 'all') || ...
        any(~isfinite(samples.y(mask)), 'all')
    invalidWorkspaceInput();
end
validX = full(double(samples.x(mask)));
validY = full(double(samples.y(mask)));
end

function [boundaryShape, rectangleInfo] = validateResult(result)
if ~isstruct(result) || ~isscalar(result) || ...
        ~isfield(result, 'boundaryShape') || ...
        ~isfield(result, 'maxRectangle') || ...
        ~isa(result.boundaryShape, 'alphaShape') || ...
        ~isscalar(result.boundaryShape)
    invalidWorkspaceInput();
end
boundaryShape = result.boundaryShape;
shapePoints = boundaryShape.Points;
if ~isnumeric(shapePoints) || ~isreal(shapePoints) || ...
        size(shapePoints, 1) < 3 || size(shapePoints, 2) ~= 2 || ...
        any(~isfinite(shapePoints), 'all')
    invalidWorkspaceInput();
end

rectangleInfo = result.maxRectangle;
if ~isstruct(rectangleInfo) || ~isscalar(rectangleInfo) || ...
        ~isfield(rectangleInfo, 'areaCells')
    invalidWorkspaceInput();
end
areaCells = rectangleInfo.areaCells;
if ~isnumeric(areaCells) || ~isreal(areaCells) || ...
        ~isscalar(areaCells) || ~isfinite(areaCells) || ...
        areaCells < 0 || areaCells ~= fix(areaCells)
    invalidWorkspaceInput();
end
rectangleInfo.areaCells = double(areaCells);
if rectangleInfo.areaCells > 0
    if ~isfield(rectangleInfo, 'bounds')
        invalidWorkspaceInput();
    end
    bounds = rectangleInfo.bounds;
    if ~isnumeric(bounds) || ~isreal(bounds) || ...
            ~isequal(size(bounds), [1, 4]) || ...
            any(~isfinite(bounds)) || ...
            bounds(2) <= bounds(1) || bounds(4) <= bounds(3)
        invalidWorkspaceInput();
    end
    rectangleInfo.bounds = full(double(bounds));
end
end

function restoreNextPlot(axesHandle, originalNextPlot)
if isscalar(axesHandle) && isgraphics(axesHandle, 'axes')
    axesHandle.NextPlot = originalNextPlot;
end
end

function invalidWorkspaceInput()
error('duallink5:viz:InvalidWorkspacePlotInput', ...
    'samples and result must contain valid two-dimensional workspace data.');
end
