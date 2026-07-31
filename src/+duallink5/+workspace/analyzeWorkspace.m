function result = analyzeWorkspace(samples, options)
if nargin < 2
    options = struct();
end
validX = samples.x(samples.validMask);
validY = samples.y(samples.validMask);
if numel(validX) < 3
    error('duallink5:workspace:InsufficientSamples', ...
        'At least three valid task samples are required.');
end
if max(validX) == min(validX) || max(validY) == min(validY)
    error('duallink5:workspace:InsufficientSpan', ...
        'Valid task samples must span nonzero x and y ranges.');
end

alpha = getOption(options, 'alpha', Inf);
if ~(isnumeric(alpha) && isreal(alpha) && isscalar(alpha) && ...
        ~isnan(alpha) && alpha > 0)
    error('duallink5:workspace:InvalidAlpha', ...
        'alpha must be a positive scalar or Inf.');
end
shape = alphaShape(validX, validY, alpha);
result.area = area(shape);
result.boundaryShape = shape;

gridSize = getOption(options, 'gridSize', [150, 150]);
if ~(isnumeric(gridSize) && isreal(gridSize) && ...
        isequal(size(gridSize), [1, 2]) && ...
        all(isfinite(gridSize)) && all(gridSize >= 2) && ...
        all(gridSize == fix(gridSize)))
    error('duallink5:workspace:InvalidGridSize', ...
        ['gridSize must contain two integers greater than or ' ...
         'equal to 2.']);
end
result.xEdges = linspace( ...
    min(validX), max(validX), gridSize(1) + 1);
result.yEdges = linspace( ...
    min(validY), max(validY), gridSize(2) + 1);
result.xGrid = (result.xEdges(1:end-1) + ...
    result.xEdges(2:end)) / 2;
result.yGrid = (result.yEdges(1:end-1) + ...
    result.yEdges(2:end)) / 2;
[X, Y] = meshgrid(result.xGrid, result.yGrid);
result.insideMask = inShape(shape, X, Y);
dx = result.xEdges(2) - result.xEdges(1);
dy = result.yEdges(2) - result.yEdges(1);
result.maxRectangle = ...
    duallink5.workspace.largestRectangleInMask( ...
    result.insideMask, dx, dy);
if result.maxRectangle.areaCells == 0
    result.maxRectangle.bounds = [NaN, NaN, NaN, NaN];
else
    rectangle = result.maxRectangle;
    result.maxRectangle.bounds = [ ...
        result.xEdges(rectangle.left), ...
        result.xEdges(rectangle.right + 1), ...
        result.yEdges(rectangle.top), ...
        result.yEdges(rectangle.bottom + 1)];
end

statusCodes = unique(samples.reasonMap(:));
counts = arrayfun( ...
    @(code) nnz(samples.reasonMap(:) == code), statusCodes);
result.reasonCounts = table(statusCodes, counts, ...
    'VariableNames', {'statusCode', 'count'});
result.nearSingularCount = ...
    nnz(samples.reasonMap == "NEAR_SINGULAR");
if isfield(samples, 'conditionNumber')
    finiteConditions = samples.conditionNumber( ...
        isfinite(samples.conditionNumber));
else
    finiteConditions = [];
end
result.singularity.nearCount = result.nearSingularCount;
if isempty(finiteConditions)
    result.singularity.maxCondition = NaN;
    result.singularity.medianCondition = NaN;
else
    result.singularity.maxCondition = max(finiteConditions);
    result.singularity.medianCondition = median(finiteConditions);
end
result.metadata = samples.metadata;
end

function value = getOption(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end
