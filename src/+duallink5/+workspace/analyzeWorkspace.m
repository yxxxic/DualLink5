function result = analyzeWorkspace(samples, options)
if nargin < 2
    options = struct();
end
samples = normalizeSamples(samples);
validX = full(double(samples.x(samples.validMask)));
validY = full(double(samples.y(samples.validMask)));
validX = validX(:);
validY = validY(:);
uniquePoints = unique([validX(:), validY(:)], 'rows');
if size(uniquePoints, 1) < 3
    error('duallink5:workspace:InsufficientSamples', ...
        'At least three unique valid task samples are required.');
end
centeredPoints = uniquePoints - mean(uniquePoints, 1);
rankTolerance = max(size(centeredPoints)) * ...
    eps(norm(centeredPoints, 2));
if rank(centeredPoints, rankTolerance) < 2
    error('duallink5:workspace:InsufficientSpan', ...
        'Valid task samples must span a two-dimensional region.');
end
validX = uniquePoints(:, 1);
validY = uniquePoints(:, 2);

alpha = getOption(options, 'alpha', Inf);
if ~(isnumeric(alpha) && isreal(alpha) && isscalar(alpha) && ...
        ~isnan(alpha) && alpha > 0)
    error('duallink5:workspace:InvalidAlpha', ...
        'alpha must be a positive scalar or Inf.');
end
alpha = full(double(alpha));
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
gridSize = full(double(gridSize));
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
    conditions = samples.conditionNumber( ...
        ~isnan(samples.conditionNumber));
else
    conditions = [];
end
result.singularity.nearCount = result.nearSingularCount;
if isempty(conditions)
    result.singularity.maxCondition = NaN;
    result.singularity.medianCondition = NaN;
else
    result.singularity.maxCondition = max(conditions);
    result.singularity.medianCondition = median(conditions);
end
result.metadata = samples.metadata;
end

function samples = normalizeSamples(samples)
requiredFields = {'x', 'y', 'validMask', 'reasonMap', 'metadata'};
if ~isstruct(samples) || ~isscalar(samples) || ...
        ~all(isfield(samples, requiredFields))
    invalidSamples();
end
if ~isnumeric(samples.x) || ~isreal(samples.x) || ...
        ~ismatrix(samples.x) || ...
        ~isnumeric(samples.y) || ~isreal(samples.y) || ...
        ~ismatrix(samples.y)
    invalidSamples();
end
sampleSize = size(samples.x);
if ~isequal(size(samples.y), sampleSize) || ...
        ~isstring(samples.reasonMap) || ...
        ~ismatrix(samples.reasonMap) || ...
        ~isequal(size(samples.reasonMap), sampleSize)
    invalidSamples();
end

mask = samples.validMask;
logicalMask = islogical(mask) && ismatrix(mask);
numericMask = isnumeric(mask) && isreal(mask) && ismatrix(mask) && ...
    all(isfinite(mask), 'all') && ...
    all(mask == 0 | mask == 1, 'all');
if ~(logicalMask || numericMask) || ~isequal(size(mask), sampleSize)
    invalidSamples();
end
samples.validMask = logical(mask);
if any(~isfinite(samples.x(samples.validMask)), 'all') || ...
        any(~isfinite(samples.y(samples.validMask)), 'all')
    invalidSamples();
end

if isfield(samples, 'conditionNumber') && ...
        (~isnumeric(samples.conditionNumber) || ...
        ~isreal(samples.conditionNumber) || ...
        ~isequal(size(samples.conditionNumber), sampleSize))
    invalidSamples();
end
end

function invalidSamples()
error('duallink5:workspace:InvalidSamples', ...
    ['samples must contain consistently sized real numeric x/y, ' ...
     'a binary validMask, string reasonMap, and metadata.']);
end

function value = getOption(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end
