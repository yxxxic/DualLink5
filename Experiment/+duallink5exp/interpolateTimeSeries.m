function result = interpolateTimeSeries(sourceTime, sourceValue, queryTime)
validateSourceVector(sourceTime);
validateSourceVector(sourceValue);
validateQueryVector(queryTime);
if numel(sourceTime) ~= numel(sourceValue)
    error('duallink5exp:TimeSeriesSizeMismatch', ...
        'sourceTime and sourceValue must have the same length.');
end

sourceTime = full(double(sourceTime(:)));
sourceValue = full(double(sourceValue(:)));
queryShape = size(queryTime);
queryVector = full(double(queryTime(:)));
resultVector = nan(size(queryVector));

validSource = isfinite(sourceTime) & isfinite(sourceValue);
sourceTime = sourceTime(validSource);
sourceValue = sourceValue(validSource);
if isempty(sourceTime)
    result = reshape(resultVector, queryShape);
    return
end

% Sort by time and original order. Keeping the last adjacent duplicate then
% makes the last sample in the original stable order win deterministically.
originalOrder = (1:numel(sourceTime)).';
[~, sortOrder] = sortrows([sourceTime, originalOrder], [1, 2]);
sourceTime = sourceTime(sortOrder);
sourceValue = sourceValue(sortOrder);
keepLastDuplicate = [diff(sourceTime) ~= 0; true];
sourceTime = sourceTime(keepLastDuplicate);
sourceValue = sourceValue(keepLastDuplicate);

finiteQuery = isfinite(queryVector);
if isscalar(sourceTime)
    matchTolerance = 16 * eps(max(1, abs(sourceTime)));
    matchingQuery = finiteQuery & ...
        abs(queryVector - sourceTime) <= matchTolerance;
    resultVector(matchingQuery) = sourceValue;
else
    resultVector(finiteQuery) = interp1( ...
        sourceTime, sourceValue, queryVector(finiteQuery), 'linear', NaN);
end
result = reshape(resultVector, queryShape);
end

function validateSourceVector(value)
if ~(isnumeric(value) && isreal(value) && ...
        (isvector(value) || isempty(value)))
    error('duallink5exp:InvalidTimeSeries', ...
        'Source times and values must be real numeric vectors.');
end
end

function validateQueryVector(value)
if ~(isnumeric(value) && isreal(value) && ...
        (isvector(value) || isempty(value)))
    error('duallink5exp:InvalidQueryTime', ...
        'Query times must be a real numeric vector.');
end
end
