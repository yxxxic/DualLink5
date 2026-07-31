function [candidates, statusCode] = solveClosure( ...
        C, E, radiusCD, radiusDE, tolerance)
if ~(isnumeric(C) && isreal(C) && numel(C) == 2 && ...
        all(isfinite(C(:)))) || ...
        ~(isnumeric(E) && isreal(E) && numel(E) == 2 && ...
        all(isfinite(E(:))))
    error('duallink5:kinematics:InvalidClosureInput', ...
        'C and E must each contain exactly two finite numeric elements.');
end

if ~isPositiveFiniteScalar(radiusCD) || ...
        ~isPositiveFiniteScalar(radiusDE) || ...
        ~isPositiveFiniteScalar(tolerance)
    error('duallink5:kinematics:InvalidClosureInput', ...
        ['radiusCD, radiusDE, and tolerance must each be a positive ' ...
         'finite numeric scalar.']);
end

C = double(C(:));
E = double(E(:));
radiusCD = double(radiusCD);
radiusDE = double(radiusDE);
tolerance = double(tolerance);

template = struct( ...
    'D', zeros(2, 1), ...
    'branchId', int8(0), ...
    'closureResidual', Inf);
candidates = repmat(template, 0, 1);

delta = E - C;
distance = norm(delta);
if distance < tolerance || ...
        distance > radiusCD + radiusDE + tolerance || ...
        distance < abs(radiusCD - radiusDE) - tolerance
    statusCode = "UNREACHABLE_CLOSURE";
    return
end

unitCE = delta / distance;
along = (radiusCD^2 - radiusDE^2 + distance^2) / (2 * distance);
heightSquared = radiusCD^2 - along^2;
squaredTolerance = tolerance * ...
    max([radiusCD, radiusDE, distance, tolerance]);
if heightSquared < -squaredTolerance
    statusCode = "UNREACHABLE_CLOSURE";
    return
end

height = sqrt(max(0, heightSquared));
basePoint = C + along * unitCE;
normal = [-unitCE(2); unitCE(1)];
if height <= tolerance
    candidate = template;
    candidate.D = basePoint;
    candidate.closureResidual = closureResidual( ...
        basePoint, C, E, radiusCD, radiusDE);
    candidates = candidate;
    statusCode = "NEAR_SINGULAR";
    return
end

candidates = repmat(template, 2, 1);
points = [basePoint + height * normal, basePoint - height * normal];
for index = 1:2
    D = points(:, index);
    candidates(index).D = D;
    candidates(index).branchId = int8(sign(cross2(delta, D - C)));
    candidates(index).closureResidual = closureResidual( ...
        D, C, E, radiusCD, radiusDE);
end
statusCode = "OK";
end

function valid = isPositiveFiniteScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value > 0;
end

function residual = closureResidual(D, C, E, radiusCD, radiusDE)
residual = max(abs([norm(D - C) - radiusCD, ...
    norm(D - E) - radiusDE]));
end

function value = cross2(first, second)
value = first(1) * second(2) - first(2) * second(1);
end
