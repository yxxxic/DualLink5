function refined = refineTypeIICurves( ...
        curves, targetDistance, geometry, bounds)
template = struct('theta', [], 'phi', []);
refined = repmat(template, 0, 1);
if targetDistance <= geometry.tolerance.length
    return
end
tolerance = 32 * eps(max([targetDistance, ...
    geometry.links.link3, geometry.links.link4]));
for curveIndex = 1:numel(curves)
    source = [curves(curveIndex).theta; curves(curveIndex).phi];
    projected = nan(size(source));
    keep = false(1, size(source, 2));
    for pointIndex = 1:size(source, 2)
        q = source(:, pointIndex);
        iteration = 0;
        while iteration < 15
            iteration = iteration + 1;
            [residual, gradient] = closureResidual( ...
                q, targetDistance, geometry);
            if abs(residual) <= tolerance
                keep(pointIndex) = true;
                break
            end
            denominator = dot(gradient, gradient);
            if ~isfinite(denominator) || denominator <= eps
                break
            end
            correction = residual * gradient.' / denominator;
            if norm(correction) > 0.25
                correction = 0.25 * correction / norm(correction);
            end
            q = min(max(q - correction, bounds(:, 1)), bounds(:, 2));
        end
        [residual, ~] = closureResidual(q, targetDistance, geometry);
        keep(pointIndex) = abs(residual) <= tolerance;
        if keep(pointIndex)
            projected(:, pointIndex) = q;
        end
    end
    projected = projected(:, keep);
    if size(projected, 2) >= 2
        refined(end + 1, 1) = struct( ...
            'theta', projected(1, :), ...
            'phi', projected(2, :)); %#ok<AGROW>
    end
end
end

function [residual, gradient] = closureResidual(q, targetDistance, geometry)
L = geometry.links;
theta = q(1);
phi = q(2);
E = L.link1 * [cos(theta); sin(theta)];
C = [L.link5 - L.link2 * cos(phi); L.link2 * sin(phi)];
delta = E - C;
distance = norm(delta);
residual = distance - targetDistance;
if distance <= eps
    gradient = [NaN, NaN];
    return
end
eTheta = L.link1 * [-sin(theta); cos(theta)];
cPhi = L.link2 * [sin(phi); cos(phi)];
gradient = [dot(delta, eTheta), -dot(delta, cPhi)] / distance;
end
