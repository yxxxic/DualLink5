function markedValid = validateFiveBarPose(pose, mode)
if ~isstruct(pose) || ~isscalar(pose)
    invalidPose();
end

markedValid = true;
if mode == "assessment"
    if ~isfield(pose, 'quality') || ...
            ~isstruct(pose.quality) || ~isscalar(pose.quality) || ...
            ~isfield(pose.quality, 'valid') || ...
            ~islogical(pose.quality.valid) || ...
            ~isscalar(pose.quality.valid)
        invalidPose();
    end
    markedValid = pose.quality.valid;
    if ~markedValid
        return
    end
    if ~isfield(pose.quality, 'closureResidual') || ...
            ~isnumeric(pose.quality.closureResidual) || ...
            ~isreal(pose.quality.closureResidual) || ...
            ~isscalar(pose.quality.closureResidual) || ...
            ~isfinite(pose.quality.closureResidual)
        invalidPose();
    end
elseif mode ~= "collision"
    invalidPose();
end

validatePoints(pose);
end

function validatePoints(pose)
if ~isfield(pose, 'points') || ...
        ~isstruct(pose.points) || ~isscalar(pose.points)
    invalidPose();
end

names = {'A', 'B', 'C', 'D', 'E'};
for index = 1:numel(names)
    name = names{index};
    if ~isfield(pose.points, name)
        invalidPose();
    end
    point = pose.points.(name);
    if ~isnumeric(point) || ~isreal(point) || ...
            ~isvector(point) || numel(point) ~= 2 || ...
            any(~isfinite(point(:)))
        invalidPose();
    end
end
end

function invalidPose()
error('duallink5:validation:InvalidPose', ...
    'pose must contain finite real two-vector points A through E.');
end
