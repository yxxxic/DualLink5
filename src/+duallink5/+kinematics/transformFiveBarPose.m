function transformed = transformFiveBarPose(pose, rotation, translation)
validRotation = isnumeric(rotation) && isreal(rotation) && ...
    isequal(size(rotation), [2, 2]) && all(isfinite(rotation), 'all');
validTranslation = isnumeric(translation) && isreal(translation) && ...
    numel(translation) == 2 && all(isfinite(translation), 'all');
if ~validRotation || ~validTranslation
    error('duallink5:kinematics:InvalidTransform', ...
        'rotation must be 2x2 and translation must be a 2-vector.');
end

translation = translation(:);
transformed = pose;
names = fieldnames(pose.points);
for index = 1:numel(names)
    name = names{index};
    transformed.points.(name) = ...
        rotation * pose.points.(name) + translation;
end

transformed.sharedLink.start = ...
    rotation * pose.sharedLink.start + translation;
transformed.sharedLink.end = ...
    rotation * pose.sharedLink.end + translation;
transformed.sharedLink.center = ...
    rotation * pose.sharedLink.center + translation;
direction = transformed.sharedLink.end - transformed.sharedLink.start;
transformed.sharedLink.orientation = atan2(direction(2), direction(1));
transformed.metadata.rotation = rotation;
transformed.metadata.translation = translation;
end
