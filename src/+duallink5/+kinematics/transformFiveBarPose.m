function transformed = transformFiveBarPose(pose, rotation, translation)
validRotation = isnumeric(rotation) && isreal(rotation) && ...
    isequal(size(rotation), [2, 2]) && all(isfinite(rotation), 'all');
validTranslation = isnumeric(translation) && isreal(translation) && ...
    numel(translation) == 2 && all(isfinite(translation), 'all');
if ~validRotation || ~validTranslation
    invalidTransform();
end

rotationWasSingle = isa(rotation, 'single');
rotation = double(rotation);
translation = double(translation(:));
rotationScale = max(1, norm(rotation, 'fro') ^ 2);
if rotationWasSingle
    rigidTolerance = 16 * double(eps(single(rotationScale)));
else
    rigidTolerance = 16 * eps(rotationScale);
end
orthogonalityResidual = norm( ...
    rotation.' * rotation - eye(2), 'fro');
determinantResidual = abs(det(rotation) - 1);
if orthogonalityResidual > rigidTolerance || ...
        determinantResidual > rigidTolerance
    invalidTransform();
end

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

function invalidTransform()
error('duallink5:kinematics:InvalidTransform', ...
    ['rotation must be a finite SO(2) matrix and translation must be ' ...
     'a finite 2-vector.']);
end
