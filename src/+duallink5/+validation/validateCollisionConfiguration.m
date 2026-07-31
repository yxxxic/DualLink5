function profile = validateCollisionConfiguration( ...
        profile, collision, allowNone)
try
    profile = string(profile);
catch
    invalidCollisionProfile();
end

allowedProfiles = ["centerline", "physicalClearance"];
if allowNone
    allowedProfiles = ["none", allowedProfiles];
end
if ~isscalar(profile) || ismissing(profile) || ...
        ~ismember(profile, allowedProfiles)
    invalidCollisionProfile();
end

if profile == "physicalClearance"
    validatePhysicalGeometry(collision);
end
end

function validatePhysicalGeometry(collision)
names = {'link1', 'link2', 'link3', 'link4', 'link5'};
for index = 1:numel(names)
    name = names{index};
    validRadius = isfield(collision.radius, name) && ...
        isscalar(collision.radius.(name)) && ...
        isnumeric(collision.radius.(name)) && ...
        isreal(collision.radius.(name)) && ...
        isfinite(collision.radius.(name)) && ...
        collision.radius.(name) >= 0;
    validLayer = isfield(collision.layerOffset, name) && ...
        isscalar(collision.layerOffset.(name)) && ...
        isnumeric(collision.layerOffset.(name)) && ...
        isreal(collision.layerOffset.(name)) && ...
        isfinite(collision.layerOffset.(name));
    if ~validRadius || ~validLayer
        error('duallink5:validation:MissingClearanceGeometry', ...
            ['physicalClearance requires nonnegative link radii ', ...
             'and finite layer offsets for link1 through link5.']);
    end
end
end

function invalidCollisionProfile()
error('duallink5:validation:InvalidCollisionProfile', ...
    ['collision profile must be none, centerline, or ', ...
     'physicalClearance as allowed by this API.']);
end
