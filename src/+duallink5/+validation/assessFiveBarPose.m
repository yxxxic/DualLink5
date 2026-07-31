function pose = assessFiveBarPose(pose, geometry, profile)
markedValid = validateFiveBarPose(pose, "assessment");
if ~markedValid
    return
end
if pose.quality.closureResidual > geometry.tolerance.residual
    pose.quality.valid = false;
    pose.quality.statusCode = "UNREACHABLE_CLOSURE";
    return
end
profile = duallink5.validation.validateCollisionConfiguration( ...
    profile, geometry.collision, true);
if profile == "none"
    pose.quality.statusCode = "OK";
    return
end

[safe, details] = duallink5.validation.isCollisionFree( ...
    pose, geometry, profile);
pose.quality.collisionFree = safe;
pose.quality.clearanceModelApplied = details.clearanceModelApplied;
pose.quality.collisionDetails = details;

if safe
    pose.quality.statusCode = "OK";
else
    pose.quality.valid = false;
    pose.quality.statusCode = "SELF_COLLISION";
end
end
