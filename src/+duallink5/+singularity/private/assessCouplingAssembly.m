function assessment = assessCouplingAssembly( ...
        q, kinematics, geometry, options, reference)
%ASSESSCOUPLINGASSEMBLY Separate theoretical and mechanical validity.
assessment = defaultAssessment(kinematics);
if ~assessment.theoreticallyReachable
    return
end

ranges = [geometry.analysis.thetaRange; geometry.analysis.phiRange];
assessment.jointLimitValid = ...
    ~any(q < ranges(:, 1).' | q > ranges(:, 2).');

candidateSign = int8(kinematics.quality.passiveOrientationSign);
assessment.passiveOrientationCompatible = candidateSign == 0 || ...
    candidateSign == reference.passiveOrientationSign;

if options.topologyProfile == "reference"
    comparison = ...
        duallink5.validation.compareAssemblyTopologyToReference( ...
        kinematics, reference.topology);
    assessment.topologyComparison = comparison;
    assessment.assemblyTopologyCompatible = comparison.compatible;
else
    assessment.assemblyTopologyCompatible = true;
end

if options.collisionProfile == "none"
    assessment.collisionFree = true;
    assessment.collisionStatusCode = "OK";
else
    [assessment.collisionFree, assessment.collisionStatusCode] = ...
        assessCollision(kinematics, geometry, options.collisionProfile);
end

assessment.mechanicallyValid = assessment.jointLimitValid && ...
    assessment.passiveOrientationCompatible && ...
    assessment.assemblyTopologyCompatible && assessment.collisionFree;
assessment.statusCode = mechanicalStatus(assessment);
end

function [collisionFree, status] = assessCollision( ...
        kinematics, geometry, profile)
[lowerSafe, ~] = duallink5.validation.isCollisionFree( ...
    kinematics.lower, geometry, profile);
[upperSafe, ~] = duallink5.validation.isCollisionFree( ...
    kinematics.upper, geometry, profile);
if ~lowerSafe || ~upperSafe
    collisionFree = false;
    status = "SELF_COLLISION";
    return
end
clearance = parallelSharedDistance(kinematics.lower.points);
collisionFree = clearance > geometry.collision.parallelSharedClearance;
if collisionFree
    status = "OK";
else
    status = "PARALLEL_SHARED_COLLISION";
end
end

function distance = parallelSharedDistance(points)
distance = min([ ...
    pointSegmentDistance(points.Pbeta1, points.E, points.D), ...
    pointSegmentDistance(points.Pbeta2, points.E, points.D), ...
    pointSegmentDistance(points.E, points.Pbeta1, points.Pbeta2), ...
    pointSegmentDistance(points.D, points.Pbeta1, points.Pbeta2)]);
end

function distance = pointSegmentDistance(point, startPoint, endPoint)
segment = endPoint - startPoint;
denominator = dot(segment, segment);
if denominator == 0
    distance = norm(point - startPoint);
    return
end
parameter = dot(point - startPoint, segment) / denominator;
parameter = max(0, min(1, parameter));
distance = norm(point - (startPoint + parameter * segment));
end

function status = mechanicalStatus(assessment)
if ~assessment.jointLimitValid
    status = "JOINT_LIMIT_VIOLATION";
elseif ~assessment.passiveOrientationCompatible
    status = "PASSIVE_ASSEMBLY_TOPOLOGY_MISMATCH";
elseif ~assessment.assemblyTopologyCompatible
    status = assessment.topologyComparison.statusCode;
elseif ~assessment.collisionFree
    status = assessment.collisionStatusCode;
else
    status = "OK";
end
end

function assessment = defaultAssessment(kinematics)
assessment = struct( ...
    'theoreticallyReachable', logical( ...
        kinematics.quality.theoreticallyReachable), ...
    'mechanicallyValid', false, ...
    'jointLimitValid', false, ...
    'passiveOrientationCompatible', false, ...
    'assemblyTopologyCompatible', false, ...
    'collisionFree', false, ...
    'collisionStatusCode', "NOT_ASSESSED", ...
    'topologyComparison', struct(), ...
    'statusCode', string(kinematics.quality.closureStatusCode));
end
