function assessment = assessIdealAssembly( ...
        q, geometry, options, topologyReference)
assessment.valid = false;
assessment.statusCode = "UNINITIALIZED";
ranges = [geometry.analysis.thetaRange; geometry.analysis.phiRange];
if any(q < ranges(:, 1).' | q > ranges(:, 2).')
    assessment.statusCode = "JOINT_LIMIT_VIOLATION";
    return
end

input = struct('lower', q, 'upper', q);
assemblyOptions = struct( ...
    'mode', "ideal", ...
    'branchMode', "fixed", ...
    'branchId', options.branchId, ...
    'collisionProfile', options.collisionProfile);
assembly = duallink5.kinematics.forwardAssembly( ...
    input, geometry, assemblyOptions);
assessment.statusCode = assembly.quality.statusCode;
if ~assembly.quality.valid
    return
end

if options.topologyProfile == "reference"
    comparison = ...
        duallink5.validation.compareAssemblyTopologyToReference( ...
        assembly, topologyReference);
    if ~comparison.compatible
        assessment.statusCode = comparison.statusCode;
        return
    end
end
assessment.valid = true;
assessment.statusCode = "OK";
end
