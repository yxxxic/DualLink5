function result = taskJacobian(q, geometry, taskSpec, options)
if nargin < 4
    options = struct();
end

if isnumeric(q)
    q = q(:).';
    input.lower = q;
    input.upper = q;
    idealOptions = options;
    idealOptions.mode = "ideal";
    assembly = duallink5.kinematics.forwardAssembly( ...
        input, geometry, idealOptions);
    if ~assembly.quality.valid
        result = invalidResult( ...
            taskSpec, assembly.quality.statusCode, false);
        return
    end
    lower = duallink5.kinematics.fiveBarJacobian( ...
        q, assembly.lower, geometry, taskSpec);
    result.logical = lower.matrix;
    result.lowerLocal = lower.matrix;
    result.upperLocal = lower.matrix;
    result.closureConditionNumber = ...
        lower.closureConditionNumber;
    result.taskConditionNumber = lower.taskConditionNumber;
    result.taskRowScale = lower.taskRowScale;
    result.conditionNumber = lower.conditionNumber;
    result.nearSingular = lower.nearSingular;
    result.statusCode = lower.statusCode;
    result.assemblyStatusCode = assembly.quality.statusCode;
    return
end

diagnosticOptions = options;
diagnosticOptions.mode = "diagnostic";
assembly = duallink5.kinematics.forwardAssembly( ...
    q, geometry, diagnosticOptions);
if ~isfield(assembly.quality, 'diagnosticAvailable') || ...
        ~assembly.quality.diagnosticAvailable
    result = invalidResult( ...
        taskSpec, assembly.quality.statusCode, true);
    return
end
lower = duallink5.kinematics.fiveBarJacobian( ...
    q.lower, assembly.lower, geometry, taskSpec);
upper = duallink5.kinematics.fiveBarJacobian( ...
    q.upper, assembly.upperLocal, geometry, taskSpec);
result.logical = [];
result.lowerLocal = lower.matrix;
result.upperLocal = upper.matrix;
result.closureConditionNumber = [ ...
    lower.closureConditionNumber, ...
    upper.closureConditionNumber];
result.taskConditionNumber = [lower.taskConditionNumber, ...
    upper.taskConditionNumber];
result.taskRowScale = lower.taskRowScale;
result.conditionNumber = [lower.conditionNumber, ...
    upper.conditionNumber];
result.nearSingular = lower.nearSingular || upper.nearSingular;
result.assemblyStatusCode = assembly.quality.statusCode;
if result.nearSingular
    result.statusCode = "NEAR_SINGULAR";
else
    result.statusCode = "DIAGNOSTIC_ONLY";
end
end

function result = invalidResult(taskSpec, statusCode, diagnostic)
includeOrientation = isfield(taskSpec, 'includeOrientation') && ...
    taskSpec.includeOrientation;
rows = 2 + double(includeOrientation);
result.logical = nan(rows, 2);
if diagnostic
    result.logical = [];
end
result.lowerLocal = nan(rows, 2);
result.upperLocal = nan(rows, 2);
result.closureConditionNumber = Inf;
result.taskConditionNumber = Inf;
result.taskRowScale = nan(1, rows);
result.conditionNumber = Inf;
result.nearSingular = statusCode == "NEAR_SINGULAR";
result.statusCode = statusCode;
result.assemblyStatusCode = statusCode;
end
