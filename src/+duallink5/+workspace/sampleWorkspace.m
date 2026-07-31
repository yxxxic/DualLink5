function samples = sampleWorkspace(grid, geometry, taskSpec, options)
if nargin < 4
    options = struct();
end
if ~isstruct(grid) || ~isscalar(grid) || ...
        ~isfield(grid, 'theta') || ~isfield(grid, 'phi') || ...
        ~isValidAngleVector(grid.theta) || ...
        ~isValidAngleVector(grid.phi)
    error('duallink5:workspace:InvalidAngleGrid', ...
        'grid.theta and grid.phi must be finite vectors.');
end

[thetaGrid, phiGrid] = ndgrid(grid.theta, grid.phi);
gridShape = size(thetaGrid);
samples.x = nan(gridShape);
samples.y = nan(gridShape);
samples.orientation = nan(gridShape);
samples.validMask = false(gridShape);
samples.reasonMap = strings(gridShape);
samples.conditionNumber = nan(gridShape);
samples.thetaGrid = thetaGrid;
samples.phiGrid = phiGrid;

for index = 1:numel(thetaGrid)
    qLogical = [thetaGrid(index), phiGrid(index)];
    q.lower = qLogical;
    q.upper = qLogical;
    assemblyOptions = options;
    assemblyOptions.mode = "ideal";
    assembly = duallink5.kinematics.forwardAssembly( ...
        q, geometry, assemblyOptions);
    samples.reasonMap(index) = assembly.quality.statusCode;
    if ~assembly.quality.valid
        continue
    end

    jacobian = duallink5.kinematics.fiveBarJacobian( ...
        qLogical, assembly.lower, geometry, taskSpec);
    samples.conditionNumber(index) = jacobian.conditionNumber;
    if jacobian.nearSingular
        samples.reasonMap(index) = "NEAR_SINGULAR";
        continue
    end

    task = duallink5.kinematics.taskPose(assembly, taskSpec);
    samples.x(index) = task.position(1);
    samples.y(index) = task.position(2);
    if task.includeOrientation
        samples.orientation(index) = task.orientation;
    end
    samples.validMask(index) = true;
end

samples.metadata.units = geometry.units;
samples.metadata.taskSpec = taskSpec;
end

function valid = isValidAngleVector(value)
valid = isnumeric(value) && isreal(value) && isvector(value) && ...
    all(isfinite(value), 'all');
end
