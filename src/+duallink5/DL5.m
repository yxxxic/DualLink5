classdef DL5 < handle
    properties (SetAccess = private)
        Geometry
        Q
        Assembly
        PointG
        IsValid
        StatusCode
    end

    properties (Dependent, SetAccess = private)
        QDegrees
        PointGMillimetres
    end

    properties (Access = private)
        TopologyReference
    end

    methods
        function robot = DL5(geometry)
            if nargin < 1 || isempty(geometry)
                geometry = duallink5.model.defaultGeometry();
            else
                geometry = duallink5.model.validateGeometry(geometry);
            end
            robot.Geometry = geometry;

            referenceQ = geometry.analysis.referenceQ;
            referenceCandidate = robot.evaluateJointAngles( ...
                referenceQ, false);
            if ~referenceCandidate.accepted
                error('duallink5:facade:InvalidInitialConfiguration', ...
                    'The configured reference pose is invalid: %s.', ...
                    referenceCandidate.statusCode);
            end
            robot.TopologyReference = ...
                duallink5.validation.prepareAssemblyTopologyReference( ...
                    referenceCandidate.assembly, ...
                    geometry.tolerance.length);
            robot.commit(referenceCandidate);
        end

        function result = trySetJointAngles(robot, q)
            q = validateJointAngles(q);
            candidate = robot.evaluateJointAngles(q, true);
            result = robot.applyCandidate(candidate, q);
        end

        function result = trySetJointAnglesDegrees(robot, theta, phi)
            if ~isFiniteRealScalar(theta) || ~isFiniteRealScalar(phi)
                invalidJointAngles();
            end
            result = robot.trySetJointAngles( ...
                deg2rad(double([theta, phi])));
        end

        function result = trySetPointG(robot, pointG)
            pointG = validatePointG(pointG);
            target = struct('kind', "pointG", 'position', pointG);
            solutions = duallink5.kinematics.inverseKinematics( ...
                target, robot.Geometry);

            if isempty(solutions)
                result = robot.reject("IK_NO_SOLUTION", NaN(1, 2));
                return
            end

            bestCandidate = [];
            bestCost = Inf;
            for index = 1:numel(solutions)
                candidate = robot.evaluateJointAngles( ...
                    solutions(index).q, true);
                if ~candidate.accepted
                    continue
                end
                difference = candidate.q - robot.Q;
                wrappedDifference = atan2( ...
                    sin(difference), cos(difference));
                cost = norm(wrappedDifference);
                if cost < bestCost
                    bestCost = cost;
                    bestCandidate = candidate;
                end
            end

            if isempty(bestCandidate)
                result = robot.reject( ...
                    "IK_NO_VALID_ASSEMBLY", NaN(1, 2));
                return
            end
            result = robot.applyCandidate(bestCandidate, bestCandidate.q);
        end

        function result = trySetPointGMillimetres(robot, x, y)
            if ~isFiniteRealScalar(x) || ~isFiniteRealScalar(y)
                invalidPointG();
            end
            result = robot.trySetPointG(1e-3 * double([x; y]));
        end

        function handles = plot(robot, axesHandle, options)
            if nargin < 3
                options = struct();
            end
            handles = duallink5.viz.plotAssembly( ...
                robot.Assembly, axesHandle, options);
        end

        function result = singularity(robot, options)
            if nargin < 2
                options = struct();
            end
            result = duallink5.singularity.evaluate( ...
                robot.Q, robot.Geometry, options);
        end

        function samples = sampleSingularitySpace(robot, grid, options)
            if nargin < 3
                options = struct();
            end
            samples = duallink5.singularity.sampleSpace( ...
                grid, robot.Geometry, options);
        end

        function value = get.QDegrees(robot)
            value = rad2deg(robot.Q);
        end

        function value = get.PointGMillimetres(robot)
            value = 1e3 * robot.PointG;
        end
    end

    methods (Access = private)
        function candidate = evaluateJointAngles( ...
                robot, q, enforceTopology)
            candidate = emptyCandidate(q);
            ranges = [robot.Geometry.analysis.thetaRange; ...
                robot.Geometry.analysis.phiRange];
            if any(q < ranges(:, 1).' | q > ranges(:, 2).')
                candidate.statusCode = "JOINT_LIMIT_VIOLATION";
                return
            end

            jointInput = struct('lower', q, 'upper', q);
            options = struct( ...
                'mode', "ideal", ...
                'branchMode', "fixed", ...
                'branchId', robot.Geometry.assembly.defaultBranch);
            assembly = duallink5.kinematics.forwardAssembly( ...
                jointInput, robot.Geometry, options);
            candidate.assembly = assembly;
            if ~assembly.quality.valid
                candidate.statusCode = assembly.quality.statusCode;
                return
            end

            if enforceTopology
                topology = ...
                    duallink5.validation.compareAssemblyTopologyToReference( ...
                        assembly, robot.TopologyReference);
                if ~topology.compatible
                    candidate.statusCode = topology.statusCode;
                    return
                end
            end

            task = duallink5.kinematics.taskPose(assembly, ...
                struct('kind', "pointG", ...
                'includeOrientation', false));
            candidate.accepted = true;
            candidate.statusCode = "OK";
            candidate.pointG = task.position;
        end

        function result = applyCandidate(robot, candidate, requestedQ)
            if candidate.accepted
                robot.commit(candidate);
                result = robot.makeResult(true, "OK", requestedQ);
            else
                result = robot.reject( ...
                    candidate.statusCode, requestedQ);
            end
        end

        function commit(robot, candidate)
            robot.Q = candidate.q;
            robot.Assembly = candidate.assembly;
            robot.PointG = candidate.pointG;
            robot.IsValid = true;
            robot.StatusCode = "OK";
        end

        function result = reject(robot, statusCode, requestedQ)
            robot.IsValid = false;
            robot.StatusCode = statusCode;
            result = robot.makeResult(false, statusCode, requestedQ);
        end

        function result = makeResult( ...
                robot, accepted, statusCode, requestedQ)
            result.accepted = logical(accepted);
            result.statusCode = string(statusCode);
            result.requestedQ = requestedQ;
            result.q = robot.Q;
            result.pointG = robot.PointG;
            result.assembly = robot.Assembly;
        end
    end
end

function candidate = emptyCandidate(q)
candidate.accepted = false;
candidate.statusCode = "UNINITIALIZED";
candidate.q = q;
candidate.pointG = NaN(2, 1);
candidate.assembly = struct();
end

function q = validateJointAngles(q)
if ~isnumeric(q) || ~isreal(q) || ~isequal(size(q), [1, 2]) || ...
        any(~isfinite(q))
    invalidJointAngles();
end
q = double(q);
end

function pointG = validatePointG(pointG)
if ~isnumeric(pointG) || ~isreal(pointG) || numel(pointG) ~= 2 || ...
        any(~isfinite(pointG(:)))
    invalidPointG();
end
pointG = double(pointG(:));
end

function valid = isFiniteRealScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value);
end

function invalidJointAngles()
error('duallink5:facade:InvalidJointAngles', ...
    'Joint angles must be a finite real 1-by-2 vector in radians.');
end

function invalidPointG()
error('duallink5:facade:InvalidPointG', ...
    'Point G must be a finite real 2-vector in metres.');
end
