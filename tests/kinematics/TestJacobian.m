classdef TestJacobian < matlab.unittest.TestCase
    properties
        Geometry
        Q
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            testCase.Geometry = duallink5.model.defaultGeometry();
            testCase.Q = deg2rad([85, 52.93]);
        end
    end

    methods (Test)
        function analyticTaskJacobiansMatchCentralDifferences(testCase)
            offset = [11e-3; -7e-3];
            specs = { ...
                struct('kind', "sharedCenter", ...
                    'includeOrientation', false), ...
                struct('kind', "sharedCenter", ...
                    'includeOrientation', true), ...
                struct('kind', "pointG", ...
                    'includeOrientation', false), ...
                struct('kind', "pointG", ...
                    'includeOrientation', true), ...
                struct('kind', "sharedOffset", 'offset', offset, ...
                    'includeOrientation', false), ...
                struct('kind', "sharedOffset", 'offset', offset, ...
                    'includeOrientation', true), ...
                struct('kind', "marker", 'offset', offset, ...
                    'orientationOffset', 0.23, ...
                    'includeOrientation', false), ...
                struct('kind', "marker", 'offset', offset, ...
                    'orientationOffset', 0.23, ...
                    'includeOrientation', true)};

            for index = 1:numel(specs)
                spec = specs{index};
                result = duallink5.kinematics.taskJacobian( ...
                    testCase.Q, testCase.Geometry, spec, struct());
                numeric = centralDifference( ...
                    testCase.Q, testCase.Geometry, spec);

                testCase.verifyEqual( ...
                    result.logical, numeric, 'AbsTol', 1e-6);
                testCase.verifyFalse(result.nearSingular);
                rows = 2 + double(spec.includeOrientation);
                expectedScale = ones(1, rows);
                if spec.includeOrientation
                    expectedScale(end) = testCase.Geometry.links.link4;
                end
                testCase.verifyEqual( ...
                    result.taskRowScale, expectedScale);
                verifyRegularConditionMetadata(testCase, result, [1, 1]);
            end
        end

        function diagnosticInputReturnsPerSideJacobians(testCase)
            q.lower = testCase.Q + deg2rad([-0.15, 0.08]);
            q.upper = testCase.Q + deg2rad([0.2, -0.1]);
            spec = struct('kind', "marker", ...
                'offset', [9e-3; 4e-3], ...
                'orientationOffset', -0.17, ...
                'includeOrientation', true);
            result = duallink5.kinematics.taskJacobian( ...
                q, testCase.Geometry, spec, ...
                struct('mode', "diagnostic"));
            lowerNumeric = localCentralDifference( ...
                q.lower, testCase.Geometry, spec);
            upperNumeric = localCentralDifference( ...
                q.upper, testCase.Geometry, spec);

            testCase.verifySize(result.lowerLocal, [3, 2]);
            testCase.verifySize(result.upperLocal, [3, 2]);
            testCase.verifyEqual( ...
                result.lowerLocal, lowerNumeric, 'AbsTol', 1e-6);
            testCase.verifyEqual( ...
                result.upperLocal, upperNumeric, 'AbsTol', 1e-6);
            testCase.verifyEmpty(result.logical);
            testCase.verifyEqual(result.taskRowScale, ...
                [1, 1, testCase.Geometry.links.link4]);
            verifyRegularConditionMetadata(testCase, result, [1, 2]);
            testCase.verifyEqual(result.failingSide, "");
            testCase.verifyEqual(result.lowerStatusCode, "OK");
            testCase.verifyEqual(result.upperStatusCode, "OK");
        end

        function positionOnlySingularResultHasCorrectShape(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                testCase.Q, testCase.Geometry, ...
                struct('collisionProfile', "none"));
            pose.points.C = [0; 0];
            pose.points.E = [80e-3; 0];
            pose.points.D = [40e-3; 0];
            spec = struct('kind', "sharedCenter", ...
                'includeOrientation', false);

            result = duallink5.kinematics.fiveBarJacobian( ...
                testCase.Q, pose, testCase.Geometry, spec);

            testCase.verifySize(result.matrix, [2, 2]);
            testCase.verifyTrue(all(isnan(result.matrix), 'all'));
            testCase.verifyTrue(result.nearSingular);
            testCase.verifyEqual(result.statusCode, "NEAR_SINGULAR");
        end

        function invalidAssemblyStatusPropagates(testCase)
            spec = struct('kind', "sharedCenter", ...
                'includeOrientation', false);
            result = duallink5.kinematics.taskJacobian( ...
                [NaN, 0.5], testCase.Geometry, spec, struct());

            testCase.verifyEqual(result.statusCode, "NONFINITE_INPUT");
            testCase.verifySize(result.logical, [2, 2]);
            testCase.verifyTrue(all(isnan(result.logical), 'all'));
        end

        function fiveBarRejectsInvalidJointVectors(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                testCase.Q, testCase.Geometry);
            spec = struct('kind', "sharedCenter");
            malformed = {[testCase.Q, 0], ...
                testCase.Q + [1i, 0], "not-a-joint-vector"};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.kinematics.fiveBarJacobian( ...
                        malformed{index}, pose, ...
                        testCase.Geometry, spec), ...
                    'duallink5:kinematics:InvalidJointVector');
            end
        end

        function fiveBarRejectsNonfiniteJointVectors(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                testCase.Q, testCase.Geometry);
            spec = struct('kind', "sharedCenter");
            malformed = {[NaN, testCase.Q(2)], ...
                [Inf, testCase.Q(2)]};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.kinematics.fiveBarJacobian( ...
                        malformed{index}, pose, ...
                        testCase.Geometry, spec), ...
                    'duallink5:kinematics:InvalidJointVector');
            end
        end

        function fiveBarRejectsMalformedPoses(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                testCase.Q, testCase.Geometry);
            missingPoints = rmfield(pose, 'points');
            missingSharedLink = rmfield(pose, 'sharedLink');
            nonfinitePoint = pose;
            nonfinitePoint.points.C = [NaN; 0];
            badOrientation = pose;
            badOrientation.sharedLink.orientation = [0, 1];
            malformed = {missingPoints, missingSharedLink, ...
                nonfinitePoint, badOrientation, repmat(pose, 1, 2)};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.kinematics.fiveBarJacobian( ...
                        testCase.Q, malformed{index}, ...
                        testCase.Geometry, ...
                        struct('kind', "sharedCenter")), ...
                    'duallink5:validation:InvalidPose');
            end
        end

        function fiveBarRejectsMalformedTaskSpecsBeforeSingularity(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                testCase.Q, testCase.Geometry);
            malformed = {struct(), ...
                struct('kind', "unsupported"), ...
                struct('kind', "sharedCenter", ...
                    'includeOrientation', 2), ...
                struct('kind', "marker"), ...
                struct('kind', "marker", 'offset', [1; 2; 3]), ...
                struct('kind', "sharedOffset", ...
                    'offset', [1 + 1i; 0])};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.kinematics.fiveBarJacobian( ...
                        testCase.Q, pose, ...
                        testCase.Geometry, malformed{index}), ...
                    'duallink5:kinematics:InvalidTaskSpec');
            end

            pose.points.C = [0; 0];
            pose.points.E = [80e-3; 0];
            pose.points.D = [40e-3; 0];
            testCase.verifyError( ...
                @() duallink5.kinematics.fiveBarJacobian( ...
                    testCase.Q, pose, testCase.Geometry, ...
                    struct('kind', "unsupported")), ...
                'duallink5:kinematics:InvalidTaskSpec');
        end

        function taskJacobianRejectsInvalidInputKinds(testCase)
            spec = struct('kind', "sharedCenter");
            diagnosticArray = repmat(struct( ...
                'lower', testCase.Q, 'upper', testCase.Q), 1, 2);
            malformed = {true, "not-a-joint-vector", ...
                [testCase.Q, 0], testCase.Q + [1i, 0], ...
                diagnosticArray};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.kinematics.taskJacobian( ...
                        malformed{index}, testCase.Geometry, ...
                        spec, struct()), ...
                    'duallink5:kinematics:InvalidJointVector');
            end

            testCase.verifyError( ...
                @() duallink5.kinematics.taskJacobian( ...
                    struct('lower', testCase.Q), ...
                    testCase.Geometry, spec, struct()), ...
                'duallink5:kinematics:InvalidAssemblyInput');
        end

        function taskJacobianRejectsMalformedOptions(testCase)
            spec = struct('kind', "sharedCenter");
            malformed = {[], 42, "ideal", repmat(struct(), 1, 2)};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.kinematics.taskJacobian( ...
                        testCase.Q, testCase.Geometry, ...
                        spec, malformed{index}), ...
                    'duallink5:kinematics:InvalidJacobianOptions');
            end
        end

        function taskJacobianValidatesSpecBeforeInvalidResult(testCase)
            malformed = {struct(), ...
                struct('kind', "sharedCenter", ...
                    'includeOrientation', 2), ...
                struct('kind', "marker", 'offset', [1; NaN])};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.kinematics.taskJacobian( ...
                        [NaN, 0.5], testCase.Geometry, ...
                        malformed{index}, struct()), ...
                    'duallink5:kinematics:InvalidTaskSpec');
            end
        end

        function tinySharedLinkIsReportedNearSingular(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                testCase.Q, testCase.Geometry);
            halfTolerance = testCase.Geometry.tolerance.length / 2;
            pose.points.C = [-halfTolerance; 0];
            pose.points.D = [0; 0];
            pose.points.E = [0; -halfTolerance];
            pose.sharedLink.orientation = pi / 2;

            result = duallink5.kinematics.fiveBarJacobian( ...
                testCase.Q, pose, testCase.Geometry, ...
                struct('kind', "sharedCenter"));

            testCase.verifyTrue(isfinite( ...
                result.closureConditionNumber));
            testCase.verifyLessThan( ...
                result.closureConditionNumber, ...
                testCase.Geometry.tolerance.singularityCondition);
            testCase.verifyTrue(all(isnan(result.matrix), 'all'));
            testCase.verifyEqual(result.taskConditionNumber, Inf);
            testCase.verifyEqual(result.conditionNumber, Inf);
            testCase.verifyTrue(result.nearSingular);
            testCase.verifyEqual(result.statusCode, "NEAR_SINGULAR");
        end

        function nonfiniteSharedLinkNormIsReportedNearSingular(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                testCase.Q, testCase.Geometry);
            pose.points.C = [realmax; -1];
            pose.points.D = [realmax; 0];
            pose.points.E = [-realmax; 0];
            pose.sharedLink.orientation = 0;

            result = duallink5.kinematics.fiveBarJacobian( ...
                testCase.Q, pose, testCase.Geometry, ...
                struct('kind', "sharedCenter"));

            testCase.verifyTrue(all(isnan(result.matrix), 'all'));
            testCase.verifyEqual(result.closureConditionNumber, Inf);
            testCase.verifyTrue(result.nearSingular);
            testCase.verifyEqual(result.statusCode, "NEAR_SINGULAR");
        end

        function invalidDiagnosticPreservesSideAndVectorMetadata(testCase)
            q.lower = testCase.Q;
            q.upper = [NaN, 0.5];
            spec = struct('kind', "sharedCenter", ...
                'includeOrientation', true);

            result = duallink5.kinematics.taskJacobian( ...
                q, testCase.Geometry, spec, ...
                struct('mode', "diagnostic"));

            testCase.verifyEmpty(result.logical);
            testCase.verifySize(result.lowerLocal, [3, 2]);
            testCase.verifySize(result.upperLocal, [3, 2]);
            testCase.verifyTrue(all(isnan(result.lowerLocal), 'all'));
            testCase.verifyTrue(all(isnan(result.upperLocal), 'all'));
            conditionFields = {'closureConditionNumber', ...
                'taskConditionNumber', 'conditionNumber'};
            for index = 1:numel(conditionFields)
                value = result.(conditionFields{index});
                testCase.verifySize(value, [1, 2]);
                testCase.verifyEqual(value, [Inf, Inf]);
            end
            testCase.verifyEqual(result.failingSide, "upper");
            testCase.verifyEqual(result.lowerStatusCode, "OK");
            testCase.verifyEqual( ...
                result.upperStatusCode, "NONFINITE_INPUT");
            testCase.verifyEqual( ...
                result.assemblyStatusCode, "NONFINITE_INPUT");
            testCase.verifyEqual(result.statusCode, "NONFINITE_INPUT");
        end
    end
end

function J = centralDifference(q, geometry, spec)
evaluator = @(angles) idealTask(angles, geometry, spec);
J = evaluateCentralDifference(q, evaluator, spec.includeOrientation);
end

function J = localCentralDifference(q, geometry, spec)
evaluator = @(angles) localTask(angles, geometry, spec);
J = evaluateCentralDifference(q, evaluator, spec.includeOrientation);
end

function J = evaluateCentralDifference(q, evaluator, includeOrientation)
step = 1e-7;
J = zeros(2 + double(includeOrientation), 2);
for column = 1:2
    delta = zeros(1, 2);
    delta(column) = step;
    plus = evaluator(q + delta);
    minus = evaluator(q - delta);
    difference = plus.vector - minus.vector;
    if includeOrientation
        difference(end) = atan2(sin(difference(end)), ...
            cos(difference(end)));
    end
    J(:, column) = difference / (2 * step);
end
end

function task = idealTask(q, geometry, spec)
input.lower = q;
input.upper = q;
assembly = duallink5.kinematics.forwardAssembly(input, geometry);
task = duallink5.kinematics.taskPose(assembly, spec);
end

function task = localTask(q, geometry, spec)
pose = duallink5.kinematics.forwardFiveBar(q, geometry);
assembly.lower = pose;
assembly.sharedLink = pose.sharedLink;
assembly.quality.valid = true;
assembly.metadata.mode = "ideal";
assembly.metadata.units = geometry.units;
task = duallink5.kinematics.taskPose(assembly, spec);
end

function verifyRegularConditionMetadata(testCase, result, expectedSize)
fields = {'closureConditionNumber', ...
    'taskConditionNumber', 'conditionNumber'};
for index = 1:numel(fields)
    value = result.(fields{index});
    testCase.verifySize(value, expectedSize);
    testCase.verifyTrue(all(isfinite(value), 'all'));
end
end
