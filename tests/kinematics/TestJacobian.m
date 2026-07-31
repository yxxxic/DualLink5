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
        function analyticCenterJacobianMatchesCentralDifference(testCase)
            spec = struct('kind', "sharedCenter", ...
                'includeOrientation', true);
            result = duallink5.kinematics.taskJacobian( ...
                testCase.Q, testCase.Geometry, spec, struct());
            numeric = centralDifference( ...
                testCase.Q, testCase.Geometry, spec);

            testCase.verifyEqual( ...
                result.logical, numeric, 'AbsTol', 1e-6);
            testCase.verifyFalse(result.nearSingular);
        end

        function diagnosticInputReturnsPerSideJacobians(testCase)
            q.lower = testCase.Q;
            q.upper = testCase.Q + deg2rad([0.2, -0.1]);
            spec = struct('kind', "sharedCenter", ...
                'includeOrientation', true);
            result = duallink5.kinematics.taskJacobian( ...
                q, testCase.Geometry, spec, ...
                struct('mode', "diagnostic"));

            testCase.verifySize(result.lowerLocal, [3, 2]);
            testCase.verifySize(result.upperLocal, [3, 2]);
            testCase.verifyEmpty(result.logical);
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
    end
end

function J = centralDifference(q, geometry, spec)
step = 1e-7;
J = zeros(3, 2);
for column = 1:2
    delta = zeros(1, 2);
    delta(column) = step;
    plus = idealTask(q + delta, geometry, spec);
    minus = idealTask(q - delta, geometry, spec);
    difference = plus.vector - minus.vector;
    difference(3) = atan2(sin(difference(3)), ...
        cos(difference(3)));
    J(:, column) = difference / (2 * step);
end
end

function task = idealTask(q, geometry, spec)
input.lower = q;
input.upper = q;
assembly = duallink5.kinematics.forwardAssembly(input, geometry);
task = duallink5.kinematics.taskPose(assembly, spec);
end
