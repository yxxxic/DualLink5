classdef TestInverseKinematics < matlab.unittest.TestCase
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
        function sharedCenterRoundTripsAllValidatedSolutions(testCase)
            g = testCase.Geometry;
            options = struct('collisionProfile', "centerline");
            sourcePose = duallink5.kinematics.forwardFiveBar( ...
                testCase.Q, g, options);
            target = struct( ...
                'kind', "sharedCenter", ...
                'position', sourcePose.sharedLink.center);

            solutions = duallink5.kinematics.inverseKinematics( ...
                target, g, options);

            testCase.verifyNumElements(solutions, 2);
            q = vertcat(solutions.q);
            [~, order] = sort(q(:, 2));
            solutions = solutions(order);
            q = vertcat(solutions.q);
            testCase.verifyEqual( ...
                [solutions.branchId], int8([-1, -1]));
            testCase.verifyEqual(rad2deg(q(:, 2)).', ...
                [52.93, 129.922883026159], 'AbsTol', 1e-9);
            wrappedDifference = atan2( ...
                sin(q - testCase.Q), cos(q - testCase.Q));
            testCase.verifyLessThan( ...
                min(vecnorm(wrappedDifference, 2, 2)), 1e-7);
            testCase.verifyTrue(all([solutions.valid]));
            testCase.verifyTrue(all([solutions.collisionFree]));
            testCase.verifyLessThan( ...
                max([solutions.closureResidual]), 1e-9);

            for index = 1:numel(solutions)
                forwardOptions = options;
                forwardOptions.branchMode = "fixed";
                forwardOptions.branchId = solutions(index).branchId;
                pose = duallink5.kinematics.forwardFiveBar( ...
                    solutions(index).q, g, forwardOptions);
                testCase.verifyEqual(pose.sharedLink.center, ...
                    target.position, 'AbsTol', 1e-8);
            end
            testCase.verifyLessThan( ...
                max([solutions.forwardResidual]), 1e-8);
        end

        function pointGRoundTripFindsValidatedSolution(testCase)
            g = testCase.Geometry;
            sourcePose = duallink5.kinematics.forwardFiveBar( ...
                testCase.Q, g);
            target = struct( ...
                'kind', "pointG", ...
                'position', sourcePose.points.G);

            solutions = duallink5.kinematics.inverseKinematics( ...
                target, g, struct());

            testCase.verifyNotEmpty(solutions);
            testCase.verifyLessThan( ...
                min([solutions.forwardResidual]), 1e-8);
        end

        function unreachableSharedCenterReturnsEmpty(testCase)
            target = struct( ...
                'kind', "sharedCenter", ...
                'position', [1; 1]);

            solutions = duallink5.kinematics.inverseKinematics( ...
                target, testCase.Geometry, struct());

            testCase.verifyEmpty(solutions);
        end
    end
end
