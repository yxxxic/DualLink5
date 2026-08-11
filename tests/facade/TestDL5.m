classdef TestDL5 < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
        end
    end

    methods (Test)
        function startsAtConfiguredReferencePose(testCase)
            robot = duallink5.DL5();

            testCase.verifyTrue(robot.IsValid);
            testCase.verifyEqual(robot.StatusCode, "OK");
            testCase.verifyEqual(robot.QDegrees, [85, 30], ...
                'AbsTol', 1e-12);
            testCase.verifyEqual(robot.PointGMillimetres, ...
                1e3 * robot.Assembly.lower.points.G, ...
                'AbsTol', 1e-10);
        end

        function validJointRequestCommitsAtomically(testCase)
            robot = duallink5.DL5();

            result = robot.trySetJointAnglesDegrees(85, 31);

            testCase.verifyTrue(result.accepted);
            testCase.verifyTrue(robot.IsValid);
            testCase.verifyEqual(robot.StatusCode, "OK");
            testCase.verifyEqual(robot.QDegrees, [85, 31], ...
                'AbsTol', 1e-12);
            testCase.verifyEqual(result.q, robot.Q, 'AbsTol', 1e-12);
            testCase.verifyEqual(result.pointG, robot.PointG, ...
                'AbsTol', 1e-12);
        end

        function topologyMismatchPreservesLastValidState(testCase)
            robot = duallink5.DL5();
            previousQ = robot.Q;
            previousPointG = robot.PointG;
            previousAssembly = robot.Assembly;

            result = robot.trySetJointAnglesDegrees(13.5, 84);

            testCase.verifyFalse(result.accepted);
            testCase.verifyFalse(robot.IsValid);
            testCase.verifyEqual(robot.StatusCode, ...
                "ASSEMBLY_TOPOLOGY_MISMATCH");
            testCase.verifyEqual(robot.Q, previousQ);
            testCase.verifyEqual(robot.PointG, previousPointG);
            testCase.verifyEqual(robot.Assembly, previousAssembly);
        end

        function pointGRequestUsesClosestValidInverseSolution(testCase)
            robot = duallink5.DL5();
            angleResult = robot.trySetJointAnglesDegrees(86, 31);
            testCase.assertTrue(angleResult.accepted);
            targetMillimetres = robot.PointGMillimetres;
            resetResult = robot.trySetJointAnglesDegrees(85, 30);
            testCase.assertTrue(resetResult.accepted);

            result = robot.trySetPointGMillimetres( ...
                targetMillimetres(1), targetMillimetres(2));

            testCase.verifyTrue(result.accepted);
            testCase.verifyEqual(robot.PointGMillimetres, ...
                targetMillimetres, 'AbsTol', 1e-6);
            testCase.verifyEqual(robot.QDegrees, [86, 31], ...
                'AbsTol', 1e-6);
        end

        function unreachablePointPreservesLastValidState(testCase)
            robot = duallink5.DL5();
            previousQ = robot.Q;
            previousPointG = robot.PointG;

            result = robot.trySetPointG([10; 10]);

            testCase.verifyFalse(result.accepted);
            testCase.verifyFalse(robot.IsValid);
            testCase.verifyEqual(robot.StatusCode, "IK_NO_SOLUTION");
            testCase.verifyEqual(robot.Q, previousQ);
            testCase.verifyEqual(robot.PointG, previousPointG);
        end

        function rejectsMalformedRequests(testCase)
            robot = duallink5.DL5();

            testCase.verifyError( ...
                @()robot.trySetJointAngles([1, NaN]), ...
                'duallink5:facade:InvalidJointAngles');
            testCase.verifyError( ...
                @()robot.trySetJointAngles([1; 2]), ...
                'duallink5:facade:InvalidJointAngles');
            testCase.verifyError( ...
                @()robot.trySetPointG([1; 2; 3]), ...
                'duallink5:facade:InvalidPointG');
        end

        function singularityAnalysisDoesNotMutateCommittedState(testCase)
            robot = duallink5.DL5();
            previousQ = robot.Q;
            previousAssembly = robot.Assembly;
            previousPointG = robot.PointG;
            previousValid = robot.IsValid;
            previousStatus = robot.StatusCode;

            result = robot.singularity(struct());

            testCase.verifyEqual(result.mode, "ideal");
            testCase.verifyEqual(result.logical.classification, "REGULAR");
            testCase.verifyEqual(robot.Q, previousQ);
            testCase.verifyEqual(robot.Assembly, previousAssembly);
            testCase.verifyEqual(robot.PointG, previousPointG);
            testCase.verifyEqual(robot.IsValid, previousValid);
            testCase.verifyEqual(robot.StatusCode, previousStatus);
        end

        function singularitySamplingDoesNotMutateCommittedState(testCase)
            robot = duallink5.DL5();
            previousQ = robot.Q;
            previousAssembly = robot.Assembly;
            grid.theta = deg2rad([84, 85, 86]);
            grid.phi = deg2rad([29, 30, 31]);

            samples = robot.sampleSingularitySpace(grid, struct());

            testCase.verifySize(samples.thetaGrid, [3, 3]);
            testCase.verifyEqual(robot.Q, previousQ);
            testCase.verifyEqual(robot.Assembly, previousAssembly);
        end
    end
end
