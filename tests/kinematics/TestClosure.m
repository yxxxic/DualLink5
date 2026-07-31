classdef TestClosure < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(~)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
        end
    end

    methods (Test)
        function returnsBothSignedBranches(testCase)
            C = [0; 0];
            E = [1; 0];

            [candidates, statusCode] = ...
                duallink5.kinematics.solveClosure(C, E, 1, 1, 1e-12);

            testCase.verifyEqual(statusCode, "OK");
            testCase.verifyNumElements(candidates, 2);
            testCase.verifyEqual(sort([candidates.branchId]), int8([-1, 1]));
            delta = E - C;
            for index = 1:numel(candidates)
                D = candidates(index).D;
                offset = D - C;
                expectedBranch = int8(sign( ...
                    delta(1) * offset(2) - delta(2) * offset(1)));
                testCase.verifyEqual( ...
                    candidates(index).branchId, expectedBranch);
                testCase.verifyEqual(norm(D - C), 1, ...
                    'AbsTol', 1e-12);
                testCase.verifyEqual(norm(D - E), 1, ...
                    'AbsTol', 1e-12);
                testCase.verifyLessThan( ...
                    candidates(index).closureResidual, 1e-12);
            end
        end

        function unreachableCirclesReturnStatus(testCase)
            [candidates, statusCode] = ...
                duallink5.kinematics.solveClosure( ...
                    [0; 0], [3; 0], 1, 1, 1e-12);

            testCase.verifyEmpty(candidates);
            testCase.verifyEqual(statusCode, "UNREACHABLE_CLOSURE");
        end

        function tangentCirclesReportSingularity(testCase)
            C = [0; 0];
            E = [2; 0];
            tolerance = 1e-12;

            [candidates, statusCode] = ...
                duallink5.kinematics.solveClosure( ...
                    C, E, 1, 1, tolerance);

            testCase.verifyNumElements(candidates, 1);
            testCase.verifyEqual(candidates.branchId, int8(0));
            testCase.verifyEqual(statusCode, "NEAR_SINGULAR");
            testCase.verifyEqual(candidates.D, [1; 0], ...
                'AbsTol', tolerance);
            testCase.verifyEqual(norm(candidates.D - C), 1, ...
                'AbsTol', tolerance);
            testCase.verifyEqual(norm(candidates.D - E), 1, ...
                'AbsTol', tolerance);
            testCase.verifyLessThanOrEqual( ...
                candidates.closureResidual, tolerance);
        end

        function nearInternalTangencyWithinSquaredToleranceIsAllowed(testCase)
            radiusCD = 0.08;
            radiusDE = 0.06;
            tolerance = 1e-8;
            distance = radiusCD - radiusDE - 0.1 * tolerance;
            C = [0; 0];
            E = [distance; 0];

            % The coarse internal-separation guard does not decide this case.
            along = (radiusCD^2 - radiusDE^2 + distance^2) / ...
                (2 * distance);
            heightSquared = radiusCD^2 - along^2;
            squaredTolerance = tolerance * ...
                max([radiusCD, radiusDE, distance, tolerance]);
            testCase.verifyGreaterThanOrEqual( ...
                distance, abs(radiusCD - radiusDE) - tolerance);
            testCase.verifyLessThan(heightSquared, 0);
            testCase.verifyGreaterThanOrEqual( ...
                heightSquared, -squaredTolerance);

            [candidates, statusCode] = ...
                duallink5.kinematics.solveClosure( ...
                    C, E, radiusCD, radiusDE, tolerance);

            testCase.verifyNumElements(candidates, 1);
            testCase.verifyEqual(candidates.branchId, int8(0));
            testCase.verifyEqual(statusCode, "NEAR_SINGULAR");
            testCase.verifyLessThanOrEqual( ...
                candidates.closureResidual, tolerance);
        end

        function nearInternalTangencyBeyondSquaredToleranceIsUnreachable( ...
                testCase)
            radiusCD = 0.08;
            radiusDE = 0.06;
            tolerance = 1e-8;
            distance = radiusCD - radiusDE - 0.5 * tolerance;
            C = [0; 0];
            E = [distance; 0];

            % This also bypasses the coarse guard; heightSquared rejects it.
            along = (radiusCD^2 - radiusDE^2 + distance^2) / ...
                (2 * distance);
            heightSquared = radiusCD^2 - along^2;
            squaredTolerance = tolerance * ...
                max([radiusCD, radiusDE, distance, tolerance]);
            testCase.verifyGreaterThanOrEqual( ...
                distance, abs(radiusCD - radiusDE) - tolerance);
            testCase.verifyLessThan(heightSquared, -squaredTolerance);

            [candidates, statusCode] = ...
                duallink5.kinematics.solveClosure( ...
                    C, E, radiusCD, radiusDE, tolerance);

            testCase.verifyEmpty(candidates);
            testCase.verifyEqual(statusCode, "UNREACHABLE_CLOSURE");
        end
    end
end
