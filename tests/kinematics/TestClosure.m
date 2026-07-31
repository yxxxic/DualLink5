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
            for index = 1:numel(candidates)
                testCase.verifyEqual(norm(candidates(index).D - C), 1, ...
                    'AbsTol', 1e-12);
                testCase.verifyEqual(norm(candidates(index).D - E), 1, ...
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
            [candidates, statusCode] = ...
                duallink5.kinematics.solveClosure( ...
                    [0; 0], [2; 0], 1, 1, 1e-12);

            testCase.verifyNumElements(candidates, 1);
            testCase.verifyEqual(candidates.branchId, int8(0));
            testCase.verifyEqual(statusCode, "NEAR_SINGULAR");
        end
    end
end
