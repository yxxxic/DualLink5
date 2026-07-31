classdef TestWorkspace < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(~)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
        end
    end

    methods (Test)
        function largestRectangleFindsKnownMask(testCase)
            mask = logical([1 1 0; 1 1 1; 1 1 1]);
            result = duallink5.workspace.largestRectangleInMask( ...
                mask, 2, 3);

            testCase.verifyEqual(result.areaCells, 6);
            testCase.verifyEqual(result.area, 36);
        end

        function emptyMaskHasZeroPhysicalExtent(testCase)
            result = duallink5.workspace.largestRectangleInMask( ...
                false(3, 4), 2, 3);

            testCase.verifyEqual(result.areaCells, 0);
            testCase.verifyEqual(result.area, 0);
            testCase.verifyEqual(result.width, 0);
            testCase.verifyEqual(result.height, 0);
        end

        function samplerReturnsReasonMapAndFiniteTasks(testCase)
            geometry = duallink5.model.defaultGeometry();
            grid.theta = linspace(deg2rad(70), deg2rad(100), 9);
            grid.phi = linspace(deg2rad(40), deg2rad(65), 9);
            taskSpec = struct( ...
                'kind', "pointG", 'includeOrientation', false);

            samples = duallink5.workspace.sampleWorkspace( ...
                grid, geometry, taskSpec, struct());

            testCase.verifySize(samples.validMask, [9, 9]);
            testCase.verifySize(samples.reasonMap, [9, 9]);
            testCase.verifyTrue(any(samples.validMask, 'all'));
            testCase.verifyTrue(all( ...
                isfinite(samples.x(samples.validMask))));
            testCase.verifyTrue(all(isfinite( ...
                samples.conditionNumber(samples.validMask))));
        end

        function fixedMechanismWorkspaceRegression(testCase)
            geometry = duallink5.model.defaultGeometry();
            grid.theta = linspace(deg2rad(80), deg2rad(90), 9);
            grid.phi = linspace(deg2rad(48), deg2rad(58), 9);
            taskSpec = struct( ...
                'kind', "pointG", 'includeOrientation', false);

            samples = duallink5.workspace.sampleWorkspace( ...
                grid, geometry, taskSpec, struct());
            result = duallink5.workspace.analyzeWorkspace( ...
                samples, struct('alpha', Inf, 'gridSize', [40, 40]));

            testCase.verifyEqual(nnz(samples.validMask), 81);
            testCase.verifyEqual(result.area, ...
                4.844403225084295e-4, 'AbsTol', 1e-12);
            testCase.verifyEqual(result.maxRectangle.area, ...
                1.515848848459369e-4, 'AbsTol', 1e-12);
        end

        function squareSamplesGiveAreaAndRectangleRegression(testCase)
            samples.x = [0, 1; 0, 1; NaN, NaN];
            samples.y = [0, 0; 1, 1; NaN, NaN];
            samples.validMask = logical([1, 1; 1, 1; 0, 0]);
            samples.reasonMap = ["OK", "OK"; "OK", "OK"; ...
                "NEAR_SINGULAR", "NONFINITE_INPUT"];
            samples.conditionNumber = [2, 3; 4, 5; Inf, NaN];
            samples.metadata = struct('units', ...
                struct('length', "m", 'angle', "rad"));

            result = duallink5.workspace.analyzeWorkspace( ...
                samples, struct('alpha', Inf, 'gridSize', [20, 20]));

            testCase.verifyEqual(result.area, 1, 'AbsTol', 1e-12);
            testCase.verifyEqual( ...
                result.maxRectangle.area, 1, 'AbsTol', 1e-12);
            testCase.verifyEqual(result.maxRectangle.bounds, ...
                [0, 1, 0, 1], 'AbsTol', 1e-12);
            testCase.verifyEqual(result.nearSingularCount, 1);
        end
    end
end
