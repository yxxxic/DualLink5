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
                4.490009119731253e-4, 'AbsTol', 1e-12);
            testCase.verifyEqual(result.maxRectangle.area, ...
                1.263207373716140e-4, 'AbsTol', 1e-12);
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
            testCase.verifyEqual( ...
                result.singularity.maxCondition, Inf);
        end

        function sampledTopologyPreservesInteriorHole(testCase)
            [X, Y] = meshgrid(0:4, 0:4);
            samples.x = X;
            samples.y = Y;
            samples.validMask = true(5);
            samples.validMask(2:4, 2:4) = false;
            samples.reasonMap = repmat("OK", 5);
            samples.reasonMap(~samples.validMask) = ...
                "PARALLEL_SHARED_COLLISION";
            samples.conditionNumber = ones(5);
            samples.metadata = struct('units', ...
                struct('length', "m", 'angle', "rad"));

            result = duallink5.workspace.analyzeWorkspace( ...
                samples, struct('gridSize', [40, 40]));

            testCase.verifyFalse(result.insideMask(21, 21));
            testCase.verifyLessThan(result.area, 16);
            testCase.verifyGreaterThan(result.area, 0);
            testCase.verifyTrue(isfield(result, 'boundaryMesh'));
        end

        function partialCellsPreserveEveryThreeCornerTriangle(testCase)
            [X, Y] = meshgrid(0:1, 0:1);
            base.x = X;
            base.y = Y;
            base.validMask = true(2);
            base.reasonMap = repmat("OK", 2);
            base.conditionNumber = ones(2);
            base.metadata = struct('units', ...
                struct('length', "m", 'angle', "rad"));

            for invalidIndex = 1:4
                samples = base;
                samples.validMask(invalidIndex) = false;
                samples.reasonMap(invalidIndex) = "OUTSIDE_WORKSPACE";

                result = duallink5.workspace.analyzeWorkspace( ...
                    samples, struct('gridSize', [10, 10]));

                testCase.verifySize( ...
                    result.boundaryMesh.ConnectivityList, [1, 3]);
                testCase.verifyEqual(result.area, 0.5, 'AbsTol', 1e-12);
            end
        end

        function numericBinaryMaskMatchesLogicalMask(testCase)
            logicalSamples = squareSamples();
            numericSamples = logicalSamples;
            numericSamples.validMask = double(logicalSamples.validMask);

            logicalResult = duallink5.workspace.analyzeWorkspace( ...
                logicalSamples, struct('alpha', Inf, ...
                'gridSize', [20, 20]));
            numericResult = duallink5.workspace.analyzeWorkspace( ...
                numericSamples, struct('alpha', Inf, ...
                'gridSize', [20, 20]));

            testCase.verifyEqual( ...
                numericResult.area, logicalResult.area);
            testCase.verifyEqual(numericResult.maxRectangle.bounds, ...
                logicalResult.maxRectangle.bounds);
        end

        function numericOnesMaskDoesNotRepeatFirstSample(testCase)
            samples = squareSamples();
            samples.x = samples.x(1:2, :);
            samples.y = samples.y(1:2, :);
            samples.validMask = ones(2, 2);
            samples.reasonMap = samples.reasonMap(1:2, :);
            samples.conditionNumber = samples.conditionNumber(1:2, :);

            result = duallink5.workspace.analyzeWorkspace( ...
                samples, struct('alpha', Inf, ...
                'gridSize', [20, 20]));

            testCase.verifyEqual(result.area, 1, 'AbsTol', 1e-12);
        end

        function invalidNumericSampleMasksUseStableError(testCase)
            samples = squareSamples();
            malformedMasks = { ...
                [1, 1; 1, 2; 0, 0], ...
                [1, 1; 1, NaN; 0, 0], ...
                [1, 1; 1, Inf; 0, 0]};

            for index = 1:numel(malformedMasks)
                samples.validMask = malformedMasks{index};
                testCase.verifyError( ...
                    @() duallink5.workspace.analyzeWorkspace( ...
                    samples, struct()), ...
                    'duallink5:workspace:InvalidSamples');
            end
        end

        function malformedSampleShapesUseStableError(testCase)
            baseline = squareSamples();
            badMask = baseline;
            badMask.validMask = true(2, 2);
            badReason = baseline;
            badReason.reasonMap = strings(2, 2);
            badCondition = baseline;
            badCondition.conditionNumber = ones(2, 2);
            badY = baseline;
            badY.y = zeros(2, 2);
            missingMetadata = rmfield(baseline, 'metadata');
            malformed = {badMask, badReason, badCondition, ...
                badY, missingMetadata};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.workspace.analyzeWorkspace( ...
                    malformed{index}, struct()), ...
                    'duallink5:workspace:InvalidSamples');
            end
        end

        function nonfiniteValidCoordinatesUseStableError(testCase)
            samples = squareSamples();
            samples.x(1, 1) = NaN;

            testCase.verifyError( ...
                @() duallink5.workspace.analyzeWorkspace( ...
                samples, struct()), ...
                'duallink5:workspace:InvalidSamples');
        end

        function duplicateCoordinatesAreInsufficientSamples(testCase)
            samples = squareSamples();
            samples.x = [0, 1; 0, 1];
            samples.y = [0, 1; 0, 1];
            samples.validMask = true(2, 2);
            samples.reasonMap = repmat("OK", 2, 2);
            samples.conditionNumber = ones(2, 2);

            testCase.verifyError( ...
                @() duallink5.workspace.analyzeWorkspace( ...
                samples, struct()), ...
                'duallink5:workspace:InsufficientSamples');
        end

        function collinearCoordinatesHaveInsufficientSpan(testCase)
            samples = squareSamples();
            samples.x = [0, 1; 1, 2];
            samples.y = samples.x;
            samples.validMask = true(2, 2);
            samples.reasonMap = repmat("OK", 2, 2);
            samples.conditionNumber = ones(2, 2);

            testCase.verifyError( ...
                @() duallink5.workspace.analyzeWorkspace( ...
                samples, struct()), ...
                'duallink5:workspace:InsufficientSpan');
        end

        function largestRectangleRejectsInvalidNumericMasks(testCase)
            malformed = { ...
                [1, 0; NaN, 1], ...
                [1, 0; Inf, 1], ...
                [1, 0; 2, 1], ...
                [1, 0; -1, 1]};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.workspace.largestRectangleInMask( ...
                    malformed{index}, 1, 1), ...
                    'duallink5:workspace:InvalidRectangleInput');
            end
        end

        function samplerRejectsEmptyAndMalformedAngleGrids(testCase)
            geometry = duallink5.model.defaultGeometry();
            taskSpec = struct('kind', "pointG");
            validGrid = struct('theta', pi, 'phi', pi);
            emptyTheta = validGrid;
            emptyTheta.theta = zeros(1, 0);
            complexPhi = validGrid;
            complexPhi.phi = pi + 1i;
            textTheta = validGrid;
            textTheta.theta = "pi";
            missingPhi = rmfield(validGrid, 'phi');
            malformed = {emptyTheta, complexPhi, textTheta, ...
                missingPhi, repmat(validGrid, 1, 2)};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.workspace.sampleWorkspace( ...
                    malformed{index}, geometry, taskSpec, struct()), ...
                    'duallink5:workspace:InvalidAngleGrid');
            end
        end

        function samplerRejectsMalformedOptionsDeterministically(testCase)
            geometry = duallink5.model.defaultGeometry();
            grid = struct('theta', pi, 'phi', pi);
            taskSpec = struct('kind', "pointG");
            malformed = {[], 42, "ideal", repmat(struct(), 1, 2)};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.workspace.sampleWorkspace( ...
                    grid, geometry, taskSpec, malformed{index}), ...
                    'duallink5:workspace:InvalidWorkspaceOptions');
            end
        end

        function samplerValidatesTaskSpecOnUnreachableGrid(testCase)
            geometry = duallink5.model.defaultGeometry();
            grid = struct('theta', pi, 'phi', pi);
            malformed = { ...
                struct(), ...
                struct('kind', "unsupported"), ...
                struct('kind', "pointG", 'includeOrientation', 2), ...
                struct('kind', "sharedOffset"), ...
                struct('kind', "marker", 'offset', [NaN; 0]), ...
                struct('kind', "pointG", 'orientationOffset', Inf)};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.workspace.sampleWorkspace( ...
                    grid, geometry, malformed{index}, struct()), ...
                    'duallink5:kinematics:InvalidTaskSpec');
            end
        end

        function samplerNormalizesTaskSpecOnce(testCase)
            geometry = duallink5.model.defaultGeometry();
            grid.theta = deg2rad(85);
            grid.phi = deg2rad(52.93);
            taskSpec = struct( ...
                'kind', 'sharedOffset', ...
                'includeOrientation', 1, ...
                'offset', [0, 0], ...
                'orientationOffset', single(0.2));

            samples = duallink5.workspace.sampleWorkspace( ...
                grid, geometry, taskSpec, struct());

            testCase.verifyEqual( ...
                samples.metadata.taskSpec.kind, "sharedOffset");
            testCase.verifyTrue( ...
                islogical(samples.metadata.taskSpec.includeOrientation));
            testCase.verifyEqual( ...
                samples.metadata.taskSpec.offset, [0; 0]);
            testCase.verifyEqual( ...
                samples.metadata.taskSpec.orientationOffset, ...
                double(single(0.2)));
            testCase.verifyTrue(samples.validMask);
            testCase.verifyTrue(isfinite(samples.orientation));
        end

        function analysisNormalizesNumericClasses(testCase)
            samples = squareSamples();
            samples.x = single(samples.x);
            samples.y = single(samples.y);
            options = struct( ...
                'alpha', single(Inf), ...
                'gridSize', uint8([255, 2]));

            result = duallink5.workspace.analyzeWorkspace( ...
                samples, options);

            testCase.verifyClass(result.xEdges, 'double');
            testCase.verifyClass(result.yEdges, 'double');
            testCase.verifySize(result.insideMask, [2, 255]);
            testCase.verifyEqual(result.area, 1, 'AbsTol', 1e-12);
        end

        function samplerNormalizesSingleAngleGrid(testCase)
            geometry = duallink5.model.defaultGeometry();
            singleGrid.theta = single(deg2rad(85));
            singleGrid.phi = single(deg2rad(52.93));
            doubleGrid.theta = double(singleGrid.theta);
            doubleGrid.phi = double(singleGrid.phi);
            taskSpec = struct('kind', "pointG");

            singleSamples = duallink5.workspace.sampleWorkspace( ...
                singleGrid, geometry, taskSpec, struct());
            doubleSamples = duallink5.workspace.sampleWorkspace( ...
                doubleGrid, geometry, taskSpec, struct());

            testCase.verifyClass(singleSamples.thetaGrid, 'double');
            testCase.verifyClass(singleSamples.phiGrid, 'double');
            testCase.verifyEqual( ...
                singleSamples.validMask, doubleSamples.validMask);
            testCase.verifyEqual(singleSamples.x, doubleSamples.x);
            testCase.verifyEqual(singleSamples.y, doubleSamples.y);
        end

        function samplerRejectsSparseTopologyModes(testCase)
            geometry = duallink5.model.defaultGeometry();
            taskSpec = struct('kind', "pointG");
            sparseQ = {[13.5, 84], [166.5, 72.75]};

            for index = 1:numel(sparseQ)
                grid.theta = deg2rad(sparseQ{index}(1));
                grid.phi = deg2rad(sparseQ{index}(2));
                samples = duallink5.workspace.sampleWorkspace( ...
                    grid, geometry, taskSpec, struct());

                testCase.verifyFalse(samples.validMask);
                testCase.verifyEqual(samples.reasonMap, ...
                    "ASSEMBLY_TOPOLOGY_MISMATCH");
                testCase.verifyTrue(isnan(samples.x));
                testCase.verifyTrue(isnan(samples.y));
                testCase.verifyTrue(isnan(samples.orientation));
                testCase.verifyTrue(isnan(samples.conditionNumber));
            end
        end

        function samplerRetainsReferenceTopologyModes(testCase)
            geometry = duallink5.model.defaultGeometry();
            taskSpec = struct('kind', "pointG");
            compatibleQ = {[85, 30], [85, 31]};

            for index = 1:numel(compatibleQ)
                grid.theta = deg2rad(compatibleQ{index}(1));
                grid.phi = deg2rad(compatibleQ{index}(2));
                samples = duallink5.workspace.sampleWorkspace( ...
                    grid, geometry, taskSpec, struct());

                testCase.verifyTrue(samples.validMask);
                testCase.verifyEqual(samples.reasonMap, "OK");
                testCase.verifyTrue(isfinite(samples.x));
                testCase.verifyTrue(isfinite(samples.y));
                testCase.verifyTrue(isfinite(samples.conditionNumber));
            end
        end

        function topologyProfileNoneRetainsSparsePose(testCase)
            geometry = duallink5.model.defaultGeometry();
            grid.theta = deg2rad(13.5);
            grid.phi = deg2rad(84);
            options = struct( ...
                'topologyProfile', "none", ...
                'collisionProfile', "none");

            samples = duallink5.workspace.sampleWorkspace( ...
                grid, geometry, struct('kind', "pointG"), options);

            testCase.verifyTrue(samples.validMask);
            testCase.verifyEqual(samples.reasonMap, "OK");
            testCase.verifyTrue(isfinite(samples.x));
            testCase.verifyTrue(isfinite(samples.y));
            testCase.verifyTrue(isfinite(samples.conditionNumber));
        end

        function samplerRejectsMalformedTopologyProfiles(testCase)
            geometry = duallink5.model.defaultGeometry();
            grid = struct('theta', deg2rad(85), 'phi', deg2rad(30));
            taskSpec = struct('kind', "pointG");
            malformed = { ...
                "invalid", ...
                ["reference", "none"], ...
                struct(), ...
                string(missing)};

            for index = 1:numel(malformed)
                options = struct('topologyProfile', malformed{index});
                testCase.verifyError( ...
                    @() duallink5.workspace.sampleWorkspace( ...
                    grid, geometry, taskSpec, options), ...
                    'duallink5:workspace:InvalidWorkspaceOptions');
            end
        end

        function invalidFiniteTopologyReferenceUsesStableError(testCase)
            geometry = duallink5.model.defaultGeometry();
            geometry.analysis.referenceQ = [pi, pi];
            grid = struct('theta', deg2rad(85), 'phi', deg2rad(30));

            testCase.verifyError( ...
                @() duallink5.workspace.sampleWorkspace( ...
                grid, geometry, struct('kind', "pointG"), struct()), ...
                'duallink5:workspace:InvalidTopologyReference');
        end

        function samplerStoresTopologyMetadata(testCase)
            geometry = duallink5.model.defaultGeometry();
            grid = struct('theta', deg2rad(85), 'phi', deg2rad(30));

            samples = duallink5.workspace.sampleWorkspace( ...
                grid, geometry, struct('kind', "pointG"), struct());

            testCase.verifyEqual( ...
                samples.metadata.topologyProfile, "reference");
            testCase.verifyEqual( ...
                samples.metadata.referenceQ, geometry.analysis.referenceQ);
        end

        function rectangleNormalizesIntegerCellSizes(testCase)
            mask = logical([1 1 0; 1 1 1; 1 1 1]);

            result = duallink5.workspace.largestRectangleInMask( ...
                mask, uint8(200), uint8(3));

            testCase.verifyClass(result.area, 'double');
            testCase.verifyClass(result.width, 'double');
            testCase.verifyClass(result.height, 'double');
            testCase.verifyEqual(result.area, 3600);
            testCase.verifyEqual(result.width, 400);
            testCase.verifyEqual(result.height, 9);
        end
    end
end

function samples = squareSamples()
samples.x = [0, 1; 0, 1; NaN, NaN];
samples.y = [0, 0; 1, 1; NaN, NaN];
samples.validMask = logical([1, 1; 1, 1; 0, 0]);
samples.reasonMap = ["OK", "OK"; "OK", "OK"; ...
    "NEAR_SINGULAR", "NONFINITE_INPUT"];
samples.conditionNumber = [2, 3; 4, 5; Inf, NaN];
samples.metadata = struct('units', ...
    struct('length', "m", 'angle', "rad"));
end
