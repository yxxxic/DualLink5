classdef TestCouplingSpace < matlab.unittest.TestCase
    properties
        Geometry
        Options
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            testCase.Geometry = duallink5.model.defaultGeometry();
            testCase.Options = struct( ...
                'collisionProfile', "none", ...
                'topologyProfile', "none");
        end
    end

    methods (Test)
        function samplesBothSidesAndExactLine(testCase)
            grid.theta = deg2rad([84, 85, 86]);
            grid.phi = deg2rad([-15, -10, -5, 0, 5, 10, 15]);
            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, testCase.Options);

            expectedShape = [3, 7];
            testCase.verifySize(samples.thetaGrid, expectedShape);
            testCase.verifySize(samples.phiGrid, expectedShape);
            testCase.verifySize(samples.constraintMatrix, [2, 2, 3, 7]);
            testCase.verifySize(samples.actuationMatrix, [2, 2, 3, 7]);
            testCase.verifySize(samples.taskJacobian, [2, 2, 3, 7]);
            testCase.verifySize(samples.passiveDirection, [2, 3, 7]);
            zeroColumn = 4;
            testCase.verifyTrue(all( ...
                samples.exactCouplingSingular(:, zeroColumn)));
            testCase.verifyTrue(all( ...
                samples.nearCouplingSingular(:, zeroColumn)));
            testCase.verifyTrue(all( ...
                samples.engineeringWarning(:, 2:6), 'all'));
            testCase.verifyFalse(any( ...
                samples.engineeringWarning(:, [1, 7]), 'all'));
            testCase.verifyEqual(samples.couplingMargin(:, 3), ...
                samples.couplingMargin(:, 5), 'AbsTol', 1e-11);
            testCase.verifyTrue(all(isfinite( ...
                samples.x(:, zeroColumn))));
            testCase.verifyTrue(all(isfinite( ...
                samples.y(:, zeroColumn))));
            testCase.verifyTrue(all(isfinite( ...
                samples.constraintMatrix(:, :, :, zeroColumn)), 'all'));
            testCase.verifyTrue(all(isfinite( ...
                samples.passiveDirection(:, :, zeroColumn)), 'all'));
            testCase.verifyEqual( ...
                samples.conditionNumber(:, zeroColumn), Inf(3, 1));
        end

        function separatesTheoreticalAndMechanicalValidity(testCase)
            grid.theta = deg2rad(85);
            grid.phi = deg2rad([-15, 15]);
            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, testCase.Options);

            testCase.verifyTrue(samples.theoreticalReachableMask(1, 1));
            testCase.verifyFalse(samples.mechanicallyValidMask(1, 1));
            testCase.verifyEqual(samples.assemblyStatusMap(1, 1), ...
                "JOINT_LIMIT_VIOLATION");
            testCase.verifyEqual(samples.reasonMap(1, 1), ...
                "JOINT_LIMIT_VIOLATION");
            testCase.verifyTrue(isfinite(samples.couplingMargin(1, 1)));
            testCase.verifyTrue(samples.theoreticalReachableMask(1, 2));
            testCase.verifyTrue(samples.mechanicallyValidMask(1, 2));
        end

        function masksAndReasonsFollowPublicClassification(testCase)
            grid.theta = deg2rad([84, 85]);
            grid.phi = deg2rad([-15, 0, 15, 30]);
            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, testCase.Options);
            expectedSafe = samples.mechanicallyValidMask & ...
                ~samples.exactCouplingSingular & ...
                ~samples.nearCouplingSingular & ...
                ~samples.engineeringWarning;

            testCase.verifyEqual(samples.safeUsableMask, expectedSafe);
            testCase.verifyEqual(samples.reasonMap(samples.safeUsableMask), ...
                repmat("OK", nnz(samples.safeUsableMask), 1));
            testCase.verifyEqual( ...
                samples.reasonMap(samples.exactCouplingSingular), ...
                repmat("COUPLING_SINGULAR", ...
                nnz(samples.exactCouplingSingular), 1));
            testCase.verifyFalse(any(samples.mechanicallyValidMask & ...
                ~samples.theoreticalReachableMask, 'all'));
        end

        function customUnreachableSamplesDoNotTerminateGrid(testCase)
            geometry = testCase.Geometry;
            geometry.links.link3 = 10e-3;
            geometry.links.link4 = 50e-3;
            grid.theta = deg2rad(85);
            grid.phi = deg2rad([30, 90]);
            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, geometry, testCase.Options);

            testCase.verifyTrue(samples.theoreticalReachableMask(1, 1));
            testCase.verifyFalse(samples.theoreticalReachableMask(1, 2));
            testCase.verifyEqual(samples.reasonMap(1, 2), ...
                "UNREACHABLE_CLOSURE");
            testCase.verifyTrue(isfinite(samples.x(1, 1)));
            testCase.verifyTrue(isnan(samples.x(1, 2)));
            testCase.verifyTrue(isnan(samples.y(1, 2)));
            testCase.verifyTrue(isnan(samples.couplingMargin(1, 2)));
            testCase.verifyTrue(all(isnan( ...
                samples.constraintMatrix(:, :, 1, 2)), 'all'));
        end

        function acceptsSingleElementAxes(testCase)
            grid.theta = deg2rad(85);
            grid.phi = deg2rad(30);
            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, testCase.Options);

            testCase.verifySize(samples.thetaGrid, [1, 1]);
            testCase.verifySize(samples.constraintMatrix, [2, 2]);
            testCase.verifySize(samples.passiveDirection, [2, 1]);
            testCase.verifyEqual(samples.reasonMap, "OK");
        end

        function preservesNonuniformNdgridAxes(testCase)
            grid.theta = deg2rad([60, 61, 80, 120]);
            grid.phi = deg2rad([-30, -7, 0, 2, 30]);
            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, testCase.Options);
            [expectedTheta, expectedPhi] = ndgrid( ...
                double(grid.theta), double(grid.phi));

            testCase.verifyEqual(samples.thetaGrid, expectedTheta);
            testCase.verifyEqual(samples.phiGrid, expectedPhi);
            testCase.verifyEqual(samples.metadata.gridShape, [4, 5]);
        end

        function referenceIsPreparedOnceOutsideLoop(testCase)
            grid.theta = deg2rad([84, 85]);
            grid.phi = deg2rad([-5, 0, 5]);
            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, testCase.Options);

            testCase.verifyEqual(samples.metadata.referenceBuildCount, 1);
            testCase.verifyEqual(samples.metadata.referenceQ, ...
                testCase.Geometry.analysis.referenceQ, 'AbsTol', 0);
            testCase.verifyNotEqual( ...
                samples.metadata.referencePassiveOrientationSign, ...
                int8(0));
            testCase.verifyEqual(samples.metadata.units, ...
                testCase.Geometry.units);
            testCase.verifyEqual(samples.metadata.displayLengthUnit, "mm");
            testCase.verifyEqual(samples.metadata.displayAngleUnit, "deg");
        end

        function preparedSamplingMatchesPublicResultsFieldByField(testCase)
            grid.theta = deg2rad([84, 85]);
            grid.phi = deg2rad([0, 30]);
            options = struct( ...
                'collisionProfile', "centerline", ...
                'topologyProfile', "reference");

            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, options);

            verifyMatchesPointEvaluations(testCase, samples, ...
                testCase.Geometry, options);
        end

        function mediumGridCompletesWithinRegressionBudget(testCase)
            grid.theta = deg2rad(linspace(60, 120, 11));
            grid.phi = deg2rad(linspace(-30, 30, 11));
            options = struct( ...
                'collisionProfile', "centerline", ...
                'topologyProfile', "reference");

            started = tic;
            duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, options);
            elapsed = toc(started);

            testCase.verifyLessThan(elapsed, 30);
        end

        function metadataFingerprintBindsTheSampleGeometry(testCase)
            grid.theta = deg2rad(85);
            grid.phi = deg2rad([0, 30]);
            baseline = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, testCase.Options);
            customGeometry = testCase.Geometry;
            customGeometry.links.link1 = customGeometry.links.link1 + 1e-3;
            customGeometry = duallink5.model.validateGeometry(customGeometry);
            custom = duallink5.singularity.sampleCouplingSpace( ...
                grid, customGeometry, testCase.Options);

            testCase.verifyTrue(isfield( ...
                baseline.metadata, 'geometryFingerprint'));
            testCase.verifyEqual(baseline.metadata.geometryFingerprint, ...
                baseline.metadata.couplingReference.geometryFingerprint);
            testCase.verifyEqual(custom.metadata.geometryFingerprint, ...
                custom.metadata.couplingReference.geometryFingerprint);
            testCase.verifyNotEqual(custom.metadata.geometryFingerprint, ...
                baseline.metadata.geometryFingerprint);
        end

        function acceptsAndPropagatesCachedReferenceAndOptions(testCase)
            seedOptions = testCase.Options;
            seedOptions.referenceQ = deg2rad([86, 30]);
            seed = duallink5.singularity.evaluateCoupling( ...
                deg2rad([86, 30]), testCase.Geometry, seedOptions);
            options = seedOptions;
            options.couplingReference = ...
                seed.metadata.couplingReference;
            options.exactThreshold = 1e-7;
            options.nearMetricThreshold = 0.08;
            options.nearConditionThreshold = 50;
            grid.theta = deg2rad([85, 86]);
            grid.phi = deg2rad([0, 30]);

            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, options);

            testCase.verifyEqual(samples.metadata.referenceQ, ...
                options.referenceQ, 'AbsTol', 0);
            actualReference = samples.metadata.couplingReference;
            testCase.verifyEqual(actualReference.q, ...
                options.couplingReference.q, 'AbsTol', 0);
            testCase.verifyEqual(actualReference.passiveOrientationSign, ...
                options.couplingReference.passiveOrientationSign);
            testCase.verifyEqual(actualReference.topology, ...
                options.couplingReference.topology);
            testCase.verifyEqual(actualReference.geometryFingerprint, ...
                options.couplingReference.geometryFingerprint);
            testCase.verifyGreaterThan(strlength( ...
                actualReference.integrityToken), 0);
            testCase.verifyEqual(samples.metadata.exactThreshold, ...
                options.exactThreshold);
            testCase.verifyEqual(samples.metadata.nearMetricThreshold, ...
                options.nearMetricThreshold);
            testCase.verifyEqual(samples.metadata.nearConditionThreshold, ...
                options.nearConditionThreshold);
            testCase.verifyEqual(samples.metadata.collisionProfile, "none");
            testCase.verifyEqual(samples.metadata.topologyProfile, "none");
        end

        function legacyReferenceIsStrictlyValidatedOnce(testCase)
            seed = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), testCase.Geometry, testCase.Options);
            legacy = rmfield(seed.metadata.couplingReference, ...
                {'geometryFingerprint', 'integrityToken'});
            legacy.passiveOrientationSign = ...
                -legacy.passiveOrientationSign;
            options = testCase.Options;
            options.couplingReference = legacy;
            grid.theta = deg2rad([84, 85]);
            grid.phi = deg2rad([20, 30]);

            testCase.verifyError(@() ...
                duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, options), ...
                'duallink5:singularity:InvalidCouplingOptions');
        end

        function rowGridPreservesPerPointTensorShapes(testCase)
            grid.theta = deg2rad(85);
            grid.phi = deg2rad([15, 30, 45]);
            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, testCase.Options);

            verifyTensorSizes(testCase, samples, [1, 3]);
            verifyMatchesPointEvaluations(testCase, samples, ...
                testCase.Geometry, testCase.Options);
        end

        function columnGridPreservesPerPointTensorShapes(testCase)
            grid.theta = deg2rad([84, 85, 86]);
            grid.phi = deg2rad(30);
            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, testCase.Options);

            verifyTensorSizes(testCase, samples, [3, 1]);
            verifyMatchesPointEvaluations(testCase, samples, ...
                testCase.Geometry, testCase.Options);
        end

        function malformedGridsUseStableError(testCase)
            valid.theta = [0, 1];
            valid.phi = [0, 1];
            malformed = {[], struct.empty, repmat(valid, 1, 2), ...
                rmfield(valid, 'theta'), rmfield(valid, 'phi'), ...
                struct('theta', [], 'phi', 0), ...
                struct('theta', [1, 0], 'phi', [0, 1]), ...
                struct('theta', [0, 0], 'phi', [0, 1]), ...
                struct('theta', [0, NaN], 'phi', [0, 1]), ...
                struct('theta', [0, Inf], 'phi', [0, 1]), ...
                struct('theta', [0, 1i], 'phi', [0, 1]), ...
                struct('theta', [0, 1; 2, 3], 'phi', [0, 1]), ...
                struct('theta', "bad", 'phi', [0, 1])};

            for index = 1:numel(malformed)
                testCase.verifyError(@() ...
                    duallink5.singularity.sampleCouplingSpace( ...
                    malformed{index}, testCase.Geometry), ...
                    'duallink5:singularity:InvalidCouplingGrid');
            end
        end
    end
end

function verifyTensorSizes(testCase, samples, expectedGridShape)
testCase.verifySize(samples.thetaGrid, expectedGridShape);
testCase.verifySize(samples.phiGrid, expectedGridShape);
testCase.verifyEqual(size(samples.constraintMatrix, 1), 2);
testCase.verifyEqual(size(samples.constraintMatrix, 2), 2);
testCase.verifyEqual(size(samples.constraintMatrix, 3), ...
    expectedGridShape(1));
testCase.verifyEqual(size(samples.constraintMatrix, 4), ...
    expectedGridShape(2));
testCase.verifyEqual(size(samples.taskJacobian, 1), 2);
testCase.verifyEqual(size(samples.taskJacobian, 2), 2);
testCase.verifyEqual(size(samples.taskJacobian, 3), ...
    expectedGridShape(1));
testCase.verifyEqual(size(samples.taskJacobian, 4), ...
    expectedGridShape(2));
testCase.verifyEqual(size(samples.passiveDirection, 1), 2);
testCase.verifyEqual(size(samples.passiveDirection, 2), ...
    expectedGridShape(1));
testCase.verifyEqual(size(samples.passiveDirection, 3), ...
    expectedGridShape(2));
end

function verifyMatchesPointEvaluations( ...
        testCase, samples, geometry, options)
for row = 1:size(samples.thetaGrid, 1)
    for column = 1:size(samples.thetaGrid, 2)
        result = duallink5.singularity.evaluateCoupling( ...
            [samples.thetaGrid(row, column), ...
            samples.phiGrid(row, column)], geometry, options);
        testCase.verifyEqual( ...
            samples.constraintMatrix(:, :, row, column), ...
            result.coupling.constraintMatrix, 'AbsTol', 1e-12);
        testCase.verifyEqual( ...
            samples.actuationMatrix(:, :, row, column), ...
            result.coupling.actuationMatrix, 'AbsTol', 1e-12);
        testCase.verifyEqual( ...
            samples.taskJacobian(:, :, row, column), ...
            result.coupling.taskJacobian, 'AbsTol', 1e-12);
        testCase.verifyEqual( ...
            samples.passiveDirection(:, row, column), ...
            result.coupling.passiveDirection, 'AbsTol', 1e-12);
        testCase.verifyEqual(samples.couplingMargin(row, column), ...
            result.coupling.margin, 'AbsTol', 1e-12);
        testCase.verifyEqual(samples.sigmaMin(row, column), ...
            result.coupling.sigmaMin, 'AbsTol', 1e-12);
        testCase.verifyEqual(samples.conditionNumber(row, column), ...
            result.coupling.conditionNumber, 'AbsTol', 1e-12);
        testCase.verifyEqual( ...
            samples.theoreticalReachableMask(row, column), ...
            result.theoreticallyReachable);
        testCase.verifyEqual(samples.mechanicallyValidMask(row, column), ...
            result.mechanicallyValid);
        testCase.verifyEqual(samples.exactCouplingSingular(row, column), ...
            result.exactCouplingSingular);
        testCase.verifyEqual(samples.nearCouplingSingular(row, column), ...
            result.nearCouplingSingular);
        testCase.verifyEqual(samples.engineeringWarning(row, column), ...
            result.engineeringWarning);
        testCase.verifyEqual(samples.reasonMap(row, column), ...
            result.statusCode);
        testCase.verifyEqual(samples.assemblyStatusMap(row, column), ...
            result.assemblyStatusCode);
        testCase.verifyEqual( ...
            samples.passiveOrientationSign(row, column), ...
            double(result.kinematics.quality.passiveOrientationSign));
        if result.theoreticallyReachable
            testCase.verifyEqual(samples.x(row, column), ...
                result.kinematics.points.G(1), 'AbsTol', 1e-12);
            testCase.verifyEqual(samples.y(row, column), ...
                result.kinematics.points.G(2), 'AbsTol', 1e-12);
        else
            testCase.verifyTrue(isnan(samples.x(row, column)));
            testCase.verifyTrue(isnan(samples.y(row, column)));
        end
    end
end
end
