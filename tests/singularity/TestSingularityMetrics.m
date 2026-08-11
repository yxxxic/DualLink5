classdef TestSingularityMetrics < matlab.unittest.TestCase
    properties
        Geometry
        RegularOptions
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            testCase.Geometry = duallink5.model.defaultGeometry();
            testCase.RegularOptions = struct( ...
                'collisionProfile', "none", ...
                'topologyProfile', "none");
        end
    end

    methods (Test)
        function referenceConfigurationMatchesIndependentValues(testCase)
            result = duallink5.singularity.evaluate( ...
                deg2rad([85, 30]), testCase.Geometry, ...
                testCase.RegularOptions);
            metrics = result.logical;

            testCase.verifyEqual(metrics.classification, "REGULAR");
            testCase.verifyEqual(metrics.closureStatusCode, "OK");
            testCase.verifyEqual( ...
                metrics.conditionNumber.constraint, ...
                2.72294646944709, 'AbsTol', 1e-11);
            testCase.verifyEqual( ...
                metrics.conditionNumber.task, ...
                2.62563451612049, 'AbsTol', 1e-11);
            testCase.verifyEqual(metrics.margins.typeITheta, ...
                0.996379406563618, 'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.margins.typeIPhi, ...
                0.869963303515486, 'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.margins.typeII, ...
                0.647208200758206, 'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.orientationSensitivity, ...
                1.6337986967138, 'AbsTol', 1e-11);
        end

        function signedIndicatorsMatchGeometricSineDefinitions(testCase)
            q = deg2rad([85, 30]);
            result = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, testCase.RegularOptions);
            metrics = result.logical;
            L = testCase.Geometry.links;
            B = [L.link5; 0];
            E = L.link1 * [cos(q(1)); sin(q(1))];
            C = [L.link5 - L.link2 * cos(q(2)); ...
                L.link2 * sin(q(2))];
            D = metrics.pointG - E + B;
            u = D - C;
            v = D - E;
            eTheta = L.link1 * [-sin(q(1)); cos(q(1))];
            cPhi = L.link2 * [sin(q(2)); cos(q(2))];
            expected = [dot(v, eTheta) / (L.link4 * L.link1), ...
                dot(u, cPhi) / (L.link3 * L.link2), ...
                (u(1) * v(2) - u(2) * v(1)) / ...
                (L.link3 * L.link4)];

            actual = [metrics.signedIndicator.typeITheta, ...
                metrics.signedIndicator.typeIPhi, ...
                metrics.signedIndicator.typeII];
            margins = [metrics.margins.typeITheta, ...
                metrics.margins.typeIPhi, metrics.margins.typeII];
            testCase.verifyEqual(actual, expected, 'AbsTol', 1e-12);
            testCase.verifyEqual(margins, abs(expected), 'AbsTol', 1e-12);
        end

        function classifiesBothTypeISubtypes(testCase)
            thetaCase = duallink5.singularity.evaluate( ...
                deg2rad([45.206365032903925, 79]), ...
                testCase.Geometry, testCase.RegularOptions);
            phiCase = duallink5.singularity.evaluate( ...
                deg2rad([56.98067735693949, 1]), ...
                testCase.Geometry, testCase.RegularOptions);

            testCase.verifyTrue(thetaCase.logical.exact.typeITheta);
            testCase.verifyFalse(thetaCase.logical.exact.typeIPhi);
            testCase.verifyEqual( ...
                thetaCase.logical.classification, "TYPE_I");
            testCase.verifyTrue(phiCase.logical.exact.typeIPhi);
            testCase.verifyFalse(phiCase.logical.exact.typeITheta);
            testCase.verifyEqual( ...
                phiCase.logical.classification, "TYPE_I");
        end

        function retainsOuterAndInnerTangencyPointG(testCase)
            outerQ = [3.1273346915827149, deg2rad(69)];
            innerQ = [0.83109759906031577, deg2rad(56.5)];
            outer = duallink5.singularity.evaluate( ...
                outerQ, testCase.Geometry, testCase.RegularOptions);
            inner = duallink5.singularity.evaluate( ...
                innerQ, testCase.Geometry, testCase.RegularOptions);

            for metrics = {outer.logical, inner.logical}
                value = metrics{1};
                testCase.verifyTrue(value.reachable);
                testCase.verifyTrue(value.exact.typeII);
                testCase.verifyEqual(value.classification, "TYPE_II");
                testCase.verifyEqual( ...
                    value.closureStatusCode, "NEAR_SINGULAR");
                testCase.verifyTrue(all(isfinite(value.pointG)));
                testCase.verifyTrue( ...
                    all(isnan(value.matrices.taskJacobian), 'all'));
                testCase.verifyEqual( ...
                    value.conditionNumber.task, Inf);
            end
        end

        function nearTangencyRetainsResolvedBranch(testCase)
            q = deg2rad([85, 30]);
            geometry = testCase.Geometry;
            L = geometry.links;
            E = L.link1 * [cos(q(1)); sin(q(1))];
            C = [L.link5 - L.link2 * cos(q(2)); ...
                L.link2 * sin(q(2))];
            distance = norm(E - C);
            geometry.links.link3 = 1e-5;
            geometry.links.link4 = distance + 1e-14 - ...
                geometry.links.link3;
            geometry = duallink5.model.validateGeometry(geometry);
            L = geometry.links;
            outerResidual = L.link3 + L.link4 - distance;
            tangencyTolerance = 32 * eps(max( ...
                [L.link3, L.link4, distance]));
            along = (L.link3^2 - L.link4^2 + distance^2) / ...
                (2 * distance);
            heightSquared = L.link3^2 - along^2;
            height = sqrt(heightSquared);
            normalizedMargin = height * distance / ...
                (L.link3 * L.link4);

            testCase.verifyGreaterThan(outerResidual, tangencyTolerance);
            testCase.verifyGreaterThan(heightSquared, 0);
            testCase.verifyLessThan(height, geometry.tolerance.length);
            testCase.verifyGreaterThan(normalizedMargin, 1e-8);

            result = duallink5.singularity.evaluate( ...
                q, geometry, testCase.RegularOptions);
            metrics = result.logical;
            testCase.verifyTrue(metrics.reachable);
            testCase.verifyEqual( ...
                metrics.branchId, geometry.assembly.defaultBranch);
            testCase.verifyFalse(metrics.exact.typeII);
            testCase.verifyTrue( ...
                all(isfinite(metrics.matrices.taskJacobian), 'all'));
            testCase.verifyTrue(all(isfinite(metrics.dpsi_dq)));
            testCase.verifyEqual(metrics.classification, "NEAR_TYPE_II");
        end

        function unreachableClosureRetainsStableStatus(testCase)
            result = duallink5.singularity.evaluate( ...
                deg2rad([180, 90]), testCase.Geometry, ...
                testCase.RegularOptions);

            testCase.verifyFalse(result.logical.reachable);
            testCase.verifyEqual(result.logical.closureStatusCode, ...
                "UNREACHABLE_CLOSURE");
            testCase.verifyEqual(result.logical.classification, ...
                "UNREACHABLE_CLOSURE");
            testCase.verifyTrue(all(isnan(result.logical.pointG)));
            testCase.verifyTrue(isnan(result.logical.closureDistance));
        end

        function customGeometryProducesTypeIII(testCase)
            geometry = testCase.Geometry;
            geometry.links.link3 = 30e-3;
            geometry.links.link4 = 32e-3;
            geometry = duallink5.model.validateGeometry(geometry);
            result = duallink5.singularity.evaluate( ...
                [0, 0], geometry, testCase.RegularOptions);

            testCase.verifyTrue(result.logical.exact.typeITheta);
            testCase.verifyTrue(result.logical.exact.typeIPhi);
            testCase.verifyTrue(result.logical.exact.typeII);
            testCase.verifyTrue(result.logical.exact.typeIII);
            testCase.verifyEqual( ...
                result.logical.classification, "TYPE_III");
        end

        function analyticTaskJacobianMatchesCentralDifference(testCase)
            q = deg2rad([85, 30]);
            result = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, testCase.RegularOptions);
            numeric = zeros(2);
            step = 1e-7;
            for column = 1:2
                delta = zeros(1, 2);
                delta(column) = step;
                plus = evaluatePointG(q + delta, testCase.Geometry);
                minus = evaluatePointG(q - delta, testCase.Geometry);
                numeric(:, column) = (plus - minus) / (2 * step);
            end

            testCase.verifyEqual( ...
                result.logical.matrices.taskJacobian, ...
                numeric, 'AbsTol', 1e-6);
        end

        function thresholdBoundariesAreDeterministic(testCase)
            q = deg2rad([85, 30]);
            base = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, testCase.RegularOptions);
            boundary = base.logical.margins.typeII;
            common = struct('collisionProfile', "none", ...
                'topologyProfile', "none", ...
                'exactThreshold', 1e-8, ...
                'nearConditionThreshold', 1e12);
            atBoundary = common;
            atBoundary.nearMetricThreshold = boundary;
            inside = common;
            inside.nearMetricThreshold = boundary + 1e-12;
            outside = common;
            outside.nearMetricThreshold = boundary - 1e-12;

            equalResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, atBoundary);
            insideResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, inside);
            outsideResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, outside);
            testCase.verifyFalse(equalResult.logical.near.typeII);
            testCase.verifyTrue(insideResult.logical.near.typeII);
            testCase.verifyFalse(outsideResult.logical.near.typeII);

            exactAt = common;
            exactAt.exactThreshold = boundary;
            exactAt.nearMetricThreshold = boundary + 0.1;
            exactBelow = exactAt;
            exactBelow.exactThreshold = boundary - 1e-12;
            exactAtResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, exactAt);
            exactBelowResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, exactBelow);
            testCase.verifyTrue(exactAtResult.logical.exact.typeII);
            testCase.verifyFalse(exactBelowResult.logical.exact.typeII);

            conditionBoundary = ...
                base.logical.conditionNumber.constraint;
            conditionAt = struct('collisionProfile', "none", ...
                'topologyProfile', "none", ...
                'exactThreshold', 1e-8, ...
                'nearMetricThreshold', 0.01, ...
                'nearConditionThreshold', conditionBoundary);
            conditionInside = conditionAt;
            conditionInside.nearConditionThreshold = ...
                conditionBoundary - 1e-12;
            conditionAtResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, conditionAt);
            conditionInsideResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, conditionInside);
            testCase.verifyFalse( ...
                conditionAtResult.logical.near.typeII);
            testCase.verifyTrue( ...
                conditionInsideResult.logical.near.typeII);
        end
    end
end

function pointG = evaluatePointG(q, geometry)
input = struct('lower', q, 'upper', q);
assembly = duallink5.kinematics.forwardAssembly( ...
    input, geometry, struct('mode', "ideal", ...
    'branchMode', "fixed", ...
    'branchId', geometry.assembly.defaultBranch, ...
    'collisionProfile', "none"));
task = duallink5.kinematics.taskPose(assembly, ...
    struct('kind', "pointG", 'includeOrientation', false));
pointG = task.position;
end
