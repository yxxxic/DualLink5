classdef TestPassiveDoublet < matlab.unittest.TestCase
    properties
        Geometry
        NoCollision
    end

    methods (TestMethodSetup)
        function configure(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            testCase.Geometry = duallink5.model.defaultGeometry();
            testCase.NoCollision = struct('collisionProfile', "none");
        end
    end

    methods (Test)
        function referencePoseMatchesIdealAssembly(testCase)
            q = deg2rad([85, 30]);
            passive = duallink5.kinematics.forwardPassiveDoublet( ...
                q, testCase.Geometry, testCase.NoCollision);
            ideal = duallink5.kinematics.forwardAssembly( ...
                struct('lower', q, 'upper', q), testCase.Geometry, ...
                testCase.NoCollision);

            testCase.verifyTrue(passive.quality.valid);
            testCase.verifyTrue(passive.quality.theoreticallyReachable);
            testCase.verifyEqual(passive.quality.statusCode, "OK");
            testCase.verifyEqual(passive.quality.closureStatusCode, "OK");
            testCase.verifyEqual( ...
                passive.quality.continuityCost, 0, 'AbsTol', 0);
            for name = ["A", "B", "C", "D", "E"]
                field = char(name);
                testCase.verifyEqual(passive.upper.points.(field), ...
                    ideal.upper.points.(field), 'AbsTol', 1e-11);
            end
            testCase.verifyEqual(passive.points.G, ...
                ideal.upper.points.B, 'AbsTol', 1e-11);
            testCase.verifyEqual(passive.sharedLink.center, ...
                ideal.sharedLink.center, 'AbsTol', 1e-12);
            testCase.verifyLessThanOrEqual( ...
                passive.residuals.maximum, 1e-11);
        end

        function zeroPhiRetainsInternalTangency(testCase)
            q = deg2rad([85, 0]);
            result = duallink5.kinematics.forwardPassiveDoublet( ...
                q, testCase.Geometry, testCase.NoCollision);

            Aupper = result.upper.points.A;
            Cupper = result.upper.points.C;
            G = result.points.G;
            testCase.verifyTrue(result.quality.valid);
            testCase.verifyTrue(result.quality.theoreticallyReachable);
            testCase.verifyEqual(result.quality.statusCode, "OK");
            testCase.verifyEqual(result.quality.closureStatusCode, ...
                "INTERNAL_TANGENCY");
            testCase.verifyEqual(norm(Aupper - Cupper), 18e-3, ...
                'AbsTol', 1e-12);
            testCase.verifyTrue(all(isfinite(G)));
            testCase.verifyLessThanOrEqual(abs(cross2( ...
                G - Aupper, G - Cupper)), 1e-14);
            testCase.verifyEqual( ...
                result.quality.passiveBranchId, int8(0));
            testCase.verifyEqual( ...
                result.quality.passiveOrientationSign, int8(0));
        end

        function positiveAndNegativePhiHaveSymmetricCouplingGeometry( ...
                testCase)
            positive = duallink5.kinematics.forwardPassiveDoublet( ...
                deg2rad([85, 5]), testCase.Geometry, ...
                testCase.NoCollision);
            negative = duallink5.kinematics.forwardPassiveDoublet( ...
                deg2rad([85, -5]), testCase.Geometry, ...
                testCase.NoCollision);

            testCase.verifyTrue(positive.quality.theoreticallyReachable);
            testCase.verifyTrue(negative.quality.theoreticallyReachable);
            testCase.verifyEqual(passiveMargin(positive), ...
                passiveMargin(negative), 'AbsTol', 1e-12);
            testCase.verifyEqual(norm(positive.upper.points.A - ...
                positive.upper.points.C), ...
                norm(negative.upper.points.A - ...
                negative.upper.points.C), 'AbsTol', 1e-12);
            testCase.verifyEqual( ...
                positive.quality.passiveOrientationSign, ...
                -negative.quality.passiveOrientationSign);
        end

        function externalTangencyRetainsFiniteClosure(testCase)
            result = duallink5.kinematics.forwardPassiveDoublet( ...
                [0, pi], testCase.Geometry, testCase.NoCollision);

            testCase.verifyTrue(result.quality.valid);
            testCase.verifyTrue(result.quality.theoreticallyReachable);
            testCase.verifyEqual(result.quality.closureStatusCode, ...
                "EXTERNAL_TANGENCY");
            testCase.verifyEqual(norm(result.upper.points.A - ...
                result.upper.points.C), ...
                testCase.Geometry.links.link5 + ...
                testCase.Geometry.links.link2, 'AbsTol', 1e-12);
            testCase.verifyTrue(all(isfinite(result.points.G)));
            testCase.verifyEqual( ...
                result.quality.passiveBranchId, int8(0));
            testCase.verifyEqual( ...
                result.quality.passiveOrientationSign, int8(0));
        end

        function equalRadiusNearCoincidentCirclesReturnTypedStatus(testCase)
            geometry = testCase.Geometry;
            geometry.links.link2 = geometry.links.link5;

            result = duallink5.kinematics.forwardPassiveDoublet( ...
                [pi / 2, 1e-15], geometry, testCase.NoCollision);

            testCase.verifyFalse(result.quality.valid);
            testCase.verifyFalse(result.quality.theoreticallyReachable);
            testCase.verifyEqual(result.quality.statusCode, ...
                "COINCIDENT_CIRCLES");
            testCase.verifyEqual(result.quality.closureStatusCode, ...
                "COINCIDENT_CIRCLES");
            testCase.verifyEmpty(result.passiveCandidates);
            testCase.verifyTrue(isnan(result.quality.closureResidual));
            testCase.verifyTrue(all(isnan(result.points.G)));
            TestPassiveDoublet.verifyInvalidUpperSchema( ...
                testCase, result.upper, "COINCIDENT_CIRCLES");
        end

        function equalRadiusSeparatedCirclesReturnTwoCandidates(testCase)
            geometry = testCase.Geometry;
            geometry.links.link2 = geometry.links.link5;

            result = duallink5.kinematics.forwardPassiveDoublet( ...
                [pi / 2, 1e-6], geometry, testCase.NoCollision);

            testCase.verifyTrue(result.quality.valid);
            testCase.verifyEqual(result.quality.closureStatusCode, "OK");
            testCase.verifyNumElements(result.passiveCandidates, 2);
            testCase.verifyEqual(sort(double( ...
                [result.passiveCandidates.branchId])), [-1, 1]);
            testCase.verifyLessThanOrEqual( ...
                result.quality.closureResidual, ...
                geometry.tolerance.residual);
        end

        function excessiveCandidateResidualReturnsTypedFailure(testCase)
            geometry = testCase.Geometry;
            geometry.tolerance.residual = 1e-20;

            result = duallink5.kinematics.forwardPassiveDoublet( ...
                deg2rad([85, 30]), geometry, testCase.NoCollision);

            testCase.verifyFalse(result.quality.valid);
            testCase.verifyFalse(result.quality.theoreticallyReachable);
            testCase.verifyEqual(result.quality.statusCode, ...
                "NUMERICAL_CLOSURE_FAILURE");
            testCase.verifyEqual(result.quality.closureStatusCode, ...
                "NUMERICAL_CLOSURE_FAILURE");
            testCase.verifyTrue(isnan(result.quality.closureResidual));
            testCase.verifyTrue(all(isnan(result.points.G)));
            TestPassiveDoublet.verifyInvalidUpperSchema( ...
                testCase, result.upper, "NUMERICAL_CLOSURE_FAILURE");
        end

        function reportsAllParallelogramLengthResiduals(testCase)
            result = testCase.referenceResult();
            names = {'side12Length', 'side23Length', ...
                'side34Length', 'side41Length'};

            for index = 1:numel(names)
                name = names{index};
                testCase.verifyLessThanOrEqual( ...
                    abs(result.residuals.alpha.(name)), 1e-12);
                testCase.verifyLessThanOrEqual( ...
                    abs(result.residuals.beta.(name)), 1e-12);
            end
            testCase.verifyLessThanOrEqual( ...
                result.residuals.alpha.oppositeParallel, 1e-12);
            testCase.verifyLessThanOrEqual( ...
                result.residuals.beta.oppositeParallel, 1e-12);
        end

        function reportsConfirmedRigidLineAndSharedPointResiduals(testCase)
            result = testCase.referenceResult();
            rigidNames = {'alphaLower', 'alphaUpper', ...
                'betaLower', 'betaUpper'};
            sharedNames = {'upperDLowerE', 'upperELowerD', ...
                'upperGLowerB', 'selectedGUpperB'};

            for index = 1:numel(rigidNames)
                testCase.verifyLessThanOrEqual( ...
                    result.residuals.rigidLines.(rigidNames{index}), ...
                    1e-12);
            end
            for index = 1:numel(sharedNames)
                testCase.verifyLessThanOrEqual( ...
                    result.residuals.shared.(sharedNames{index}), ...
                    1e-12);
            end
        end

        function reportsUpperLinkAndIdealSymmetryResiduals(testCase)
            result = testCase.referenceResult();
            linkNames = {'link1', 'link2', 'link3', 'link4', 'link5'};
            symmetryNames = {'A', 'B', 'C', 'D', 'E', 'G'};

            for index = 1:numel(linkNames)
                value = result.residuals.upperLinks.(linkNames{index});
                testCase.verifyTrue(isfinite(value));
                testCase.verifyLessThanOrEqual(abs(value), 1e-11);
            end
            for index = 1:numel(symmetryNames)
                value = result.residuals.idealSymmetry.( ...
                    symmetryNames{index});
                testCase.verifyTrue(isfinite(value));
                testCase.verifyLessThanOrEqual(abs(value), 1e-11);
            end
        end

        function continuousModeFollowsPreviousPassiveCandidate(testCase)
            q = deg2rad([85, 30]);
            fixed = testCase.referenceResult();
            otherIndex = find([fixed.passiveCandidates.branchId] ~= ...
                fixed.quality.passiveBranchId, 1, 'first');
            previous = fixed;
            previous.points.G = fixed.passiveCandidates(otherIndex).G;
            options = struct( ...
                'collisionProfile', "none", ...
                'lowerBranchMode', "continuous", ...
                'previousPose', previous, ...
                'maxContinuityCost', 1e-12);

            result = duallink5.kinematics.forwardPassiveDoublet( ...
                q, testCase.Geometry, options);

            testCase.verifyTrue(result.quality.valid);
            testCase.verifyEqual(result.points.G, previous.points.G, ...
                'AbsTol', 1e-12);
            testCase.verifyNotEqual(result.quality.passiveBranchId, ...
                fixed.quality.passiveBranchId);
            testCase.verifyEqual(result.quality.continuityCost, 0, ...
                'AbsTol', 1e-14);
        end

        function enabledProfileRejectsCollidingPassiveCandidate(testCase)
            q = deg2rad([85, 30]);
            reference = testCase.referenceResult();
            otherIndex = find([reference.passiveCandidates.branchId] ~= ...
                reference.quality.passiveBranchId, 1, 'first');
            previous = reference;
            previous.points.G = ...
                reference.passiveCandidates(otherIndex).G;
            options = struct( ...
                'collisionProfile', "centerline", ...
                'lowerBranchMode', "continuous", ...
                'previousPose', previous, ...
                'maxContinuityCost', 1e-12);

            result = duallink5.kinematics.forwardPassiveDoublet( ...
                q, testCase.Geometry, options);

            testCase.verifyFalse(result.quality.valid);
            testCase.verifyEqual(result.quality.statusCode, ...
                "SELF_COLLISION");
            testCase.verifyFalse(result.upper.quality.valid);
            testCase.verifyFalse(result.upper.quality.collisionFree);
            testCase.verifyEqual(result.upper.quality.statusCode, ...
                "SELF_COLLISION");
        end

        function continuousModePreventsUnreportedBranchJump(testCase)
            q = deg2rad([85, 30]);
            previous = testCase.referenceResult();
            options = struct( ...
                'collisionProfile', "none", ...
                'lowerBranchMode', "continuous", ...
                'previousPose', previous, ...
                'maxContinuityCost', 1e-12);

            result = duallink5.kinematics.forwardPassiveDoublet( ...
                q + deg2rad([0.1, 0.1]), testCase.Geometry, options);

            testCase.verifyFalse(result.quality.valid);
            testCase.verifyTrue(result.quality.theoreticallyReachable);
            testCase.verifyEqual(result.quality.statusCode, ...
                "BRANCH_DISCONTINUITY");
            testCase.verifyGreaterThan(result.quality.continuityCost, ...
                options.maxContinuityCost);
        end

        function lowerBranchIdIsForwardedToLowerSolver(testCase)
            options = struct('collisionProfile', "none", ...
                'lowerBranchId', int8(1));

            result = duallink5.kinematics.forwardPassiveDoublet( ...
                deg2rad([85, 30]), testCase.Geometry, options);

            testCase.verifyEqual(result.lower.quality.branchId, int8(1));
        end

        function enabledProfileRejectsParallelSharedCollision(testCase)
            q = deg2rad([180, 68.25]);

            disabled = duallink5.kinematics.forwardPassiveDoublet( ...
                q, testCase.Geometry, testCase.NoCollision);
            enabled = duallink5.kinematics.forwardPassiveDoublet( ...
                q, testCase.Geometry, ...
                struct('collisionProfile', "centerline"));

            testCase.verifyTrue(disabled.quality.valid);
            testCase.verifyTrue(isnan( ...
                disabled.quality.parallelSharedClearance));
            testCase.verifyFalse(enabled.quality.valid);
            testCase.verifyEqual(enabled.quality.statusCode, ...
                "PARALLEL_SHARED_COLLISION");
            testCase.verifyLessThanOrEqual( ...
                enabled.quality.parallelSharedClearance, ...
                testCase.Geometry.collision.parallelSharedClearance);
        end

        function customUnreachableGeometryReturnsTypedStatus(testCase)
            geometry = testCase.Geometry;
            geometry.links.link3 = 1e-3;
            geometry.links.link4 = 1e-3;

            result = duallink5.kinematics.forwardPassiveDoublet( ...
                deg2rad([85, 30]), geometry, testCase.NoCollision);

            testCase.verifyFalse(result.quality.valid);
            testCase.verifyFalse(result.quality.theoreticallyReachable);
            testCase.verifyEqual(result.quality.statusCode, ...
                "UNREACHABLE_CLOSURE");
            testCase.verifyEqual(result.quality.closureStatusCode, ...
                "UNREACHABLE_CLOSURE");
            testCase.verifyTrue(isnan(result.quality.passiveBranchId));
            testCase.verifyTrue( ...
                isnan(result.quality.passiveOrientationSign));
            testCase.verifyTrue(isnan(result.quality.closureResidual));
            testCase.verifyTrue(isnan(result.quality.continuityCost));
            testCase.verifyTrue(isnan(result.residuals.maximum));
            testCase.verifyTrue(isstruct(result.lower));
            testCase.verifyTrue(isstruct(result.upper));
            TestPassiveDoublet.verifyInvalidUpperSchema( ...
                testCase, result.upper, "UNREACHABLE_CLOSURE");
            testCase.verifyTrue(isstruct(result.sharedLink));
            testCase.verifyTrue(isstruct(result.residuals));
        end

        function invalidInputsUseStableErrors(testCase)
            invalid = {[1; 2], [1, NaN], [1, Inf], ...
                [1, 2 + 1i], [1, 2, 3], "1,2", {1, 2}};

            for index = 1:numel(invalid)
                value = invalid{index};
                testCase.verifyError( ...
                    @()duallink5.kinematics.forwardPassiveDoublet( ...
                    value, testCase.Geometry), ...
                    'duallink5:kinematics:InvalidPassiveDoubletInput');
            end
        end

        function invalidOptionsUseStableErrors(testCase)
            q = deg2rad([85, 30]);
            invalid = { ...
                42, repmat(struct(), 1, 2), ...
                struct('branchMode', "bad"), ...
                struct('collisionProfile', "bad"), ...
                struct('lowerBranchMode', "bad"), ...
                struct('lowerBranchId', 0), ...
                struct('previousPose', struct()), ...
                struct('lowerBranchMode', "continuous"), ...
                struct('maxContinuityCost', 0), ...
                struct('maxContinuityCost', -1), ...
                struct('maxContinuityCost', NaN), ...
                struct('maxContinuityCost', [1, 2]), ...
                struct('maxContinuityCost', 1 + 1i)};

            for index = 1:numel(invalid)
                options = invalid{index};
                testCase.verifyError( ...
                    @()duallink5.kinematics.forwardPassiveDoublet( ...
                    q, testCase.Geometry, options), ...
                    'duallink5:kinematics:InvalidPassiveDoubletOptions');
            end
        end
    end

    methods (Access = private)
        function result = referenceResult(testCase)
            result = duallink5.kinematics.forwardPassiveDoublet( ...
                deg2rad([85, 30]), testCase.Geometry, ...
                testCase.NoCollision);
        end
    end

    methods (Static, Access = private)
        function verifyInvalidUpperSchema(testCase, upper, statusCode)
            testCase.verifyTrue(isstruct(upper.points));
            testCase.verifyTrue(isstruct(upper.sharedLink));
            testCase.verifyEqual(fieldnames(upper.sharedLink), ...
                {'start'; 'end'; 'center'; 'orientation'});
            testCase.verifyEqual(upper.sharedLink.start, NaN(2, 1));
            testCase.verifyEqual(upper.sharedLink.end, NaN(2, 1));
            testCase.verifyEqual(upper.sharedLink.center, NaN(2, 1));
            testCase.verifyTrue(isnan(upper.sharedLink.orientation));
            expected = {'valid'; 'branchId'; 'closureResidual'; ...
                'continuityCost'; 'collisionFree'; ...
                'clearanceModelApplied'; 'statusCode'};
            testCase.verifyEqual(fieldnames(upper.quality), expected);
            testCase.verifyFalse(upper.quality.valid);
            testCase.verifyTrue(isnan(upper.quality.branchId));
            testCase.verifyTrue(isnan(upper.quality.closureResidual));
            testCase.verifyTrue(isnan(upper.quality.continuityCost));
            testCase.verifyTrue(isnan(upper.quality.collisionFree));
            testCase.verifyFalse(upper.quality.clearanceModelApplied);
            testCase.verifyEqual(upper.quality.statusCode, statusCode);
        end
    end
end

function value = passiveMargin(result)
A = result.upper.points.A;
C = result.upper.points.C;
G = result.points.G;
uAG = (G - A) / norm(G - A);
uCG = (G - C) / norm(G - C);
value = abs(cross2(uAG, uCG));
end

function value = cross2(first, second)
value = first(1) * second(2) - first(2) * second(1);
end
