classdef TestCouplingMetrics < matlab.unittest.TestCase
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
        function referenceMetricsMatchIndependentGeometry(testCase)
            q = deg2rad([85, 30]);
            result = duallink5.singularity.evaluateCoupling( ...
                q, testCase.Geometry, testCase.Options);
            A = result.kinematics.upper.points.A;
            C = result.kinematics.upper.points.C;
            G = result.kinematics.points.G;
            uAG = (G - A) / norm(G - A);
            uCG = (G - C) / norm(G - C);
            expectedMatrix = [uAG.'; uCG.'];
            expectedMargin = abs(cross2(uAG, uCG));

            testCase.verifyEqual(result.coupling.constraintMatrix, ...
                expectedMatrix, 'AbsTol', 1e-12);
            testCase.verifyEqual(result.coupling.margin, ...
                expectedMargin, 'AbsTol', 1e-12);
            testCase.verifyFalse(result.exactCouplingSingular);
            testCase.verifyFalse(result.engineeringWarning);
            testCase.verifyTrue(result.theoreticallyReachable);
            testCase.verifyTrue(result.mechanicallyValid);
            testCase.verifyEqual(result.statusCode, "OK");
        end

        function zeroPhiHasPassiveSelfMotion(testCase)
            result = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 0]), testCase.Geometry, testCase.Options);
            linkDirection = result.kinematics.points.G - ...
                result.kinematics.upper.points.A;
            linkDirection = linkDirection / norm(linkDirection);

            testCase.verifyTrue(result.theoreticallyReachable);
            testCase.verifyTrue(result.exactCouplingSingular);
            testCase.verifyTrue(result.nearCouplingSingular);
            testCase.verifyTrue(all(isfinite( ...
                result.kinematics.points.G)));
            testCase.verifyEqual(rank( ...
                result.coupling.constraintMatrix, 1e-12), 1);
            testCase.verifyEqual(result.coupling.sigmaMin, 0, ...
                'AbsTol', 1e-12);
            testCase.verifyEqual(result.coupling.conditionNumber, Inf);
            testCase.verifyLessThanOrEqual(abs(dot( ...
                linkDirection, result.coupling.passiveDirection)), 1e-12);
            testCase.verifyEqual(norm( ...
                result.coupling.passiveDirection), 1, 'AbsTol', 1e-12);
            testCase.verifyTrue(all(isnan( ...
                result.coupling.taskJacobian), 'all'));
            testCase.verifyEqual(result.statusCode, "COUPLING_SINGULAR");
        end

        function positiveAndNegativeFiveDegreesHaveSymmetricMargin(testCase)
            positive = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 5]), testCase.Geometry, testCase.Options);
            negative = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, -5]), testCase.Geometry, testCase.Options);

            testCase.verifyEqual(positive.coupling.margin, ...
                negative.coupling.margin, 'AbsTol', 1e-12);
            testCase.verifyTrue(positive.engineeringWarning);
            testCase.verifyTrue(negative.engineeringWarning);
        end

        function warningBandIncludesBothBoundaries(testCase)
            values = [-10, -5, 0, 5, 10];
            for phi = values
                result = duallink5.singularity.evaluateCoupling( ...
                    deg2rad([85, phi]), testCase.Geometry, ...
                    testCase.Options);
                testCase.verifyTrue(result.engineeringWarning);
            end
            for phi = [-10.01, 10.01]
                outside = duallink5.singularity.evaluateCoupling( ...
                    deg2rad([85, phi]), testCase.Geometry, ...
                    testCase.Options);
                testCase.verifyFalse(outside.engineeringWarning);
            end
        end

        function analyticVelocityMatricesMatchCentralDifference(testCase)
            q = deg2rad([85, 30]);
            result = duallink5.singularity.evaluateCoupling( ...
                q, testCase.Geometry, testCase.Options);
            numericCenters = zeros(4, 2);
            numericG = zeros(2);
            step = 1e-7;
            for column = 1:2
                delta = zeros(1, 2);
                delta(column) = step;
                plus = passivePoints(q + delta, testCase.Geometry);
                minus = passivePoints(q - delta, testCase.Geometry);
                numericCenters(:, column) = ...
                    ([plus.A; plus.C] - [minus.A; minus.C]) / ...
                    (2 * step);
                numericG(:, column) = ...
                    (plus.G - minus.G) / (2 * step);
            end
            A = result.kinematics.upper.points.A;
            C = result.kinematics.upper.points.C;
            G = result.kinematics.points.G;
            uAG = (G - A) / testCase.Geometry.links.link5;
            uCG = (G - C) / testCase.Geometry.links.link2;
            expectedB = [ ...
                uAG.' * numericCenters(1:2, :); ...
                uCG.' * numericCenters(3:4, :)];

            testCase.verifyEqual(result.coupling.actuationMatrix, ...
                expectedB, 'AbsTol', 1e-7);
            testCase.verifyEqual(result.coupling.taskJacobian, ...
                numericG, 'AbsTol', 1e-7);
        end

        function singularValuesAreOrderedAndDirectionIsUnit(testCase)
            result = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), testCase.Geometry, testCase.Options);
            [~, singularMatrix, V] = svd( ...
                result.coupling.constraintMatrix);

            testCase.verifyGreaterThanOrEqual( ...
                result.coupling.singularValues(1), ...
                result.coupling.singularValues(2));
            testCase.verifyEqual(result.coupling.singularValues, ...
                diag(singularMatrix), 'AbsTol', 1e-12);
            testCase.verifyEqual(result.coupling.sigmaMin, ...
                result.coupling.singularValues(end), 'AbsTol', 0);
            testCase.verifyEqual(abs(dot( ...
                result.coupling.passiveDirection, V(:, end))), ...
                1, 'AbsTol', 1e-12);
            testCase.verifyEqual(norm( ...
                result.coupling.passiveDirection), 1, 'AbsTol', 1e-12);
        end

        function negativePhiRemainsTheoreticalButViolatesJointLimit(testCase)
            result = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, -30]), testCase.Geometry, testCase.Options);
            assessment = result.metadata.assemblyAssessment;

            testCase.verifyTrue(result.theoreticallyReachable);
            testCase.verifyFalse(result.mechanicallyValid);
            testCase.verifyFalse(assessment.jointLimitValid);
            testCase.verifyFalse( ...
                assessment.passiveOrientationCompatible);
            testCase.verifyTrue( ...
                assessment.assemblyTopologyCompatible);
            testCase.verifyTrue(assessment.collisionFree);
            testCase.verifyEqual(result.assemblyStatusCode, ...
                "JOINT_LIMIT_VIOLATION");
            testCase.verifyTrue(isfinite(result.coupling.margin));
            testCase.verifyEqual(result.statusCode, ...
                "JOINT_LIMIT_VIOLATION");
        end

        function simultaneousMechanicalFailuresAreAllAssessed(testCase)
            geometry = testCase.Geometry;
            geometry.collision.parallelSharedClearance = 1;
            geometry = duallink5.model.validateGeometry(geometry);
            result = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, -30]), geometry, struct( ...
                'collisionProfile', "centerline", ...
                'topologyProfile', "reference"));
            assessment = result.metadata.assemblyAssessment;

            testCase.verifyTrue(result.theoreticallyReachable);
            testCase.verifyFalse(assessment.jointLimitValid);
            testCase.verifyFalse( ...
                assessment.passiveOrientationCompatible);
            testCase.verifyFalse( ...
                assessment.assemblyTopologyCompatible);
            testCase.verifyFalse(assessment.collisionFree);
            testCase.verifyFalse(result.mechanicallyValid);
            testCase.verifyEqual(result.assemblyStatusCode, ...
                "JOINT_LIMIT_VIOLATION");
        end

        function lowerBranchModeAcceptsOnlyFixed(testCase)
            fixed = testCase.Options;
            fixed.lowerBranchMode = "fixed";
            result = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), testCase.Geometry, fixed);

            testCase.verifyEqual( ...
                result.metadata.lowerBranchMode, "fixed");
            for value = {"continuous", "bad", ["fixed", "fixed"], 1}
                options = testCase.Options;
                options.lowerBranchMode = value{1};
                testCase.verifyError(@() ...
                    duallink5.singularity.evaluateCoupling( ...
                    deg2rad([85, 30]), testCase.Geometry, options), ...
                    'duallink5:singularity:InvalidCouplingOptions');
            end
        end

        function oppositePassiveBranchIsTopologyMismatch(testCase)
            negativeReference = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, -30]), testCase.Geometry, struct( ...
                'collisionProfile', "none", ...
                'topologyProfile', "none", ...
                'referenceQ', deg2rad([85, -30])));
            options = testCase.Options;
            options.referenceQ = deg2rad([85, -30]);
            options.couplingReference = ...
                negativeReference.metadata.couplingReference;
            candidate = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), testCase.Geometry, options);

            testCase.verifyTrue(candidate.theoreticallyReachable);
            testCase.verifyFalse(candidate.mechanicallyValid);
            testCase.verifyEqual(candidate.assemblyStatusCode, ...
                "PASSIVE_ASSEMBLY_TOPOLOGY_MISMATCH");
            testCase.verifyTrue(isfinite(candidate.coupling.margin));
        end

        function collisionAndExactSingularityRemainIndependent(testCase)
            geometry = testCase.Geometry;
            geometry.collision.parallelSharedClearance = 1;
            geometry = duallink5.model.validateGeometry(geometry);
            result = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 0]), geometry, struct( ...
                'collisionProfile', "centerline", ...
                'topologyProfile', "none"));

            testCase.verifyTrue(result.exactCouplingSingular);
            testCase.verifyFalse(result.mechanicallyValid);
            testCase.verifyEqual(result.assemblyStatusCode, ...
                "SELF_COLLISION");
            testCase.verifyEqual(result.statusCode, "COUPLING_SINGULAR");
        end

        function thresholdEqualitiesAreDeterministic(testCase)
            q = deg2rad([85, 30]);
            base = duallink5.singularity.evaluateCoupling( ...
                q, testCase.Geometry, testCase.Options);
            margin = base.coupling.margin;
            condition = base.coupling.conditionNumber;
            common = testCase.Options;
            common.exactThreshold = margin / 2;
            common.nearConditionThreshold = condition + 1;
            common.nearMetricThreshold = margin;
            equalMargin = duallink5.singularity.evaluateCoupling( ...
                q, testCase.Geometry, common);
            common.nearMetricThreshold = margin + 1e-12;
            insideMargin = duallink5.singularity.evaluateCoupling( ...
                q, testCase.Geometry, common);

            testCase.verifyFalse(equalMargin.nearCouplingSingular);
            testCase.verifyTrue(insideMargin.nearCouplingSingular);

            common.nearMetricThreshold = margin / 2 + 1e-6;
            common.nearConditionThreshold = condition;
            equalCondition = duallink5.singularity.evaluateCoupling( ...
                q, testCase.Geometry, common);
            common.nearConditionThreshold = condition - 1e-12;
            insideCondition = duallink5.singularity.evaluateCoupling( ...
                q, testCase.Geometry, common);
            testCase.verifyFalse(equalCondition.nearCouplingSingular);
            testCase.verifyTrue(insideCondition.nearCouplingSingular);

            common.nearConditionThreshold = condition + 1;
            common.nearMetricThreshold = min(0.99, margin + 0.1);
            common.exactThreshold = margin;
            equalExact = duallink5.singularity.evaluateCoupling( ...
                q, testCase.Geometry, common);
            common.exactThreshold = margin - 1e-12;
            belowExact = duallink5.singularity.evaluateCoupling( ...
                q, testCase.Geometry, common);
            testCase.verifyTrue(equalExact.exactCouplingSingular);
            testCase.verifyEqual( ...
                equalExact.coupling.conditionNumber, ...
                base.coupling.conditionNumber, 'AbsTol', 1e-12);
            testCase.verifyFalse(belowExact.exactCouplingSingular);
        end

        function unreachableAndLowerSingularUseNaNContracts(testCase)
            unreachable = duallink5.singularity.evaluateCoupling( ...
                deg2rad([180, 90]), testCase.Geometry, testCase.Options);
            testCase.verifyFalse(unreachable.theoreticallyReachable);
            testCase.verifyFalse(unreachable.mechanicallyValid);
            testCase.verifyEqual(unreachable.statusCode, ...
                "UNREACHABLE_CLOSURE");
            testCase.verifyTrue(all(isnan( ...
                unreachable.coupling.constraintMatrix), 'all'));

            lowerSingular = duallink5.singularity.evaluateCoupling( ...
                [3.1273346915827149, deg2rad(69)], ...
                testCase.Geometry, testCase.Options);
            testCase.verifyTrue(lowerSingular.theoreticallyReachable);
            testCase.verifyEqual(lowerSingular.statusCode, ...
                "LOWER_CLOSURE_SINGULAR");
            testCase.verifyTrue(all(isfinite( ...
                lowerSingular.coupling.constraintMatrix), 'all'));
            testCase.verifyTrue(all(isnan( ...
                lowerSingular.coupling.actuationMatrix), 'all'));
            testCase.verifyTrue(all(isnan( ...
                lowerSingular.coupling.taskJacobian), 'all'));
        end

        function lowerTangencyCollisionIsAssessedFromFinitePoses(testCase)
            q = [3.1273346915827149, deg2rad(69)];
            none = duallink5.singularity.evaluateCoupling( ...
                q, testCase.Geometry, struct( ...
                'collisionProfile', "none", ...
                'topologyProfile', "none"));
            centerline = duallink5.singularity.evaluateCoupling( ...
                q, testCase.Geometry, struct( ...
                'collisionProfile', "centerline", ...
                'topologyProfile', "none"));
            noneAssessment = none.metadata.assemblyAssessment;
            centerAssessment = centerline.metadata.assemblyAssessment;

            testCase.verifyEqual(none.statusCode, ...
                "LOWER_CLOSURE_SINGULAR");
            testCase.verifyEqual(centerline.statusCode, ...
                "LOWER_CLOSURE_SINGULAR");
            testCase.verifyEqual( ...
                none.kinematics.lower.quality.statusCode, ...
                centerline.kinematics.lower.quality.statusCode);
            testCase.verifyTrue(noneAssessment.collisionFree);
            testCase.verifyNotEqual( ...
                centerAssessment.collisionStatusCode, "NEAR_SINGULAR");
            testCase.verifyEqual(centerAssessment.collisionFree, ...
                finitePoseCollisionFree(centerline.kinematics, ...
                testCase.Geometry));
        end

        function lowerTangencyIgnoresCouplingExactThreshold(testCase)
            options = testCase.Options;
            options.exactThreshold = 1e-20;
            result = duallink5.singularity.evaluateCoupling( ...
                [3.1273346915827149, deg2rad(69)], ...
                testCase.Geometry, options);

            testCase.verifyEqual(result.statusCode, ...
                "LOWER_CLOSURE_SINGULAR");
            testCase.verifyFalse(result.engineeringWarning);
            testCase.verifyTrue(all(isnan( ...
                result.coupling.actuationMatrix), 'all'));
            testCase.verifyTrue(all(isnan( ...
                result.coupling.taskJacobian), 'all'));
        end

        function recoveredTangencyMatchesStableKinematicSchema(testCase)
            regular = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), testCase.Geometry, testCase.Options);
            recovered = duallink5.singularity.evaluateCoupling( ...
                [3.1273346915827149, deg2rad(69)], ...
                testCase.Geometry, testCase.Options);
            regularKinematics = regular.kinematics;
            recoveredKinematics = recovered.kinematics;

            verifyStructSchema(testCase, recoveredKinematics, ...
                regularKinematics, "kinematics");
            candidate = recoveredKinematics.passiveCandidates(1);
            testCase.verifyTrue(all(isfinite(candidate.G)));
            testCase.verifyTrue(ismember(double(candidate.branchId), ...
                [-1, 0, 1]));
            testCase.verifyEqual(candidate.orientationSign, ...
                candidate.branchId);
            testCase.verifyTrue(isfinite(candidate.closureResidual));
            testCase.verifyTrue(isfinite( ...
                recoveredKinematics.residuals.maximum));
            testCase.verifyTrue(isfield( ...
                recoveredKinematics.metadata, 'units'));
            testCase.verifyTrue(isfield( ...
                recoveredKinematics.quality, 'parallelSharedClearance'));
        end

        function defaultReferenceIgnoresCandidateCollisionProfile(testCase)
            geometry = testCase.Geometry;
            geometry.collision.parallelSharedClearance = 1;
            geometry = duallink5.model.validateGeometry(geometry);
            result = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), geometry, struct( ...
                'collisionProfile', "centerline", ...
                'topologyProfile', "none"));

            testCase.verifyEqual( ...
                result.metadata.couplingReference.q, ...
                geometry.analysis.referenceQ, 'AbsTol', 0);
            testCase.verifyNotEqual( ...
                result.metadata.couplingReference.passiveOrientationSign, ...
                int8(0));
            testCase.verifyEqual(result.assemblyStatusCode, ...
                "PARALLEL_SHARED_COLLISION");
        end

        function rejectsMalformedInputsOptionsAndReferences(testCase)
            invalidQ = {[1; 2], [1, NaN], [1, 2, 3], true, ...
                single([1, Inf])};
            for index = 1:numel(invalidQ)
                testCase.verifyError(@() ...
                    duallink5.singularity.evaluateCoupling( ...
                    invalidQ{index}, testCase.Geometry, struct()), ...
                    'duallink5:singularity:InvalidCouplingInput');
            end

            invalidOptions = {[], struct.empty, repmat(struct(), 1, 2), ...
                struct('unsupported', true), ...
                struct('collisionProfile', "bad"), ...
                struct('collisionProfile', "physicalClearance"), ...
                struct('topologyProfile', "bad"), ...
                struct('lowerBranchId', 0), ...
                struct('exactThreshold', 0.05, ...
                'nearMetricThreshold', 0.05), ...
                struct('nearMetricThreshold', 1), ...
                struct('nearConditionThreshold', 1), ...
                struct('referenceQ', [1; 2])};
            for index = 1:numel(invalidOptions)
                testCase.verifyError(@() ...
                    duallink5.singularity.evaluateCoupling( ...
                    deg2rad([85, 30]), testCase.Geometry, ...
                    invalidOptions{index}), ...
                    'duallink5:singularity:InvalidCouplingOptions');
            end

            valid = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), testCase.Geometry, testCase.Options);
            reference = valid.metadata.couplingReference;
            malformed = {rmfield(reference, 'q'), ...
                setfield(reference, 'q', [1; 2]), ... %#ok<SFLD>
                setfield(reference, 'geometryVersion', "other"), ... %#ok<SFLD>
                setfield(reference, 'passiveOrientationSign', int8(0)), ... %#ok<SFLD>
                setfield(reference, 'topology', struct())}; %#ok<SFLD>
            for index = 1:numel(malformed)
                options = testCase.Options;
                options.couplingReference = malformed{index};
                testCase.verifyError(@() ...
                    duallink5.singularity.evaluateCoupling( ...
                    deg2rad([85, 30]), testCase.Geometry, options), ...
                    'duallink5:singularity:InvalidCouplingOptions');
            end
        end

        function rejectsCachedTopologyOutsideSignatureDomain(testCase)
            valid = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), testCase.Geometry, testCase.Options);
            reference = valid.metadata.couplingReference;
            validRow = firstSignatureRow(reference.topology.signature);

            bogus = reference;
            bogus.topology.signature = ["bogus", "lower.link1"];
            selfPair = reference;
            selfPair.topology.signature = ["lower.link1", "lower.link1"];
            unsortedPair = reference;
            unsortedPair.topology.signature = fliplr(validRow);
            duplicate = reference;
            duplicate.topology.signature = [validRow; validRow];
            nonexistentLinks = reference;
            nonexistentLinks.topology.signature = ...
                ["lower.link4", "upper.link4"];
            malformed = {bogus, selfPair, unsortedPair, duplicate, ...
                nonexistentLinks};

            for index = 1:numel(malformed)
                options = testCase.Options;
                options.topologyProfile = "none";
                options.couplingReference = malformed{index};
                testCase.verifyError(@() ...
                    duallink5.singularity.evaluateCoupling( ...
                    deg2rad([85, 30]), testCase.Geometry, options), ...
                    'duallink5:singularity:InvalidCouplingOptions');
            end

            options = testCase.Options;
            options.couplingReference = reference;
            reused = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), testCase.Geometry, options);
            testCase.verifyEqual( ...
                reused.metadata.couplingReference, reference);
        end


        function cachedReferenceIsSemanticallyBound(testCase)
            valid = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), testCase.Geometry, testCase.Options);
            reference = valid.metadata.couplingReference;
            validRow = firstSignatureRow(reference.topology.signature);

            badQ = reference;
            badQ.q = [pi, pi];
            badSign = reference;
            badSign.passiveOrientationSign = ...
                -reference.passiveOrientationSign;
            badSignature = reference;
            badSignature.topology.signature = validAlternativeSignature( ...
                validRow, reference.topology.signature);
            badTolerance = reference;
            badTolerance.topology.lengthTolerance = ...
                2 * testCase.Geometry.tolerance.length;
            malformed = {badQ, badSign, badSignature, badTolerance};

            for index = 1:numel(malformed)
                options = testCase.Options;
                options.couplingReference = malformed{index};
                testCase.verifyError(@() ...
                    duallink5.singularity.evaluateCoupling( ...
                    deg2rad([85, 30]), testCase.Geometry, options), ...
                    'duallink5:singularity:InvalidCouplingOptions');
            end

            changedGeometry = testCase.Geometry;
            changedGeometry.links.link3 = ...
                changedGeometry.links.link3 + 1e-3;
            changedGeometry = ...
                duallink5.model.validateGeometry(changedGeometry);
            options = testCase.Options;
            options.couplingReference = reference;
            testCase.verifyError(@() ...
                duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), changedGeometry, options), ...
                'duallink5:singularity:InvalidCouplingOptions');

            options.couplingReference = reference;
            reused = duallink5.singularity.evaluateCoupling( ...
                deg2rad([85, 30]), testCase.Geometry, options);
            testCase.verifyEqual( ...
                reused.metadata.couplingReference, reference);
        end
    end
end

function points = passivePoints(q, geometry)
result = duallink5.kinematics.forwardPassiveDoublet( ...
    q, geometry, struct('collisionProfile', "none"));
assert(result.quality.valid);
points = struct( ...
    'A', result.upper.points.A, ...
    'C', result.upper.points.C, ...
    'G', result.points.G);
end

function value = cross2(first, second)
value = first(1) * second(2) - first(2) * second(1);
end

function row = firstSignatureRow(signature)
if isempty(signature)
    row = ["lower.link1", "upper.link1"];
else
    row = signature(1, :);
end
end

function safe = finitePoseCollisionFree(kinematics, geometry)
[lowerSafe, ~] = duallink5.validation.isCollisionFree( ...
    kinematics.lower, geometry, "centerline");
[upperSafe, ~] = duallink5.validation.isCollisionFree( ...
    kinematics.upper, geometry, "centerline");
points = kinematics.lower.points;
clearance = min([ ...
    pointSegmentDistance(points.Pbeta1, points.E, points.D), ...
    pointSegmentDistance(points.Pbeta2, points.E, points.D), ...
    pointSegmentDistance(points.E, points.Pbeta1, points.Pbeta2), ...
    pointSegmentDistance(points.D, points.Pbeta1, points.Pbeta2)]);
safe = lowerSafe && upperSafe && ...
    clearance > geometry.collision.parallelSharedClearance;
end

function distance = pointSegmentDistance(point, startPoint, endPoint)
segment = endPoint - startPoint;
denominator = dot(segment, segment);
if denominator == 0
    distance = norm(point - startPoint);
    return
end
parameter = dot(point - startPoint, segment) / denominator;
parameter = max(0, min(1, parameter));
distance = norm(point - (startPoint + parameter * segment));
end

function verifyStructSchema(testCase, actual, expected, path)
testCase.verifyEqual(sort(fieldnames(actual)), ...
    sort(fieldnames(expected)), sprintf('%s fields differ.', path));
names = fieldnames(expected);
for index = 1:numel(names)
    name = names{index};
    if isstruct(expected.(name)) && isscalar(expected.(name)) && ...
            isstruct(actual.(name)) && isscalar(actual.(name))
        verifyStructSchema(testCase, actual.(name), expected.(name), ...
            path + "." + name);
    end
end
end

function signature = validAlternativeSignature(validRow, original)
alternatives = [ ...
    "alpha.12", "lower.link1"; ...
    "beta.12", "upper.link1"; ...
    "lower.link1", "upper.link1"];
for index = 1:size(alternatives, 1)
    row = sort(alternatives(index, :));
    if ~isequal(row, validRow) && ...
            ~any(all(original == row, 2))
        signature = row;
        return
    end
end
signature = sort(["alpha.12", "upper.link2"]);
end
