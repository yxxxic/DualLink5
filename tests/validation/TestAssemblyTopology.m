classdef TestAssemblyTopology < matlab.unittest.TestCase
    properties
        Geometry
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            testCase.Geometry = duallink5.model.defaultGeometry();
        end
    end

    methods (Test)
        function referenceAndNearbyPoseAreCompatible(testCase)
            reference = makeAssembly(testCase.Geometry, [85, 30]);
            nearby = makeAssembly(testCase.Geometry, [85, 31]);

            result = duallink5.validation.compareAssemblyTopology( ...
                nearby, reference, testCase.Geometry);

            testCase.verifyTrue(result.compatible);
            testCase.verifyTrue(islogical(result.compatible));
            testCase.verifyEqual(result.statusCode, "OK");
            testCase.verifyEqual( ...
                result.candidateSignature, result.referenceSignature);
            verifySignature(testCase, result.candidateSignature);
            verifySignature(testCase, result.referenceSignature);
            testCase.verifyEqual(result.addedCrossings, strings(0, 2));
            testCase.verifyEqual(result.removedCrossings, strings(0, 2));
        end

        function sparseModesAreRejected(testCase)
            reference = makeAssembly(testCase.Geometry, [85, 30]);
            sparseQ = {[13.5, 84], [166.5, 72.75]};

            for index = 1:numel(sparseQ)
                candidate = makeAssembly( ...
                    testCase.Geometry, sparseQ{index});
                result = ...
                    duallink5.validation.compareAssemblyTopology( ...
                        candidate, reference, testCase.Geometry);

                testCase.verifyFalse(result.compatible);
                testCase.verifyEqual(result.statusCode, ...
                    "ASSEMBLY_TOPOLOGY_MISMATCH");
                testCase.verifyGreaterThan( ...
                    size(result.addedCrossings, 1) + ...
                    size(result.removedCrossings, 1), 0);
                verifySignature(testCase, result.addedCrossings);
                verifySignature(testCase, result.removedCrossings);
            end
        end

        function malformedAssembliesAreRejected(testCase)
            reference = makeAssembly(testCase.Geometry, [85, 30]);
            badPoint = reference;
            badPoint.lower.points.Palpha3 = [NaN; 0];
            wrongShape = reference;
            wrongShape.upper.points.E = [0; 0; 0];
            nonscalarPoints = reference;
            nonscalarPoints.lower.points = ...
                repmat(reference.lower.points, 1, 2);
            malformed = {42, repmat(reference, 1, 2), ...
                rmfield(reference, 'upper'), badPoint, wrongShape, ...
                nonscalarPoints};

            for index = 1:numel(malformed)
                testCase.verifyError(@() ...
                    duallink5.validation.compareAssemblyTopology( ...
                        malformed{index}, reference, testCase.Geometry), ...
                    'duallink5:validation:InvalidAssemblyTopology');
            end

            testCase.verifyError(@() ...
                duallink5.validation.compareAssemblyTopology( ...
                    reference, badPoint, testCase.Geometry), ...
                'duallink5:validation:InvalidAssemblyTopology');
        end

        function endpointOnlyContactIsNotAnInteriorCrossing(testCase)
            assembly = makeAssembly(testCase.Geometry, [85, 30]);
            assembly.lower.points.A = [0; 0];
            assembly.lower.points.B = [1; 0];
            assembly.lower.points.Palpha1 = [1; 0];
            assembly.lower.points.Palpha2 = [1; 1];

            result = duallink5.validation.compareAssemblyTopology( ...
                assembly, assembly, testCase.Geometry);

            endpointPair = ["alpha.12", "lower.link5"];
            testCase.verifyFalse(any(all( ...
                result.candidateSignature == endpointPair, 2)));
        end

        function shortLinkEndpointToleranceIsScaledPerLink(testCase)
            assembly = makeAssembly(testCase.Geometry, [85, 30]);
            tolerance = testCase.Geometry.tolerance.length;
            assembly.lower.points.A = [0; 0];
            assembly.lower.points.B = [1; 0];
            assembly.lower.points.Palpha1 = [0.5; -0.5 * tolerance];
            assembly.lower.points.Palpha2 = ...
                [0.5; 1e-3 - 0.5 * tolerance];

            result = duallink5.validation.compareAssemblyTopology( ...
                assembly, assembly, testCase.Geometry);

            endpointPair = ["alpha.12", "lower.link5"];
            testCase.verifyFalse(any(all( ...
                result.candidateSignature == endpointPair, 2)));
        end
    end
end

function assembly = makeAssembly(geometry, qDegrees)
q = deg2rad(qDegrees);
jointInput = struct('lower', q, 'upper', q);
assembly = duallink5.kinematics.forwardAssembly( ...
    jointInput, geometry, struct('collisionProfile', "none"));
end

function verifySignature(testCase, signature)
testCase.verifyTrue(isstring(signature));
testCase.verifyEqual(size(signature, 2), 2);
testCase.verifyEqual(signature, sortrows(signature, [1, 2]));
end
