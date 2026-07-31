classdef TestPoseValidation < matlab.unittest.TestCase
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
        function verifiedPoseIsCollisionFree(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                deg2rad([85, 52.93]), testCase.Geometry);

            testCase.verifyTrue(pose.quality.valid);
            testCase.verifyTrue(pose.quality.collisionFree);
            testCase.verifyEqual(pose.quality.statusCode, "OK");
        end

        function crossingCenterlinesAreRejected(testCase)
            pose.points = struct( ...
                'A', [-1; 0], 'B', [1; 0], 'C', [0; -1], ...
                'D', [0; 1], 'E', [2; 2]);

            [safe, details] = duallink5.validation.isCollisionFree( ...
                pose, testCase.Geometry, "centerline");

            testCase.verifyFalse(safe);
            testCase.verifyNotEmpty(details.collidingPairs);
        end

        function coincidentNonJointEndpointsAreRejected(testCase)
            pose.points = struct( ...
                'A', [0; 0], 'B', [2; 0], 'C', [1; 0], ...
                'D', [1; 1], 'E', [1; 0]);

            [safe, details] = duallink5.validation.isCollisionFree( ...
                pose, testCase.Geometry, "centerline");

            testCase.verifyFalse(safe);
            testCase.verifyTrue(any(all( ...
                details.collidingPairs == ["link1", "link2"], 2)));
        end

        function physicalProfileRequiresRadii(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                deg2rad([85, 52.93]), testCase.Geometry, ...
                struct('collisionProfile', "none"));

            testCase.verifyError( ...
                @() duallink5.validation.isCollisionFree( ...
                    pose, testCase.Geometry, "physicalClearance"), ...
                'duallink5:validation:MissingClearanceGeometry');
        end

        function collisionCheckerRejectsMalformedPoses(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                deg2rad([85, 52.93]), testCase.Geometry, ...
                struct('collisionProfile', "none"));
            missingPoint = pose;
            missingPoint.points = rmfield(missingPoint.points, 'A');
            nonscalarPoints = pose;
            nonscalarPoints.points = repmat(pose.points, 1, 2);
            nanPoint = pose;
            nanPoint.points.A = [NaN; 0];
            wrongShape = pose;
            wrongShape.points.A = [0; 0; 0];
            malformed = { ...
                struct(), ...
                repmat(struct('points', pose.points), 1, 2), ...
                missingPoint, nonscalarPoints, nanPoint, wrongShape};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.validation.isCollisionFree( ...
                        malformed{index}, testCase.Geometry, "centerline"), ...
                    'duallink5:validation:InvalidPose');
            end
        end

        function assessmentRejectsMalformedValidPoses(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                deg2rad([85, 52.93]), testCase.Geometry, ...
                struct('collisionProfile', "none"));
            missingQuality = rmfield(pose, 'quality');
            missingValid = pose;
            missingValid.quality = rmfield(missingValid.quality, 'valid');
            nonscalarValid = pose;
            nonscalarValid.quality.valid = [true, false];
            nanResidual = pose;
            nanResidual.quality.closureResidual = NaN;
            nanPoint = pose;
            nanPoint.points.C = [0; NaN];
            malformed = { ...
                struct(), missingQuality, missingValid, nonscalarValid, ...
                nanResidual, nanPoint};

            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @() duallink5.validation.assessFiveBarPose( ...
                        malformed{index}, testCase.Geometry, "none"), ...
                    'duallink5:validation:InvalidPose');
            end
        end

        function wellFormedInvalidPosePassesThroughAssessment(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                [NaN, 0.5], testCase.Geometry);

            assessed = duallink5.validation.assessFiveBarPose( ...
                pose, testCase.Geometry, "centerline");

            testCase.verifyEqual(assessed, pose);
        end

        function longSegmentDoesNotInflateShortSegmentTolerance(testCase)
            pose.points = struct( ...
                'A', [0; 0], 'B', [0; 1], ...
                'C', [1e9; 1e9 + 1], 'D', [2; 3], 'E', [1; 1]);

            [~, details] = duallink5.validation.isCollisionFree( ...
                pose, testCase.Geometry, "centerline");

            testCase.verifyFalse(any(all( ...
                details.collidingPairs == ["link1", "link2"], 2)));
        end

        function degenerateLinkUsesPointSegmentSemantics(testCase)
            pose.points = struct( ...
                'A', [0; 0], 'B', [-1; 0], 'C', [1; 0], ...
                'D', [2; 2], 'E', [0; 0]);

            [~, touching] = duallink5.validation.isCollisionFree( ...
                pose, testCase.Geometry, "centerline");

            testCase.verifyTrue(any(all( ...
                touching.collidingPairs == ["link1", "link2"], 2)));

            pose.points.B = [-1; 1];
            pose.points.C = [1; 1];
            [~, separated] = duallink5.validation.isCollisionFree( ...
                pose, testCase.Geometry, "centerline");

            testCase.verifyFalse(any(all( ...
                separated.collidingPairs == ["link1", "link2"], 2)));
        end

        function physicalConfigurationPrecedesKinematicEarlyReturns(testCase)
            options = struct('collisionProfile', "physicalClearance");
            jointVectors = {[NaN, 0.5], [pi, pi]};

            for index = 1:numel(jointVectors)
                q = jointVectors{index};
                testCase.verifyError( ...
                    @() duallink5.kinematics.forwardFiveBar( ...
                        q, testCase.Geometry, options), ...
                    'duallink5:validation:MissingClearanceGeometry');
            end

            configured = testCase.Geometry;
            names = {'link1', 'link2', 'link3', 'link4', 'link5'};
            for index = 1:numel(names)
                name = names{index};
                configured.collision.radius.(name) = 0;
                configured.collision.layerOffset.(name) = 0;
            end
            pose = duallink5.kinematics.forwardFiveBar( ...
                [NaN, 0.5], configured, options);
            testCase.verifyEqual( ...
                pose.quality.statusCode, "NONFINITE_INPUT");
        end

        function directCheckerRejectsNoneProfile(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                deg2rad([85, 52.93]), testCase.Geometry, ...
                struct('collisionProfile', "none"));

            testCase.verifyError( ...
                @() duallink5.validation.isCollisionFree( ...
                    pose, testCase.Geometry, "none"), ...
                'duallink5:validation:InvalidCollisionProfile');
        end
    end
end
