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
    end
end
