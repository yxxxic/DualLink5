classdef TestComputeGTrajectory < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addPaths(~)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            addpath(fullfile(projectRoot, 'Experiment'));
        end
    end

    methods (Test)
        function convertsDegreeStreamsAndPreservesInvalidRows(testCase)
            geometry = duallink5.model.defaultGeometry();
            theta = [85; NaN];
            phi = [52.93; 50];

            result = duallink5exp.computeGTrajectory( ...
                theta, phi, geometry);

            testCase.verifyEqual( ...
                result.x(1), 9.604250744006e-3, 'AbsTol', 1e-11);
            testCase.verifyEqual( ...
                result.y(1), 0.185384718125293, 'AbsTol', 1e-11);
            testCase.verifyTrue(isnan(result.x(2)));
            testCase.verifyEqual(result.status(2), "NONFINITE_INPUT");
        end
    end
end
