classdef TestDL5MotionApp < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
        end
    end

    methods (Test)
        function constructsHiddenAppWithValidReferencePose(testCase)
            robot = duallink5.DL5();
            app = duallink5.ui.DL5MotionApp( ...
                robot, struct('visible', "off"));
            testCase.addTeardown(@()delete(app));

            testCase.verifyTrue(isvalid(app));
            testCase.verifyEqual(app.Robot, robot);
            testCase.verifyTrue(isvalid(app.Figure));
            testCase.verifyEqual(string(app.Figure.Visible), "off");
            testCase.verifyTrue(robot.IsValid);
        end

        function rejectsInvalidConstructorOptions(testCase)
            robot = duallink5.DL5();

            testCase.verifyError(@()duallink5.ui.DL5MotionApp( ...
                robot, struct('visible', "sometimes")), ...
                'duallink5:ui:InvalidOptions');
            testCase.verifyError(@()duallink5.ui.DL5MotionApp( ...
                42, struct('visible', "off")), ...
                'duallink5:ui:InvalidRobot');
        end
    end
end
