classdef TestGeometry < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(~)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
        end
    end

    methods (Test)
        function defaultLengthsUseSIAndApprovedMapping(testCase)
            g = duallink5.model.defaultGeometry();

            testCase.verifyEqual(g.links.link1, 80e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link2, 62e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link3, 69e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link4, 80e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link5, 80e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.units.length, "m");
            testCase.verifyEqual(g.units.angle, "rad");
        end

        function parallelDimensionsUseEndpointNames(testCase)
            g = duallink5.model.defaultGeometry();
            L = g.parallel.lengths;

            testCase.verifyEqual(L.E_Palpha1, 30e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(L.Palpha1_Palpha4, 60e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(L.D_Pbeta2, 30e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(L.Pbeta2_Pbeta3, 60e-3, 'AbsTol', 1e-15);
        end

        function invalidUnitIsRejected(testCase)
            g = duallink5.model.defaultGeometry();
            g.units.length = "mm";
            testCase.verifyError(@() duallink5.model.validateGeometry(g), ...
                'duallink5:model:InvalidUnits');
        end

        function missingDirectionFieldIsRejected(testCase)
            g=duallink5.model.defaultGeometry();
            g.parallel.direction=rmfield(g.parallel.direction,'betaSide');
            testCase.verifyError(@()duallink5.model.validateGeometry(g), ...
                'duallink5:model:InvalidGeometry');
        end
    end
end
