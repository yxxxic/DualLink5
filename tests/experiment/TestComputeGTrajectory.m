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

        function rowAndColumnStreamsAreEquivalent(testCase)
            geometry = duallink5.model.defaultGeometry();

            rowResult = duallink5exp.computeGTrajectory( ...
                single([85, 86]), single([52.93, 53]), geometry);
            columnResult = duallink5exp.computeGTrajectory( ...
                single([85; 86]), single([52.93; 53]), geometry);

            testCase.verifyEqual(rowResult.x, columnResult.x, ...
                'AbsTol', 1e-12);
            testCase.verifyEqual(rowResult.y, columnResult.y, ...
                'AbsTol', 1e-12);
            testCase.verifyEqual(rowResult.status, columnResult.status);
            testCase.verifySize(rowResult.x, [2, 1]);
            testCase.verifyClass(rowResult.x, 'double');
            testCase.verifyEqual(rowResult.units.position, "m");
            testCase.verifyEqual(rowResult.units.inputAngle, "deg");
        end

        function rejectsInvalidAngleStreams(testCase)
            geometry = duallink5.model.defaultGeometry();
            invalidStreams = {reshape(1:4, 2, 2), ...
                [85 + 1i; 86], '85', {85; 86}};

            for index = 1:numel(invalidStreams)
                stream = invalidStreams{index};
                validStream = 50 * ones(numel(stream), 1);
                testCase.verifyError( ...
                    @() duallink5exp.computeGTrajectory( ...
                        stream, validStream, geometry), ...
                    'duallink5exp:InvalidAngleStream');
                testCase.verifyError( ...
                    @() duallink5exp.computeGTrajectory( ...
                        validStream, stream, geometry), ...
                    'duallink5exp:InvalidAngleStream');
            end
        end

        function rejectsMismatchedStreamLengths(testCase)
            geometry = duallink5.model.defaultGeometry();

            testCase.verifyError( ...
                @() duallink5exp.computeGTrajectory( ...
                    [85; 86], 52.93, geometry), ...
                'duallink5exp:SizeMismatch');
        end

        function validatesGeometryBeforeEmptyOrNonfiniteRows(testCase)
            malformedGeometry = struct();

            testCase.verifyError( ...
                @() duallink5exp.computeGTrajectory( ...
                    NaN, NaN, malformedGeometry), ...
                'duallink5:model:InvalidGeometry');
            testCase.verifyError( ...
                @() duallink5exp.computeGTrajectory( ...
                    [], [], malformedGeometry), ...
                'duallink5:model:InvalidGeometry');

            geometry = duallink5.model.defaultGeometry();
            result = duallink5exp.computeGTrajectory([], [], geometry);
            testCase.verifySize(result.x, [0, 1]);
            testCase.verifySize(result.status, [0, 1]);
        end

        function interpolationSortsAndUsesLastDuplicate(testCase)
            sourceTime = [2, 0, 1, 1, NaN, 3];
            sourceValue = [20, 0, 10, 12, 999, NaN];
            queryTime = [-1, 0, 0.5, 1, 1.5, 2, 3, NaN];

            actual = duallink5exp.interpolateTimeSeries( ...
                sourceTime, sourceValue, queryTime);

            expected = [NaN, 0, 6, 12, 16, 20, NaN, NaN];
            testCase.verifyEqual(actual, expected, 'AbsTol', 1e-12);
            testCase.verifySize(actual, size(queryTime));
        end

        function interpolationHandlesSingleAndEmptySources(testCase)
            queryTime = [5; 5 + 8 * eps(5); 5 + 32 * eps(5); NaN];

            singleResult = duallink5exp.interpolateTimeSeries( ...
                [NaN, 5], [1, 42], queryTime);
            emptyResult = duallink5exp.interpolateTimeSeries( ...
                [NaN, Inf], [1, 2], queryTime);
            normalizedResult = duallink5exp.interpolateTimeSeries( ...
                single([0, 1]), single([0, 1]), single(0.5));

            testCase.verifyEqual(singleResult, [42; 42; NaN; NaN]);
            testCase.verifyEqual(emptyResult, nan(size(queryTime)));
            testCase.verifySize(singleResult, size(queryTime));
            testCase.verifyClass(normalizedResult, 'double');
        end

        function interpolationRejectsInvalidInputs(testCase)
            invalidVectors = {reshape(1:4, 2, 2), ...
                [1 + 1i; 2], '12', {1; 2}};

            for index = 1:numel(invalidVectors)
                invalid = invalidVectors{index};
                valid = zeros(numel(invalid), 1);
                testCase.verifyError( ...
                    @() duallink5exp.interpolateTimeSeries( ...
                        invalid, valid, 0), ...
                    'duallink5exp:InvalidTimeSeries');
                testCase.verifyError( ...
                    @() duallink5exp.interpolateTimeSeries( ...
                        valid, invalid, 0), ...
                    'duallink5exp:InvalidTimeSeries');
                testCase.verifyError( ...
                    @() duallink5exp.interpolateTimeSeries( ...
                        0, 0, invalid), ...
                    'duallink5exp:InvalidQueryTime');
            end

            testCase.verifyError( ...
                @() duallink5exp.interpolateTimeSeries([0, 1], 0, 0), ...
                'duallink5exp:TimeSeriesSizeMismatch');
        end

        function scriptsUseSharedInterpolationHelper(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            scriptFiles = { ...
                fullfile(projectRoot, 'Experiment', 'plotModeledG.m'), ...
                fullfile(projectRoot, 'Experiment', 'exp0612.m'), ...
                fullfile(projectRoot, 'Experiment', 'exp0618.m')};

            for index = 1:numel(scriptFiles)
                scriptText = string(fileread(scriptFiles{index}));
                testCase.verifyEqual(count(scriptText, ...
                    "duallink5exp.interpolateTimeSeries"), 2);
                testCase.verifyFalse(contains(scriptText, "interp1("));
                testCase.verifyFalse(contains(scriptText, "unique("));
            end
        end

        function exp0618GuardsExactInventoryBeforeLoop(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            scriptText = string(fileread( ...
                fullfile(projectRoot, 'Experiment', 'exp0618.m')));

            testCase.verifyTrue(contains(scriptText, ...
                "duallink5exp:ExperimentInventoryMismatch"));
            testCase.verifyTrue(contains(scriptText, ...
                "for i = 1:expected_pairs"));
        end

        function exp0417UsesSelfLocatedInput(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            scriptText = string(fileread( ...
                fullfile(projectRoot, 'Experiment', 'exp0417.m')));

            testCase.verifyTrue(contains(scriptText, ...
                "dataFile = fullfile(experimentDir, " + ...
                "'record_20260417_153928.csv');"));
            testCase.verifyTrue(contains(scriptText, ...
                "data = readtable(dataFile);"));
        end
    end
end
