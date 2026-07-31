classdef TestVisualization < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(testCase)
            originalPath = path;
            testCase.addTeardown(@() path(originalPath));
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot,'startup.m'));
        end
    end

    methods (Test)
        function plotUsesProvidedAxesWithoutChangingRootDefaults(testCase)
            g = duallink5.model.defaultGeometry();
            pose = duallink5.kinematics.forwardFiveBar(deg2rad([85,52.93]),g);
            originalFont = get(groot,'defaultAxesFontName');
            figureHandle = figure('Visible','off');
            cleanup = onCleanup(@() close(figureHandle));
            axesHandle = axes(figureHandle);

            handles = duallink5.viz.plotMechanism(pose,axesHandle,struct());

            testCase.verifyNotEmpty(handles.links);
            testCase.verifyEqual(get(groot,'defaultAxesFontName'),originalFont);
            testCase.verifyTrue(all([handles.links.Parent]==axesHandle));
            clear cleanup
        end

        function workspacePlotAndExporterUseExplicitHandles(testCase)
            samples.x=[0,1;0,1];
            samples.y=[0,0;1,1];
            samples.validMask=true(2);
            samples.reasonMap=repmat("OK",2);
            samples.metadata=struct('units', ...
                struct('length',"m",'angle',"rad"));
            result=duallink5.workspace.analyzeWorkspace( ...
                samples,struct('alpha',Inf,'gridSize',[10,10]));
            figureHandle=figure('Visible','off');
            figureCleanup=onCleanup(@()close(figureHandle));
            axesHandle=axes(figureHandle);
            outputFile=[tempname,'.png'];
            fileCleanup=onCleanup(@()deleteIfExists(outputFile));

            handles=duallink5.viz.plotWorkspaceResult( ...
                samples,result,axesHandle);
            duallink5.viz.exportFigure( ...
                figureHandle,outputFile,struct('resolution',100));

            testCase.verifyEqual(handles.boundary.Parent,axesHandle);
            testCase.verifyTrue(isfile(outputFile));
            clear fileCleanup figureCleanup
        end

        function numericWorkspaceMaskPlotsEverySelectedPoint(testCase)
            [samples,result] = squareWorkspaceFixture();
            samples.validMask = ones(2);
            figureHandle = figure('Visible','off');
            cleanup = onCleanup(@()close(figureHandle));
            axesHandle = axes(figureHandle);

            handles = duallink5.viz.plotWorkspaceResult( ...
                samples,result,axesHandle);

            actual = sortrows(scatterCoordinates(handles.samples));
            expected = sortrows([samples.x(:),samples.y(:)]);
            testCase.verifyEqual(actual,expected);
            clear cleanup
        end

        function workspacePlotRejectsMalformedInputs(testCase)
            [samples,result] = squareWorkspaceFixture();
            figureHandle = figure('Visible','off');
            cleanup = onCleanup(@()close(figureHandle));
            axesHandle = axes(figureHandle);
            badShape = samples;
            badShape.y = zeros(1,4);
            badMask = samples;
            badMask.validMask = [1,2;1,0];
            badCoordinate = samples;
            badCoordinate.x(1,1) = NaN;
            malformedSamples = {badShape,badMask,badCoordinate};

            for index = 1:numel(malformedSamples)
                testCase.verifyError( ...
                    @()duallink5.viz.plotWorkspaceResult( ...
                    malformedSamples{index},result,axesHandle), ...
                    'duallink5:viz:InvalidWorkspacePlotInput');
            end

            badBoundary = result;
            badBoundary.boundaryShape = [];
            badBounds = result;
            badBounds.maxRectangle.bounds = [1,0,0,1];
            malformedResults = {badBoundary,badBounds};
            for index = 1:numel(malformedResults)
                testCase.verifyError( ...
                    @()duallink5.viz.plotWorkspaceResult( ...
                    samples,malformedResults{index},axesHandle), ...
                    'duallink5:viz:InvalidWorkspacePlotInput');
            end
            clear cleanup
        end

        function workspacePlotRequiresLiveScalarAxes(testCase)
            [samples,result] = squareWorkspaceFixture();
            figureHandle = figure('Visible','off');
            cleanup = onCleanup(@()close(figureHandle));
            firstAxes = axes(figureHandle);
            secondAxes = axes(figureHandle);
            deletedAxes = axes(figureHandle);
            delete(deletedAxes);
            invalidHandles = {figureHandle,[firstAxes,secondAxes],deletedAxes};

            for index = 1:numel(invalidHandles)
                testCase.verifyError( ...
                    @()duallink5.viz.plotWorkspaceResult( ...
                    samples,result,invalidHandles{index}), ...
                    'duallink5:viz:InvalidGraphicsHandle');
            end
            clear cleanup
        end

        function workspacePlotRestoresExactNextPlot(testCase)
            [samples,result] = squareWorkspaceFixture();
            figureHandle = figure('Visible','off');
            cleanup = onCleanup(@()close(figureHandle));
            axesHandle = axes(figureHandle);
            axesHandle.NextPlot = 'replacechildren';

            duallink5.viz.plotWorkspaceResult(samples,result,axesHandle);

            testCase.verifyEqual(axesHandle.NextPlot,'replacechildren');

            overflowResult = result;
            overflowResult.maxRectangle.bounds = ...
                [-realmax,realmax,-realmax,realmax];
            didError = false;
            try
                duallink5.viz.plotWorkspaceResult( ...
                    samples,overflowResult,axesHandle);
            catch
                didError = true;
            end
            testCase.verifyTrue(didError);
            testCase.verifyEqual(axesHandle.NextPlot,'replacechildren');
            clear cleanup
        end

        function mechanismRejectsInvalidHandlesOptionsAndPoses(testCase)
            pose = verifiedPoseFixture();
            figureHandle = figure('Visible','off');
            cleanup = onCleanup(@()close(figureHandle));
            firstAxes = axes(figureHandle);
            secondAxes = axes(figureHandle);
            deletedAxes = axes(figureHandle);
            delete(deletedAxes);
            invalidHandles = {figureHandle,[firstAxes,secondAxes],deletedAxes};
            for index = 1:numel(invalidHandles)
                testCase.verifyError( ...
                    @()duallink5.viz.plotMechanism( ...
                    pose,invalidHandles{index},struct()), ...
                    'duallink5:viz:InvalidGraphicsHandle');
            end

            invalidOptions = {[],42,repmat(struct(),1,2), ...
                struct('lineWidth',0),struct('lineWidth',Inf), ...
                struct('lineWidth',"2"),struct('showLabels',2), ...
                struct('showLabels',NaN),struct('showLabels',[])};
            for index = 1:numel(invalidOptions)
                firstAxes.NextPlot = 'replacechildren';
                testCase.verifyError( ...
                    @()duallink5.viz.plotMechanism( ...
                    pose,firstAxes,invalidOptions{index}), ...
                    'duallink5:viz:InvalidPlotOptions');
                testCase.verifyEqual( ...
                    firstAxes.NextPlot,'replacechildren');
            end

            invalidSolverPose = duallink5.kinematics.forwardFiveBar( ...
                [NaN,0],duallink5.model.defaultGeometry());
            missingPoints = rmfield(pose,'points');
            badValidFlag = pose;
            badValidFlag.quality.valid = 2;
            badPoint = pose;
            badPoint.points.A = [0;0;0];
            badCenter = pose;
            badCenter.sharedLink.center = [NaN;0];
            invalidPoses = {invalidSolverPose,missingPoints, ...
                badValidFlag,badPoint,badCenter};
            for index = 1:numel(invalidPoses)
                firstAxes.NextPlot = 'replacechildren';
                testCase.verifyError( ...
                    @()duallink5.viz.plotMechanism( ...
                    invalidPoses{index},firstAxes,struct()), ...
                    'duallink5:viz:InvalidPose');
                testCase.verifyEqual( ...
                    firstAxes.NextPlot,'replacechildren');
            end
            clear cleanup
        end

        function mechanismRestoresExactNextPlot(testCase)
            pose = verifiedPoseFixture();
            figureHandle = figure('Visible','off');
            cleanup = onCleanup(@()close(figureHandle));
            axesHandle = axes(figureHandle);
            axesHandle.NextPlot = 'replacechildren';

            duallink5.viz.plotMechanism( ...
                pose,axesHandle,struct('showLabels',false));
            testCase.verifyEqual(axesHandle.NextPlot,'replacechildren');
            clear cleanup
        end

        function exporterRejectsInvalidHandlesOptionsAndFiles(testCase)
            figureHandle = figure('Visible','off');
            secondFigure = figure('Visible','off');
            cleanup = onCleanup(@()close([figureHandle,secondFigure]));
            axesHandle = axes(figureHandle);
            deletedFigure = figure('Visible','off');
            delete(deletedFigure);
            outputFile = [tempname,'.png'];
            fileCleanup = onCleanup(@()deleteIfExists(outputFile));
            invalidHandles = {deletedFigure,axesHandle, ...
                [figureHandle,secondFigure]};
            for index = 1:numel(invalidHandles)
                testCase.assertError( ...
                    @()duallink5.viz.exportFigure( ...
                    invalidHandles{index},outputFile,struct()), ...
                    'duallink5:viz:InvalidGraphicsHandle');
            end

            invalidOptions = {[],42,repmat(struct(),1,2), ...
                struct('resolution',0),struct('resolution',1.5), ...
                struct('resolution',Inf),struct('resolution',"100")};
            for index = 1:numel(invalidOptions)
                testCase.assertError( ...
                    @()duallink5.viz.exportFigure( ...
                    figureHandle,outputFile,invalidOptions{index}), ...
                    'duallink5:viz:InvalidExportOptions');
            end

            invalidFiles = {"",'',"output", ...
                [tempname,'.bmp'],fullfile(tempname,'output.png'), ...
                ["first.png","second.png"]};
            for index = 1:numel(invalidFiles)
                testCase.assertError( ...
                    @()duallink5.viz.exportFigure( ...
                    figureHandle,invalidFiles{index},struct()), ...
                    'duallink5:viz:InvalidExportFile');
            end
            clear fileCleanup cleanup
        end
    end
end

function pose = verifiedPoseFixture()
geometry = duallink5.model.defaultGeometry();
pose = duallink5.kinematics.forwardFiveBar( ...
    deg2rad([85,52.93]),geometry);
end

function [samples,result] = squareWorkspaceFixture()
samples.x = [0,1;0,1];
samples.y = [0,0;1,1];
samples.validMask = true(2);
samples.reasonMap = repmat("OK",2);
samples.metadata = struct('units', ...
    struct('length',"m",'angle',"rad"));
result = duallink5.workspace.analyzeWorkspace( ...
    samples,struct('alpha',Inf,'gridSize',[10,10]));
end

function coordinates = scatterCoordinates(scatterHandles)
parts = cell(numel(scatterHandles),1);
for index = 1:numel(scatterHandles)
    parts{index} = [scatterHandles(index).XData(:), ...
        scatterHandles(index).YData(:)];
end
coordinates = vertcat(parts{:});
end

function deleteIfExists(fileName)
if isfile(fileName), delete(fileName); end
end
