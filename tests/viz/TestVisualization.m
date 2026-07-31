classdef TestVisualization < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(~)
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
    end
end

function deleteIfExists(fileName)
if isfile(fileName), delete(fileName); end
end
