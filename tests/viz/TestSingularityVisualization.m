classdef TestSingularityVisualization < matlab.unittest.TestCase
    properties
        Samples
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            geometry = duallink5.model.defaultGeometry();
            grid.theta = deg2rad(0:15:180);
            grid.phi = deg2rad(0:15:90);
            testCase.Samples = duallink5.singularity.sampleSpace( ...
                grid, geometry, struct());
        end
    end

    methods (Test)
        function jointPlotReturnsLayerHandlesAndRestoresAxes(testCase)
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];
            axesHandles(1).NextPlot = 'replacechildren';
            axesHandles(2).NextPlot = 'replace';

            handles = duallink5.viz.plotJointSingularitySpace( ...
                testCase.Samples, axesHandles);

            required = {'conditionImage', 'categoryImage', 'nearBand', ...
                'typeITheta', 'typeIPhi', 'typeIIOuter', ...
                'typeIIInner', 'reference', 'conditionColorbar', ...
                'categoryColorbar', 'legend'};
            testCase.verifyTrue(all(isfield(handles, required)));
            testCase.verifyTrue(isgraphics(handles.conditionImage));
            testCase.verifyTrue(isgraphics(handles.categoryImage));
            testCase.verifyTrue(isgraphics(handles.reference));
            testCase.verifyEqual( ...
                axesHandles(1).NextPlot, 'replacechildren');
            testCase.verifyEqual(axesHandles(2).NextPlot, 'replace');
            testCase.verifyEqual(axesHandles(1).XLabel.String, ...
                '\theta [deg]');
            testCase.verifyEqual(axesHandles(1).YLabel.String, ...
                '\phi [deg]');
            clear cleanup
        end

        function jointPlotRejectsBadAxesAndSamples(testCase)
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            oneAxes = axes(figureHandle);
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                testCase.Samples, oneAxes), ...
                'duallink5:viz:InvalidGraphicsHandle');
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                testCase.Samples, [oneAxes, oneAxes]), ...
                'duallink5:viz:InvalidGraphicsHandle');

            delete(oneAxes);
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];
            malformed = rmfield(testCase.Samples, 'classificationMap');
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                malformed, axesHandles), ...
                'duallink5:viz:InvalidSingularityPlotInput');

            malformed = testCase.Samples;
            malformed.conditionNumber.task = strings( ...
                size(malformed.thetaGrid));
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                malformed, axesHandles), ...
                'duallink5:viz:InvalidSingularityPlotInput');

            malformed = testCase.Samples;
            malformed.mechanicallyValidMask = double( ...
                malformed.mechanicallyValidMask);
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                malformed, axesHandles), ...
                'duallink5:viz:InvalidSingularityPlotInput');

            malformed = testCase.Samples;
            malformed.metadata.referenceQ(1) = NaN;
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                malformed, axesHandles), ...
                'duallink5:viz:InvalidSingularityPlotInput');

            malformed = testCase.Samples;
            malformed.curves.typeITheta = struct.empty;
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                malformed, axesHandles), ...
                'duallink5:viz:InvalidSingularityPlotInput');
            clear cleanup
        end

        function jointPlotRejectsMalformedCurveContract(testCase)
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];
            malformed = testCase.Samples;
            malformed.curves.typeITheta = struct( ...
                'theta', [0, 1], 'phi', 0, ...
                'pointG', zeros(2, 2), ...
                'adjacentMechanical', true(1, 2));

            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                malformed, axesHandles), ...
                'duallink5:viz:InvalidSingularityPlotInput');
            clear cleanup
        end

        function jointPlotSupportsSinglePointGrid(testCase)
            geometry = duallink5.model.defaultGeometry();
            grid.theta = deg2rad(85);
            grid.phi = deg2rad(30);
            samples = duallink5.singularity.sampleSpace( ...
                grid, geometry, struct('collisionProfile', "none", ...
                'topologyProfile', "none"));
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];

            duallink5.viz.plotJointSingularitySpace(samples, axesHandles);

            testCase.verifyGreaterThan(diff(axesHandles(1).XLim), 0);
            testCase.verifyGreaterThan(diff(axesHandles(1).YLim), 0);
            clear cleanup
        end

        function jointPlotValidatesConditionNumberSemantics(testCase)
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];
            invalidValues = {-1, 0, -Inf};
            for index = 1:numel(invalidValues)
                malformed = testCase.Samples;
                malformed.conditionNumber.task(1) = invalidValues{index};
                testCase.verifyError( ...
                    @()duallink5.viz.plotJointSingularitySpace( ...
                    malformed, axesHandles), ...
                    'duallink5:viz:InvalidSingularityPlotInput');
            end

            allowed = testCase.Samples;
            allowed.conditionNumber.task(1) = NaN;
            allowed.conditionNumber.task(2) = Inf;
            handles = duallink5.viz.plotJointSingularitySpace( ...
                allowed, axesHandles);
            testCase.verifyTrue(isgraphics(handles.conditionImage));
            clear cleanup
        end

        function jointPlotSetsConditionLimitsFromCurrentData(testCase)
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];
            axesHandles(1).CLim = [42, 43];
            condition = log10(testCase.Samples.conditionNumber.task);
            valid = testCase.Samples.theoreticalReachableMask & ...
                isfinite(condition);
            expected = [min(condition(valid)), max(condition(valid))];
            testCase.assertLessThan(expected(1), expected(2));

            duallink5.viz.plotJointSingularitySpace( ...
                testCase.Samples, axesHandles);
            testCase.verifyEqual(axesHandles(1).CLim, expected, ...
                'AbsTol', 1e-12);

            fallback = testCase.Samples;
            fallback.conditionNumber.task(:) = Inf;
            fallback.conditionNumber.task(1:2:end) = NaN;
            axesHandles(1).CLim = [42, 43];
            duallink5.viz.plotJointSingularitySpace(fallback, axesHandles);
            testCase.verifyEqual(axesHandles(1).CLim, [0, 1]);

            fallback.conditionNumber.task(:) = 10;
            axesHandles(1).CLim = [42, 43];
            duallink5.viz.plotJointSingularitySpace(fallback, axesHandles);
            testCase.verifyEqual(axesHandles(1).CLim, [0, 1]);
            clear cleanup
        end

        function jointPlotSetsConstantConditionLimits(testCase)
            samples = testCase.Samples;
            samples.conditionNumber.task(:) = 100;
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];
            axesHandles(1).CLim = [42, 43];

            duallink5.viz.plotJointSingularitySpace(samples, axesHandles);

            limits = axesHandles(1).CLim;
            testCase.verifyLessThan(limits(1), limits(2));
            testCase.verifyLessThanOrEqual(limits(1), 2);
            testCase.verifyGreaterThanOrEqual(limits(2), 2);
            clear cleanup
        end

        function jointPlotUsesNonuniformCellEdges(testCase)
            geometry = duallink5.model.defaultGeometry();
            grid.theta = deg2rad([0, 5, 40, 180]);
            grid.phi = deg2rad([0, 10, 90]);
            samples = duallink5.singularity.sampleSpace( ...
                grid, geometry, struct('collisionProfile', "none", ...
                'topologyProfile', "none"));
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];

            handles = duallink5.viz.plotJointSingularitySpace( ...
                samples, axesHandles);

            expectedThetaEdges = [-2.5, 2.5, 22.5, 110, 250];
            expectedPhiEdges = [-5, 5, 50, 130];
            testCase.verifyEqual( ...
                handles.conditionImage.XData(1, :), ...
                expectedThetaEdges, 'AbsTol', 1e-12);
            testCase.verifyEqual( ...
                handles.conditionImage.YData(:, 1).', ...
                expectedPhiEdges, 'AbsTol', 1e-12);
            testCase.verifyEqual(handles.categoryImage.XData(1, :), ...
                expectedThetaEdges, 'AbsTol', 1e-12);
            testCase.verifyEqual(handles.categoryImage.YData(:, 1).', ...
                expectedPhiEdges, 'AbsTol', 1e-12);
            testCase.verifyEqual(axesHandles(1).XLim, ...
                expectedThetaEdges([1, end]), 'AbsTol', 1e-12);
            testCase.verifyEqual(axesHandles(1).YLim, ...
                expectedPhiEdges([1, end]), 'AbsTol', 1e-12);
            clear cleanup
        end

        function jointPlotRejectsMalformedGridContract(testCase)
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];

            descendingTheta = testCase.Samples;
            descendingTheta.thetaGrid = flipud(descendingTheta.thetaGrid);
            descendingPhi = testCase.Samples;
            descendingPhi.phiGrid = fliplr(descendingPhi.phiGrid);
            nonseparableTheta = testCase.Samples;
            nonseparableTheta.thetaGrid(1, 2) = ...
                nonseparableTheta.thetaGrid(1, 2) + 0.01;
            nonseparablePhi = testCase.Samples;
            nonseparablePhi.phiGrid(2, 1) = ...
                nonseparablePhi.phiGrid(2, 1) + 0.01;
            malformedGrids = {descendingTheta, descendingPhi, ...
                nonseparableTheta, nonseparablePhi};
            for index = 1:numel(malformedGrids)
                testCase.verifyError( ...
                    @()duallink5.viz.plotJointSingularitySpace( ...
                    malformedGrids{index}, axesHandles), ...
                    'duallink5:viz:InvalidSingularityPlotInput');
            end
            clear cleanup
        end

        function jointPlotRejectsInconsistentMasksAndReasons(testCase)
            target = find(testCase.Samples.safeUsableMask, 1);
            testCase.assertNotEmpty(target);

            safeWithoutMechanical = testCase.Samples;
            safeWithoutMechanical.mechanicallyValidMask(target) = false;
            mechanicalWithoutTheoretical = testCase.Samples;
            mechanicalWithoutTheoretical.theoreticalReachableMask(target) = ...
                false;
            mechanicalWithoutTheoretical.safeUsableMask(target) = false;
            collisionMarkedSafe = testCase.Samples;
            collisionMarkedSafe.reasonMap(target) = "SELF_COLLISION";
            topologyMarkedSafe = testCase.Samples;
            topologyMarkedSafe.reasonMap(target) = ...
                "ASSEMBLY_TOPOLOGY_MISMATCH";
            nearWithRegularReason = testCase.Samples;
            nearWithRegularReason.near.any(target) = true;
            nearWithRegularReason.safeUsableMask(target) = false;
            nearReasonWithoutNearMask = testCase.Samples;
            nearReasonWithoutNearMask.reasonMap(target) = "NEAR_SINGULAR";
            inconsistent = {safeWithoutMechanical, ...
                mechanicalWithoutTheoretical, collisionMarkedSafe, ...
                topologyMarkedSafe, nearWithRegularReason, ...
                nearReasonWithoutNearMask};

            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];
            for index = 1:numel(inconsistent)
                testCase.verifyError( ...
                    @()duallink5.viz.plotJointSingularitySpace( ...
                    inconsistent{index}, axesHandles), ...
                    'duallink5:viz:InvalidSingularityPlotInput');
            end
            clear cleanup
        end

        function jointLimitViolationUsesUnreachableCategory(testCase)
            samples = testCase.Samples;
            target = find(samples.theoreticalReachableMask, 1);
            [row, column] = ind2sub(size(samples.thetaGrid), target);
            samples.reasonMap(target) = "JOINT_LIMIT_VIOLATION";
            samples.mechanicallyValidMask(target) = false;
            samples.safeUsableMask(target) = false;
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];

            handles = duallink5.viz.plotJointSingularitySpace( ...
                samples, axesHandles);

            testCase.verifyEqual( ...
                handles.categoryImage.CData(column, row), 0);
            clear cleanup
        end
    end
end
