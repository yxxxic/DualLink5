classdef TestCouplingVisualization < matlab.unittest.TestCase
    properties
        Geometry
        Samples
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            run(fullfile(fileparts(fileparts(testDir)), 'startup.m'));
            testCase.Geometry = duallink5.model.defaultGeometry();
            grid.theta = deg2rad([84, 85, 86]);
            grid.phi = deg2rad([-15, -10, -5, 0, 5, 10, 15]);
            testCase.Samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, struct( ...
                'collisionProfile', "none", 'topologyProfile', "none"));
        end
    end

    methods (Test)
        function rendersMapsAndComputedRepresentativeMechanisms(testCase)
            [figureHandle, axesHandles, cleanup] = makeAxes();
            beforeFigures = findall(groot, 'Type', 'figure');
            handles = duallink5.viz.plotCouplingSingularitySpace( ...
                testCase.Samples, axesHandles, struct());

            required = {'jointMap', 'taskScatter', 'poseArrows', ...
                'representativeDirections', 'sourceDirections', ...
                'representativeStatus', 'representativeQ', ...
                'theoreticalLine', 'warningBoundaries', 'warningBand', ...
                'reference', 'jointColorbar', 'taskColorbar', 'assemblies'};
            testCase.verifyTrue(all(isfield(handles, required)));
            testCase.verifyEqual(handles.theoreticalLine.DisplayName, ...
                '\phi = 0 deg theoretical singularity');
            testCase.verifyEqual(sort([handles.warningBoundaries.YData]), ...
                [-10, -10, 10, 10], 'AbsTol', 1e-12);
            testCase.verifyEqual(axesHandles(1).XLabel.String, '\theta [deg]');
            testCase.verifyEqual(axesHandles(1).YLabel.String, '\phi [deg]');
            testCase.verifyEqual(axesHandles(2).XLabel.String, 'x_G [mm]');
            testCase.verifyEqual(axesHandles(2).YLabel.String, 'y_G [mm]');
            testCase.verifyEqual(axesHandles(2).DataAspectRatio(1:2), [1, 1]);
            testCase.verifyEqual(axesHandles(1).CLim, axesHandles(2).CLim);
            testCase.verifyEqual(handles.taskScatter.CData(:), ...
                handles.riskData(testCase.Samples.theoreticalReachableMask));
            testCase.verifySize(handles.poseArrows, [3, 2]);
            testCase.verifySize(handles.representativeDirections, [2, 3]);
            testCase.verifySize(handles.sourceDirections, [2, 3]);
            testCase.verifyEqual(handles.representativeStatus, ...
                ["OK"; "ENGINEERING_WARNING"; "COUPLING_SINGULAR"]);
            selected = handles.representativeIndices;
            testCase.verifyTrue(testCase.Samples.safeUsableMask(selected(1)));
            testCase.verifyTrue(testCase.Samples.engineeringWarning(selected(2)));
            testCase.verifyFalse(testCase.Samples.exactCouplingSingular(selected(2)));
            testCase.verifyTrue(testCase.Samples.exactCouplingSingular(selected(3)));
            for index = 1:3
                testCase.verifyGreaterThan(abs(dot( ...
                    handles.representativeDirections(:, index), ...
                    handles.sourceDirections(:, index))), 1 - 1e-10);
                first = handles.poseArrows(index, 1);
                second = handles.poseArrows(index, 2);
                testCase.verifyEqual([first.UData; first.VData], ...
                    -[second.UData; second.VData], 'AbsTol', 1e-12);
                assembly = handles.assemblies(index);
                testCase.verifyNumElements(assembly.lowerLinks, 4);
                testCase.verifyNumElements(assembly.upperLinks, 4);
                testCase.verifyNumElements(assembly.alphaLinks, 4);
                testCase.verifyNumElements(assembly.betaLinks, 4);
            end
            testCase.verifyEqual(axesHandles(3).XLim, axesHandles(4).XLim);
            testCase.verifyEqual(axesHandles(4).XLim, axesHandles(5).XLim);
            testCase.verifyEqual(axesHandles(3).YLim, axesHandles(4).YLim);
            testCase.verifyEqual(axesHandles(4).YLim, axesHandles(5).YLim);
            titleText = string(arrayfun(@(item)item.Title.String, ...
                axesHandles(3:5), 'UniformOutput', false));
            testCase.verifyTrue(any(contains(lower(titleText), "regular")));
            testCase.verifyTrue(any(contains(lower(titleText), "warning")));
            testCase.verifyTrue(any(contains(lower(titleText), "exact")));
            testCase.verifyEqual(findall(groot, 'Type', 'figure'), beforeFigures);
            clear cleanup
        end

        function representativeLabelsDoNotOverlapAtAnalysisSize(testCase)
            [figureHandle, axesHandles, cleanup] = makeAnalysisAxes();
            duallink5.viz.plotCouplingSingularitySpace( ...
                testCase.Samples, axesHandles, struct());
            drawnow;

            verifyRepresentativeTextGaps( ...
                testCase, figureHandle, axesHandles);
            clear cleanup
        end

        function supportsPixelUnitRepresentativeAxes(testCase)
            [figureHandle, axesHandles, cleanup] = makeAnalysisAxes();
            set(axesHandles, 'Units', 'pixels');
            duallink5.viz.plotCouplingSingularitySpace( ...
                testCase.Samples, axesHandles, struct());
            drawnow;

            testCase.verifyEqual(string({axesHandles.Units}), ...
                repmat("pixels", 1, 5));
            verifyRepresentativeTextGaps( ...
                testCase, figureHandle, axesHandles);
            clear cleanup
        end

        function compactLayoutStaysInsideFigure(testCase)
            [figureHandle, axesHandles, cleanup] = makeCompactAnalysisAxes();
            duallink5.viz.plotCouplingSingularitySpace( ...
                testCase.Samples, axesHandles, struct());
            drawnow;

            verifyRepresentativeTextGaps( ...
                testCase, figureHandle, axesHandles);
            verifyRepresentativeCanvasBounds( ...
                testCase, figureHandle, axesHandles);
            clear cleanup
        end

        function usesVoronoiEdgesAndSafeColorLimits(testCase)
            samples = resample(testCase, [84, 85, 88], [-15, -7, 0, 10]);
            [~, axesHandles, cleanup] = makeAxes();
            handles = duallink5.viz.plotCouplingSingularitySpace( ...
                samples, axesHandles, struct());
            testCase.verifyEqual(handles.jointMap.XData(1, :), ...
                [83.5, 84.5, 86.5, 89.5], 'AbsTol', 1e-12);
            testCase.verifyEqual(handles.jointMap.YData(:, 1).', ...
                [-19, -11, -3.5, 5, 15], 'AbsTol', 1e-12);
            testCase.verifyTrue(all(isfinite(axesHandles(1).CLim)));
            testCase.verifyGreaterThan(diff(axesHandles(1).CLim), 0);

            constant = samples;
            constant.conditionNumber(constant.theoreticalReachableMask) = 4;
            duallink5.viz.plotCouplingSingularitySpace( ...
                constant, axesHandles, struct());
            testCase.verifyGreaterThan(diff(axesHandles(1).CLim), 0);
            infinite = constant;
            exactIndex = find(infinite.theoreticalReachableMask, 1);
            infinite.conditionNumber(exactIndex) = Inf;
            handles = duallink5.viz.plotCouplingSingularitySpace( ...
                infinite, axesHandles, struct());
            testCase.verifyTrue(all(isfinite(handles.riskData( ...
                infinite.theoreticalReachableMask))));
            testCase.verifyGreaterThan(diff(axesHandles(1).CLim), 0);
            clear cleanup
        end

        function supportsSingletonGrid(testCase)
            samples = resample(testCase, 85, 0);
            [~, axesHandles, cleanup] = makeAxes();
            handles = duallink5.viz.plotCouplingSingularitySpace( ...
                samples, axesHandles, struct());
            testCase.verifyTrue(isgraphics(handles.jointMap));
            testCase.verifyGreaterThan(diff(axesHandles(1).XLim), 0);
            testCase.verifyGreaterThan(diff(axesHandles(1).YLim), 0);
            testCase.verifySize(handles.poseArrows, [3, 2]);
            testCase.verifyEqual(handles.representativeStatus, ...
                repmat("COUPLING_SINGULAR", 3, 1));
            poseTitles = string(arrayfun(@(item)item.Title.String, ...
                axesHandles(3:5), 'UniformOutput', false));
            testCase.verifyFalse(any(contains(poseTitles, "Regular")));
            testCase.verifyFalse(any(contains( ...
                poseTitles, "Engineering warning")));
            testCase.verifyTrue(all(contains(poseTitles, "Exact")));
            clear cleanup
        end

        function rejectsGeometryMismatchBeforePlotting(testCase)
            customGeometry = testCase.Geometry;
            customGeometry.links.link1 = customGeometry.links.link1 + 1e-3;
            customGeometry = duallink5.model.validateGeometry(customGeometry);
            grid.theta = deg2rad([84, 85, 86]);
            grid.phi = deg2rad([-15, -10, -5, 0, 5, 10, 15]);
            customSamples = duallink5.singularity.sampleCouplingSpace( ...
                grid, customGeometry, struct( ...
                'collisionProfile', "none", 'topologyProfile', "none"));
            [~, axesHandles, cleanup] = makeAxes();
            axesHandles(1).NextPlot = 'replacechildren';
            beforeChildren = arrayfun(@(item)numel(item.Children), axesHandles);

            testCase.verifyError(@() ...
                duallink5.viz.plotCouplingSingularitySpace( ...
                customSamples, axesHandles, struct()), ...
                'duallink5:viz:InvalidCouplingPlotInput');
            testCase.verifyEqual( ...
                arrayfun(@(item)numel(item.Children), axesHandles), ...
                beforeChildren);
            testCase.verifyEqual(axesHandles(1).NextPlot, 'replacechildren');

            handles = duallink5.viz.plotCouplingSingularitySpace( ...
                customSamples, axesHandles, struct('geometry', customGeometry));
            first = handles.representativeIndices(1);
            result = duallink5.singularity.evaluateCoupling( ...
                handles.representativeQ(1, :), customGeometry, struct( ...
                'collisionProfile', "none", 'topologyProfile', "none"));
            testCase.verifyEqual(result.kinematics.points.G, ...
                [customSamples.x(first); customSamples.y(first)], ...
                'AbsTol', 1e-12);
            clear cleanup
        end

        function paintsTheoreticallyUnreachableCellsGray(testCase)
            geometry = testCase.Geometry;
            geometry.links.link3 = 0.01;
            geometry.links.link4 = 0.05;
            geometry = duallink5.model.validateGeometry(geometry);
            grid.theta = deg2rad(85);
            grid.phi = deg2rad([30, 90]);
            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, geometry, struct( ...
                'collisionProfile', "none", 'topologyProfile', "none"));
            testCase.assertEqual(samples.theoreticalReachableMask, ...
                [true, false]);
            [~, axesHandles, cleanup] = makeAxes();

            handles = duallink5.viz.plotCouplingSingularitySpace( ...
                samples, axesHandles, struct('geometry', geometry, ...
                'representativeQ', repmat(deg2rad([85, 30]), 3, 1)));

            testCase.verifyTrue(isgraphics(handles.unreachableMask));
            testCase.verifyEqual(handles.unreachableMask.FaceColor, ...
                [0.68, 0.68, 0.68], 'AbsTol', 1e-12);
            mask = handles.unreachableMask.CData;
            testCase.verifyEqual(mask(1, :), zeros(1, size(mask, 2)));
            testCase.verifyEqual(mask(end, :), ones(1, size(mask, 2)));
            testCase.verifyEqual(handles.unreachableMask.AlphaData, ...
                mask, 'AbsTol', 1e-12);
            testCase.verifyEqual(handles.unreachableMask.XData, ...
                handles.jointMap.XData, 'AbsTol', 1e-12);
            testCase.verifyEqual(handles.unreachableMask.YData, ...
                handles.jointMap.YData, 'AbsTol', 1e-12);
            testCase.verifyTrue(any(contains(string( ...
                handles.legend.String), "unreachable")));
            clear cleanup
        end

        function restoresNextPlotNormallyAndAfterPlotError(testCase)
            [figureHandle, axesHandles, cleanup] = makeAxes();
            modes = {'replacechildren', 'replace', 'add', ...
                'replacechildren', 'replace'};
            for index = 1:5
                axesHandles(index).NextPlot = modes{index};
            end
            duallink5.viz.plotCouplingSingularitySpace( ...
                testCase.Samples, axesHandles, struct());
            testCase.verifyEqual(string({axesHandles.NextPlot}), string(modes));

            incompatibleGeometry = testCase.Geometry;
            incompatibleGeometry.links.link1 = 1;
            incompatibleGeometry.links.link5 = 1;
            incompatibleGeometry = duallink5.model.validateGeometry( ...
                incompatibleGeometry);
            testCase.verifyError(@() ...
                duallink5.viz.plotCouplingSingularitySpace( ...
                testCase.Samples, axesHandles, ...
                struct('geometry', incompatibleGeometry)), ...
                'duallink5:viz:InvalidCouplingPlotInput');
            testCase.verifyEqual(string({axesHandles.NextPlot}), string(modes));
            clear cleanup
        end

        function rejectsBadAxesSamplesAndOptionsWithStableIdentifier(testCase)
            [figureHandle, axesHandles, cleanup] = makeAxes();
            deadAxes = axes(figureHandle); delete(deadAxes);
            badAxes = {axesHandles(1:4), ...
                [axesHandles(1:4), axesHandles(1)], deadAxes};
            for index = 1:numel(badAxes)
                verifyInvalid(testCase, testCase.Samples, ...
                    badAxes{index}, struct());
            end

            malformed = {rmfield(testCase.Samples, 'metadata'), ...
                setNested(testCase.Samples, 'phiGrid', zeros(3, 7)), ...
                setNested(testCase.Samples, 'safeUsableMask', true(3, 7)), ...
                setNested(testCase.Samples, 'conditionNumber', zeros(3, 7)), ...
                setNested(testCase.Samples, 'passiveDirection', zeros(2, 3, 6)), ...
                setNested(testCase.Samples, 'x', zeros(2)), ...
                setNested(testCase.Samples, 'reasonMap', cell(3, 7))};
            badMetadata = testCase.Samples;
            badMetadata.metadata = rmfield(badMetadata.metadata, 'geometryVersion');
            malformed{end + 1} = badMetadata;
            missingReferenceMetadata = testCase.Samples;
            missingReferenceMetadata.metadata = rmfield( ...
                missingReferenceMetadata.metadata, 'referenceSign');
            malformed{end + 1} = missingReferenceMetadata;
            badReference = testCase.Samples;
            badReference.metadata.couplingReference.topology = struct();
            malformed{end + 1} = badReference;
            badExact = testCase.Samples;
            badExact.exactCouplingSingular(1) = true;
            badExact.theoreticalReachableMask(1) = false;
            malformed{end + 1} = badExact;
            exactNotNear = testCase.Samples;
            exactNotNear.nearCouplingSingular( ...
                exactNotNear.exactCouplingSingular) = false;
            malformed{end + 1} = exactNotNear;
            badUnitDirection = testCase.Samples;
            directionIndex = find( ...
                badUnitDirection.theoreticalReachableMask, 1);
            directions = reshape(badUnitDirection.passiveDirection, 2, []);
            directions(:, directionIndex) = [2; 0];
            badUnitDirection.passiveDirection = reshape( ...
                directions, size(badUnitDirection.passiveDirection));
            malformed{end + 1} = badUnitDirection;
            wrongTensorShape = testCase.Samples;
            wrongTensorShape.passiveDirection = reshape( ...
                wrongTensorShape.passiveDirection, 2, []);
            malformed{end + 1} = wrongTensorShape;
            unreachableInf = testCase.Samples;
            badCoordinateIndex = find( ...
                unreachableInf.theoreticalReachableMask, 1);
            unreachableInf.x(badCoordinateIndex) = Inf;
            malformed{end + 1} = unreachableInf;
            emptyReason = testCase.Samples;
            emptyReason.reasonMap(badCoordinateIndex) = "";
            malformed{end + 1} = emptyReason;
            noTheory = testCase.Samples;
            noTheory.theoreticalReachableMask(:) = false;
            noTheory.mechanicallyValidMask(:) = false;
            noTheory.exactCouplingSingular(:) = false;
            noTheory.nearCouplingSingular(:) = false;
            noTheory.engineeringWarning(:) = false;
            noTheory.safeUsableMask(:) = false;
            noTheory.lowerClosureSingular(:) = false;
            noTheory.x(:) = NaN; noTheory.y(:) = NaN;
            noTheory.conditionNumber(:) = NaN;
            noTheory.couplingMargin(:) = NaN; noTheory.sigmaMin(:) = NaN;
            noTheory.reasonMap(:) = "UNREACHABLE_CLOSURE";
            noTheory.assemblyStatusMap(:) = "UNREACHABLE_CLOSURE";
            noTheory.passiveOrientationSign(:) = NaN;
            malformed{end + 1} = noTheory;
            bogusReason = testCase.Samples;
            bogusReason.reasonMap(1) = "BOGUS";
            malformed{end + 1} = bogusReason;
            bogusAssembly = testCase.Samples;
            bogusAssembly.assemblyStatusMap(1) = "BOGUS";
            malformed{end + 1} = bogusAssembly;
            for index = 1:numel(malformed)
                testCase.verifyError(@() ...
                    duallink5.viz.plotCouplingSingularitySpace( ...
                    malformed{index}, axesHandles, struct()), ...
                    'duallink5:viz:InvalidCouplingPlotInput', ...
                    sprintf('malformed sample case %d', index));
            end
            unreachableGeometry = testCase.Geometry;
            unreachableGeometry.links.link3 = 0.01;
            unreachableGeometry.links.link4 = 0.05;
            unreachableGeometry = duallink5.model.validateGeometry( ...
                unreachableGeometry);
            unreachableSamples = duallink5.singularity.sampleCouplingSpace( ...
                struct('theta', deg2rad(85), ...
                'phi', deg2rad([30, 90])), unreachableGeometry, struct( ...
                'collisionProfile', "none", 'topologyProfile', "none"));
            unreachableIndex = find( ...
                ~unreachableSamples.theoreticalReachableMask, 1);
            unreachableSamples.reasonMap(unreachableIndex) = "OK";
            unreachableSamples.assemblyStatusMap(unreachableIndex) = "OK";
            verifyInvalid(testCase, unreachableSamples, axesHandles, ...
                struct('geometry', unreachableGeometry, ...
                'representativeQ', repmat(deg2rad([85, 30]), 3, 1)));

            invalidOptions = {[], 1, repmat(struct(), 1, 2), ...
                struct('unknown', 1), struct('arrowLengthMm', 0), ...
                struct('arrowLengthMm', Inf), ...
                struct('representativeQ', zeros(2)), ...
                struct('geometry', struct())};
            for index = 1:numel(invalidOptions)
                verifyInvalid(testCase, testCase.Samples, axesHandles, ...
                    invalidOptions{index});
            end
            clear cleanup
        end
    end

    methods (Access = private)
        function samples = resample(testCase, thetaDegrees, phiDegrees)
            grid.theta = deg2rad(thetaDegrees);
            grid.phi = deg2rad(phiDegrees);
            samples = duallink5.singularity.sampleCouplingSpace( ...
                grid, testCase.Geometry, struct( ...
                'collisionProfile', "none", 'topologyProfile', "none"));
        end
    end
end

function [figureHandle, axesHandles, cleanup] = makeAxes()
figureHandle = figure('Visible', 'off');
cleanup = onCleanup(@()closeIfLive(figureHandle));
layout = tiledlayout(figureHandle, 2, 3);
axesHandles = gobjects(1, 5);
for index = 1:5
    axesHandles(index) = nexttile(layout);
end
end

function [figureHandle, axesHandles, cleanup] = makeAnalysisAxes()
figureHandle = figure('Visible', 'off', ...
    'Units', 'pixels', 'Position', [40, 40, 1600, 820]);
cleanup = onCleanup(@()closeIfLive(figureHandle));
positions = [ ...
    0.055, 0.10, 0.39, 0.82; ...
    0.50, 0.55, 0.20, 0.36; ...
    0.76, 0.69, 0.21, 0.25; ...
    0.76, 0.385, 0.21, 0.25; ...
    0.76, 0.08, 0.21, 0.25];
axesHandles = gobjects(1, 5);
for index = 1:5
    axesHandles(index) = axes(figureHandle, ...
        'Position', positions(index, :));
end
end

function [figureHandle, axesHandles, cleanup] = makeCompactAnalysisAxes()
[figureHandle, axesHandles, cleanup] = makeAnalysisAxes();
axesHandles(3).Position = [0.76, 0.58, 0.21, 0.25];
axesHandles(4).Position = [0.76, 0.30, 0.21, 0.25];
axesHandles(5).Position = [0.76, 0.05, 0.21, 0.25];
end

function verifyRepresentativeTextGaps( ...
        testCase, figureHandle, axesHandles)
minimumGapPixels = 6;
for index = 3:4
    upperLabel = figureTextBounds( ...
        axesHandles(index).XLabel);
    lowerTitle = figureTextBounds( ...
        axesHandles(index + 1).Title);
    actualGap = upperLabel(2) - ...
        (lowerTitle(2) + lowerTitle(4));
    testCase.verifyGreaterThanOrEqual( ...
        actualGap, minimumGapPixels, ...
        sprintf(['Representative axes %d and %d need ', ...
        'non-overlapping label/title extents.'], index, index + 1));
end
end

function verifyRepresentativeCanvasBounds( ...
        testCase, figureHandle, axesHandles)
figurePosition = getpixelposition(figureHandle);
for index = 3:5
    axesPosition = getpixelposition(axesHandles(index), true);
    labelBounds = figureTextBounds( ...
        axesHandles(index).XLabel);
    titleBounds = figureTextBounds( ...
        axesHandles(index).Title);
    testCase.verifyGreaterThanOrEqual(axesPosition(2), 1);
    testCase.verifyLessThanOrEqual( ...
        axesPosition(2) + axesPosition(4), figurePosition(4));
    testCase.verifyGreaterThanOrEqual(labelBounds(2), 1);
    testCase.verifyLessThanOrEqual( ...
        titleBounds(2) + titleBounds(4), figurePosition(4));
end
end

function bounds = figureTextBounds(textHandle)
originalUnits = textHandle.Units;
unitCleanup = onCleanup(@()set(textHandle, 'Units', originalUnits));
textHandle.Units = 'pixels';
bounds = textHandle.Extent;
axesPosition = getpixelposition(textHandle.Parent, true);
bounds(1:2) = bounds(1:2) + axesPosition(1:2);
clear unitCleanup
end

function closeIfLive(handle)
if isgraphics(handle, 'figure'), close(handle); end
end

function verifyInvalid(testCase, samples, axesHandles, options)
testCase.verifyError(@()duallink5.viz.plotCouplingSingularitySpace( ...
    samples, axesHandles, options), ...
    'duallink5:viz:InvalidCouplingPlotInput');
end

function value = setNested(value, name, replacement)
value.(name) = replacement;
end
