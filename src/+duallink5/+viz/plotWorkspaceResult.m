function handles = plotWorkspaceResult(samples, result, axesHandle)
holdState = ishold(axesHandle);
hold(axesHandle, 'on');
handles.samples = scatter(axesHandle, samples.x(samples.validMask), ...
    samples.y(samples.validMask), 8, '.');
handles.boundary = plot(result.boundaryShape, 'Parent', axesHandle, ...
    'FaceAlpha', 0.08, 'EdgeColor', [0 0.45 0.74]);
rectangleInfo = result.maxRectangle;
handles.rectangle = gobjects(0);
if rectangleInfo.areaCells > 0
    bounds = rectangleInfo.bounds;
    handles.rectangle = rectangle(axesHandle, 'Position', ...
        [bounds(1), bounds(3), bounds(2) - bounds(1), ...
        bounds(4) - bounds(3)], 'EdgeColor', 'r', 'LineWidth', 2);
end
axis(axesHandle, 'equal');
xlabel(axesHandle, 'x [m]');
ylabel(axesHandle, 'y [m]');
if ~holdState
    hold(axesHandle, 'off');
end
end
