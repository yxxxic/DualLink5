function handles = plotMechanism(pose, axesHandle, options)
if nargin < 3
    options = struct();
end
lineWidth = getOption(options, 'lineWidth', 2);
showLabels = getOption(options, 'showLabels', true);
p = pose.points;

holdState = ishold(axesHandle);
hold(axesHandle, 'on');
segments = {p.A, p.E; p.B, p.C; p.C, p.D; p.E, p.D; p.A, p.B};
handles.links = gobjects(5, 1);
for index = 1:5
    startPoint = segments{index, 1};
    endPoint = segments{index, 2};
    handles.links(index) = plot(axesHandle, ...
        [startPoint(1), endPoint(1)], [startPoint(2), endPoint(2)], ...
        '-o', 'LineWidth', lineWidth);
end
handles.sharedCenter = plot(axesHandle, pose.sharedLink.center(1), ...
    pose.sharedLink.center(2), 'kp', 'MarkerFaceColor', 'y');

if showLabels
    names = {'A', 'B', 'C', 'D', 'E', 'G'};
    handles.labels = gobjects(numel(names), 1);
    for index = 1:numel(names)
        point = p.(names{index});
        handles.labels(index) = text(axesHandle, point(1), point(2), ...
            ['  ', names{index}], 'Interpreter', 'none');
    end
end
axis(axesHandle, 'equal');
xlabel(axesHandle, 'x [m]');
ylabel(axesHandle, 'y [m]');
if ~holdState
    hold(axesHandle, 'off');
end
end

function value = getOption(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end
