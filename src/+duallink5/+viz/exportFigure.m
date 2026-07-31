function exportFigure(figureHandle, fileName, options)
if nargin < 3
    options = struct();
end
resolution = getOption(options, 'resolution', 300);
exportgraphics(figureHandle, fileName, 'Resolution', resolution);
end

function value = getOption(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end
