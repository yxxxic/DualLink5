function exportFigure(figureHandle, fileName, options)
if nargin < 3
    options = struct();
end
if ~(isscalar(figureHandle) && isgraphics(figureHandle, 'figure'))
    error('duallink5:viz:InvalidGraphicsHandle', ...
        'figureHandle must be a live scalar figure handle.');
end
resolution = validateOptions(options);
fileName = validateFileName(fileName);
exportgraphics(figureHandle, fileName, 'Resolution', resolution);
end

function resolution = validateOptions(options)
if ~isstruct(options) || ~isscalar(options)
    invalidOptions();
end
if isfield(options, 'resolution')
    resolution = options.resolution;
else
    resolution = 300;
end
if ~isnumeric(resolution) || ~isreal(resolution) || ...
        ~isscalar(resolution) || ~isfinite(resolution) || ...
        resolution <= 0 || resolution ~= fix(resolution)
    invalidOptions();
end
resolution = full(double(resolution));
end

function fileName = validateFileName(fileName)
validString = isstring(fileName) && isscalar(fileName) && ...
    ~ismissing(fileName) && strlength(fileName) > 0;
validCharacter = ischar(fileName) && isrow(fileName) && ...
    ~isempty(fileName);
if ~(validString || validCharacter)
    invalidFile();
end
fileName = char(fileName);
[parentPath, baseName, extension] = fileparts(fileName);
supportedExtensions = {'.png', '.jpg', '.jpeg', '.tif', '.tiff', ...
    '.pdf', '.eps', '.svg'};
if isempty(baseName) || ...
        ~ismember(lower(extension), supportedExtensions) || ...
        (~isempty(parentPath) && ~isfolder(parentPath))
    invalidFile();
end
end

function invalidOptions()
error('duallink5:viz:InvalidExportOptions', ...
    'options.resolution must be a positive integer.');
end

function invalidFile()
error('duallink5:viz:InvalidExportFile', ...
    'fileName must use a supported extension and existing parent path.');
end
