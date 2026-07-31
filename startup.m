projectRoot = fileparts(mfilename('fullpath'));
sourceDir = fullfile(projectRoot, 'src');
pathEntries = strsplit(path, pathsep);
if ~any(strcmp(pathEntries, sourceDir))
    addpath(sourceDir);
end
clear projectRoot sourceDir pathEntries
