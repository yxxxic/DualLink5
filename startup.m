addDualLink5SourcePath(mfilename('fullpath'));

function addDualLink5SourcePath(startupFile)
projectRoot = fileparts(startupFile);
sourceDir = fullfile(projectRoot, 'src');
pathEntries = strsplit(path, pathsep);
if ~any(strcmp(pathEntries, sourceDir))
    addpath(sourceDir);
end
end
