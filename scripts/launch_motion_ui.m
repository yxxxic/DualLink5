exampleDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(exampleDir);
run(fullfile(projectRoot, 'startup.m'));

dl5MotionApp = duallink5.launchMotionApp();
