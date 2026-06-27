% setup_claude.m
% Run this once inside MATLAB to configure Claude Code for your machine.
% Works on Mac and Windows automatically.

matlabBinPath = fullfile(matlabroot, 'bin');

if ispc
    pathSeparator = ';';
    systemPaths   = 'C:\Windows\System32;C:\Windows';
else
    pathSeparator = ':';
    systemPaths   = '/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin';
end

fullPath = [matlabBinPath, pathSeparator, systemPaths];

settingsDir  = fullfile(pwd, '.claude');
settingsFile = fullfile(settingsDir, 'settings.local.json');

if ~exist(settingsDir, 'dir')
    mkdir(settingsDir);
end

json = sprintf('{\n  "env": {\n    "PATH": "%s"\n  }\n}\n', fullPath);

fid = fopen(settingsFile, 'w');
if fid == -1
    error('Could not write settings file. Check folder permissions.');
end
fprintf(fid, '%s', json);
fclose(fid);

fprintf('Claude configured successfully.\n');
fprintf('MATLAB path: %s\n', matlabBinPath);
fprintf('Settings written to: %s\n', settingsFile);
