%Path Setup
clc; clear; close all;
repoRoot = fileparts(mfilename('fullpath'));

allPaths = strsplit(genpath(repoRoot), pathsep);
keepPaths = {};

for k = 1:numel(allPaths)
    p = allPaths{k};
    if isempty(p)
        continue;
    end
    if contains(p, [filesep '.git']) || endsWith(p, [filesep '.git'])
        continue;
    end
    keepPaths{end+1} = p;
end

addpath(keepPaths{:});
disp('Project paths loaded.');
disp(['Repo root: ', repoRoot]);

%test