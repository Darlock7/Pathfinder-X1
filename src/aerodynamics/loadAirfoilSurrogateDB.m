function airfoilDB = loadAirfoilSurrogateDB(repoRoot)
% loadAirfoilSurrogateDB
%
% Purpose:
%   Load the saved surrogate database from repoRoot/data/models/airfoilDB.mat

    arguments
        repoRoot char
    end

    dbFile = fullfile(repoRoot, 'data', 'models', 'airfoilDB.mat');

    if exist(dbFile, 'file') ~= 2
        error('loadAirfoilSurrogateDB:MissingDB', ...
            ['Could not find airfoil surrogate database:\n%s\n' ...
             'Run buildAirfoilSurrogates.m first.'], dbFile);
    end

    S = load(dbFile, 'airfoilDB');

    if ~isfield(S, 'airfoilDB')
        error('loadAirfoilSurrogateDB:BadFile', ...
            'File exists but does not contain variable "airfoilDB".');
    end

    airfoilDB = S.airfoilDB;
end