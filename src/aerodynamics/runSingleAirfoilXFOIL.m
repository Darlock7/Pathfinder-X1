function foilOut = runSingleAirfoilXFOIL(foilName, Re, alpha_deg, opts)
% runSingleAirfoilXFOIL
%
% Purpose:
%   Thin wrapper around existing airfoilAnalysisXFOIL(...) so that one foil
%   can be run at one Reynolds number.
%
% Inputs:
%   foilName    : char/string, e.g. 'e222.dat'
%   Re          : scalar Reynolds number [-]
%   alpha_deg   : vector [deg]
%   opts        : struct with fields:
%                   .xfoilFolder
%                   .airfoilFolder
%                   .Mach
%                   .maxIter
%                   .cleanupFiles
%                   .printSummary
%
% Output:
%   foilOut     : struct containing one-airfoil polar + derived metrics

    arguments
        foilName
        Re (1,1) double {mustBePositive}
        alpha_deg (:,1) double
        opts struct
    end

    if ~isfield(opts, 'xfoilFolder') || isempty(opts.xfoilFolder)
        error('runSingleAirfoilXFOIL:MissingXFOILFolder', ...
            'opts.xfoilFolder must be provided.');
    end
    if ~isfield(opts, 'airfoilFolder') || isempty(opts.airfoilFolder)
        error('runSingleAirfoilXFOIL:MissingAirfoilFolder', ...
            'opts.airfoilFolder must be provided.');
    end
    if ~isfield(opts, 'Mach') || isempty(opts.Mach)
        opts.Mach = 0.0;
    end
    if ~isfield(opts, 'maxIter') || isempty(opts.maxIter)
        opts.maxIter = 150;
    end
    if ~isfield(opts, 'cleanupFiles') || isempty(opts.cleanupFiles)
        opts.cleanupFiles = true;
    end
    if ~isfield(opts, 'printSummary') || isempty(opts.printSummary)
        opts.printSummary = false;
    end

    foilName = char(string(foilName));
    isNACA = startsWith(upper(strtrim(foilName)), 'NACA');

    copiedLocal = false;
    srcFile = fullfile(opts.airfoilFolder, foilName);
    dstFile = fullfile(pwd, foilName);

    if ~isNACA
        if exist(srcFile, 'file') ~= 2
            error('runSingleAirfoilXFOIL:FileNotFound', ...
                'Could not find airfoil file:\n%s', srcFile);
        end

        if exist(dstFile, 'file') ~= 2
            copyfile(srcFile, dstFile);
            copiedLocal = true;
        end
    end

    airfoilIn = struct();
    airfoilIn.rootFoil = foilName;
    airfoilIn.tipFoil  = foilName;

    airfoilIn.Re_root = Re;
    airfoilIn.Re_tip  = Re;

    airfoilIn.alpha_deg    = alpha_deg(:).';
    airfoilIn.xfoilFolder  = opts.xfoilFolder;
    airfoilIn.Mach         = opts.Mach;
    airfoilIn.maxIter      = opts.maxIter;
    airfoilIn.cleanupFiles = opts.cleanupFiles;
    airfoilIn.printSummary = opts.printSummary;

    tmp = airfoilAnalysisXFOIL(airfoilIn);

    if ~isfield(tmp, 'root')
        error('runSingleAirfoilXFOIL:MissingRoot', ...
            'Expected output field "root" was not found.');
    end

    foilOut = tmp.root;

    if copiedLocal && exist(dstFile, 'file') == 2
        delete(dstFile);
    end
end