function idx = findAirfoilInDB(airfoilDB, foilName)
% findAirfoilInDB
%
% Purpose:
%   Return the index of a requested foil in the airfoil surrogate database.

    arguments
        airfoilDB struct
        foilName
    end

    target = string(foilName);
    names = string({airfoilDB.foils.name});

    idx = find(strcmpi(names, target), 1);

    if isempty(idx)
        error('findAirfoilInDB:UnknownFoil', ...
            'Airfoil "%s" was not found in the surrogate database.', char(target));
    end
end