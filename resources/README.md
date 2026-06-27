# resources/ — surrogate model databases

The pipeline builds fast **surrogate models** for airfoils and propellers from
public reference databases. The raw databases are large and third-party, so they
are **git-ignored** — download them locally to regenerate a surrogate.

| Model | Generator (tracked) | Raw DB (git-ignored, download locally) | Source |
|---|---|---|---|
| Airfoil | `airfoil_surrogate_model/airfoil_surrogate_model.py` | `airfoil_shape_data_files/` | UIUC Airfoil Coordinates Database |
| Propeller | `propeller_surrogate_model/` (code) | `UIUC-propDB/`, `propeller_performance_data_files/` | UIUC Propeller Database |

Sources:
- UIUC Airfoil Coordinates Database — https://m-selig.ae.illinois.edu/ads/coord_database.html
- UIUC Propeller Data Site — https://m-selig.ae.illinois.edu/props/propDB.html

Once downloaded into the folders above, run the generator scripts to (re)build the
surrogate(s) the MATLAB pipeline loads at runtime.
