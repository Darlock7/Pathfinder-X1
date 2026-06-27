"""
Airfoil Aerodynamic Analysis via Neuralfoil

Generates aerodynamic polar data (Cl, Cd) for airfoils across Reynolds number
and angle of attack ranges using neuralfoil neural network surrogate models.

INSTALLATION:
  Install neuralfoil (neural network aerodynamic surrogate):
    pip install neuralfoil

  Alternatively, with conda:
    conda install -c conda-forge neuralfoil

INPUT DATA:
  Airfoil geometry files in selig format in airfoil_shape_data_files/ directory.
  Each file contains the airfoil coordinate points (x/c, y/c).

OUTPUT:
  Two visualization plots:
  1. Multiple airfoils at a fixed Reynolds number (compare designs)
  2. Single airfoil across multiple Reynolds number slices (analyze Re effects)

  Aerodynamic parameters extracted for each airfoil:
  - Cl0: zero-lift coefficient
  - Cl_alpha: lift curve slope (dCl/dalpha in linear region)
  - Cl_max: maximum lift coefficient
  - Cd_min: minimum drag coefficient
  - Cl_min_drag: lift coefficient at which Cd is minimum (typically near zero lift)
  - k_induced: parabolic drag coefficient (Cd_profile = Cd_min + k*(Cl - Cl_min_drag)^2)

USAGE:
  python airfoil_surrogate_model.py

TO USE IN CODE:
  from airfoil_surrogate_model import AirfoilSurrogateModel
  aero_model = AirfoilSurrogateModel('airfoil_shape_data_files/naca2412.dat')
  cl = aero_model.get_cl(alpha=5.0, Re=1e6)      # Get Cl at Re=1M, alpha=5deg
  cd = aero_model.get_cd(alpha=5.0, Re=1e6)      # Get Cd at Re=1M, alpha=5deg
  Cl0, Cl_alpha, Cl_max, Cd_min, Cl_min_drag, k_induced = aero_model.extract_parameters()
  # Returns aerodynamic parameters at median Re; can be unpacked directly
  # where Cd_min is minimum drag coefficient and Cl_min_drag is the Cl at which Cd is minimum
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple
from neuralfoil import get_aero_from_dat_file

# Named tuple for aerodynamic parameters (supports unpacking and attribute access)
AeroParams = namedtuple('AeroParams', ['Cl0', 'Cl_alpha', 'Cl_max', 'Cd_min', 'Cl_min_drag', 'k_induced'])


class AirfoilDataFile:
    """Load and parse airfoil geometry and aerodynamic data."""

    def __init__(self, file_path, re_values=None, alpha_values=None):
        """
        Initialize airfoil analyzer.

        Args:
            file_path: Path to selig format airfoil .dat file
            re_values: List of Reynolds numbers to analyze (default: [0.5e6, 1e6, 2e6, 5e6])
            alpha_values: Array of angles of attack (default: -10 to 20 degrees)
        """
        self.file_path = file_path
        self.airfoil_name = os.path.basename(file_path).replace('.dat', '')

        if re_values is None:
            self.re_values = [0.5e6, 1e6, 2e6, 5e6]
        else:
            self.re_values = re_values

        if alpha_values is None:
            self.alpha_values = np.linspace(-10, 20, 61)
        else:
            self.alpha_values = alpha_values

        print(f"Loading airfoil: {self.airfoil_name}")
        self.data = self._compute_aero_polars()

    def _compute_aero_polars(self):
        """Compute Cl and Cd across Re and alpha ranges using neuralfoil."""
        print(f"Computing aerodynamic polars for {self.airfoil_name}...")

        cl_values = []
        cd_values = []
        re_list = []
        alpha_list = []

        for re in self.re_values:
            for alpha in self.alpha_values:
                try:
                    result = get_aero_from_dat_file(
                        self.file_path,
                        alpha=alpha,
                        Re=re,
                        n_crit=9.0
                    )
                    cl_values.append(result['CL'])
                    cd_values.append(result['CD'])
                    re_list.append(re)
                    alpha_list.append(alpha)
                except Exception as e:
                    print(f"  Warning: Failed to compute aero at Re={re:.1e}, alpha={alpha:.1f}: {e}")
                    continue

        print(f"  Computed {len(cl_values)} points")

        return {
            "name": self.airfoil_name,
            "file": self.file_path,
            "Re": np.array(re_list),
            "alpha": np.array(alpha_list),
            "Cl": np.array(cl_values),
            "Cd": np.array(cd_values),
        }

    def get_loaded_data(self):
        return self.data

    def extract_parameters(self, re=None):
        """
        Extract key aerodynamic parameters from computed polars.

        Args:
            re: Reynolds number to extract parameters for. If None, uses median Re.
                Returns parameters as a named tuple: (Cl0, Cl_alpha, Cl_max, Cd_min, Cl_min_drag, k_induced)

        Returns:
            AeroParams namedtuple with fields: Cl0, Cl_alpha, Cl_max, Cd_min, Cl_min_drag, k_induced
            Can be unpacked: Cl0, Cl_alpha, Cl_max, Cd_min, Cl_min_drag, k_induced = extract_parameters()
        """
        data = self.data

        # Use median Reynolds number if not specified
        if re is None:
            re = np.median(self.re_values)

        # Find closest Re value in dataset
        re_closest_idx = np.argmin(np.abs(np.array(self.re_values) - re))
        re_actual = self.re_values[re_closest_idx]

        mask = np.isclose(data["Re"], re_actual)
        if np.sum(mask) < 3:
            raise ValueError(f"Insufficient data at Re={re_actual:.1e}")

        # Use boolean masking to get slices
        alpha_slice = data["alpha"][mask]
        cl_slice = data["Cl"][mask]
        cd_slice = data["Cd"][mask]

        # Ensure arrays are 1-D
        alpha_slice = np.atleast_1d(alpha_slice)
        cl_slice = np.atleast_1d(cl_slice)
        cd_slice = np.atleast_1d(cd_slice)

        # Sort by alpha for consistent interpolation
        sort_idx = np.argsort(alpha_slice)
        alpha_slice = alpha_slice[sort_idx]
        cl_slice = cl_slice[sort_idx]
        cd_slice = cd_slice[sort_idx]

        # Helper to extract scalars from numpy arrays
        def to_float(val):
            if hasattr(val, 'item'):
                return val.item()
            return float(val)

        # Lift curve slope (dCl/dalpha) and Cl0 via linear fit in linear region
        # Cl = Cl_alpha * alpha + Cl0  =>  Cl0 = Cl at alpha=0, alpha0 = -Cl0/Cl_alpha
        linear_region = (alpha_slice >= -2) & (alpha_slice <= 8)
        if np.sum(linear_region) > 2:
            alpha_linear = np.asarray(alpha_slice[linear_region]).flatten()
            cl_linear = np.asarray(cl_slice[linear_region]).flatten()
            z = np.polyfit(alpha_linear, cl_linear, 1)
            cl_alpha = to_float(z[0])
            cl0 = to_float(z[1])
        else:
            cl_alpha = to_float((cl_slice[-1] - cl_slice[0]) / (alpha_slice[-1] - alpha_slice[0]))
            cl0 = to_float(cl_slice[int(np.argmin(np.abs(alpha_slice)))])

        # Maximum Cl (minimum is not used in the returned tuple)
        cl_max = to_float(np.max(cl_slice))

        # Minimum drag
        cd_min = to_float(np.min(cd_slice))

        # Parabolic profile drag fit: Cd_profile = Cd_min + k*(Cl - Cl_min_drag)^2
        # Uses centered parabola (no linear term) to match typical airfoil behavior
        cd_min = to_float(np.min(cd_slice))
        cl_min_drag = to_float(cl_slice[int(np.argmin(cd_slice))])

        # Fit parabola centered at (Cl_min_drag, Cd_min)
        if np.sum(linear_region) > 3:
            cl_lin = np.asarray(cl_slice[linear_region]).flatten()
            cd_lin = np.asarray(cd_slice[linear_region]).flatten()
            # Shift Cl to center parabola and fit: Cd - Cd_min = k*(Cl - Cl_min_drag)^2
            cl_shifted = cl_lin - cl_min_drag
            cd_shifted = cd_lin - cd_min
            z_cd = np.polyfit(cl_shifted, cd_shifted, 2)
            k_ind = to_float(z_cd[0])  # Coefficient of (Cl - Cl_min_drag)^2
        else:
            k_ind = 0.0

        # Return as AeroParams namedtuple for direct unpacking
        return AeroParams(Cl0=cl0, Cl_alpha=cl_alpha, Cl_max=cl_max, Cd_min=cd_min, Cl_min_drag=cl_min_drag, k_induced=k_ind)

    def extract_parameters_all_re(self):
        """
        Extract aerodynamic parameters for ALL Reynolds numbers in dataset.

        Returns:
            Dictionary with one entry per Reynolds number, each containing:
            {Cl0, Cl_alpha, Cl_max, Cd_min, Cd_min, k_induced, ...}
        """
        data = self.data
        params = {}

        # For each Reynolds number, extract parameters
        for re in self.re_values:
            mask = np.isclose(data["Re"], re)
            if np.sum(mask) < 3:
                continue

            # Use boolean masking to get slices
            alpha_slice = data["alpha"][mask]
            cl_slice = data["Cl"][mask]
            cd_slice = data["Cd"][mask]

            # Ensure arrays are 1-D
            alpha_slice = np.atleast_1d(alpha_slice)
            cl_slice = np.atleast_1d(cl_slice)
            cd_slice = np.atleast_1d(cd_slice)

            # Sort by alpha for consistent interpolation
            sort_idx = np.argsort(alpha_slice)
            alpha_slice = alpha_slice[sort_idx]
            cl_slice = cl_slice[sort_idx]
            cd_slice = cd_slice[sort_idx]

            # Helper to extract scalars from numpy arrays
            def to_float(val):
                if hasattr(val, 'item'):
                    return val.item()
                return float(val)

            # Lift curve slope (dCl/dalpha) and Cl0 via linear fit in linear region
            # Cl = Cl_alpha * alpha + Cl0  =>  Cl0 = Cl at alpha=0, alpha0 = -Cl0/Cl_alpha
            linear_region = (alpha_slice >= -2) & (alpha_slice <= 8)
            if np.sum(linear_region) > 2:
                alpha_linear = np.asarray(alpha_slice[linear_region]).flatten()
                cl_linear = np.asarray(cl_slice[linear_region]).flatten()
                z = np.polyfit(alpha_linear, cl_linear, 1)
                cl_alpha = to_float(z[0])
                cl0 = to_float(z[1])
            else:
                cl_alpha = to_float((cl_slice[-1] - cl_slice[0]) / (alpha_slice[-1] - alpha_slice[0]))
                cl0 = to_float(cl_slice[int(np.argmin(np.abs(alpha_slice)))])

            # Zero-lift angle of attack: alpha where Cl = 0
            alpha0 = -cl0 / cl_alpha if cl_alpha != 0 else 0.0

            # Maximum and minimum Cl
            cl_max = to_float(np.max(cl_slice))
            cl_max_alpha = to_float(alpha_slice[int(np.argmax(cl_slice))])
            cl_min = to_float(np.min(cl_slice))
            cl_min_alpha = to_float(alpha_slice[int(np.argmin(cl_slice))])

            # Minimum drag
            cd_min = to_float(np.min(cd_slice))
            cd_min_alpha = to_float(alpha_slice[int(np.argmin(cd_slice))])

            # Parabolic profile drag fit: Cd_profile = Cd_min + k*(Cl - Cl_min_drag)^2
            # Uses centered parabola (no linear term) to match typical airfoil behavior
            cd_min_value = to_float(np.min(cd_slice))
            cl_at_cd_min = to_float(cl_slice[int(np.argmin(cd_slice))])

            # Fit parabola centered at (Cl_min_drag, Cd_min)
            if np.sum(linear_region) > 3:
                cl_lin = np.asarray(cl_slice[linear_region]).flatten()
                cd_lin = np.asarray(cd_slice[linear_region]).flatten()
                # Shift Cl to center parabola and fit: Cd - Cd_min = k*(Cl - Cl_min_drag)^2
                cl_shifted = cl_lin - cl_at_cd_min
                cd_shifted = cd_lin - cd_min_value
                z_cd = np.polyfit(cl_shifted, cd_shifted, 2)
                k_ind = to_float(z_cd[0])  # Coefficient of (Cl - Cl_min_drag)^2
            else:
                k_ind = 0.0

            cd0 = cd_min_value  # Cd_min is the minimum drag coefficient

            # Helper function to ensure scalar conversion
            def to_scalar(val):
                if hasattr(val, 'item'):
                    return val.item()
                return float(val) if not isinstance(val, (int, float)) else val

            params[f"Re_{re:.1e}"] = {
                "Re": float(re),
                "Cl0": to_scalar(cl0),
                "alpha0": to_scalar(alpha0),
                "Cl_alpha": to_scalar(cl_alpha),
                "Cl_max": to_scalar(cl_max),
                "Cl_max_alpha": to_scalar(cl_max_alpha),
                "Cl_min": to_scalar(cl_min),
                "Cl_min_alpha": to_scalar(cl_min_alpha),
                "Cd_min": to_scalar(cd_min),
                "Cd_min_alpha": to_scalar(cd_min_alpha),
                "Cl_min_drag": to_scalar(cl_at_cd_min),
                "k_induced": to_scalar(k_ind),
            }

        return params

    def get_data_slice(self, re):
        """Get all data points for a specific Reynolds number."""
        data = self.data
        mask = data["Re"] == re
        return {
            "alpha": data["alpha"][mask],
            "Cl": data["Cl"][mask],
            "Cd": data["Cd"][mask],
        }


class NeuralfoilSurrogateToolbox:
    """Prediction methods for aerodynamic coefficients."""

    @staticmethod
    def interpolate_polar(alpha_data, coeff_data, alpha_query):
        """Interpolate aerodynamic coefficient at requested alpha."""
        return np.interp(alpha_query, alpha_data, coeff_data)

    @staticmethod
    def predict_cl(data_slice, alpha):
        return NeuralfoilSurrogateToolbox.interpolate_polar(data_slice["alpha"], data_slice["Cl"], alpha)

    @staticmethod
    def predict_cd(data_slice, alpha):
        return NeuralfoilSurrogateToolbox.interpolate_polar(data_slice["alpha"], data_slice["Cd"], alpha)


class AirfoilSurrogateModel:
    """Load airfoil and generate neuralfoil aerodynamic surrogate model."""

    def __init__(self, airfoil_file_path: str, re_values=None, alpha_values=None):
        """
        Initialize airfoil analyzer.

        Args:
            airfoil_file_path: Path to selig format airfoil .dat file
            re_values: List of Reynolds numbers to analyze (default: [0.5e6, 1e6, 2e6, 5e6])
            alpha_values: Array of angles of attack (default: -10 to 20 degrees)
        """
        self.airfoil_file = AirfoilDataFile(airfoil_file_path, re_values, alpha_values)
        self.data = self.airfoil_file.get_loaded_data()

    def get_cl(self, alpha: float, Re: float) -> float:
        """Predict lift coefficient Cl at (alpha [deg], Re)."""
        data_slice = self.airfoil_file.get_data_slice(Re)
        return NeuralfoilSurrogateToolbox.predict_cl(data_slice, alpha)

    def get_cd(self, alpha: float, Re: float) -> float:
        """Predict drag coefficient Cd at (alpha [deg], Re)."""
        data_slice = self.airfoil_file.get_data_slice(Re)
        return NeuralfoilSurrogateToolbox.predict_cd(data_slice, alpha)

    def extract_parameters(self, re=None):
        """
        Extract key aerodynamic parameters from computed polars.

        Args:
            re: Reynolds number to extract parameters for. If None, uses median Re.

        Returns:
            AeroParams namedtuple with fields: (Cl0, Cl_alpha, Cl_max, Cd_min, Cd_min, k_induced)
            Can be unpacked: Cl0, Cl_alpha, Cl_max, Cd_min, Cd_min, k_induced = extract_parameters()
        """
        return self.airfoil_file.extract_parameters(re)

    def extract_parameters_all_re(self):
        """
        Extract aerodynamic parameters for ALL Reynolds numbers in dataset.

        Returns:
            Dictionary with one entry per Reynolds number.
        """
        return self.airfoil_file.extract_parameters_all_re()

    def get_data_slice(self, re):
        """Get all data points for a specific Reynolds number."""
        return self.airfoil_file.get_data_slice(re)

    @property
    def re_values(self):
        """Get list of Reynolds numbers in the dataset."""
        return self.airfoil_file.re_values


def _plot_subplot(ax, alpha_range, data_slice, color, label, coeff_key):
    """Plot polar curve on a subplot (alpha-based)."""
    coeff_data = data_slice[coeff_key]
    alpha_data = data_slice["alpha"]

    # Sort by alpha for smooth plotting
    sort_idx = np.argsort(alpha_data)
    alpha_sorted = alpha_data[sort_idx]
    coeff_sorted = coeff_data[sort_idx]

    ax.plot(alpha_sorted, coeff_sorted, color=color, linewidth=2, label=label, zorder=3)


def _plot_drag_polar(ax, data_slice, color, label):
    """Plot drag polar (Cl vs Cd) on a subplot."""
    cl_data = np.asarray(data_slice["Cl"]).flatten()
    cd_data = np.asarray(data_slice["Cd"]).flatten()
    alpha_data = np.asarray(data_slice["alpha"]).flatten()

    # Sort by alpha (data order) for smooth polar trace
    sort_idx = np.argsort(alpha_data)
    cl_sorted = cl_data[sort_idx]
    cd_sorted = cd_data[sort_idx]

    ax.plot(cd_sorted, cl_sorted, color=color, linewidth=2, label=label, zorder=2)


def _plot_cl_approximations(ax, params, re, alpha_range, color):
    """Plot linear Cl approximation and reference line segments on Cl vs alpha subplot."""
    p = params.get(f"Re_{re:.1e}")
    if not p:
        return

    cl_alpha = p["Cl_alpha"]
    cl0 = p["Cl0"]      # Cl at alpha = 0
    cl_max = p["Cl_max"]

    # Linear approximation: Cl = Cl0 + Cl_α * α
    cl_linear = cl0 + cl_alpha * alpha_range
    ax.plot(alpha_range, cl_linear, color=color, linestyle='--', linewidth=1.5,
            alpha=0.6, zorder=2, label='_nolegend_')

    # Cl0 reference line: short segment straddling the y-axis (alpha=0)
    alpha_cl0_segment = np.array([-2, 2])
    cl0_segment = np.full_like(alpha_cl0_segment, cl0, dtype=float)
    ax.plot(alpha_cl0_segment, cl0_segment, color=color, linestyle='--',
            linewidth=1, alpha=0.4, zorder=1, label='_nolegend_')

    # Cl_max reference line: from where linear fit intersects Cl_max onwards
    if cl_alpha != 0:
        alpha_cl_max_intersect = (cl_max - cl0) / cl_alpha
        alpha_cl_max_segment = np.array([alpha_cl_max_intersect, alpha_range[-1]])
        cl_max_segment = np.full_like(alpha_cl_max_segment, cl_max, dtype=float)
        ax.plot(alpha_cl_max_segment, cl_max_segment, color=color, linestyle='--',
                linewidth=1, alpha=0.4, zorder=1, label='_nolegend_')


def _plot_cd_approximation(ax, params, re, data_slice, color):
    """Plot parabolic Cd approximation on drag polar (x=Cd, y=Cl)."""
    p = params.get(f"Re_{re:.1e}")
    if not p:
        return

    cd_min = p["Cd_min"]
    cl_min_drag = p["Cl_min_drag"]  # Cl at which Cd is minimum
    k_ind = p["k_induced"]

    # Plot parabolic fit centered at min drag: Cd_profile = Cd_min + k*(Cl - Cl_min_drag)^2
    cl_range = np.linspace(-0.5, 2.0, 200)
    cd_quad = cd_min + k_ind * (cl_range - cl_min_drag)**2

    # x-axis is Cd, y-axis is Cl on the drag polar
    ax.plot(cd_quad, cl_range, color=color, linestyle='--', linewidth=2.5,
            alpha=0.9, zorder=4, label='_nolegend_')

    # Cd_min limit is a vertical line (Cd is on x-axis)
    ax.axvline(x=cd_min, color=color, linestyle=':', linewidth=1.5, alpha=0.7, zorder=2)


def _configure_subplot(ax, title, ylabel):
    """Configure subplot axes, labels, and grid."""
    ax.set_xlabel('Angle of Attack α (degrees)')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)


if __name__ == "__main__":
    # ===== EXAMPLE USAGE (Verify TO USE IN CODE section works) =====
    print("="*80)
    print("EXAMPLE: Direct airfoil analysis")
    print("="*80)
    try:
        aero_model = AirfoilSurrogateModel("airfoil_shape_data_files/naca2412.dat")
        cl_sample = aero_model.get_cl(alpha=5.0, Re=1e6)
        cd_sample = aero_model.get_cd(alpha=5.0, Re=1e6)
        Cl0, Cl_alpha, Cl_max, Cd_min, Cd_min, k_induced = aero_model.extract_parameters()
        print(f"Loaded NACA2412 airfoil")
        print(f"  At alpha=5.0°, Re=1.0e6:")
        print(f"    Cl (lift coeff)       = {cl_sample:.4f}")
        print(f"    Cd (drag coeff)       = {cd_sample:.5f}")
        print(f"  Extracted parameters (at median Reynolds number):")
        print(f"    Cl0 (zero-lift coeff)      = {Cl0:.4f}")
        print(f"    Cl_alpha (lift curve slope)= {Cl_alpha:.4f} /deg")
        print(f"    Cl_max (max lift)          = {Cl_max:.4f}")
        print(f"    Cd_min (profile drag)         = {Cd_min:.5f}")
        print(f"    Cd_min (min drag)          = {Cd_min:.5f}")
        print(f"    k_induced (ind. drag coeff)= {k_induced:.5f}")
    except Exception as e:
        print(f"Error in example: {e}")

    print("\n" + "="*80)
    print("GENERATING PLOTS")
    print("="*80 + "\n")

    DATA_DIR = "airfoil_shape_data_files"
    AIRFOIL_FILES = [
        "e423.dat",
        "s1223.dat",
        "naca0010.dat",
        "naca2412.dat",
    ]

    try:
        # ===== PLOT 1: Multiple airfoils at a fixed Reynolds number =====
        colors = plt.cm.tab10(np.linspace(0, 1, len(AIRFOIL_FILES)))
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        fig.suptitle("Airfoil Aerodynamic Polars: Multiple Designs (Re = 1.0M)", fontsize=14)

        fixed_re = 1e6
        alpha_range = np.linspace(-10, 20, 100)

        for idx, airfoil_file in enumerate(AIRFOIL_FILES):
            print(f"\nProcessing {airfoil_file}...")
            analyzer = AirfoilSurrogateModel(os.path.join(DATA_DIR, airfoil_file))
            data_slice = analyzer.get_data_slice(fixed_re)

            if len(data_slice["alpha"]) == 0:
                print(f"  Skipping {airfoil_file}: no data at Re={fixed_re:.1e}")
                continue

            airfoil_name = airfoil_file.replace(".dat", "")
            params_all_re = analyzer.extract_parameters_all_re()

            # Plot Cl vs alpha
            _plot_subplot(axes[0], data_slice["alpha"], data_slice, colors[idx],
                         airfoil_name, "Cl")
            _plot_cl_approximations(axes[0], params_all_re, fixed_re, alpha_range, colors[idx])

            # Plot drag polar (Cl vs Cd)
            _plot_drag_polar(axes[1], data_slice, colors[idx], airfoil_name)
            _plot_cd_approximation(axes[1], params_all_re, fixed_re, data_slice, colors[idx])

        _configure_subplot(axes[0], "$C_l$ vs α", "$C_l$ (Lift Coefficient)")
        axes[1].set_xlabel('$C_d$ (Drag Coefficient)')
        axes[1].set_ylabel('$C_l$ (Lift Coefficient)')
        axes[1].set_title('Drag Polar')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='best', fontsize=9)
        axes[1].set_xlim(0, 0.05)
        axes[1].set_ylim(-0.25, 2.5)

        plt.tight_layout()
        plt.show()

        # ===== PLOT 2: Single airfoil at multiple Reynolds numbers =====
        print("\n\nGenerating second plot: single airfoil at multiple Reynolds numbers...")

        analyzer = AirfoilSurrogateModel(os.path.join(DATA_DIR, AIRFOIL_FILES[0]))
        airfoil_name = AIRFOIL_FILES[0].replace(".dat", "")
        params_all_re = analyzer.extract_parameters_all_re()

        # Select 4 Reynolds numbers
        re_slices = sorted(analyzer.re_values)
        if len(re_slices) > 4:
            re_indices = np.linspace(0, len(re_slices) - 1, 4, dtype=int)
            re_slices = [re_slices[i] for i in re_indices]

        re_colors = plt.cm.viridis(np.linspace(0, 1, len(re_slices)))

        fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
        fig2.suptitle(f"Airfoil Aerodynamic Polars: {airfoil_name} at Multiple Re", fontsize=14)

        for re_idx, re in enumerate(re_slices):
            data_slice = analyzer.get_data_slice(re)

            if len(data_slice["alpha"]) == 0:
                print(f"  Skipping Re={re:.1e}: no data")
                continue

            re_label = f"Re = {re:.1e}"

            _plot_subplot(axes2[0], data_slice["alpha"], data_slice,
                         re_colors[re_idx], re_label, "Cl")
            _plot_cl_approximations(axes2[0], params_all_re, re, alpha_range, re_colors[re_idx])

            _plot_drag_polar(axes2[1], data_slice, re_colors[re_idx], re_label)
            _plot_cd_approximation(axes2[1], params_all_re, re, data_slice, re_colors[re_idx])

        _configure_subplot(axes2[0], "$C_l$ vs α", "$C_l$ (Lift Coefficient)")
        axes2[1].set_xlabel('$C_d$ (Drag Coefficient)')
        axes2[1].set_ylabel('$C_l$ (Lift Coefficient)')
        axes2[1].set_title('Drag Polar')
        axes2[1].grid(True, alpha=0.3)
        axes2[1].legend(loc='best', fontsize=9)
        axes2[1].set_xlim(0, 0.05)
        axes2[1].set_ylim(-0.25, 2.5)

        plt.tight_layout()
        plt.show()

        # ===== Extract and print aerodynamic parameters =====
        print("\n\n" + "="*80)
        print("EXTRACTED AERODYNAMIC PARAMETERS")
        print("="*80)

        for airfoil_file in AIRFOIL_FILES:
            analyzer = AirfoilSurrogateModel(os.path.join(DATA_DIR, airfoil_file))
            params = analyzer.extract_parameters_all_re()

            print(f"\n{airfoil_file.replace('.dat', '').upper()}")
            print("-" * 80)

            for re_key, re_params in sorted(params.items()):
                re = re_params["Re"]
                print(f"\n  Re = {re:.2e}:")
                print(f"    Cl0 (zero-lift coeff):        {re_params['Cl0']:7.4f}")
                print(f"    α0 (zero-lift angle):         {re_params['alpha0']:7.2f}°")
                print(f"    Cl_α (lift curve slope):      {re_params['Cl_alpha']:7.4f} /deg")
                print(f"    Cl_max (max lift):            {re_params['Cl_max']:7.4f}  @ α = {re_params['Cl_max_alpha']:6.2f}°")
                print(f"    Cl_min (min lift):            {re_params['Cl_min']:7.4f}  @ α = {re_params['Cl_min_alpha']:6.2f}°")
                print(f"    Cd_min (min drag):            {re_params['Cd_min']:7.5f}  @ α = {re_params['Cd_min_alpha']:6.2f}°")
                print(f"    Cd_min (profile drag @ Cl=0):    {re_params['Cd_min']:7.5f}")
                print(f"    k (induced drag coeff):       {re_params['k_induced']:7.5f}")

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
