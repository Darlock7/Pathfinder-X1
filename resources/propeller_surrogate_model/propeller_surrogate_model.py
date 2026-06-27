"""
APC Propeller Surrogate Model Generator

Builds 2D RMTB surrogate models of APC propeller performance data. Models predict
thrust coefficient (Ct), power coefficient (Cp), torque coefficient (Cq), and
efficiency (Pe) as functions of RPM (n) and advance ratio (J).
The surrogate models are trained on data from APC Propellers.
Reference: https://smt.readthedocs.io/en/latest/_src_docs/surrogate_models/rmts.html

INSTALLATION:
  Install the SMT (Surrogate Modeling Toolbox) library:
    pip install smt

  Alternatively, with conda:
    conda install -c conda-forge smt

INPUT DATA:
  APC performance files in propeller_performance_data_files/ directory (PER3_*.dat format).
  Each file contains multiple RPM sections with J vs Ct/Cp/Pe data.

OUTPUT:
  Two visualization plots:
  1. Multiple propellers at median RPM (compare designs)
  2. Single propeller at multiple RPM slices (analyze RPM effects)

USAGE:
  python propeller_surrogate_model.py

TO USE THE MODELS IN CODE:
  from propeller_surrogate_model import PropellerSurrogateModel
  prop_model = PropellerSurrogateModel('propeller_performance_data_files/PER3_10x10E.dat')
  ct = prop_model.get_ct(n=2000, J=0.5)    # Predict Ct at 2000 RPM, J=0.5
  cp = prop_model.get_cp(n=2000, J=0.5)    # Predict Cp at 2000 RPM, J=0.5
  cq = prop_model.get_cq(n=2000, J=0.5)    # Predict Cq at 2000 RPM, J=0.5
  pe = prop_model.get_pe(n=2000, J=0.5)    # Predict efficiency Pe at 2000 RPM, J=0.5
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from smt.surrogate_models import RMTB


class ApcDataFile:
    """Load and parse APC propeller performance data files."""

    def __init__(self, file_path):
        self.file_path = file_path
        print(f"Loading APC prop data from {file_path}...")
        self.data = self._parse_data_file(file_path)

    def _parse_data_file(self, file_path):
        """Extract N, J, Ct, Cp, Pe from all RPM sections in the file."""
        N_values = []
        J_values = []
        Ct_values = []
        Cp_values = []
        Pe_values = []

        with open(file_path, 'r') as f:
            lines = f.readlines()

        in_data_section = False
        current_rpm = None

        for i, line in enumerate(lines):
            if 'PROP RPM' in line:
                try:
                    parts = line.split('=')
                    current_rpm = float(parts[1].strip())
                    in_data_section = False
                except (ValueError, IndexError):
                    pass

            if not in_data_section and current_rpm is not None:
                if 'V' in line and 'J' in line and 'Ct' in line and 'Cp' in line:
                    in_data_section = True
                continue

            if in_data_section and current_rpm is not None:
                if '(mph)' in line or '(Adv Ratio)' in line:
                    continue

                parts = line.split()
                if len(parts) >= 5:
                    try:
                        if parts[0] == 'V' or '=' in line or not parts[0][0].isdigit():
                            continue
                        J = float(parts[1])
                        Pe = float(parts[2])
                        Ct = float(parts[3])
                        Cp = float(parts[4])
                        N_values.append(current_rpm)
                        J_values.append(J)
                        Ct_values.append(Ct)
                        Cp_values.append(Cp)
                        Pe_values.append(Pe)
                    except (ValueError, IndexError):
                        continue

        Cq_values = np.array(Cp_values) / (2 * np.pi)

        return {
            "file": file_path,
            "status": "loaded",
            "N": np.array(N_values),
            "J": np.array(J_values),
            "Ct": np.array(Ct_values),
            "Cp": np.array(Cp_values),
            "Cq": Cq_values,
            "Pe": np.array(Pe_values),
        }

    def get_loaded_data(self):
        return self.data


class RmtbSurrogateToolbox:
    """RMTB surrogate model builder and predictor."""

    @staticmethod
    def generate_rmtc_model(data):
        """Train RMTB model: (N/1000, J) -> (Ct, Cp, Cq, Pe)."""
        print("Generating RMTB model...")

        N_norm = data["N"] / 1000.0
        J_norm = data["J"]
        xt = np.column_stack([N_norm, J_norm])
        yt = np.column_stack([data["Ct"], data["Cp"], data["Cq"], data["Pe"]])

        xlimits = np.array([[N_norm.min(), N_norm.max()], [J_norm.min(), J_norm.max()]])
        sm = RMTB(
            xlimits=xlimits,
            num_ctrl_pts=20,
            energy_weight=1e-8,
            regularization_weight=0e-15,
            nonlinear_maxiter=10,
        )

        sm.set_training_values(xt, yt)
        sm.train()

        return {
            "model_type": "RMTC",
            "status": "generated",
            "metadata": data["file"],
            "rmtc": sm,
            "N_data": data["N"],
            "J_data": data["J"],
            "Ct_data": data["Ct"],
            "Cp_data": data["Cp"],
            "Cq_data": data["Cq"],
            "Pe_data": data["Pe"],
        }

    @staticmethod
    def _predict(model, n, J, output_idx):
        """Predict output_idx-th output (0=Ct, 1=Cp, 2=Cq, 3=Pe)."""
        if model and model["model_type"] == "RMTC":
            N_scaled = n / 1000.0
            return float(model["rmtc"].predict_values(np.array([[N_scaled, J]]))[0, output_idx])
        return None

    @staticmethod
    def predict_ct(model, n, J):
        return RmtbSurrogateToolbox._predict(model, n, J, 0)

    @staticmethod
    def predict_cp(model, n, J):
        return RmtbSurrogateToolbox._predict(model, n, J, 1)

    @staticmethod
    def predict_cq(model, n, J):
        return RmtbSurrogateToolbox._predict(model, n, J, 2)

    @staticmethod
    def predict_pe(model, n, J):
        return RmtbSurrogateToolbox._predict(model, n, J, 3)


class PropellerSurrogateModel:
    """Load APC data and generate RMTB surrogate model for propeller performance."""

    def __init__(self, apc_data_file_name: str):
        self.data_file_name = apc_data_file_name
        self.apc_data = ApcDataFile(apc_data_file_name)
        self.rmtc_model = RmtbSurrogateToolbox.generate_rmtc_model(self.apc_data.get_loaded_data())

    def get_ct(self, n: float, J: float) -> float | None:
        """Predict thrust coefficient Ct at (n RPM, J)."""
        return RmtbSurrogateToolbox.predict_ct(self.rmtc_model, n, J)

    def get_cp(self, n: float, J: float) -> float | None:
        """Predict power coefficient Cp at (n RPM, J)."""
        return RmtbSurrogateToolbox.predict_cp(self.rmtc_model, n, J)

    def get_cq(self, n: float, J: float) -> float | None:
        """Predict torque coefficient Cq at (n RPM, J)."""
        return RmtbSurrogateToolbox.predict_cq(self.rmtc_model, n, J)

    def get_pe(self, n: float, J: float) -> float | None:
        """Predict efficiency Pe at (n RPM, J)."""
        return RmtbSurrogateToolbox.predict_pe(self.rmtc_model, n, J)


def _plot_subplot(ax, J_RANGE, generator, N_SLICE, J_data, data_values, color, label, pred_func):
    """Plot interpolant curve and training data markers on a subplot."""
    Y_vs_J = np.array([pred_func(N_SLICE, J) for J in J_RANGE])
    ax.plot(J_RANGE, Y_vs_J, color=color, linewidth=2, zorder=3)
    ax.scatter(J_data, data_values, color=color, s=100, marker='o',
              edgecolors='black', linewidth=1, label=label, zorder=4, alpha=0.8)


def _configure_subplot(ax, title, ylabel):
    """Configure subplot axes, labels, and grid."""
    ax.set_xlabel('J (Advance Ratio)')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')


if __name__ == "__main__":
    # ===== EXAMPLE USAGE (Verify TO USE IN CODE section works) =====
    print("="*80)
    print("EXAMPLE: Direct propeller analysis")
    print("="*80)
    try:
        prop_model = PropellerSurrogateModel("propeller_performance_data_files/PER3_10x10E.dat")
        ct_sample = prop_model.get_ct(n=2000, J=0.5)
        cp_sample = prop_model.get_cp(n=2000, J=0.5)
        cq_sample = prop_model.get_cq(n=2000, J=0.5)
        pe_sample = prop_model.get_pe(n=2000, J=0.5)
        print(f"Loaded PER3_10x10E propeller")
        print(f"  At n=2000 RPM, J=0.5:")
        print(f"    Ct (thrust coeff)  = {ct_sample:.4f}")
        print(f"    Cp (power coeff)   = {cp_sample:.4f}")
        print(f"    Cq (torque coeff)  = {cq_sample:.4f}")
        print(f"    Pe (efficiency)    = {pe_sample:.4f}")
    except Exception as e:
        print(f"Error in example: {e}")

    print("\n" + "="*80)
    print("GENERATING PLOTS")
    print("="*80 + "\n")

    DATA_DIR = "propeller_performance_data_files"
    PROPELLER_FILES = [
        "PER3_9x45E.dat",
        "PER3_9x47SF.dat",
        "PER3_9x6E.dat",
        "PER3_10x47SF.dat",
    ]

    try:
        # ===== PLOT 1: Multiple propellers at median RPM =====
        colors = plt.cm.tab10(np.linspace(0, 1, len(PROPELLER_FILES)))
        fig, axes = plt.subplots(2, 2, figsize=(16, 10))
        fig.suptitle("Propeller Surrogate Models: Multiple Designs", fontsize=16)

        for idx, data_file in enumerate(PROPELLER_FILES):
            print(f"\nProcessing {data_file}...")
            gen = PropellerSurrogateModel(os.path.join(DATA_DIR, data_file))
            data = gen.apc_data.get_loaded_data()

            J_RANGE = np.linspace(data["J"].min(), data["J"].max(), 100)

            median_N = np.median(data["N"])
            unique_N = np.unique(data["N"])
            N_SLICE = unique_N[np.argmin(np.abs(unique_N - median_N))]

            mask = np.abs(data["N"] - N_SLICE) < 1.0
            J_slice = data["J"][mask]
            prop_name = data_file.replace("PER3_", "").replace(".dat", "")

            # Plot Ct
            _plot_subplot(axes[0, 0], J_RANGE, gen, N_SLICE, J_slice, data["Ct"][mask],
                         colors[idx], prop_name, gen.get_ct)

            # Plot Cp
            _plot_subplot(axes[0, 1], J_RANGE, gen, N_SLICE, J_slice, data["Cp"][mask],
                         colors[idx], prop_name, gen.get_cp)

            # Plot Cq
            _plot_subplot(axes[1, 0], J_RANGE, gen, N_SLICE, J_slice, data["Cq"][mask],
                         colors[idx], prop_name, gen.get_cq)

            # Plot Pe
            _plot_subplot(axes[1, 1], J_RANGE, gen, N_SLICE, J_slice, data["Pe"][mask],
                         colors[idx], prop_name, gen.get_pe)

        _configure_subplot(axes[0, 0], "$C_T$ vs J", "$C_T$ (Thrust Coefficient)")
        _configure_subplot(axes[0, 1], "$C_P$ vs J", "$C_P$ (Power Coefficient)")
        _configure_subplot(axes[1, 0], "$C_Q$ vs J", "$C_Q$ (Torque Coefficient)")
        _configure_subplot(axes[1, 1], "Efficiency vs J", "Efficiency (Pe)")

        plt.tight_layout()
        plt.show()

        # ===== PLOT 2: Single propeller at multiple RPMs =====
        print("\n\nGenerating second plot: single propeller at multiple RPMs...")

        gen = PropellerSurrogateModel(os.path.join(DATA_DIR, PROPELLER_FILES[0]))
        data = gen.apc_data.get_loaded_data()
        J_RANGE = np.linspace(data["J"].min(), data["J"].max(), 100)

        unique_N = np.unique(data["N"])
        rpm_indices = np.linspace(0, len(unique_N) - 1, min(4, len(unique_N)), dtype=int)
        rpm_slices = unique_N[rpm_indices]
        rpm_colors = plt.cm.viridis(np.linspace(0, 1, len(rpm_slices)))

        fig2, axes2 = plt.subplots(2, 2, figsize=(16, 10))
        prop_name = PROPELLER_FILES[0].replace("PER3_", "").replace(".dat", "")
        fig2.suptitle(f"Single Propeller ({prop_name}) at Multiple RPMs", fontsize=16)

        for rpm_idx, N_SLICE in enumerate(rpm_slices):
            mask = np.abs(data["N"] - N_SLICE) < 1.0
            J_slice = data["J"][mask]
            rpm_label = f"{N_SLICE:.0f} RPM"

            _plot_subplot(axes2[0, 0], J_RANGE, gen, N_SLICE, J_slice, data["Ct"][mask],
                         rpm_colors[rpm_idx], rpm_label, gen.get_ct)
            _plot_subplot(axes2[0, 1], J_RANGE, gen, N_SLICE, J_slice, data["Cp"][mask],
                         rpm_colors[rpm_idx], rpm_label, gen.get_cp)
            _plot_subplot(axes2[1, 0], J_RANGE, gen, N_SLICE, J_slice, data["Cq"][mask],
                         rpm_colors[rpm_idx], rpm_label, gen.get_cq)
            _plot_subplot(axes2[1, 1], J_RANGE, gen, N_SLICE, J_slice, data["Pe"][mask],
                         rpm_colors[rpm_idx], rpm_label, gen.get_pe)

        _configure_subplot(axes2[0, 0], "$C_T$ vs J", "$C_T$ (Thrust Coefficient)")
        _configure_subplot(axes2[0, 1], "$C_P$ vs J", "$C_P$ (Power Coefficient)")
        _configure_subplot(axes2[1, 0], "$C_Q$ vs J", "$C_Q$ (Torque Coefficient)")
        _configure_subplot(axes2[1, 1], "Efficiency vs J", "Efficiency (Pe)")

        plt.tight_layout()
        plt.show()

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
