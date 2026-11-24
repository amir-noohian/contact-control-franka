#!/usr/bin/env python3
"""Plot IAC1 normal estimation followed by hybrid force and motion control."""

from plot_cartesian_iac1_control import HYBRID_DATA_DIR, main


if __name__ == "__main__":
    main(default_directory=HYBRID_DATA_DIR, description=__doc__)
