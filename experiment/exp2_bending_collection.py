"""Collect four-phase quasi-static force cycles with tendons T0 and T1."""

if __package__:
    from .exp1_bending_collection import run_cli
else:
    from exp1_bending_collection import run_cli


if __name__ == "__main__":
    run_cli(pair=True)
