import argparse
from dem2mujoco import preview_dem

def main():
    p = argparse.ArgumentParser()
    p.add_argument("dem", help="Path to DEM .tif")
    p.add_argument("--out", default="dem2mujoco_out")
    p.add_argument("--n", type=int, default=257)
    p.add_argument("--sigma", type=float, default=1.0)
    p.add_argument("--z", type=float, default=3.0)
    p.add_argument("--xhalf", type=float, default=20.0)
    p.add_argument("--yhalf", type=float, default=20.0)
    args = p.parse_args()

    preview_dem(
        args.dem,
        workdir=args.out,
        target_n=args.n,
        smooth_sigma=args.sigma,
        z_scale_m=args.z,
        x_half=args.xhalf,
        y_half=args.yhalf,
    )

if __name__ == "__main__":
    main()
