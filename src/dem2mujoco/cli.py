import argparse
from dem2mujoco.visualize import plot_dem_3d

def main():
    p = argparse.ArgumentParser()
    p.add_argument("tif")
    p.add_argument("--html", default="dem_3d.html", help="The name of the html file to be generated")
    p.add_argument("--target", type=int, default=400, help="")
    args = p.parse_args()

    plot_dem_3d(args.tif, target=args.target, out_html_path=args.html, show=True)
    print(f"Wrote {args.html}.")
