import numpy as np
import rasterio
import plotly.graph_objects as go

def plot_dem_3d(
    dem_file_path: str, 
    target: int = 400, 
    clip_percent = (2, 98), 
    z_aspect = 0.2, 
    show_figure = True, 
    out_html_path = None
):
    with rasterio.open(path) as src:
        dem = src.read(1).astype(np.float32)
        nodata = src.nodata
        transform = src.transform

    if nodata is not None:
        dem = np.where(dem == nodata, np.nan, dem)

    step = max(1, max(dem.shape) // target)
    Z = dem[::step, ::step]
    Z = np.where(np.isnan(Z), np.nanmean(Z), Z)

    zmin, zmax = np.percentile(Z, clip_percent)
    Z = np.clip(Z, zmin, zmax)

    px = float(transform.a) * step
    py = abs(float(transform.e)) * step

    ny, nx = Z.shape
    x = np.arange(nx) * px
    y = np.arange(ny) * py

    fig = go.Figure(data=[go.Surface(x=x, y=y, z=Z, colorscale="earth")])
    fig.update_layout(
        title="Interactive DEM (meters)",
        scene=dict(
            xaxis_title="x (m)",
            yaxis_title="y (m)",
            zaxis_title="elevation (m)",
            aspectmode="manual",
            aspectratio=dict(x=1, y=1, z=z_aspect),
        ),
        margin=dict(l=0, r=0, b=0, t=40),
    )

    if out_html_path:
        fig.write_html(out_html_path, include_plotlyjs="cdn")
    if show_figure:
        fig.show()

    return fig
