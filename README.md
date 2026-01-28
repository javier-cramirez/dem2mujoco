**W.I.P**

You can fetch DEMs from [OpenTopography](https://opentopography.org/), given you have a license. If you are a student at higher institution,
it should be the case that you are able to be granted one. 

## How it Works

### Multi-Dimensional Terrain Smoothing

DEMs *can be* noisy. Whether they were generated from LiDaR point clouds, via photogrammetery, or radar, they may contain artifacts or other noise.
For our purposes (robot environments), we want a smooth terrain that the robot can navigate (smooth as in not weird or noisy where it shouldn't be). 

As such, we can use filters (like the Gaussian filter) applied to the heightfield generated from our DEM data to "smooth" the terrain. 

In 2D images, low-pass filtering can produce a "smoothened" effect. Unwanted space between distorted pixels can be bridged together by such filters. 
Using a Gaussian filter, the cutoff frequency is the variance $\sigma^2$. Therefore, $\sigma^2$ controls the amount of "smoothing". 

(image credit to the [scipy.ndimage docs](https://docs.scipy.org/doc/scipy/tutorial/ndimage.html))<img width="1536" height="864" alt="image" src="https://github.com/user-attachments/assets/bbee3e08-ea86-4acc-b726-c56d73f51b98"/>


