**W.I.P**

THis is a small tool for my robotics simulation projects. Anything outside of that, I do not know how it'd fare.

### Instructions

You can fetch tiff files from [OpenTopography](https://opentopography.org/), given you have a license. If you are a student at higher institution,
it should be the case that you are able to be granted one. 

Once downloaded, we will convert them to DEMs which we will use for the next step.

<img width="500" height="133" alt="image" src="https://github.com/user-attachments/assets/117b40c8-4521-49a6-ada1-4cecfa553b24" />

(image credit to [MacOdrum Library](https://library.carleton.ca/guides/help/dem-formats))

### Some Background Behind the Abstraction

DEMs *can be* noisy. Whether they were generated from LiDaR point clouds, via photogrammetery, or radar, they may contain artifacts or other noise.
For our purposes (robot environments), we want a smooth terrain that the robot can navigate (smooth as in not weird or noisy where it shouldn't be). 

As such, we can use filters (like the Gaussian filter) applied to the heightfield generated from our DEM data to "smooth" the terrain. 

### Extra Stuff
From signal processing land, we know that low-pass filters only pass along frequencies below a certain cutoff, $D_0$. 

In 2D images, low-pass filtering can produce a "smoothened" effect. Unwanted space between distorted pixels can be bridged together by such filters. 
Using a Gaussian filter, the cutoff frequency, $D_0$, is the variance $\sigma^2$. Therefore, $\sigma^2$ controls the amount of "smoothing" in our image. 

<img width="1536" height="864" alt="image" src="https://github.com/user-attachments/assets/bbee3e08-ea86-4acc-b726-c56d73f51b98"/>

(image credit to the [scipy.ndimage docs](https://docs.scipy.org/doc/scipy/tutorial/ndimage.html))

