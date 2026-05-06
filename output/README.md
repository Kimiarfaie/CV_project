# Submitted output

This output directory was generated using the submitted SGBM + WLS stereo depth pipeline.

## Method

The depth map was produced from the rectified grayscale ZED stereo pair using OpenCV StereoSGBM. The resulting disparity map was post-processed using OpenCV WLS disparity filtering with left-right matching. The filtered disparity was converted to metric depth using the focal length from `CameraInfo` and the stereo baseline from the camera extrinsics.

A light gamma preprocessing step was applied before stereo matching with:

- gamma: 0.75

This improved matching stability in darker and lower-contrast regions and reduced the mean reconstruction error in my experiments.

## Final configuration

Depth backend:

- OpenCV StereoSGBM
- `numDisparities`: 64
- `blockSize`: 3
- `disp12MaxDiff`: 1
- `uniquenessRatio`: 8
- `speckleWindowSize`: 100
- `speckleRange`: 1
- `preFilterCap`: 63
- mode: `STEREO_SGBM_MODE_SGBM_3WAY`

Post-processing:

- WLS disparity filtering enabled
    - WLS lambda: 8000
    - WLS sigmaColor: 2.0
- Gamma preprocessing enabled
    - gamma: 0.75

TSDF fusion:

- `pc_downsampling`: 5