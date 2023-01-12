# Camera Calibration
Standard [OpenCV Camera Calibration](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)


## Setup
Run `setup.sh` to create a virtual environment and install dependencies:
```shell
./setup.sh
```

Activate the virtual environment:
```shell
source venv/bin/activate
```


## Usage
Take photos (at least 10) of the chessboard pattern `patterns/chessboard.png` from various angles. Replace the sample photos in the `images` folder with the ones you took. Run the script:
```shell
python calibrate images/

Intrinsic Parameters:
fx: 534.0708836434115
fy: 534.1191459522821
cx: 341.5340755262212
cy: 232.94565259795462

Distortion Coefficients:
k1: -0.29297163675274435
k2: +0.10770696223452401
p1: +0.0013103837668586803
p2: -3.110188108687015e-05
k3: +0.04347981037293046

Re-projection error: 0.023686000375385673
```


## Parameters
| Variables | Description |
| :---: | :--- |
| fx, fy | Focal length |
| cx, cy | Optical center |
| k1, k2, k3 | Radial distortion |
| p1, p2 | Tangential distortion |

**Re-projection error**: The closer this value is to zero, the more accurate the parameters are.


## Troubleshooting
_`pip install` fails with `ModuleNotFoundError: No module named 'skbuild'`_

Upgrade `pip` with `pip install --upgrade pip`. For details, see: https://github.com/opencv/opencv-python#frequently-asked-questions
