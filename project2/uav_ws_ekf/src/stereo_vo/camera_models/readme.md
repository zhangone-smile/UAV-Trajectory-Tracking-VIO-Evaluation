Trimmed runtime subset of [camodocal](https://github.com/hengli/camodocal).

This package keeps the camera model interfaces used by `stereo_vo` for YAML
loading, projection, and back-projection. Offline calibration tools and sample
calibration images are intentionally removed from this onboard runtime tree.

# Undistortion:

See [Camera.h](https://github.com/dvorak0/camera_model/blob/master/include/camodocal/camera_models/Camera.h) for general interface: 

 - liftProjective: Lift points from the image plane to the projective space.
 - spaceToPlane: Projects 3D points to the image plane (Pi function)
