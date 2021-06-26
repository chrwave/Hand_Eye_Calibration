# Hand-Eye Calibration
Included in this repository is a guide for hand-eye calibration (eye in hand) using Python and OpenCV.\
The guide uses the "DTU UR Equipment Functions" by Bisssen and pandaman35 (https://github.com/Bisssen/DTU_UR_equipment_functions). However, the necessary functions has already been included in this repository.

The guide "Guide_To_HandEye_Calibration.pdf" describes the process of using the Python script to obtain the transformation matrix from end-effector to camera as well as the camera matrix and distortion coefficients.

The folder "ExamplePictures" features 10 images taken with a camera mounted on the end-effector of an UR5. Notice that the end-effector is rotated, placed at different heights etc.

# Using the Transformation
The end-effector to camera transformation obtained with the hand-eye calibration, describes the rotation and translation of the camera relative to the end-effector. As an example of application, a image analysis script has been included in the repository. The script will filter red circles using BLOB-detection from a camera feed and return the center coordinate (point between the two circles). The hand-eye transformation is then applied to convert the coordinate to a corresponding gripper coordinate. A vector is defined between the two circles, such that the rotation of the end effector always will be parallel to the red circles.\
The necessary scripts are found in the folder "ApplicationExample". The requires packages are similar to those from the hand-eye calibration. Notice that the user must insert the hand-eye transformation matrix into the script.

# Setting up the Config
In the Libary there is a config file where the ip-address have to match the UR's ip-address. NB! Your own ip-address have to be within the same scope as the UR's ip-address

# Camera Intrinsics
In the "Libary" there is a file called "camera2robotcalib.py", in here the camera intrinsics can be edited to fit the camera used (which should be found by performing camera calibration). There is also a function that allows you to set the parameters which shouldnt be used yet, since there havent been added a function to read the camera intrinsic from the function yet.
