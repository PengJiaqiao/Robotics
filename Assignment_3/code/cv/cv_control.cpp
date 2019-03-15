#include "cv_control.h"

using namespace cv;
using namespace std;

// Parameters for Visual Servoing
float f;					// Focal length in pixel
float img_width;			// width of image sensor in pixel
float img_height;			// height of image sensor in pixel
float diameter_real;		// real diameter of the circle
float diameter_desired_px;	// desired diameter of the circle in pixels
float dt;					// Time step
float t0;                  // how fast the velocity controller should converge
int   hcd_min_distance;
PrVector3 desired_s_opencvf;

void initVisualServoing(float _f, float _img_width, float _img_height, float _diameter_real, float _diameter_desired_px, float _dt, PrVector3 _desired_s_opencvf)
{
    f = _f;
    img_width = _img_width;
    img_height = _img_height;
    diameter_real = _diameter_real;
    diameter_desired_px = _diameter_desired_px;
    dt = _dt;
    desired_s_opencvf = _desired_s_opencvf;
    t0 = 0.3;   //time constant for controller convergence. Smaller values result in higher velocities. Transforms an error into a velocity
    hcd_min_distance = 2000;
}

/*****************************************************************************************************************/
/* YOUR WORK STARTS HERE!!! */

/** 
* findCircleFeature
* Find circles in the image using the OpenCV Hough Circle detector
*
* Input parameters:
*  img: the camera image, you can print text in it with 
* 	    putText(img,"Hello World",cvPoint(0,12),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,0,255))
*	    see http://opencv.willowgarage.com/documentation/cpp/drawing_functions.html#cv-puttext
*
*  backproject: grayscale image with high values where the color of the image is like the selected color.
*
* Output:
*  crcl: as a result of this function you should write the center and radius of the detected circle into crcl
*/
bool findCircleFeature(Mat& img, Mat &backproject, Circle& crcl) {
  // https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html

  // step 1: convert source image to grayscale
  // backproject is already a grayscale image

  // step 2: apply gaussian blur with kernel of size 9x9 and std of 2
  GaussianBlur(backproject, backproject, Size(9, 9), 2, 2);
  
  // step 3: hough circle transform
  vector<Vec3f> circles; // will hold x_c, y_c and radius of the detected circles
  HoughCircles(backproject, circles, CV_HOUGH_GRADIENT, 1, backproject.rows/8, 200, 100, 0, 0);

  // if there was no circle found; abort
  if (circles.size() == 0) return false;
  
  // TODO: there might be multiple circles found - is it ok to always chose the first?

  // save the center-point and radius of the detected circle
  crcl.center = Point2f(circles[0][0], circles[0][1]);
  crcl.radius = circles[0][2];

  // show the outline of the detected circle
  circle(img, crcl.center, crcl.radius, Scalar(0,0,255), 3, 8, 0);

  // mark the center of the detected circle with a dot
  circle(img, crcl.center, 1, CV_RGB(0, 255, 0), 1, 8, 0);
  
  // TODO: Make sure your implementation is tolerant to adverse camera images
  // and document the steps you did to ensure this
  // - Gaussian blur to reduce noise
  // - Abort, if no circle present
  // - What if multiple circles were found?

  return true;
}


/**
* getImageJacobianCFToFF
* Compute the Image Jacobian to map from 
* camera velocity in Camera Frame to feature velocities in Feature Frame
* 
* You should use getImageJacobianFFToCF in controlRobot
*
* Input parameters:
*  u and v: the current center of the circle in feature frame [pixels]
*  z: the estimated depth of the circle [meters]
*  f: the focal length [pixels]
*  diameter: the real diameter of the circle [meters]
*
* Output:
*  Jv: assign your image 3x3 Jacobian.
*/
void getImageJacobianCFToFF(PrMatrix3 &Jv, float u, float v, float z, float f, float diameter) {
  Jv[0][0] = -f/z;
  Jv[0][1] =  0.0;
  Jv[0][2] =  u/z;

  Jv[1][0] =  0.0;
  Jv[1][1] = -f/z;
  Jv[1][2] =  v/z;

  Jv[2][0] =  0.0;
  Jv[2][1] =  0.0;
  Jv[2][2] = (f*diameter)/(z*z);
}


/**
* estimateCircleDepth
* Estimates and returns the depth of the circle
*
* Input parameters:
*  f: the focal length [pixels]
*  diameter: the real diameter of the circle [meters]
*  crcl: the parameters of the detected circle in the image
*
* Output return:
*  depth of the circle wrt the camera [meters]
*/
float estimateCircleDepth(float f, float diameter, Circle &crcl) {
  return f/(2*crcl.radius) * diameter;
}


/**
* transformFromOpenCVFToFF
* Transform a feature vector from openCV frame (origin in upper left corner of the image) to feature frame (origin at the center of the image)
*
* Input parameter:
*  vector_opencvf: feature vector defined in opencv frame
*
* Output:
*  vector_ff: feature vector defined in feature frame
*/
void transformFromOpenCVFToFF(PrVector3 vector_opencvf, PrVector3& vector_ff) {
  vector_ff[0] = vector_opencvf[0] - img_width/2;  // img_width  = 320 (cv_main.cpp)
  vector_ff[1] = vector_opencvf[1] - img_height/2; // img_height = 240 (cv_main.cpp) 
  vector_ff[2] = vector_opencvf[2]; // the diameter shouldn't change?
}


/**
* transformVelocityFromCFToEEF
* Transform the desired velocity vector from camera frame to end-effector frame
* You can hard code this transformation according to the fixed transformation between the camera and the end effector
* (see the sketch in your assignment)
*
* Input parameter:
*  vector_cf: velocity vector defined in camera frame
*
* Output:
*  vector_eef: velocity vector defined in end-effector frame
*/
void transformVelocityFromCFToEEF(PrVector3 vector_cf, PrVector3& vector_eef) {
  // hardcoded transformation based on figure 2 in the exercise sheet
  vector_eef[0] = -vector_cf[1]; // X_{EE} == -Y_C
  vector_eef[1] =  vector_cf[0]; // Y_{EE} ==  X_C 
  vector_eef[2] =  vector_cf[2]; // Z_{EE} ==  Z_C 
}


/**
* transformVelocityFromEEFToBF
* Transform the desired velocity vector from end-effector frame to base frame
* You cannot hard code this transformation because it depends of the current orientation of the end-effector wrt the base
* Make use of the current state of the robot x (the pose of the end-effector in base frame coordinates)
*
* Input parameters:
*  x_current_bf: current state of the robot - pose of the end-effector in base frame coordinates
*  vector_eef: velocity vector defined in end-effector frame
*
* Output:
*  vector_bf: velocity vector defined in base frame
*/
void transformVelocityFromEEFToBF(PrVector x_current_bf, PrVector3 vector_eef, PrVector3& vector_bf) {
  PrMatrix3 R; // Euler-Rodrigues formula, see: https://en.wikipedia.org/wiki/Euler%E2%80%93Rodrigues_formula

  // first row
  R[0][0] = pow(x_current_bf[3], 2) + pow(x_current_bf[4], 2) - pow(x_current_bf[5], 2) - pow(x_current_bf[6], 2);  // a^2 + b^2 - c^2 - d^2
  R[0][1] = 2 * (x_current_bf[4] * x_current_bf[5] - x_current_bf[3] * x_current_bf[6]);                            // 2 * (bc - ad)
  R[0][2] = 2 * (x_current_bf[4] * x_current_bf[6] + x_current_bf[3] * x_current_bf[5]);                            // 2 * (bd + ac)

  // second row
  R[1][0] =  2 * (x_current_bf[4] * x_current_bf[5] + x_current_bf[3] * x_current_bf[6]);                           // 2 * (bc + ad)
  R[1][1] = pow(x_current_bf[3], 2) + pow(x_current_bf[5], 2) - pow(x_current_bf[4], 2) - pow(x_current_bf[6], 2);  // a^2 + c^2 - b^2 - d^2
  R[1][2] =  2 * (x_current_bf[5] * x_current_bf[6] - x_current_bf[3] * x_current_bf[4]);                           // 2 * (cd - ab)

  // third row
  R[2][0] = 2 * (x_current_bf[4] * x_current_bf[6] - x_current_bf[3] * x_current_bf[5]);                            // 2 * (bd - ac)
  R[2][1] = 2 * (x_current_bf[5] * x_current_bf[6] + x_current_bf[3] * x_current_bf[4]);                            // 2 * (cd + ab)
  R[2][2] = pow(x_current_bf[3], 2) + pow(x_current_bf[6], 2) - pow(x_current_bf[4], 2) - pow(x_current_bf[5], 2);  // a^2 + d^2 - b^2 - c^2

  // actual transformation: dx_{B} = R * dx_{EE}
  vector_bf[0] = R[0][0] * vector_eef[0] + R[0][1] * vector_eef[1] + R[0][2] * vector_eef[2];
  vector_bf[1] = R[1][0] * vector_eef[0] + R[1][1] * vector_eef[1] + R[1][2] * vector_eef[2];
  vector_bf[2] = R[2][0] * vector_eef[0] + R[2][1] * vector_eef[1] + R[2][2] * vector_eef[2];

}


/*
* controlRobot
* This function computes the command to be send to the robot using Visual Servoing so that the robot tracks the circle
*
* Here you should:
* - compute the error in feature frame
* - compute the circle depth
* - compute the image jacobian from feature frame in camera frame
* - compute the desired ee velocity in feature frame
* - compute the desired ee velocity in camera frame
* - compute the desired ee velocity in ee frame
* - compute the desired ee velocity in base frame
* - compute the step in the direction of the desired ee velocity in base frame
* - form the comand to be sent to the robot (previous pose + computed step)
*
* The function will only be called if findCircleFeature returns true (if a circle is detected in the image)
*
* Input parameters:
*  crcl: the parameters of the detected circle in the image
*  x:	current robot configuration in operational space (7 dof: 3 first values are position, 4 last values is orientation quaternion)
*  img: the camera image for drawing debug text
*
* Output:
*  cmdbuf: should contain the command for the robot controler, for example: 
*			"goto 0.0 0.0 90.0 0.0 0.0 0.0"
*/
void controlRobot(Circle& crcl, PrVector &x, Mat& img, char *cmdbuf) {
  if (crcl.radius == 0) {
      sprintf(cmdbuf,"float");
      return;
  }

  PrVector3 current_s_opencvf;
  current_s_opencvf[0] = crcl.center.x;
  current_s_opencvf[1] = crcl.center.y;
  current_s_opencvf[2] = 2*crcl.radius;

  PrVector3 desired_s_ff;
  transformFromOpenCVFToFF(desired_s_opencvf, desired_s_ff);
  PrVector3 current_s_ff;
  transformFromOpenCVFToFF(current_s_opencvf, current_s_ff);

  PrVector3 error_s_ff = desired_s_ff - current_s_ff;

  float z = estimateCircleDepth(f, diameter_real, crcl);

  PrMatrix3 Jv;
  getImageJacobianCFToFF(Jv, current_s_ff[0], current_s_ff[1], z, f, diameter_real);

  PrMatrix3 Jv_inv;
  Jv.pseudoInverse(Jv_inv);

  // Compute the desired velocity of the feature in feature frame
  PrVector3 vel_f_ff = error_s_ff / t0;

  // Compute the desired velocity of the end effector in camera frame
  PrVector3 vel_ee_cf = Jv_inv*vel_f_ff;

  PrVector3 vel_ee_eef;
  transformVelocityFromCFToEEF(vel_ee_cf, vel_ee_eef);

  PrVector3 vel_ee_bf;
  transformVelocityFromEEFToBF(x, vel_ee_eef, vel_ee_bf);

  // Compute the next EE position for the next timestep given the desired EE velocity:
  PrVector3 step_ee_bf = vel_ee_bf * dt;

  PrVector desired_ee_pose_bf = x;
  desired_ee_pose_bf[0] += step_ee_bf[0];
  desired_ee_pose_bf[1] += step_ee_bf[1];
  desired_ee_pose_bf[2] += step_ee_bf[2];

  // limit the EE position to stay within the robot's workspace to avoid singular positions
  // assume a sphere of 0.85m radius around the base frame as the robot's workspace
  bool reachable = pow(desired_ee_pose_bf[0], 2) + pow(desired_ee_pose_bf[1], 2) + pow(desired_ee_pose_bf[2], 2) < pow(0.85, 2);
  if (not reachable) return;

  // Command the robot to go to the new desired position:
  sprintf(cmdbuf,"goto %.4f %.4f %.4f %.4f %.4f %.4f %.4f", desired_ee_pose_bf[0], desired_ee_pose_bf[1], desired_ee_pose_bf[2], 0.50, 0.50, -0.50, 0.50);

  putText(img, cmdbuf, cv::Point(5,50), FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0), 1.2);
}
