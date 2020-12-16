#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
//#include "LinearMath/btMatrix3x3.h"
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#define Num 20
#define PI 3.1415926

Eigen::Matrix4d T0ToE;
Eigen::Matrix4d T1ToE;
Eigen::Matrix4d T2ToE;
Eigen::Matrix4d T3ToE;
Eigen::Matrix4d T8To0;
Eigen::Matrix4d T8To1;
Eigen::Matrix4d T8To2;
Eigen::Matrix4d T8To3;
Eigen::Matrix4d T9To8;
Eigen::Matrix4d TUcTo8;
Eigen::Matrix4d TLcTo9;
Eigen::Matrix4d TLcToUc;

Eigen::Matrix4d TCToE_0;
Eigen::Matrix4d TCToE_1;
Eigen::Matrix4d TCToE_2;
Eigen::Matrix4d TCToE_3;

//Eigen::Quaterniond q0;
//Eigen::Quaterniond q1;
//Eigen::Quaterniond q2;
//Eigen::Quaterniond q3;

geometry_msgs::Pose upperPartPoseInRightEndEffectorFrame;
geometry_msgs::Pose lowerPartPoseInUpperPartFrame;

void initialize(bool & isBlackBox) {

	T0ToE.setIdentity();
	T1ToE.setIdentity();
	T2ToE.setIdentity();
	T3ToE.setIdentity();
	Eigen::Matrix4d T1;
	T1.setIdentity();
	Eigen::Matrix4d T2;
	T2.setIdentity();
	Eigen::Matrix4d T3;
	T3.setIdentity();
	T1.block(0, 0, 3, 3) = Eigen::AngleAxisd(45 * PI / 180,
			Eigen::Vector3d::UnitZ()).matrix();
	T2.block(0, 0, 3, 3) = Eigen::AngleAxisd(-90 * PI / 180,
			Eigen::Vector3d::UnitX()).matrix();
	T3(2, 3) = 0.062;
	T3(1, 3) = -0.0325;
	T0ToE = T1 * T2 * T3;
	std::cout << "T0ToE\n" << T0ToE << std::endl;

	T1.setIdentity();
	T2.setIdentity();
	T3.setIdentity();
	T1.block(0, 0, 3, 3) = Eigen::AngleAxisd(-45 * PI / 180,
			Eigen::Vector3d::UnitZ()).matrix();
	T2.block(0, 0, 3, 3) = Eigen::AngleAxisd(-90 * PI / 180,
			Eigen::Vector3d::UnitX()).matrix();
	T3(2, 3) = 0.062;
	T3(1, 3) = -0.0325;
	T1ToE = T1 * T2 * T3;
	std::cout << "T1ToE\n" << T1ToE << std::endl;

	T1.setIdentity();
	T2.setIdentity();
	T3.setIdentity();
	T1.block(0, 0, 3, 3) = Eigen::AngleAxisd(225 * PI / 180,
			Eigen::Vector3d::UnitZ()).matrix();
	T2.block(0, 0, 3, 3) = Eigen::AngleAxisd(-90 * PI / 180,
			Eigen::Vector3d::UnitX()).matrix();
	T3(2, 3) = 0.062;
	T3(1, 3) = -0.0325;
	T2ToE = T1 * T2 * T3;
	std::cout << "T2ToE\n" << T2ToE << std::endl;

	T1.setIdentity();
	T2.setIdentity();
	T3.setIdentity();
	T1.block(0, 0, 3, 3) = Eigen::AngleAxisd(135 * PI / 180,
			Eigen::Vector3d::UnitZ()).matrix();
	T2.block(0, 0, 3, 3) = Eigen::AngleAxisd(-90 * PI / 180,
			Eigen::Vector3d::UnitX()).matrix();
	T3(2, 3) = 0.062;
	T3(1, 3) = -0.0325;
	T3ToE = T1 * T2 * T3;
	std::cout << "T3ToE\n" << T3ToE << std::endl;

	T1.setIdentity();
	T2.setIdentity();
	T3.setIdentity();
	TUcTo8.setIdentity();
	T1.block(0, 0, 3, 3) = Eigen::AngleAxisd(-90 * PI / 180,
			Eigen::Vector3d::UnitX()).matrix();
	T2.block(0, 0, 3, 3) = Eigen::AngleAxisd(-90 * PI / 180,
			Eigen::Vector3d::UnitZ()).matrix();
	if (isBlackBox) {
		//The black box
		T3(0, 3) = -0.10;
		T3(1, 3) = -0.0925;
		T3(2, 3) = -0.0525;
	} else {
		//The plastic box
		T3(0, 3) = -0.0590;
		T3(1, 3) = -0.0510;
		T3(2, 3) = -0.0420;
	}
	TUcTo8 = T1 * T2 * T3;
	std::cout << "TUcTo8 \n" << TUcTo8 << std::endl;

	T1.setIdentity();
	T2.setIdentity();
	T3.setIdentity();
	TLcTo9.setIdentity();
	T1.block(0, 0, 3, 3) = Eigen::AngleAxisd(-90 * PI / 180,
			Eigen::Vector3d::UnitX()).matrix();
	T2.block(0, 0, 3, 3) = Eigen::AngleAxisd(-90 * PI / 180,
			Eigen::Vector3d::UnitZ()).matrix();
	//The plastic box
	T3(0, 3) = -0.0590;
	T3(1, 3) = -0.0510;
	T3(2, 3) = -0.0420;
	TLcTo9 = T1 * T2 * T3;
	std::cout << "TLcTo9\n" << TLcTo9 << std::endl;

	T8To0.setIdentity();
	T8To1.setIdentity();
	T8To2.setIdentity();
	T8To3.setIdentity();
	T9To8.setIdentity();
	TLcToUc.setIdentity();

	TCToE_0.setIdentity();
	TCToE_1.setIdentity();
	TCToE_2.setIdentity();
	TCToE_3.setIdentity();

//	YPR0To8.setZero();
//	YPR1To8.setZero();
//	YPR2To8.setZero();
//	YPR3To8.setZero();
//	YPRCToE.setZero();

//	q0.setIdentity();
//	q1.setIdentity();
//	q2.setIdentity();
//	q3.setIdentity();
}

Eigen::Matrix3d YPRtoRotationMatrix(Eigen::Vector3d & ypr) {
	Eigen::Matrix3d R;
	R.setZero();
	R << cos(ypr(0)) * cos(ypr(1)), cos(ypr(0)) * sin(ypr(1)) * sin(ypr(2))
			- sin(ypr(0)) * cos(ypr(2)), cos(ypr(0)) * sin(ypr(1)) * cos(ypr(2))
			+ sin(ypr(0)) * sin(ypr(2)), sin(ypr(0)) * cos(ypr(1)), sin(ypr(0))
			* sin(ypr(1)) * sin(ypr(2)) + cos(ypr(0)) * cos(ypr(2)), sin(ypr(0))
			* sin(ypr(1)) * cos(ypr(2)) - cos(ypr(0)) * sin(ypr(2)), -sin(
			ypr(1)), cos(ypr(1)) * sin(ypr(2)), cos(ypr(1)) * cos(ypr(2));
	return R;
}
//
//Eigen::Vector3d QuaterniontoEulerAngleZyx(const tf::Quaternion q) {
//	// roll (x-axis rotation)
//	double sinr_cosp = +2.0 * (q.getW() * q.getX() + q.getY() * q.getZ());
//	double cosr_cosp = +1.0 - 2.0 * (q.getX() * q.getX() + q.getY() * q.getY());
//	Eigen::Vector3d result;
//	result(2) = atan2(sinr_cosp, cosr_cosp);
//
//	// pitch (y-axis rotation)
//	double sinp = +2.0 * (q.getW() * q.getY() - q.getZ() * q.getX());
//	if (fabs(sinp) >= 1)
//		result(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//	else
//		result(1) = asin(sinp);
//
//	// yaw (z-axis rotation)
//	double siny_cosp = +2.0 * (q.getW() * q.getZ() + q.getX() * q.getY());
//	double cosy_cosp = +1.0 - 2.0 * (q.getY() * q.getY() + q.getZ() * q.getZ());
//	result(0) = atan2(siny_cosp, cosy_cosp);
//
//	return result;
//}
//
//Eigen::Vector4d YPRtoQuaternion(Eigen::Vector3d ypr) // yaw (Z), pitch (Y), roll (X)
//		{
//	// Abbreviations for the various angular functions
//	double cy = cos(ypr(0) * 0.5);
//	double sy = sin(ypr(0) * 0.5);
//	double cp = cos(ypr(1) * 0.5);
//	double sp = sin(ypr(1) * 0.5);
//	double cr = cos(ypr(2) * 0.5);
//	double sr = sin(ypr(2) * 0.5);
//
//	Eigen::Vector4d q;
//	q(3) = cr * cp * cy + sr * sp * sy;
//	q(0) = sr * cp * cy - cr * sp * sy;
//	q(1) = cr * sp * cy + sr * cp * sy;
//	q(2) = cr * cp * sy - sr * sp * cy;
//
//	return q;
//}

void computeVariance(double & var, double x, double * array) {
	for (int i = 0; i < Num; i++) {
		if (i == Num - 1) {
			array[i] = x;
		} else {
			array[i] = array[i + 1];
		}

	}
	double sum = 0;
	for (int i = 0; i < Num; i++) {
		sum += array[i];
	}
	double average = 0;
	average = sum / Num;
	for (int i = 0; i < Num; i++) {
		var += (array[i] - average) * (array[i] - average);
	}
	var /= Num;
	var = sqrt(var);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "my_tf_listener");

	ros::NodeHandle node;

	ros::Publisher upperPartPoseInRightEndEffectorFrame_pub = node.advertise<
			geometry_msgs::Pose>("upperPartPoseInRightEndEffectorFrame", 1000);
	ros::Publisher lowerPartPoseInUpperPartFrame_pub = node.advertise<
			geometry_msgs::Pose>("lowerPartPoseInUpperPartFrame", 1000);

	tf::TransformListener listener;

	tf::StampedTransform transform0To8;
	tf::StampedTransform transform1To8;
	tf::StampedTransform transform2To8;
	tf::StampedTransform transform3To8;
	tf::StampedTransform transform9To8;

	std::vector<std::string> frames;
	double cache0To8[Num] = { 0 };
	double cache1To8[Num] = { 0 };
	double cache2To8[Num] = { 0 };
	double cache3To8[Num] = { 0 };
	double cache9To8[Num] = { 0 };
	double variance0To8 = 0;
	double variance1To8 = 0;
	double variance2To8 = 0;
	double variance3To8 = 0;
	double variance9To8 = 0;
	double sum = 0;
	double average = 0;
	bool flag = false;
	bool flag2 = false;
	bool flag_pub0 = false;
	bool flag_pub1 = false;
	bool flag_pub2 = false;
	bool flag_pub3 = false;
	bool flag_pub_UcLc = false;
	bool marker_flag0 = false;
	bool marker_flag1 = false;
	bool marker_flag2 = false;
	bool marker_flag3 = false;
	bool marker_flag9 = false;
	bool flag_lowerPart = false;

	bool isBlackBox = false;
	node.param<bool>(ros::this_node::getName() + "/isBlackBox", isBlackBox,
			false);
	initialize(isBlackBox);

	ros::Rate rate(10.0);
	while (node.ok()) {

		listener.getFrameStrings(frames);
		if (frames.size() != 0) {
			for (auto id : frames) {
				if (id == "ar_marker_8") {
					flag = true;
					for (auto subId : frames) {

						if (subId == "ar_marker_0") {
							flag2 = true;
							try {
								listener.lookupTransform("/ar_marker_0",
										"/ar_marker_8", ros::Time(0),
										transform0To8);
								marker_flag0 = true;
							} catch (tf::TransformException &ex) {
								ROS_WARN("%s", ex.what());
								marker_flag0 = false;
//								rate.sleep();
//								continue;
							}
						} else if (subId == "ar_marker_1") {
							flag2 = true;
							try {
								listener.lookupTransform("/ar_marker_1",
										"/ar_marker_8", ros::Time(0),
										transform1To8);
								marker_flag1 = true;
							} catch (tf::TransformException &ex) {
								ROS_WARN("%s", ex.what());
								marker_flag1 = false;
//								rate.sleep();
//								continue;
							}
						} else if (subId == "ar_marker_2") {
							flag2 = true;
							try {
								listener.lookupTransform("/ar_marker_2",
										"/ar_marker_8", ros::Time(0),
										transform2To8);
								marker_flag2 = true;
							} catch (tf::TransformException &ex) {
								ROS_WARN("%s", ex.what());
								marker_flag2 = false;
//								rate.sleep();
//								continue;
							}
						} else if (subId == "ar_marker_3") {
							flag2 = true;
							try {
								listener.lookupTransform("/ar_marker_3",
										"/ar_marker_8", ros::Time(0),
										transform3To8);
								marker_flag3 = true;
							} catch (tf::TransformException &ex) {
								ROS_WARN("%s", ex.what());
								marker_flag3 = false;
//								rate.sleep();
//								continue;
							}
						} else if (subId == "ar_marker_9") {
							flag_lowerPart = true;
							try {
								listener.lookupTransform("/ar_marker_8",
										"/ar_marker_9", ros::Time(0),
										transform9To8);
								marker_flag9 = true;
							} catch (tf::TransformException &ex) {
								ROS_WARN("%s", ex.what());
								marker_flag9 = false;
							}
						}
					}

					if (!flag2) {
						ROS_ERROR("I cannot localise the robot!");
						continue;
					} else if (!marker_flag0 && !marker_flag1 && !marker_flag2
							&& !marker_flag3) {
						ROS_ERROR(
								"I cannot localise the robot because I lost all the markers!");
					}

					if (!flag_lowerPart) {
						ROS_ERROR("I cannot see the lower part!");
//						continue;
					}

					//compute the transformation
					if (marker_flag0) {
						computeVariance(variance0To8,
								transform0To8.getOrigin().getX(), cache0To8);
					} else {
						ROS_WARN("I cannot see marker0!");
					}
					if (marker_flag1) {
						computeVariance(variance1To8,
								transform1To8.getOrigin().getX(), cache1To8);
					} else {
						ROS_WARN("I cannot see marker1!");
					}
					if (marker_flag2) {
						computeVariance(variance2To8,
								transform2To8.getOrigin().getX(), cache2To8);
					} else {
						ROS_WARN("I cannot see marker2!");
					}
					if (marker_flag3) {
						computeVariance(variance3To8,
								transform3To8.getOrigin().getX(), cache3To8);
					} else {
						ROS_WARN("I cannot see marker3!");
					}
					if (marker_flag9) {
						computeVariance(variance9To8,
								transform9To8.getOrigin().getX(), cache9To8);
					} else {
						ROS_WARN("I cannot see marker9!");
					}

					//Publish the relative tranformation between the end-effector and the object
					if (variance0To8 == 0.05 && variance1To8 == 0.05
							&& variance2To8 == 0.05 && variance3To8 == 0.05) {
						ROS_ERROR("We lost all the  markers on the robot!");
						continue;
					}
					if (marker_flag0) {
						if (variance0To8 == 0.05) {
							ROS_WARN("We lost marker_0 or marker_8!");
							flag_pub0 = false;
						} else {
							flag_pub0 = true;

							//compute TCE
							tf::Vector3 m0 = tf::Matrix3x3(
									transform0To8.getRotation()).getColumn(0);
							tf::Vector3 m1 = tf::Matrix3x3(
									transform0To8.getRotation()).getColumn(1);
							tf::Vector3 m2 = tf::Matrix3x3(
									transform0To8.getRotation()).getColumn(2);
							T8To0(0, 0) = m0.getX();
							T8To0(1, 0) = m0.getY();
							T8To0(2, 0) = m0.getZ();
							T8To0(0, 1) = m1.getX();
							T8To0(1, 1) = m1.getY();
							T8To0(2, 1) = m1.getZ();
							T8To0(0, 2) = m2.getX();
							T8To0(1, 2) = m2.getY();
							T8To0(2, 2) = m2.getZ();
							T8To0(0, 3) = transform0To8.getOrigin().getX();
							T8To0(1, 3) = transform0To8.getOrigin().getY();
							T8To0(2, 3) = transform0To8.getOrigin().getZ();

							TCToE_0 = T0ToE * T8To0 * TUcTo8;

							//Convert transformation to ros msg
							double m00, m01, m02, m10, m11, m12, m20, m21, m22;
							m00 = TCToE_0(0, 0);
							m01 = TCToE_0(0, 1);
							m02 = TCToE_0(0, 2);
							m10 = TCToE_0(1, 0);
							m11 = TCToE_0(1, 1);
							m12 = TCToE_0(1, 2);
							m20 = TCToE_0(2, 0);
							m21 = TCToE_0(2, 1);
							m22 = TCToE_0(2, 2);
							tf::Quaternion quat;
							tf::Matrix3x3(m00, m01, m02, m10, m11, m12, m20,
									m21, m22).getRotation(quat);
							geometry_msgs::Quaternion msgQuat;
							tf::quaternionTFToMsg(quat, msgQuat);

							upperPartPoseInRightEndEffectorFrame.orientation =
									msgQuat;
							upperPartPoseInRightEndEffectorFrame.position.x =
									TCToE_0(0, 3);
							upperPartPoseInRightEndEffectorFrame.position.y =
									TCToE_0(1, 3);
							upperPartPoseInRightEndEffectorFrame.position.z =
									TCToE_0(2, 3);

							//Print out
							std::cout
									<< "The transformation from marker_0 to marker_8 in marker_0 frame: [x, y, z] "
									<< transform0To8.getOrigin().getX() << ", "
									<< transform0To8.getOrigin().getY() << ", "
									<< transform0To8.getOrigin().getZ()
									<< std::endl;
							std::cout << "variance0To8: " << variance0To8
									<< std::endl;
						}
					}
					if (marker_flag1) {
						if (variance1To8 == 0.05) {
							ROS_WARN("We lost marker_1 or marker_8!");
							flag_pub1 = false;
						} else {
							flag_pub1 = true;

							tf::Vector3 m0 = tf::Matrix3x3(
									transform1To8.getRotation()).getColumn(0);
							tf::Vector3 m1 = tf::Matrix3x3(
									transform1To8.getRotation()).getColumn(1);
							tf::Vector3 m2 = tf::Matrix3x3(
									transform1To8.getRotation()).getColumn(2);
							T8To1(0, 0) = m0.getX();
							T8To1(1, 0) = m0.getY();
							T8To1(2, 0) = m0.getZ();
							T8To1(0, 1) = m1.getX();
							T8To1(1, 1) = m1.getY();
							T8To1(2, 1) = m1.getZ();
							T8To1(0, 2) = m2.getX();
							T8To1(1, 2) = m2.getY();
							T8To1(2, 2) = m2.getZ();
							T8To1(0, 3) = transform1To8.getOrigin().getX();
							T8To1(1, 3) = transform1To8.getOrigin().getY();
							T8To1(2, 3) = transform1To8.getOrigin().getZ();

							TCToE_1 = T1ToE * T8To1 * TUcTo8;

							double m00, m01, m02, m10, m11, m12, m20, m21, m22;
							m00 = TCToE_1(0, 0);
							m01 = TCToE_1(0, 1);
							m02 = TCToE_1(0, 2);
							m10 = TCToE_1(1, 0);
							m11 = TCToE_1(1, 1);
							m12 = TCToE_1(1, 2);
							m20 = TCToE_1(2, 0);
							m21 = TCToE_1(2, 1);
							m22 = TCToE_1(2, 2);
							tf::Quaternion quat;
							tf::Matrix3x3(m00, m01, m02, m10, m11, m12, m20,
									m21, m22).getRotation(quat);
							geometry_msgs::Quaternion msgQuat;
							tf::quaternionTFToMsg(quat, msgQuat);

							upperPartPoseInRightEndEffectorFrame.orientation =
									msgQuat;
							upperPartPoseInRightEndEffectorFrame.position.x =
									TCToE_1(0, 3);
							upperPartPoseInRightEndEffectorFrame.position.y =
									TCToE_1(1, 3);
							upperPartPoseInRightEndEffectorFrame.position.z =
									TCToE_1(2, 3);

							std::cout
									<< "The transformation from marker_1 to marker_8 in marker_1 frame: [x, y, z] "
									<< transform1To8.getOrigin().getX() << ", "
									<< transform1To8.getOrigin().getY() << ", "
									<< transform1To8.getOrigin().getZ()
									<< std::endl;
							std::cout << "variance1To8: " << variance1To8
									<< std::endl;
						}
					}
					if (marker_flag2) {
						if (variance2To8 == 0.05) {
							ROS_WARN("We lost marker_2 or marker_8!");
							flag_pub2 = false;
						} else {
							flag_pub2 = true;

							tf::Vector3 m0 = tf::Matrix3x3(
									transform2To8.getRotation()).getColumn(0);
							tf::Vector3 m1 = tf::Matrix3x3(
									transform2To8.getRotation()).getColumn(1);
							tf::Vector3 m2 = tf::Matrix3x3(
									transform2To8.getRotation()).getColumn(2);
							T8To2(0, 0) = m0.getX();
							T8To2(1, 0) = m0.getY();
							T8To2(2, 0) = m0.getZ();
							T8To2(0, 1) = m1.getX();
							T8To2(1, 1) = m1.getY();
							T8To2(2, 1) = m1.getZ();
							T8To2(0, 2) = m2.getX();
							T8To2(1, 2) = m2.getY();
							T8To2(2, 2) = m2.getZ();
							T8To2(0, 3) = transform2To8.getOrigin().getX();
							T8To2(1, 3) = transform2To8.getOrigin().getY();
							T8To2(2, 3) = transform2To8.getOrigin().getZ();

							TCToE_2 = T2ToE * T8To2 * TUcTo8;

							double m00, m01, m02, m10, m11, m12, m20, m21, m22;
							m00 = TCToE_2(0, 0);
							m01 = TCToE_2(0, 1);
							m02 = TCToE_2(0, 2);
							m10 = TCToE_2(1, 0);
							m11 = TCToE_2(1, 1);
							m12 = TCToE_2(1, 2);
							m20 = TCToE_2(2, 0);
							m21 = TCToE_2(2, 1);
							m22 = TCToE_2(2, 2);
							tf::Quaternion quat;
							tf::Matrix3x3(m00, m01, m02, m10, m11, m12, m20,
									m21, m22).getRotation(quat);
							geometry_msgs::Quaternion msgQuat;
							tf::quaternionTFToMsg(quat, msgQuat);

							upperPartPoseInRightEndEffectorFrame.orientation =
									msgQuat;
							upperPartPoseInRightEndEffectorFrame.position.x =
									TCToE_2(0, 3);
							upperPartPoseInRightEndEffectorFrame.position.y =
									TCToE_2(1, 3);
							upperPartPoseInRightEndEffectorFrame.position.z =
									TCToE_2(2, 3);

							std::cout
									<< "The transformation from marker_2 to marker_8 in marker_2 frame: [x, y, z] "
									<< transform2To8.getOrigin().getX() << ", "
									<< transform2To8.getOrigin().getY() << ", "
									<< transform2To8.getOrigin().getZ()
									<< std::endl;
							std::cout << "variance2To8: " << variance2To8
									<< std::endl;
						}
					}
					if (marker_flag3) {
						if (variance3To8 == 0.05) {
							ROS_WARN("We lost marker_3 or marker_8!");
							flag_pub3 = false;
						} else {
							flag_pub3 = true;

							tf::Vector3 m0 = tf::Matrix3x3(
									transform3To8.getRotation()).getColumn(0);
							tf::Vector3 m1 = tf::Matrix3x3(
									transform3To8.getRotation()).getColumn(1);
							tf::Vector3 m2 = tf::Matrix3x3(
									transform3To8.getRotation()).getColumn(2);
							T8To3(0, 0) = m0.getX();
							T8To3(1, 0) = m0.getY();
							T8To3(2, 0) = m0.getZ();
							T8To3(0, 1) = m1.getX();
							T8To3(1, 1) = m1.getY();
							T8To3(2, 1) = m1.getZ();
							T8To3(0, 2) = m2.getX();
							T8To3(1, 2) = m2.getY();
							T8To3(2, 2) = m2.getZ();
							T8To3(0, 3) = transform3To8.getOrigin().getX();
							T8To3(1, 3) = transform3To8.getOrigin().getY();
							T8To3(2, 3) = transform3To8.getOrigin().getZ();

							TCToE_3 = T3ToE * T8To3 * TUcTo8;

							double m00, m01, m02, m10, m11, m12, m20, m21, m22;
							m00 = TCToE_3(0, 0);
							m01 = TCToE_3(0, 1);
							m02 = TCToE_3(0, 2);
							m10 = TCToE_3(1, 0);
							m11 = TCToE_3(1, 1);
							m12 = TCToE_3(1, 2);
							m20 = TCToE_3(2, 0);
							m21 = TCToE_3(2, 1);
							m22 = TCToE_3(2, 2);
							tf::Quaternion quat;
							tf::Matrix3x3(m00, m01, m02, m10, m11, m12, m20,
									m21, m22).getRotation(quat);
							geometry_msgs::Quaternion msgQuat;
							tf::quaternionTFToMsg(quat, msgQuat);

							upperPartPoseInRightEndEffectorFrame.orientation =
									msgQuat;
							upperPartPoseInRightEndEffectorFrame.position.x =
									TCToE_3(0, 3);
							upperPartPoseInRightEndEffectorFrame.position.y =
									TCToE_3(1, 3);
							upperPartPoseInRightEndEffectorFrame.position.z =
									TCToE_3(2, 3);

							std::cout
									<< "The transformation from marker_3 to marker_8 in marker_3 frame: [x, y, z] "
									<< transform3To8.getOrigin().getX() << ", "
									<< transform3To8.getOrigin().getY() << ", "
									<< transform3To8.getOrigin().getZ()
									<< std::endl;
							std::cout << "variance3To8: " << variance3To8
									<< std::endl;
						}
					}

					if (marker_flag9) {
						if (variance9To8 == 0.05) {
							ROS_WARN("We lost marker_9 or marker_8!");
							flag_pub_UcLc=false;
						} else {
							flag_pub_UcLc=true;
							tf::Vector3 m0 = tf::Matrix3x3(
									transform9To8.getRotation()).getColumn(0);
							tf::Vector3 m1 = tf::Matrix3x3(
									transform9To8.getRotation()).getColumn(1);
							tf::Vector3 m2 = tf::Matrix3x3(
									transform9To8.getRotation()).getColumn(2);
							T9To8(0, 0) = m0.getX();
							T9To8(1, 0) = m0.getY();
							T9To8(2, 0) = m0.getZ();
							T9To8(0, 1) = m1.getX();
							T9To8(1, 1) = m1.getY();
							T9To8(2, 1) = m1.getZ();
							T9To8(0, 2) = m2.getX();
							T9To8(1, 2) = m2.getY();
							T9To8(2, 2) = m2.getZ();
							T9To8(0, 3) = transform9To8.getOrigin().getX();
							T9To8(1, 3) = transform9To8.getOrigin().getY();
							T9To8(2, 3) = transform9To8.getOrigin().getZ();

							TLcToUc = TUcTo8.inverse() * T9To8 * TLcTo9;

							double m00, m01, m02, m10, m11, m12, m20, m21, m22;
							m00 = TLcToUc(0, 0);
							m01 = TLcToUc(0, 1);
							m02 = TLcToUc(0, 2);
							m10 = TLcToUc(1, 0);
							m11 = TLcToUc(1, 1);
							m12 = TLcToUc(1, 2);
							m20 = TLcToUc(2, 0);
							m21 = TLcToUc(2, 1);
							m22 = TLcToUc(2, 2);
							tf::Quaternion quat;
							tf::Matrix3x3(m00, m01, m02, m10, m11, m12, m20,
									m21, m22).getRotation(quat);
							geometry_msgs::Quaternion msgQuat;
							tf::quaternionTFToMsg(quat, msgQuat);

							lowerPartPoseInUpperPartFrame.orientation = msgQuat;
							lowerPartPoseInUpperPartFrame.position.x = TLcToUc(
									0, 3);
							lowerPartPoseInUpperPartFrame.position.y = TLcToUc(
									1, 3);
							lowerPartPoseInUpperPartFrame.position.z = TLcToUc(
									2, 3);

							std::cout
									<< "The position of marker_9 in marker_8 frame: [x, y, z] "
									<< transform9To8.getOrigin().getX() << ", "
									<< transform9To8.getOrigin().getY() << ", "
									<< transform9To8.getOrigin().getZ()
									<< std::endl;
							std::cout << "variance9To8: " << variance9To8
									<< std::endl;
						}
					}

					//publish ros msg
					if (flag_pub0 || flag_pub1 || flag_pub2 || flag_pub3) {
						upperPartPoseInRightEndEffectorFrame_pub.publish(
								upperPartPoseInRightEndEffectorFrame);
					}
					if (flag_pub_UcLc) {
						lowerPartPoseInUpperPartFrame_pub.publish(
								lowerPartPoseInUpperPartFrame);
					}

					if (marker_flag0 && marker_flag1) {
						ROS_ERROR(
								"***********************I see two markers 0 and 1*****************************");
					}
					if (marker_flag0 && marker_flag3) {
						ROS_ERROR(
								"***********************I see two markers 0 and 3*****************************");
					}
					if (marker_flag1 && marker_flag2) {
						ROS_ERROR(
								"***********************I see two markers 1 and 2*****************************");
					}
					if (marker_flag2 && marker_flag3) {
						ROS_ERROR(
								"***********************I see two markers 2 and 3*****************************");
					}

				}
			}
			if (!flag) {
				ROS_ERROR("I cannot see the upper object!");
				rate.sleep();
				continue;
			}
		}

		rate.sleep();
	}
	return 0;
}
;
