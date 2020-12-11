#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#define Num 20

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

//  ros::service::waitForService("spawn");
//  ros::ServiceClient add_turtle =
//    node.serviceClient<turtlesim::Spawn>("spawn");
//  turtlesim::Spawn srv;
//  add_turtle.call(srv);
//
//  ros::Publisher turtle_vel =
//    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

	tf::TransformListener listener;
	tf::StampedTransform transformB2M8;
	tf::StampedTransform transform0To8;
	tf::StampedTransform transform1To8;
	tf::StampedTransform transform2To8;
	tf::StampedTransform transform3To8;
	tf::StampedTransform transform;
	std::vector<std::string> frames;
	double cache0To8[Num] = { 0 };
	double cache1To8[Num] = { 0 };
	double cache2To8[Num] = { 0 };
	double cache3To8[Num] = { 0 };
	double variance0To8 = 0;
	double variance1To8 = 0;
	double variance2To8 = 0;
	double variance3To8 = 0;
	double sum = 0;
	double average = 0;
	bool flag = false;
	bool flag2 = false;
	bool flag_pub0=false;
	bool flag_pub1=false;
	bool flag_pub2=false;
	bool flag_pub3=false;
	bool marker_flag0 = false;
	bool marker_flag1 = false;
	bool marker_flag2 = false;
	bool marker_flag3 = false;

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
										marker_flag0=true;
							} catch (tf::TransformException &ex) {
								ROS_WARN("%s", ex.what());
								marker_flag0=false;
//								rate.sleep();
//								continue;
							}
						} else if (subId == "ar_marker_1") {
							flag2 = true;
							try {
								listener.lookupTransform("/ar_marker_1",
										"/ar_marker_8", ros::Time(0),
										transform1To8);
										marker_flag1=true;
							} catch (tf::TransformException &ex) {
								ROS_WARN("%s", ex.what());
								marker_flag1=false;
//								rate.sleep();
//								continue;
							}
						} else if (subId == "ar_marker_2") {
							flag2 = true;
							try {
								listener.lookupTransform("/ar_marker_2",
										"/ar_marker_8", ros::Time(0),
										transform2To8);
										marker_flag2=true;
							} catch (tf::TransformException &ex) {
								ROS_WARN("%s", ex.what());
								marker_flag2=false;
//								rate.sleep();
//								continue;
							}
						} else if (subId == "ar_marker_3") {
							flag2 = true;
							try {
								listener.lookupTransform("/ar_marker_3",
										"/ar_marker_8", ros::Time(0),
										transform3To8);
										marker_flag3=true;
							} catch (tf::TransformException &ex) {
								ROS_WARN("%s", ex.what());
								marker_flag3=false;
//								rate.sleep();
//								continue;
							}
						}
					}

					if (!flag2) {
						ROS_ERROR("I cannot localise the robot!");
						continue;
					}

					//compute the transformation
					if(marker_flag0){
					computeVariance(variance0To8,
							transform0To8.getOrigin().getX(), cache0To8);
							}else{
								ROS_WARN("I cannot see marker0!");
							}
							if(marker_flag1){
					computeVariance(variance1To8,
							transform1To8.getOrigin().getX(), cache1To8);
							}else{
								ROS_WARN("I cannot see marker1!");
							}
							if(marker_flag2){
					computeVariance(variance2To8,
							transform2To8.getOrigin().getX(), cache2To8);
							}else{
								ROS_WARN("I cannot see marker2!");
							}
							if(marker_flag3){
					computeVariance(variance3To8,
							transform3To8.getOrigin().getX(), cache3To8);
							}else{
								ROS_WARN("I cannot see marker3!");
							}

					//Publish the relative tranformation between the end-effector and the object
					if (variance0To8 == 0.05 && variance1To8 == 0.05
							&& variance2To8 == 0.05 && variance3To8 == 0.05) {
						ROS_ERROR("We lost all the  markers!");
						continue;
					}
					if(marker_flag0){
						if (variance0To8 == 0.05) {
							ROS_WARN("We lost marker_0 or marker_8!");
							flag_pub0=false;
						} else {
							flag_pub0=true;
							std::cout
									<< "The transformation from marker_0 to marker_8 in marker_0 frame: [x, y, z] "
									<< transform0To8.getOrigin().getX()<<", "<<transform0To8.getOrigin().getY()<<", "<<transform0To8.getOrigin().getZ()
									<< std::endl;
							std::cout << "variance0To8: " << variance0To8
									<< std::endl;
						}
					}
					if(marker_flag1){
						if (variance1To8 == 0.05) {
							ROS_WARN("We lost marker_1 or marker_8!");
							flag_pub1=false;
						} else {
							flag_pub1=true;
							std::cout
									<< "The transformation from marker_1 to marker_8 in marker_1 frame: [x, y, z] "
									<< transform1To8.getOrigin().getX()<<", "<<transform1To8.getOrigin().getY()<<", "<<transform1To8.getOrigin().getZ()
									<< std::endl;
							std::cout << "variance1To8: " << variance1To8
									<< std::endl;
						}
					}
					if(marker_flag2){
						if (variance2To8 == 0.05) {
							ROS_WARN("We lost marker_2 or marker_8!");
							flag_pub2=false;
						} else {
							flag_pub2=true;
							std::cout
									<< "The transformation from marker_2 to marker_8 in marker_2 frame: [x, y, z] "
									<< transform2To8.getOrigin().getX()<<", "<<transform2To8.getOrigin().getY()<<", "<<transform2To8.getOrigin().getZ()
									<< std::endl;
							std::cout << "variance2To8: " << variance2To8
									<< std::endl;
						}
					}
					if(marker_flag3){
						if (variance3To8 == 0.05) {
							ROS_WARN("We lost marker_3 or marker_8!");
							flag_pub3=false;
						} else {
							flag_pub3=true;
							std::cout
									<< "The transformation from marker_3 to marker_8 in marker_3 frame: [x, y, z] "
									<< transform3To8.getOrigin().getX()<<", "<<transform3To8.getOrigin().getY()<<", "<<transform3To8.getOrigin().getZ()
									<< std::endl;
							std::cout << "variance3To8: " << variance3To8
									<< std::endl;
						}
					}

				}
			}
			if (!flag) {
				ROS_ERROR("I cannot see the object!");
				continue;
			}
		}

		rate.sleep();
	}
	return 0;
}
;
