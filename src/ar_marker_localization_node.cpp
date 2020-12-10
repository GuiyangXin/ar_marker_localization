#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#define Num 20

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

	ros::Rate rate(100.0);
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
								listener.lookupTransform("/ar_marker_8",
										"/ar_marker_0", ros::Time(0),
										transform0To8);
							} catch (tf::TransformException &ex) {
								ROS_ERROR("%s", ex.what());
//								rate.sleep();
//								continue;
							}
						} else if (subId == "ar_marker_1") {
							flag2 = true;
							try {
								listener.lookupTransform("/ar_marker_8",
										"/ar_marker_1", ros::Time(0),
										transform1To8);
							} catch (tf::TransformException &ex) {
								ROS_ERROR("%s", ex.what());
//								rate.sleep();
//								continue;
							}
						} else if (subId == "ar_marker_2") {
							flag2 = true;
							try {
								listener.lookupTransform("/ar_marker_8",
										"/ar_marker_2", ros::Time(0),
										transform2To8);
							} catch (tf::TransformException &ex) {
								ROS_ERROR("%s", ex.what());
//								rate.sleep();
//								continue;
							}
						} else if (subId == "ar_marker_3") {
							flag2 = true;
							try {
								listener.lookupTransform("/ar_marker_8",
										"/ar_marker_3", ros::Time(0),
										transform3To8);
							} catch (tf::TransformException &ex) {
								ROS_ERROR("%s", ex.what());
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
					for (int i = 0; i < Num; i++) {
						if (i == Num - 1) {
							cache0To8[i] = transform0To8.getOrigin().getX();
						} else {
							cache0To8[i] = cache0To8[i + 1];
						}

					}
					sum = 0;
					for (int i = 0; i < Num; i++) {
						sum += cache0To8[i];
					}
					average = 0;
					average = sum / Num;
					for (int i = 0; i < Num; i++) {
						variance0To8 += (cache0To8[i] - average)
								* (cache0To8[i] - average);
					}
					variance0To8 /= Num;
					if (variance0To8 == 0) {
						ROS_ERROR("We lost marker_0 or marker_8!");
					}

					for (int i = 0; i < Num; i++) {
						if (i == Num - 1) {
							cache1To8[i] = transform1To8.getOrigin().getX();
						} else {
							cache1To8[i] = cache1To8[i + 1];
						}
					}
					sum = 0;
					for (int i = 0; i < Num; i++) {
						sum += cache1To8[i];
					}
					average = 0;
					average = sum / Num;
					for (int i = 0; i < Num; i++) {
						variance1To8 += (cache1To8[i] - average)
								* (cache1To8[i] - average);
					}
					variance1To8 /= Num;
					if (variance1To8 == 0) {
						ROS_ERROR("We lost marker_1 or marker_8!");
					}

					for (int i = 0; i < Num; i++) {
						if (i == Num - 1) {
							cache2To8[i] = transform2To8.getOrigin().getX();
						} else {
							cache2To8[i] = cache2To8[i + 1];
						}
					}
					sum = 0;
					for (int i = 0; i < Num; i++) {
						sum += cache2To8[i];
					}
					average = 0;
					average = sum / Num;
					for (int i = 0; i < Num; i++) {
						variance2To8 += (cache2To8[i] - average)
								* (cache2To8[i] - average);
					}
					variance2To8 /= Num;
					if (variance2To8 == 0) {
						ROS_ERROR("We lost marker_2 or marker_8!");
					}

					for (int i = 0; i < Num; i++) {
						if (i == Num - 1) {
							cache3To8[i] = transform3To8.getOrigin().getX();
						} else {
							cache3To8[i] = cache3To8[i + 1];
						}
					}
					sum = 0;
					for (int i = 0; i < Num; i++) {
						sum += cache3To8[i];
					}
					average = 0;
					average = sum / Num;
					for (int i = 0; i < Num; i++) {
						variance3To8 += (cache3To8[i] - average)
								* (cache3To8[i] - average);
					}
					variance3To8 /= Num;
					if (variance3To8 == 0) {
						ROS_ERROR("We lost marker_3 or marker_8!");
					}

					if (variance0To8 == 0 && variance1To8 == 0
							&& variance2To8 == 0 && variance3To8 == 0) {
						ROS_ERROR("We lost all the  markers!");
						continue;
					}
					if (variance0To8 != 0) {
						std::cout
								<< "The transformation from marker_0 to marker_8: "
								<< transform0To8.getOrigin().getX()
								<< std::endl;
					}
					if (variance1To8 != 0) {
						std::cout
								<< "The transformation from marker_1 to marker_8: "
								<< transform1To8.getOrigin().getX()
								<< std::endl;
					}
					if (variance2To8 != 0) {
						std::cout
								<< "The transformation from marker_2 to marker_8: "
								<< transform2To8.getOrigin().getX()
								<< std::endl;
					}
					if (variance3To8 != 0) {
						std::cout
								<< "The transformation from marker_3 to marker_8: "
								<< transform3To8.getOrigin().getX()
								<< std::endl;
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
