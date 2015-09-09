#include <cstdlib>

#include <ros/ros.h>

#include <designators/Designator.h>
#include <designator_integration_msgs/Designator.h>

#include <tf/tf.h>


bool g_bReceivedFirstMessage;
designator_integration_msgs::Designator g_dsgLastMessage;

ros::Subscriber g_subSubscriber;
ros::Publisher g_pubPublisher;


void republishLastMessage() {
  g_pubPublisher.publish(g_dsgLastMessage);
}


designator_integration_msgs::Designator transformDesignatorMessage(designator_integration_msgs::Designator dsgMessage) {
  designator_integration::Designator* desigData = new designator_integration::Designator(dsgMessage);
  
  // Do some transformations here (tf, ...).
  
  designator_integration_msgs::Designator dsgReturn = desigData->serializeToMessage();
  delete desigData;
  
  return dsgReturn;
}


void subscriptionCallback(const designator_integration_msgs::Designator& dsgMessage) {
  if(!g_bReceivedFirstMessage) {
    g_bReceivedFirstMessage = true;
    ROS_INFO("Received first message, starting to republish messages.");
  }
  
  g_dsgLastMessage = transformDesignatorMessage(dsgMessage);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "designator_relay_publisher");
  ros::NodeHandle nhHandle("~");
  
  g_bReceivedFirstMessage = false;
  std::string strNode = argv[0];
  
  if(argc == 3) {
    std::string strSourceTopic = argv[1];
    std::string strTargetTopic = argv[2];
    
    g_subSubscriber = nhHandle.subscribe(strSourceTopic, 1, subscriptionCallback);
    g_pubPublisher = nhHandle.advertise<designator_integration_msgs::Designator>(strTargetTopic, 1);
    
    ROS_INFO("Designator Relay Publisher started (%s -> %s).", strSourceTopic.c_str(), strTargetTopic.c_str());
    
    ros::Duration durDuration(1.0); // One second
    ros::Time timStart = ros::Time::now();
    ros::Time timCurrent;
    
    while(ros::ok()) {
      ros::spinOnce();
      
      timCurrent = ros::Time::now();
      if(timCurrent - timStart >= durDuration) {
	timStart = timCurrent;
	
	if(g_bReceivedFirstMessage) {
	  republishLastMessage();
	}
      }
    }
  } else {
    ROS_ERROR("Usage: %s <source-topic> <target-topic>", strNode.c_str());
  }
  
  return EXIT_SUCCESS;
}
