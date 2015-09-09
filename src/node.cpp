#include <cstdlib>

#include <ros/ros.h>

#include <designators/Designator.h>
#include <designator_integration_msgs/DesignatorResponse.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


bool g_bReceivedFirstMessage;
designator_integration_msgs::DesignatorResponse g_dsgrLastMessage;

ros::Subscriber g_subSubscriber;
ros::Publisher g_pubPublisher;

tf::Transformer g_tfrTF(true, ros::Duration(1000.0));


void republishLastMessage() {
  g_pubPublisher.publish(g_dsgrLastMessage);
}


geometry_msgs::PoseStamped transformPose(geometry_msgs::PoseStamped psPose, std::string strTargetFrame) {
  geometry_msgs::PoseStamped psReturn;
  
  std::string strError = "";
  if(g_tfrTF.waitForTransform(strTargetFrame, psPose.header.frame_id, psPose.header.stamp, ros::Duration(100.0), ros::Duration(0.01), &strError)) {
    try {
      tf::Stamped<tf::Pose> spPose, spReturn;
      poseStampedMsgToTF(psPose, spPose);
      
      g_tfrTF.transformPose(strTargetFrame, spPose, spReturn);
      
      poseStampedTFToMsg(spReturn, psReturn);
    } catch (tf::TransformException texException) {
      ROS_ERROR("Error while transforming pose from frame '%s' to '%s': %s", psPose.header.frame_id.c_str(), strTargetFrame.c_str(), texException.what());
    }
  } else {
    ROS_WARN("Didn't find transform from frame '%s' to '%s': %s", psPose.header.frame_id.c_str(), strTargetFrame.c_str(), strError.c_str());
  }
  
  return psReturn;
}


bool transformDesignatorPose(designator_integration::KeyValuePair* kvpTransform, std::string strPoseField, std::string strTargetFrame) {
  if(kvpTransform->childForKey(strPoseField)) {
    kvpTransform->setValue(strPoseField, transformPose(kvpTransform->poseStampedValue(strPoseField), strTargetFrame));
    
    return true;
  }
  
  return false;
}


designator_integration_msgs::DesignatorResponse transformDesignatorMessage(designator_integration_msgs::DesignatorResponse dsgrMessage) {
  designator_integration_msgs::DesignatorResponse dsgrReturn;
  
  for(designator_integration_msgs::Designator dsgDesignator : dsgrMessage.designators) {
    designator_integration::Designator* desigData = new designator_integration::Designator(dsgDesignator);
    
    //transformDesignatorPose(desigData, "pose", "/map");
    //transformDesignatorPose(desigData->childForKey("boundingbox"), "pose", "/map");
    
    desigData->printDesignator();
    std::cout << std::endl;
    
    dsgrReturn.designators.push_back(desigData->serializeToMessage());
    delete desigData;
  }
  
  return dsgrReturn;
}


void subscriptionCallback(const designator_integration_msgs::DesignatorResponse& dsgrMessage) {
  if(!g_bReceivedFirstMessage) {
    g_bReceivedFirstMessage = true;
    ROS_INFO("Received first message, starting to republish messages.");
  }
  
  g_dsgrLastMessage = transformDesignatorMessage(dsgrMessage);
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
    g_pubPublisher = nhHandle.advertise<designator_integration_msgs::DesignatorResponse>(strTargetTopic, 1);
    
    ROS_INFO("Designator Relay Publisher started (%s -> %s).", strSourceTopic.c_str(), strTargetTopic.c_str());
    
    ros::Duration durDuration(1.0); // One second
    ros::Time timStart = ros::Time::now();
    ros::Time timCurrent;
    
    while(nhHandle.ok()) {
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
