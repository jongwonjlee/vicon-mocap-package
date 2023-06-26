/*
    vicon_mocap_package
    Copyright (C) 2023 Jongwon Lee

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Author: David Hanley (dhanley@ed.ac.uk)
    Maintainer: Jongwon Lee (jongwon5@illinois.edu)
    Last date of change: 06/26/2023
*/
#include <iostream>


// Vicon Includes
#include "ViconDataStreamSDK/Linux64/DataStreamClient.h"

// ROS Includes
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "vicon_mocap_package/quality_msg.h"

using namespace std;
using namespace ViconDataStreamSDK::CPP;

// Initialize the message type
geometry_msgs::PoseStamped mocap_msg;
vicon_mocap_package::quality_msg mocap_qual;


int main(int argc, char **argv)
{
    // The ros::init() function needs to see argc and argv so that it can perform
    // any ROS arguments and name remapping that were provided at the command line.
    // For programmatic remappings you can use a different version of init() which takes
    // remappings directly, but for most command-line programs, passing argc and argv is
    // the easiest way to do it.  The third argument to init() is the name of the node.
    //
    // You must call one of the versions of ros::init() before using any other
    // part of the ROS system.
    ros::init(argc, argv, "talker");

    // Initialize the ROS handle
    ros::NodeHandle nh;
    
    // Set up ROS node and publisher 
    ros::Publisher pub_mocap1 = nh.advertise<geometry_msgs::PoseStamped>("mocap/posestamped", 1000);
    ros::Publisher pub_mocap2 = nh.advertise<vicon_mocap_package::quality_msg>("mocap/qualityscore", 1000);

    // Create a ViconDataStreamSDK Client Object
    Client ViconClient;
    Output_GetVersion Output_ver = ViconClient.GetVersion();
    cout << "Version: " << Output_ver.Major << "."
                        << Output_ver.Minor << "."
                        << Output_ver.Point << "."
                        << Output_ver.Revision << endl;

    // Connect to Vicon Server
    string host = "192.168.1.2:801"; // Change this to the actual host name or IP address
    Output_Connect Output_conn = ViconClient.Connect( host );

    // Check if system is connected to vicon server
    Output_IsConnected Output_isconn = ViconClient.IsConnected();
    if (Output_isconn.Connected == false)
    {
	cout << "System Failed to Connect to Vicon Server" << endl;
    return 0;
    }    
    else
    {
	cout << "System Successfully Connected to Vicon Server" << endl;
    }

    // Enable kinematic segment data in the Vicon DataStream
    Output_EnableSegmentData Output_enseg = ViconClient.EnableSegmentData();

    // Check if kinematic segment data is enabled
    Output_IsSegmentDataEnabled Output_issegen = ViconClient.IsSegmentDataEnabled();
    if (Output_issegen.Enabled == false)
    {
	cout << "Segment Data Not Enabled" << endl;
    }
    else
    {
	cout << "Segment Data is Enabled" << endl;
    }


    // Set the Vicon Stream Mode
    ViconClient.SetStreamMode( StreamMode::ServerPush );
    //ViconClient.SetStreamMode( StreamMode::ClientPull );
    //ViconClient.SetStreamMode( StreamMode::ClientPullPreFetch );

    // Set the Axis Mapping. Here we are using the default, mapping.
    ViconClient.SetAxisMapping( Direction::Forward, Direction::Left, Direction::Up );

    while(true)
    {
	// Request a new frame to be fetched from the Vicon DataStream server
    	Output_GetFrame getframe = ViconClient.GetFrame();

    	// Check if GetFrame is a Success 
    	if (getframe.Result == Result::Success)
    	{	
        	// Get the frame number
        	int frame_number = ViconClient.GetFrameNumber().FrameNumber;
        	cout << "Frame Number: " << frame_number << endl;
		mocap_msg.header.seq = frame_number;

		// Get the frame rate
		double frame_rate = ViconClient.GetFrameRate().FrameRateHz;
		cout << "Frame Rate: " << frame_rate << endl;

        	// Get the number of subjects
		int subject_count = ViconClient.GetSubjectCount().SubjectCount;
		cout << "Number of subjects: " << subject_count << endl;

		// Get the name of the subjects
		string subject_name = ViconClient.GetSubjectName(0).SubjectName;
	
		// Get the number of segments
		Output_GetSegmentCount seg_count = ViconClient.GetSegmentCount( "Test_1" );

		// Get the segment name
		string seg_name = ViconClient.GetSegmentName(subject_name, 0).SegmentName;
        	cout << "Object Name: " << seg_name << endl;
		mocap_msg.header.frame_id = seg_name;

		// Get the global translation in mm
		Output_GetSegmentGlobalTranslation segmentGlobalTrans = ViconClient.GetSegmentGlobalTranslation(subject_name, seg_name); 
                mocap_msg.pose.position.x = segmentGlobalTrans.Translation[0] / 1000.0;
		mocap_msg.pose.position.y = segmentGlobalTrans.Translation[1] / 1000.0;
		mocap_msg.pose.position.z = segmentGlobalTrans.Translation[2] / 1000.0;
        	cout << "Position (m): " << mocap_msg.pose.position.x << ", " << mocap_msg.pose.position.y << ", " << mocap_msg.pose.position.z << endl;
	
		// Get the global quaternion of the form (x, y, z, w) where w is the real component 
		// and x, y, z are the imaginary components. Note that this is different from that used 
		// in many other applications, which use (w, x, y, z).
		Output_GetSegmentGlobalRotationQuaternion segmentGlobalQuat = ViconClient.GetSegmentGlobalRotationQuaternion(subject_name, seg_name);
                mocap_msg.pose.orientation.x = segmentGlobalQuat.Rotation[0];
 		mocap_msg.pose.orientation.y = segmentGlobalQuat.Rotation[1];
		mocap_msg.pose.orientation.z = segmentGlobalQuat.Rotation[2];
		mocap_msg.pose.orientation.w = segmentGlobalQuat.Rotation[3];
		cout << "Orientation (x,y,z,w): " << mocap_msg.pose.orientation.x << ", " << mocap_msg.pose.orientation.y << ", " << mocap_msg.pose.orientation.z << ", " << mocap_msg.pose.orientation.w << endl;

		// Get the rotation of a subject in global Euler XYZ coordinates
		//Output_GetSegmentGlobalRotationEulerXYZ segmentGlobalEuler = ViconClient.GetSegmentGlobalRotationEulerXYZ(subject_name, seg_name);

		// Get Object Quality Score
		// The quality score is the RMS error of a rigid body compared to its model
		double subQuality = ViconClient.GetObjectQuality( subject_name ).Quality;
                mocap_qual.header.seq = frame_number;
		mocap_qual.quality_score.data = subQuality;
        	cout << "Quality Score: " << mocap_qual.quality_score.data << endl;
                cout << "-----------------------------------------------\n";

		// Publish Mocap data as a ROS topic
		pub_mocap1.publish(mocap_msg);
		pub_mocap2.publish(mocap_qual);
    	}
    	else
    	{
		cout << "Did not get a new frame!" << endl;
		cout << "------------------------" << endl;
    	}
    }
    // Disconnect Vicon Server
    Output_Disconnect Output_dis = ViconClient.Disconnect();

    return 0;
}

