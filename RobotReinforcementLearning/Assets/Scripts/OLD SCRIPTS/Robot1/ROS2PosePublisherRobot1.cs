using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System.Security.Cryptography;

public class ROS2PosePublisherRobot1 : MonoBehaviour
{
    public Transform robotBase; // reference to the Transform representing the robot's base
    public ROSConnection rosA;
    public string poseTopic = "unity/robot_pose"; // ROS topic to publish pose messages for the second robot

    void Start()
    {
        rosA = ROSConnection.GetOrCreateInstance();
        rosA.RegisterPublisher<PoseStampedMsg>(poseTopic);
    }

    void Update()
    {
        // Log the position of the robot's base (debugging)
        Vector3 basePosition = robotBase.position;
        Quaternion baseRotation = robotBase.rotation; // Get the rotation of the robot's base
        Debug.Log($"Robot Base Position: {basePosition}");

        // Publish pose message with position and orientation
        PublishPoseMessage(basePosition, baseRotation);
    }

    void PublishPoseMessage(Vector3 basePosition, Quaternion baseRotation)
    {
        PoseStampedMsg poseMessage = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time - Mathf.Floor(Time.time)) * 1e9)
                },
                frame_id = "base_link"
            },
            pose = new PoseMsg
            {
                position = new PointMsg { x = basePosition.x, y = basePosition.z, z = 0.0 },
                orientation = new QuaternionMsg
                {
                    x = 0.0,
                    y = 0.0,
                    z = baseRotation.y, //coordinate system in unity is different
                    w = baseRotation.w
                }
            }
        };

        rosA.Publish(poseTopic, poseMessage);
    }
}