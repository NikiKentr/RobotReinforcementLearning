using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class KeyboardControl5 : MonoBehaviour
{
    public ArticulationWheelController wheelController;

    public float speed = 1.5f;
    public float angularSpeed = 1.5f;
    private float targetLinearSpeed;
    private float targetAngularSpeed;

    public Transform point2; // Reference to the second point

    public GameObject robotBase; // Reference to the robot's base

    private ROSConnection ros;
    public string poseTopic = "unity/robot2_pose"; // ROS topic to publish pose messages for the second robot

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(poseTopic);
    }

    void Update()
    {
        // Get key input
        targetLinearSpeed = Input.GetAxisRaw("Vertical2") * speed;
        targetAngularSpeed = -Input.GetAxisRaw("Horizontal2") * angularSpeed;

        // Log the position of the robot's base (debugging)
        Vector3 basePosition = robotBase.transform.position;
        Quaternion baseRotation = robotBase.transform.rotation; // Get the rotation of the robot's base
        Debug.Log($"Robot Base Position: {basePosition}");

        // Log the positions of the points
        Vector3 point2Position = point2.position;
        Debug.Log($"Point2 Position: {point2Position}");

        // Publish pose message with position and orientation
        PublishPoseMessage(basePosition, baseRotation);
    }

    void FixedUpdate()
    {
        wheelController.SetRobotVelocity(targetLinearSpeed, targetAngularSpeed);
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
                    z = baseRotation.y,
                    w = baseRotation.w
                }
            }
        };

        ros.Publish(poseTopic, poseMessage);
    }
}
