using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;

public class ROSOdometryPublisher : MonoBehaviour
{
    public Transform robotBase; // Reference to the Transform representing the robot's base
    public ROSConnection rosConnection; // Reference to the specific ROSConnection instance
    public string odometryTopic; // ROS topic to publish odometry messages for the robot

    private MonoBehaviour keyboardControl; // Reference to the keyboard control script (could be KeyboardControlRobot1 or KeyboardControlRobot2)

    public float publishInterval = 0.5f; // Publish interval in seconds (500 milliseconds)
    private float lastPublishTime;

    void Start()
    {
        if (rosConnection == null)
        {
            Debug.LogError("ROSConnection is not assigned.");
            return;
        }

        // Register the publisher for Odometry messages
        rosConnection.RegisterPublisher<OdometryMsg>(odometryTopic);

        // Try to find the KeyboardControlRobot1 or KeyboardControlRobot2 script attached to the same GameObject
        keyboardControl = GetComponent<KeyboardControlRobot1>(); // Try to find KeyboardControlRobot1
        if (keyboardControl == null)
        {
            keyboardControl = GetComponent<KeyboardControlRobot2>(); // Try to find KeyboardControlRobot2
        }

        if (keyboardControl == null)
        {
            Debug.LogError("Keyboard control script (KeyboardControlRobot1 or KeyboardControlRobot2) is not found on this GameObject.");
        }
        else
        {
            Debug.Log("Keyboard control script found.");
        }

        lastPublishTime = Time.time;
    }

    void Update()
    {
        if (keyboardControl == null)
        {
            Debug.LogError("Keyboard control script is not assigned.");
            return;
        }

        // Check if enough time has passed since last publish
        if (Time.time - lastPublishTime > publishInterval)
        {
            Vector3 basePosition = robotBase.position;
            Quaternion baseRotation = robotBase.rotation;

            // Retrieve the linear and angular velocities from the keyboard control script dynamically
            float targetLinearSpeed = (float)keyboardControl.GetType().GetProperty("targetLinearSpeed").GetValue(keyboardControl);
            float targetAngularSpeed = (float)keyboardControl.GetType().GetProperty("targetAngularSpeed").GetValue(keyboardControl);

            Vector3 linearVelocity = targetLinearSpeed * robotBase.forward;
            Vector3 angularVelocity = new Vector3(0, targetAngularSpeed, 0);

            PublishOdometryMessage(basePosition, baseRotation, linearVelocity, angularVelocity);

            // Update last publish time
            lastPublishTime = Time.time;
        }
    }

    void PublishOdometryMessage(Vector3 basePosition, Quaternion baseRotation, Vector3 linearVel, Vector3 angularVel)
    {
        // Create Odometry message
        OdometryMsg odometryMessage = new OdometryMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time - Mathf.Floor(Time.time)) * 1e9)
                },
                frame_id = "odom"
            },
            child_frame_id = "base_link",
            pose = new PoseWithCovarianceMsg
            {
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
                },
                covariance = new double[36] // Assuming zero covariance for simplicity
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = new Vector3Msg { x = linearVel.x, y = linearVel.y, z = linearVel.z },
                    angular = new Vector3Msg { x = angularVel.x, y = angularVel.y, z = angularVel.z }
                },
                covariance = new double[36] // Assuming zero covariance for simplicity
            }
        };

        // Log the message being published
        Debug.Log($"Publishing Odometry Message: Linear Vel - {odometryMessage.twist.twist.linear}, Angular Vel - {odometryMessage.twist.twist.angular}");

        // Publish the Odometry message
        rosConnection.Publish(odometryTopic, odometryMessage);
    }
}
