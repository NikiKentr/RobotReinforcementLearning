using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class ROSPosePublisher : MonoBehaviour
{
    public Transform robotBase; // Reference to the Transform representing the robot's base
    public ROSConnection rosConnection; // Reference to the specific ROSConnection instance
    public string poseTopic; // ROS topic to publish pose messages for the robot

    public float publishInterval = 0.5f; // Publish interval in seconds (500 milliseconds)
    private float lastPublishTime;

    void Start()
    {
        if (rosConnection == null)
        {
            Debug.LogError("ROSConnection is not assigned.");
            return;
        }
        rosConnection.RegisterPublisher<PoseStampedMsg>(poseTopic);

        // Initialize last publish time
        lastPublishTime = Time.time;
    }

    void Update()
    {
        // Check if enough time has passed since last publish
        if (Time.time - lastPublishTime > publishInterval)
        {
            Vector3 basePosition = robotBase.position;
            Quaternion baseRotation = robotBase.rotation;

            PublishPoseMessage(basePosition, baseRotation);

            // Update last publish time
            lastPublishTime = Time.time;
        }
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

        rosConnection.Publish(poseTopic, poseMessage);
    }
}
