using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class NewKeyboardControl : MonoBehaviour
{
    private ROSConnection ros;
    public string poseTopic = "pose"; // The ROS topic to publish pose messages
    public float positionSpeed = 2.0f; // Speed of position change
    public float rotationSpeed = 1.0f; // Speed of rotation change
    private Vector3 position = Vector3.zero; // Current position
    private Quaternion rotation = Quaternion.identity; // Current rotation

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(poseTopic);
    }

    void Update()
    {
        HandleKeyboardInput();
        PublishPoseMessage();
    }

    void HandleKeyboardInput()
    {
        // Update position based on WASD keys
        position += new Vector3(Input.GetAxis("Horizontal"), 0, Input.GetAxis("Vertical")) * positionSpeed * Time.deltaTime;

        // Update rotation based on QE keys
        if (Input.GetKey(KeyCode.Q))
        {
            rotation = Quaternion.Euler(rotation.eulerAngles + new Vector3(0, -rotationSpeed * Time.deltaTime, 0));
        }
        if (Input.GetKey(KeyCode.E))
        {
            rotation = Quaternion.Euler(rotation.eulerAngles + new Vector3(0, rotationSpeed * Time.deltaTime, 0));
        }
    }

    void PublishPoseMessage()
    {
        PoseMsg poseMessage = new PoseMsg
        {
            position = new PointMsg { x = position.x, y = position.y, z = position.z },
            orientation = new QuaternionMsg { x = rotation.x, y = rotation.y, z = rotation.z, w = rotation.w }
        };

        ros.Publish(poseTopic, poseMessage);
    }
}
