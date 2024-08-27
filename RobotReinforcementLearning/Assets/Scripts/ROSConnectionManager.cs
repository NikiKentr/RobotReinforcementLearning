using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ROSConnectionManager : MonoBehaviour
{
    public ROSConnection rosA; // Reference to the first ROS connection
    public ROSConnection rosB; // Reference to the second ROS connection

    void Start()
    {
        if (rosA == null || rosB == null)
        {
            Debug.LogError("ROSConnections are not assigned.");
        }
    }
}