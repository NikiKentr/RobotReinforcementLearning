using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KeyboardControlRobot1 : MonoBehaviour
{
    public ArticulationWheelController wheelController;

    public float speed = 1.5f;
    public float angularSpeed = 1.5f;
    public float targetLinearSpeed { get; private set; }
    public float targetAngularSpeed { get; private set; }

    public Transform point1; // Reference to the second point

    void Update()
    {
        // Get key input
        targetLinearSpeed = Input.GetAxisRaw("Vertical") * speed;
        targetAngularSpeed = -Input.GetAxisRaw("Horizontal") * angularSpeed;

        // Log the speeds
        //Debug.Log($"[Robot1] Linear Speed: {targetLinearSpeed}, Angular Speed: {targetAngularSpeed}");

        // Log the positions of the points
        Vector3 point1Position = point1.position;
        //Debug.Log($"Point1 Position: {point1Position}");
    }

    void FixedUpdate()
    {
        wheelController.SetRobotVelocity(targetLinearSpeed, targetAngularSpeed);
    }
}

