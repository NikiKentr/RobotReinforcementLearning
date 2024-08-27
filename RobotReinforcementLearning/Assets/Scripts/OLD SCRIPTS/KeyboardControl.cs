using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
///     This script detects keyboard controls and
///     use them to control the mobile robot
/// </summary>
public class KeyboardControl : MonoBehaviour
{
    public ArticulationWheelController wheelController;

    public float speed = 1.5f;
    public float angularSpeed = 1.5f;
    private float targetLinearSpeed;
    private float targetAngularSpeed;

    public Transform point1; // Reference to the first point

    void Start() {}

    void Update()
    {
        // Get key input
        targetLinearSpeed = Input.GetAxisRaw("Vertical") * speed;
        targetAngularSpeed = -Input.GetAxisRaw("Horizontal") * angularSpeed;

        // Log the positions of the points
        Vector3 point1Position = point1.position;


        Debug.Log($"Point1 Position: {point1Position}");

    }

    void FixedUpdate()
    {
        wheelController.SetRobotVelocity(targetLinearSpeed, targetAngularSpeed);
    }
}
