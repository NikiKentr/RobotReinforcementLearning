using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
///     This script converts linear velocity and 
///     angular velocity to joint velocities for
///     differential drive robot.
/// </summary>
public class ArticulationWheelController : MonoBehaviour
{
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;
    public float wheelTrackLength;
    public float wheelRadius;

    private float vRight;
    private float vLeft;

    void Start() {
        // Ensure wheels are set to velocity control mode
        SetupWheel(leftWheel);
        SetupWheel(rightWheel);
    }
    private void SetupWheel(ArticulationBody wheel)
    {
        wheel.jointType = ArticulationJointType.RevoluteJoint;
        var drive = wheel.xDrive;
        drive.stiffness = 0; // Set to 0 for pure velocity control
        drive.damping = 0.05f; // Low damping for smooth motion
        drive.forceLimit = float.PositiveInfinity; // Allow as much force as needed
        drive.lowerLimit = -float.PositiveInfinity; // No rotation limits
        drive.upperLimit = float.PositiveInfinity;
        wheel.xDrive = drive;
    }
    void Update() {}

    public void SetRobotVelocity(float targetLinearSpeed, float targetAngularSpeed)
    {
        // Stop the wheel if target velocity is 0
        if (targetLinearSpeed == 0 && targetAngularSpeed == 0)
        {
            StopWheel(leftWheel);
            StopWheel(rightWheel);
        }
        else
        {
            // Convert from linear x and angular z velocity to wheel speed
            vRight = targetAngularSpeed*(wheelTrackLength/2) + targetLinearSpeed;
            vLeft = -targetAngularSpeed*(wheelTrackLength/2) + targetLinearSpeed;

            SetWheelVelocity(leftWheel, vLeft / wheelRadius * Mathf.Rad2Deg);
            SetWheelVelocity(rightWheel, vRight / wheelRadius * Mathf.Rad2Deg);
        }
    }

    private void SetWheelVelocity(ArticulationBody wheel, float targetVelocity)
    {
        ArticulationDrive drive = wheel.xDrive;
        drive.targetVelocity = targetVelocity;
        wheel.xDrive = drive;
        //ArticulationDrive drive = wheel.xDrive;
        //drive.target = drive.target + jointVelocity * Time.fixedDeltaTime;
        //wheel.xDrive = drive;
    }

    private void StopWheel(ArticulationBody wheel)
    {
        // Set desired angle as current angle to stop the wheel
        ArticulationDrive drive = wheel.xDrive;
        drive.target = wheel.jointPosition[0] * Mathf.Rad2Deg;
        wheel.xDrive = drive;
    }
}
