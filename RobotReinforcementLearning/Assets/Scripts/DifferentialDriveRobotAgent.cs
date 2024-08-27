using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Assimp.Configs;

public class DifferentialDriveRobotAgent : Agent
{
    public ArticulationWheelController wheelController;
    public float maxLinearSpeed = 10f;
    public float maxAngularSpeed = 15f;
    public Transform targetSphere;

    public DifferentialDriveRobotAgent otherRobot; //for recognising eachother

    public float maxDistanceBetweenRobots = 1.9f;
    public float minDistanceBetweenRobots = 0.4f;

    public float optimalDistanceBetweenRobots = 0.95f; // Midpoint of min and max

    private ArticulationBody robotBody;
    private bool isInitialized = false;

    public Transform frontReference;

    private float currentLinearSpeed = 0f;
    private float currentAngularSpeed = 0f;
    public float speedSmoothTime = 0.05f;
    private Vector2 speedSmoothVelocity;

    private Vector3 previousPosition;

    public override void Initialize()
    {
        robotBody = GetComponent<ArticulationBody>();
        if (robotBody == null)
        {
            Debug.LogError($"ArticulationBody component is missing on {gameObject.name}.");
            return;
        }
        if (wheelController == null)
        {
            Debug.LogError($"ArticulationWheelController is not assigned on {gameObject.name}.");
            return;
        }
        if (targetSphere == null)
        {
            Debug.LogError($"Target Sphere is not set for {gameObject.name}.");
            return;
        }
        if (otherRobot == null)
        {
            Debug.LogError($"Other Robot is not set for {gameObject.name}.");
            return;
        }

        previousPosition = transform.position;
        isInitialized = true;
        Debug.Log($"Agent {gameObject.name} initialized successfully.");
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (!isInitialized)
        {
            Debug.LogError($"Agent {gameObject.name} is not properly initialized. Check for missing components.");
            return;
        }

        if (targetSphere == null || otherRobot == null) return;  

        Vector3 robotFront = GetRobotFront();

        // Angle to face the other robot
        Vector3 directionToOtherRobot = (otherRobot.transform.position - transform.position).normalized;
        float angleToOtherRobot = Vector3.SignedAngle(robotFront, directionToOtherRobot, Vector3.up);
        sensor.AddObservation(angleToOtherRobot / 180f);

        // Midpoint between robots
        Vector3 midpoint = (transform.position + otherRobot.transform.position) / 180f; //devision This normalizes the angle to a range between -1 and 1
        // learn to minimize the absolute value of this angle

        // Relative position of the other robot
        Vector3 relativePositionOtherRobot = transform.InverseTransformPoint(otherRobot.transform.position);
        sensor.AddObservation(relativePositionOtherRobot);

        // Distance to other robot
        float distanceToOtherRobot = Vector3.Distance(transform.position, otherRobot.transform.position);
        sensor.AddObservation(distanceToOtherRobot);


        // Relative position of the target to the midpoint
        Vector3 relativePositionTarget = transform.InverseTransformPoint(targetSphere.position - midpoint);
        sensor.AddObservation(relativePositionTarget);


        // Current velocity
        sensor.AddObservation(robotBody.velocity);
        sensor.AddObservation(robotBody.angularVelocity);

        // Current speeds
        sensor.AddObservation(currentLinearSpeed / maxLinearSpeed);
        sensor.AddObservation(currentAngularSpeed / maxAngularSpeed);

        if (wheelController.leftWheel != null && wheelController.rightWheel != null) 
        {
            sensor.AddObservation(wheelController.leftWheel.jointVelocity[0]);
            sensor.AddObservation(wheelController.rightWheel.jointVelocity[0]);
        }
        else
        {
            Debug.LogWarning($"Wheel references are missing in ArticulationWheelController on {gameObject.name}.");
            sensor.AddObservation(Vector3.zero); // Add placeholder observations
            sensor.AddObservation(Vector3.zero);
        }

        //sensor.AddObservation(Vector3.Distance(transform.position, targetSphere.position));
    }




    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (!isInitialized)
        {
            Debug.LogError($"Agent {gameObject.name} received actions before initialization.");
            return;
        }

        var continuousActions = actionBuffers.ContinuousActions;

        if (continuousActions.Length < 2)
        {
            Debug.LogError($"Not enough continuous actions provided for {gameObject.name}. Expected 2, got {continuousActions.Length}");
            return;
        }

        float targetLinearSpeed = continuousActions[0] * maxLinearSpeed;
        float targetAngularSpeed = continuousActions[1] * maxAngularSpeed;

        // Strongly discourage rotation
        float rotationPenalty = -Mathf.Abs(targetAngularSpeed) / maxAngularSpeed;
        AddReward(rotationPenalty * 1.0f);

        // Smooth out the speed changes
        currentLinearSpeed = Mathf.SmoothDamp(currentLinearSpeed, targetLinearSpeed, ref speedSmoothVelocity.x, speedSmoothTime);
        currentAngularSpeed = Mathf.SmoothDamp(currentAngularSpeed, targetAngularSpeed, ref speedSmoothVelocity.y, speedSmoothTime);

        //wheelController.SetRobotVelocity(targetLinearSpeed, targetAngularSpeed);
        wheelController.SetRobotVelocity(currentLinearSpeed, currentAngularSpeed);

        if (targetSphere != null && otherRobot != null)
        {
            Vector3 robotFront = GetRobotFront();

            Vector3 directionToOtherRobot = (otherRobot.transform.position - transform.position).normalized;
            float distanceToOtherRobot = Vector3.Distance(transform.position, otherRobot.transform.position);
            float angleToOtherRobot = Vector3.Angle(robotFront, directionToOtherRobot);

            Vector3 midpoint = (transform.position + otherRobot.transform.position) / 2f;
            float distanceToTarget = Vector3.Distance(midpoint, targetSphere.position);
            Vector3 directionToTarget = (targetSphere.position - midpoint).normalized;

            // Reward for facing the other robot
            float alignmentReward = Mathf.Exp(-angleToOtherRobot / 22f);
            // devided by 45 robots are encouraged to align precisely, but not punished too harshly for moderate misalignments.

            // Reward for maintaining correct distance
            float distanceReward = 0f;

            if (distanceToOtherRobot >= minDistanceBetweenRobots && distanceToOtherRobot <= maxDistanceBetweenRobots)
            {
                distanceReward = 1f;
            }
            else
            {
                float optimalDistance = (maxDistanceBetweenRobots + minDistanceBetweenRobots) / 2f;
                float maxDeviation = Mathf.Max(maxDistanceBetweenRobots - optimalDistance, optimalDistance - minDistanceBetweenRobots);
                float deviation = Mathf.Abs(distanceToOtherRobot - optimalDistance);
                distanceReward = 1f / (1f + deviation / maxDeviation);
            }
            // Reward for moving towards the target
            float movementTowardsTarget = Vector3.Dot((transform.position - previousPosition).normalized, directionToTarget);
            float targetReward = Mathf.Clamp01((movementTowardsTarget + 1f) / 2f);
            //float targetReward = movementTowardsTarget > 0 ? movementTowardsTarget : 0;

            // Reward for moving towards the target (as a pair)
            //float targetReward = 1f / (1f + distanceToTarget);

            // Penalty for rotation
            //float rotationPenalty = -Mathf.Abs(currentAngularSpeed) / maxAngularSpeed;


            // Reward for coordinated movement
            float speedDifference = Mathf.Abs(currentLinearSpeed - otherRobot.currentLinearSpeed);
            float coordinationReward = Mathf.Exp(-speedDifference / maxLinearSpeed);

            // Reward for efficient movement (one forward, one backward)
            float efficientMovementReward = 0f;
            float myMovementEfficiency = Vector3.Dot(robotBody.velocity.normalized, directionToTarget);
            float otherMovementEfficiency = Vector3.Dot(otherRobot.robotBody.velocity.normalized, directionToTarget);

            if ((myMovementEfficiency > 0 && otherMovementEfficiency < 0) ||
                (myMovementEfficiency < 0 && otherMovementEfficiency > 0))
            {
                // Calculate how "ideal" the movement is
                float idealEfficiency = Mathf.Abs(myMovementEfficiency) + Mathf.Abs(otherMovementEfficiency);
                efficientMovementReward = idealEfficiency / 2f; // Normalize to [0, 1]
            }
            else
            {
                efficientMovementReward = 0f;
            }


            // Combine rewards
            float totalReward =
                alignmentReward * 0.4f +
                distanceReward * 0.2f +
                targetReward * 0.2f +
                coordinationReward * 0.2f +
                efficientMovementReward * 0.3f -
                rotationPenalty *0.4f;

            AddReward(totalReward);

            previousPosition = transform.position;

            // End episode if target reached
            if (distanceToTarget < 0.1f)
            {
                EndEpisode();
            }
        }
        
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical");
        continuousActionsOut[1] = -Input.GetAxis("Horizontal");

        ///NEED CHANGES AT SOME POINT COME BACK TO IT
    }

    private Vector3 GetRobotFront()
    {
        if (frontReference != null)
        {
            return (frontReference.position - transform.position).normalized;
        }
        return -transform.right;
    }

    public override void OnEpisodeBegin()
    {
        if (!isInitialized)
        {
            Initialize();
        }


        if (robotBody != null)
        {
            robotBody.velocity = Vector3.zero;
            robotBody.angularVelocity = Vector3.zero;
        }
        currentLinearSpeed = 0f;
        currentAngularSpeed = 0f;
        wheelController.SetRobotVelocity(0, 0);
        previousPosition = transform.position;
    }
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.blue;
        Vector3 robotFront = GetRobotFront();
        Gizmos.DrawRay(transform.position, robotFront * 0.5f);

        // Draw a green line for the actual forward direction
        //Gizmos.color = Color.green;
        //Gizmos.DrawRay(transform.position, transform.forward * 2f);
    }
}