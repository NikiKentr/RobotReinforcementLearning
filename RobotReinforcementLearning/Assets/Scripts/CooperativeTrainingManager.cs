using UnityEngine;
using Unity.MLAgents;

public class CooperativeTrainingManager : MonoBehaviour
{
    public DifferentialDriveRobotAgent agent1;
    public DifferentialDriveRobotAgent agent2;
    public Transform sphereTarget;
    public float maxEpisodeTime = 1000f; // 5 minutes
    public float areaSize = 5f;
    public float targetReachedDistance = 0.5f;
    public float minDistanceBetweenAgents = 1f;

    private float episodeTimer;
    private int targetsReached = 0;

    private SimpleMultiAgentGroup agentGroup;

    void Start()
    {
        if (agent1 == null || agent2 == null || sphereTarget == null)
        {
            Debug.LogError("Please assign both agents and the sphere target in the inspector.");
            return;
        }

        agentGroup = new SimpleMultiAgentGroup();
        agentGroup.RegisterAgent(agent1);
        agentGroup.RegisterAgent(agent2);

        agent1.targetSphere = sphereTarget;
        agent2.targetSphere = sphereTarget;

        agent1.otherRobot = agent2;
        agent2.otherRobot = agent1;

        ResetArea();

    }

    void FixedUpdate()
    {
        episodeTimer += Time.fixedDeltaTime;

        if (episodeTimer >= maxEpisodeTime)
        {
            EndEpisode();
        }
        else if (CheckTargetReached())
        {
            RewardAgents();
            MoveTarget();
        }
    }

    private bool CheckTargetReached()
    {
        Vector3 midpoint = (agent1.transform.position + agent2.transform.position) / 2f;
        return Vector3.Distance(midpoint, sphereTarget.position) < targetReachedDistance;
        //return Vector3.Distance(agent1.transform.position, sphereTarget.position) < targetReachedDistance ||
               //Vector3.Distance(agent2.transform.position, sphereTarget.position) < targetReachedDistance;
    }

    private void RewardAgents()
    {
        // Reward both agents for reaching the target
        agentGroup.AddGroupReward(1.0f);

        agent1.AddReward(1.0f);
        agent2.AddReward(1.0f);
        targetsReached++;
        Debug.Log($"Target reached! Total targets reached: {targetsReached}");
    }

    private void MoveTarget()
    {
        // Move the target to a new random position
        sphereTarget.position = new Vector3(
            Random.Range(-areaSize / 2f, areaSize / 2f),
            sphereTarget.position.y,
            Random.Range(-areaSize / 2f, areaSize / 2f)
        );
    }

    private void EndEpisode()
    {
        Debug.Log($"Episode ended. Time: {episodeTimer:F2}s, Targets reached: {targetsReached}");
        agent1.EndEpisode();
        agent2.EndEpisode();
        agentGroup.EndGroupEpisode();
        ResetArea();
    }

    private void ResetArea()
    {
        episodeTimer = 0f;
        targetsReached = 0;

        // Reset target position
        MoveTarget();

        // Reset robot positions
        Vector3 midpoint = new Vector3(
            Random.Range(-areaSize / 2f, areaSize / 2f),
            0f,
            Random.Range(-areaSize / 2f, areaSize / 2f)
        );

        float initialDistance = Random.Range(agent1.minDistanceBetweenRobots, agent1.maxDistanceBetweenRobots);
        Vector3 offset = new Vector3(initialDistance / 2f, 0, 0);

        ResetRobotPosition(agent1, midpoint + offset);
        ResetRobotPosition(agent2, midpoint - offset);

        // Reset each robot's position
        ResetRobotPosition(agent1, midpoint + offset);
        ResetRobotPosition(agent2, midpoint - offset);

        //ResetRobotPosition(agent1);
        //ResetRobotPosition(agent2);



        // Ensure robots face each other
        agent1.transform.LookAt(agent2.transform);
        agent2.transform.LookAt(agent1.transform);
    }

    private void ResetRobotPosition(DifferentialDriveRobotAgent agent, Vector3 position)
    {
        agent.transform.position = position;
        ArticulationBody agentBody = agent.GetComponent<ArticulationBody>();
        if (agentBody != null)
        {
            agentBody.velocity = Vector3.zero;
            agentBody.angularVelocity = Vector3.zero;
        }
        agent.wheelController.SetRobotVelocity(0, 0);


        /*agent.transform.position = new Vector3(
            Random.Range(-areaSize / 2f, areaSize / 2f),
            agent.transform.position.y,
            Random.Range(-areaSize / 2f, areaSize / 2f)
        );
        agent.transform.rotation = Quaternion.Euler(0, Random.Range(0f, 360f), 0);
        ArticulationBody agentBody = agent.GetComponent<ArticulationBody>();
        if (agentBody != null)
        {
            agentBody.velocity = Vector3.zero;
            agentBody.angularVelocity = Vector3.zero;
        }
        agent.wheelController.SetRobotVelocity(0, 0);
        */
    }
}