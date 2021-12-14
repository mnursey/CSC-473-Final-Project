using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;
using System.Collections.Generic;

public class SingleAxisAgent : Agent
{

    //The direction an agent will walk during training.
    [Header("Target To Walk Towards")]
    //public Transform TargetPrefab; //Target prefab to use in Dynamic envs
    public Transform m_Target; //Target the agent will walk towards during training.

    [Header("Body Parts")]
    [Space(10)]
    public Transform body;
    public List<SingleAxisTranslationJoint> arms = new List<SingleAxisTranslationJoint>();

    public float TargetWalkingSpeed = 10f;

    // Store rigidbody components
    public Rigidbody bodyRigidBody;
    Vector3 bodyDefaultPosition;

    [Header("Initialization Parameters")]
    [Space(10)]

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    public OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    public DirectionIndicator m_DirectionIndicator;

    public override void Initialize()
    {
        bodyDefaultPosition = body.position;
    }

    /// <summary>
    /// Spawns a target prefab at pos
    /// </summary>
    /// <param name="prefab"></param>
    /// <param name="pos"></param>
    void SpawnTarget(Transform prefab, Vector3 pos)
    {
        m_Target = Instantiate(prefab, pos, Quaternion.identity, transform.parent);
    }

    public override void OnEpisodeBegin()
    {
        
        body.position = bodyDefaultPosition;

        //Random start rotation to help generalize
        body.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        bodyRigidBody.velocity = Vector3.zero;
        bodyRigidBody.angularVelocity = Vector3.zero;

        foreach (SingleAxisTranslationJoint arm in arms)
        {
            arm.Reset();
        }
        
        UpdateOrientationObjects();
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = m_OrientationCube.transform.forward;

        // Distance from ground (0.0 to 1.0)
        RaycastHit hit;
        float maxRaycastDist = 10;
        if (Physics.Raycast(body.position, Vector3.down, out hit, maxRaycastDist))
        {
            sensor.AddObservation(hit.distance / maxRaycastDist);
        }
        else
        {
            // We didn't hit anything... so we're at max input
            sensor.AddObservation(1);
        }

        // Direction of goal (Vector3),
        // Add pos of target relative to orientation cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_Target.transform.position));

        //Body rotation delta
        sensor.AddObservation(Quaternion.FromToRotation(body.forward, cubeForward));

        //avg body vel relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bodyRigidBody.velocity));

        foreach (SingleAxisTranslationJoint arm in arms)
        {
            sensor.AddObservation(arm.GetPosDiff());
            sensor.AddObservation(arm.IsGrounded());
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var continuousActions = actionBuffers.ContinuousActions;

        // Update dynamic spring strength
        for (int i = 0; i < arms.Count; i++)
        {
            arms[i].UpdateTranslationPosition(continuousActions[i]);
        }
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        var cubeForward = m_OrientationCube.transform.forward;

        // Set reward for this step according to mixture of the following elements.

        // Set reward for this step according to mixture of the following elements.
        // a. Match target speed
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        var matchSpeedReward = GetMatchingVelocityReward(cubeForward * TargetWalkingSpeed, bodyRigidBody.velocity);

        // b. Rotation alignment with target direction.
        //This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
        var lookAtTargetReward = (Vector3.Dot(cubeForward, body.forward) + 1) * .5F;

        AddReward(matchSpeedReward * lookAtTargetReward);

    }


    /// <summary>
    /// Update OrientationCube and DirectionIndicator
    /// </summary>
    void UpdateOrientationObjects()
    {
        m_OrientationCube.UpdateOrientation(body, m_Target);

        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }

    /// <summary>
    /// Normalized value of the difference in actual speed vs goal walking speed.
    /// </summary>
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, TargetWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / TargetWalkingSpeed, 2), 2);
    }

    /// <summary>
    /// Agent touched the target
    /// </summary>
    public void TouchedTarget()
    {
        AddReward(18f);
    }
}
