using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgentsExamples;
using UnityEngine;
using Random = UnityEngine.Random;

[RequireComponent(typeof(JointDriveController))] // Required to set joint forces
public class ATSTAgent : Agent
{

    [Header("Walk Speed")]
    [Range(0.1f, m_maxWalkingSpeed)]
    [SerializeField]
    [Tooltip(
        "The speed the agent will try to match.\n\n" +
        "TRAINING:\n" +
        "For VariableSpeed envs, this value will randomize at the start of each training episode.\n" +
        "Otherwise the agent will try to match the speed set here.\n\n" +
        "INFERENCE:\n" +
        "During inference, VariableSpeed agents will modify their behavior based on this value " +
        "whereas the CrawlerDynamic & CrawlerStatic agents will run at the speed specified during training "
    )]
    //The walking speed to try and achieve
    private float m_TargetWalkingSpeed = m_maxWalkingSpeed;

    const float m_maxWalkingSpeed = 15; //The max walking speed

    //The current target walking speed. Clamped because a value of zero will cause NaNs
    public float TargetWalkingSpeed
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, 0.1f, m_maxWalkingSpeed); }
    }

    //The direction an agent will walk during training.
    [Header("Target To Walk Towards")]
    //public Transform TargetPrefab; //Target prefab to use in Dynamic envs
    public Transform m_Target; //Target the agent will walk towards during training.

    [Header("Body Parts")] [Space(10)] public Transform body;

    public Transform leg0Upper;
    public Transform leg0Mid;
    public Transform leg0lower;
    public Transform leg0Foot;

    public Transform leg1Upper;
    public Transform leg1Mid;
    public Transform leg1lower;
    public Transform leg1Foot;

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;

    [Header("Foot Grounded Visualization")]
    [Space(10)]
    public bool useFootGroundedVisualization;


    public MeshRenderer foot0;
    public MeshRenderer foot1;

    public Material groundedMaterial;
    public Material unGroundedMaterial;

    [Header("Reward settings")]
    [Space(10)]
    public TargetController goal;
    public GameObject groundA;
    public GameObject groundB;

    public GameObject groundHolder;

    [Header("Raycast settings")]
    [Space(10)]

    public LayerMask raycastMask;

    public override void Initialize()
    {
        //SpawnTarget(TargetPrefab, transform.position); //spawn target

        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_JdController = GetComponent<JointDriveController>();

        //Setup each body part
        m_JdController.SetupBodyPart(body);

        m_JdController.SetupBodyPart(leg0Upper);
        m_JdController.SetupBodyPart(leg0Mid);
        m_JdController.SetupBodyPart(leg0lower);
        m_JdController.SetupBodyPart(leg0Foot);

        m_JdController.SetupBodyPart(leg1Upper);
        m_JdController.SetupBodyPart(leg1Mid);
        m_JdController.SetupBodyPart(leg1lower);
        m_JdController.SetupBodyPart(leg1Foot);
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

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        //Random start rotation to help generalize
        body.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);
        groundHolder.transform.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        UpdateOrientationObjects();

        //Set our goal walking speed
        TargetWalkingSpeed = Random.Range(0.1f, m_maxWalkingSpeed);

        groundA.transform.position = new Vector3(groundA.transform.position.x, 0, groundA.transform.position.z);
        groundB.transform.position = new Vector3(groundB.transform.position.x, -Random.Range(0.001f, 5f), groundB.transform.position.z);

        goal.currentSpawnPosIndex = 0;
        goal.MoveTargetToRandomPosition();

    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

        if (bp.rb.transform != body)
        {
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
        }
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = m_OrientationCube.transform.forward;

        //velocity we want to match
        var velGoal = cubeForward * TargetWalkingSpeed;
        //ragdoll's avg vel
        var avgVel = GetAvgVelocity();

        //current ragdoll velocity. normalized
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        //avg body vel relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        //vel goal relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        //rotation delta
        sensor.AddObservation(Quaternion.FromToRotation(body.forward, cubeForward));

        //Add pos of target relative to orientation cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_Target.transform.position));

        RaycastHit hit;
        float maxRaycastDist = 15;

        /*
        if (Physics.Raycast(body.position, Vector3.down, out hit, maxRaycastDist, raycastMask))
        {
            sensor.AddObservation(hit.distance / maxRaycastDist);
            Debug.DrawRay(body.position, Vector3.down, Color.blue);
        }
        else
            sensor.AddObservation(1);
        */

        for(int m = -2; m <= 2; m++)
        {
            for (int n = -2; n <= 2; n++)
            {
                Vector3 localPosition = new Vector3(2 * m, 0, 2 * n);
                Vector3 worldPosition = body.TransformPoint(localPosition);
                worldPosition.y = body.transform.position.y;

                if (Physics.Raycast(worldPosition, Vector3.down, out hit, maxRaycastDist, raycastMask))
                {
                    sensor.AddObservation(hit.distance / maxRaycastDist);
                    Debug.DrawRay(worldPosition, Vector3.down * maxRaycastDist, Color.blue);
                }
                else
                    sensor.AddObservation(1);
            }
        }

        /*
        Vector3 lFootRay = new Vector3(leg0Foot.transform.position.x, body.transform.position.y, leg0Foot.transform.position.z);
        if (Physics.Raycast(lFootRay, Vector3.down, out hit, maxRaycastDist, raycastMask))
        {
            sensor.AddObservation(hit.distance / maxRaycastDist);
            Debug.DrawRay(lFootRay, Vector3.down, Color.blue);
        }
        else
            sensor.AddObservation(1);

        Vector3 rFootRay = new Vector3(leg1Foot.transform.position.x, body.transform.position.y, leg1Foot.transform.position.z);
        if (Physics.Raycast(rFootRay, Vector3.down, out hit, maxRaycastDist, raycastMask))
        {
            sensor.AddObservation(hit.distance / maxRaycastDist);
            Debug.DrawRay(rFootRay, Vector3.down, Color.blue);
        }
        else
            sensor.AddObservation(1);
        *.
        */

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }

        /*
        if (rewardCyclicWalking)
        {
            HandleCyclicWalkInput(sensor);
        }
        */
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // The dictionary with all the body parts in it are in the jdController
        var bpDict = m_JdController.bodyPartsDict;

        var continuousActions = actionBuffers.ContinuousActions;
        var i = -1;

        // Pick a new target joint rotation
        bpDict[leg0Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg0Mid].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg0lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg0Foot].SetJointTargetRotation(continuousActions[++i], 0, continuousActions[++i]);

        bpDict[leg1Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg1Mid].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg1lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg1Foot].SetJointTargetRotation(continuousActions[++i], 0, continuousActions[++i]);

        // Update joint strength
        bpDict[leg0Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg0Mid].SetJointStrength(continuousActions[++i]);
        bpDict[leg0lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg0Foot].SetJointStrength(continuousActions[++i]);

        bpDict[leg1Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg1Mid].SetJointStrength(continuousActions[++i]);
        bpDict[leg1lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg1Foot].SetJointStrength(continuousActions[++i]);
    }

    /*
    private void Update()
    {
        // Visualize raycasts
        RaycastHit hit;

        float maxRaycastDist = 30;

        int numberOfVisionRaycasts = 12;
        float degreesPerPoint = 360f / numberOfVisionRaycasts;

        for (int i = 0; i < numberOfVisionRaycasts; i++)
        {
            float angle = degreesPerPoint * i;
            if (Physics.Raycast(body.position, Quaternion.Euler(0, angle, 0) * m_OrientationCube.transform.forward * maxRaycastDist, out hit, maxRaycastDist, raycastMask))
            {
                Debug.DrawLine(body.position, hit.point, Color.yellow);
            }
            else
            {
                Debug.DrawRay(body.position, Quaternion.Euler(0, angle, 0) * m_OrientationCube.transform.forward * maxRaycastDist, Color.green);
            }
        }
    }
    */

    void FixedUpdate()
    {
        //timeR += Time.deltaTime;

        UpdateOrientationObjects();

        // If enabled the feet will light up green when the foot is grounded.
        // This is just a visualization and isn't necessary for function
        if (useFootGroundedVisualization)
        {
            foot0.material = m_JdController.bodyPartsDict[leg0Foot].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot1.material = m_JdController.bodyPartsDict[leg1Foot].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
        }

        var cubeForward = m_OrientationCube.transform.forward;

        // Set reward for this step according to mixture of the following elements.
        // a. Match target speed
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        var matchSpeedReward = GetMatchingVelocityReward(cubeForward * TargetWalkingSpeed, GetAvgVelocity());

        // b. Rotation alignment with target direction.
        //This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
        var lookAtTargetReward = (Vector3.Dot(cubeForward, body.forward) + 1) * .5F;

        AddReward(matchSpeedReward * lookAtTargetReward);

        /*
        if(rewardCyclicWalking)
        {
            HandleCyclicWalkReward();
        }
        */
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
    ///Returns the average velocity of all of the body parts
    ///Using the velocity of the body only has shown to result in more erratic movement from the limbs
    ///Using the average helps prevent this erratic movement
    /// </summary>
    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;
        Vector3 avgVel = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.velocity;
        }

        avgVel = velSum / numOfRb;
        return avgVel;
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
        if(goal.currentSpawnPosIndex == 0)
        {
            groundB.transform.position = new Vector3(groundB.transform.position.x, -Random.Range(0.001f, 5f) + groundA.transform.position.y, groundB.transform.position.z);
        }

        if (goal.currentSpawnPosIndex == 1)
        {
            groundA.transform.position = new Vector3(groundA.transform.position.x, -Random.Range(0.001f, 5f) + groundB.transform.position.y, groundA.transform.position.z);
        }

        //AddReward(1f);
    }
}
