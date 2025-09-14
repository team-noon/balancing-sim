using Unity.Mathematics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class RobotAgent : Agent
{

    Vector3 startingPos;

    [SerializeField] JointDriver jointDriver;

    [Header("Body Parts")]
    [SerializeField] Transform body;
    BodyPart bodyBP;

    [SerializeField] Transform head;
    BodyPart headBP;

    [SerializeField] Transform leftPalm;
    BodyPart leftPalmBP;
    [SerializeField] Transform rightPalm;
    BodyPart rightPalmBP;
    [Header("Jointed body parts")]
    // legs
    //r
    [SerializeField] Transform rightFoot;
    [SerializeField] Transform rightLowerLeg;
    [SerializeField] Transform rightUpperLeg;

    //l
    [SerializeField] Transform leftFoot;
    [SerializeField] Transform leftLowerLeg;
    [SerializeField] Transform leftUpperLeg;

    //arms
    //r
    [SerializeField] Transform rightLowerArm;
    [SerializeField] Transform rightUpperArm;

    //l
    [SerializeField] Transform leftLowerArm;
    [SerializeField] Transform leftUpperArm;

    [Header("Penalty / reward settings")]
    [SerializeField] AnimationCurve balancePenalty;
    [SerializeField] float maxBalancePenalty;

    [SerializeField] float largeMovementPenaltyMultiplier = -1;

    [SerializeField] float distancePenalty = -1;



    [Header("Box Throwing parameters")]
    [SerializeField] GameObject box;
    [Tooltip("Min time between box throws (in steps)")]
    [SerializeField] int minBoxTime;
    [Tooltip("Max time between box throws (in steps)")]
    [SerializeField] int maxBoxTime;
    [SerializeField] float minBoxDistance = 2;
    [SerializeField] float maxBoxDistance = 5;
    [SerializeField] Transform boxHolder;

    private int nextBoxThrow;
    void Start()
    {
        nextBoxThrow = StepCount + maxBoxTime;

        bodyBP = new BodyPart(body);
        headBP = new BodyPart(head);
        leftPalmBP = new BodyPart(leftPalm);
        rightPalmBP = new BodyPart(rightPalm);

        // legs
        jointDriver.addBodyPart(rightFoot);
        jointDriver.addBodyPart(rightLowerLeg);
        jointDriver.addBodyPart(rightUpperLeg);

        jointDriver.addBodyPart(leftFoot);
        jointDriver.addBodyPart(leftLowerLeg);
        jointDriver.addBodyPart(leftUpperLeg);

        // arms
        jointDriver.addBodyPart(rightLowerArm);
        jointDriver.addBodyPart(rightUpperArm);

        jointDriver.addBodyPart(leftLowerArm);
        jointDriver.addBodyPart(leftUpperArm);
    }



    private void FixedUpdate()
    {
        if (Vector3.Distance(startingPos, transform.position) > 1000)
        {
            Debug.Log($"[RobotAgent] Out of bounds! Adding reward: -1000f and ending episode.");
            SetReward(-1000f);
            EndEpisode();
        }

        float balanceReward = maxBalancePenalty * balancePenalty.Evaluate(Vector3.Angle(transform.up, Vector3.up) / 180);
        //Debug.Log($"[RobotAgent] Adding balance reward: {balanceReward}");
        AddReward(balanceReward);

        if (StepCount >= nextBoxThrow)
        {
            ThrowBox();
            nextBoxThrow = StepCount + Mathf.FloorToInt(UnityEngine.Random.Range(minBoxTime, maxBoxTime));
        }

        // reward for being as close to the starting position.
        float distanceReward = Vector2.Distance(new Vector2(bodyBP.startPosition.x, bodyBP.startPosition.z), new Vector2(body.position.x, body.position.z)) * distancePenalty;
        AddReward(distanceReward);
    }

    float[] lastBuffer = null;

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (lastBuffer == null)
        {
            lastBuffer = new float[actions.ContinuousActions.Length];

            // right foot
            lastBuffer[0] = jointDriver.BodyParts[rightFoot].startingJointRotation.x;
            lastBuffer[1] = jointDriver.BodyParts[rightFoot].startingJointRotation.z;

            // left foot
            lastBuffer[2] = jointDriver.BodyParts[leftFoot].startingJointRotation.x;
            lastBuffer[3] = jointDriver.BodyParts[leftFoot].startingJointRotation.z;

            // right lower leg
            lastBuffer[4] = jointDriver.BodyParts[rightLowerLeg].startingJointRotation.x;

            // left lower leg
            lastBuffer[5] = jointDriver.BodyParts[leftLowerLeg].startingJointRotation.x;

            // right upper leg
            lastBuffer[6] = jointDriver.BodyParts[rightUpperLeg].startingJointRotation.x;
            lastBuffer[7] = jointDriver.BodyParts[rightUpperLeg].startingJointRotation.y;
            lastBuffer[8] = jointDriver.BodyParts[rightUpperLeg].startingJointRotation.z;

            // left upper leg
            lastBuffer[9] = jointDriver.BodyParts[leftUpperLeg].startingJointRotation.x;
            lastBuffer[10] = jointDriver.BodyParts[leftUpperLeg].startingJointRotation.y;
            lastBuffer[11] = jointDriver.BodyParts[leftUpperLeg].startingJointRotation.z;

            // right upper arm
            lastBuffer[12] = jointDriver.BodyParts[rightUpperArm].startingJointRotation.x;
            lastBuffer[13] = jointDriver.BodyParts[rightUpperArm].startingJointRotation.z;

            // left upper arm
            lastBuffer[14] = jointDriver.BodyParts[leftUpperArm].startingJointRotation.x;
            lastBuffer[15] = jointDriver.BodyParts[leftUpperArm].startingJointRotation.z;

            // right lower arm
            lastBuffer[16] = jointDriver.BodyParts[rightLowerArm].startingJointRotation.x;

            // left lower arm
            lastBuffer[17] = jointDriver.BodyParts[leftLowerArm].startingJointRotation.x;
        }

        int i = -1;

        jointDriver.BodyParts[rightFoot].SetTargetRotation(actions.ContinuousActions[++i], 0, actions.ContinuousActions[++i]);
        jointDriver.BodyParts[leftFoot].SetTargetRotation(actions.ContinuousActions[++i], 0, actions.ContinuousActions[++i]);

        jointDriver.BodyParts[rightLowerLeg].SetTargetRotation(actions.ContinuousActions[++i], 0, 0);
        jointDriver.BodyParts[leftLowerLeg].SetTargetRotation(actions.ContinuousActions[++i], 0, 0);

        jointDriver.BodyParts[rightUpperLeg].SetTargetRotation(actions.ContinuousActions[++i], actions.ContinuousActions[++i], actions.ContinuousActions[++i]);
        jointDriver.BodyParts[leftUpperLeg].SetTargetRotation(actions.ContinuousActions[++i], actions.ContinuousActions[++i], actions.ContinuousActions[++i]);

        jointDriver.BodyParts[rightUpperArm].SetTargetRotation(actions.ContinuousActions[++i], 0, actions.ContinuousActions[++i]);
        jointDriver.BodyParts[leftUpperArm].SetTargetRotation(actions.ContinuousActions[++i], 0, actions.ContinuousActions[++i]);

        jointDriver.BodyParts[rightLowerArm].SetTargetRotation(actions.ContinuousActions[++i], 0, 0);
        jointDriver.BodyParts[leftLowerArm].SetTargetRotation(actions.ContinuousActions[++i], 0, 0);


        int j = 0;

        while (j < lastBuffer.Length)
        {
            float movementPenalty = math.pow(actions.ContinuousActions[j] - lastBuffer[j], 2) * largeMovementPenaltyMultiplier;
            if (movementPenalty != 0)
            {
                Debug.Log($"[RobotAgent] Adding movement penalty reward: {movementPenalty} for action index {j}");
            }

            AddReward(movementPenalty);

            lastBuffer[j] = actions.ContinuousActions[j];
            j++;
        }

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // rotation and velocity of the body
        sensor.AddObservation(body.transform.rotation.eulerAngles);
        sensor.AddObservation(bodyBP.Joint.linearVelocity);
        sensor.AddObservation(bodyBP.Joint.angularVelocity);

        // arms
        // elbow
        sensor.AddObservation(leftUpperArm.eulerAngles.x - leftLowerArm.eulerAngles.x);
        sensor.AddObservation(rightUpperArm.eulerAngles.x - rightLowerArm.eulerAngles.x);

        //shoulder
        sensor.AddObservation(body.eulerAngles.y - leftUpperArm.eulerAngles.x);
        sensor.AddObservation(body.eulerAngles.y - rightUpperArm.eulerAngles.x);

        sensor.AddObservation(body.eulerAngles.z - leftUpperArm.eulerAngles.z);
        sensor.AddObservation(body.eulerAngles.z - rightUpperArm.eulerAngles.z);

        //legs
        //hip
        sensor.AddObservation(body.eulerAngles - leftUpperLeg.eulerAngles);
        sensor.AddObservation(body.eulerAngles - rightUpperLeg.eulerAngles);

        //knee
        sensor.AddObservation(leftUpperLeg.eulerAngles.x - leftLowerLeg.eulerAngles.x);
        sensor.AddObservation(rightUpperLeg.eulerAngles.x - rightLowerLeg.eulerAngles.x);

        //foot
        sensor.AddObservation(leftLowerLeg.eulerAngles.x - leftFoot.eulerAngles.x);
        sensor.AddObservation(rightLowerLeg.eulerAngles.x - rightFoot.eulerAngles.x);

        sensor.AddObservation(leftLowerLeg.eulerAngles.z - leftFoot.eulerAngles.z);
        sensor.AddObservation(rightLowerLeg.eulerAngles.z - rightFoot.eulerAngles.z);
    }

    public override void OnEpisodeBegin()
    {
        nextBoxThrow = StepCount + maxBoxTime;

        // Delete all children of boxHolder
        for (int i = boxHolder.childCount - 1; i >= 0; i--)
        {
            Destroy(boxHolder.GetChild(i).gameObject);
        }

        startingPos = transform.position;
        // Reset the root first
        bodyBP.Reset(isRoot: true);

        // Reset all other parts
        foreach (var entry in jointDriver.BodyParts)
        {
            if (entry.Key != body) // skip root, already reset
                entry.Value.Reset();
        }

        headBP.Reset();
        leftPalmBP.Reset();
        rightPalmBP.Reset();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        // Default values
        float value = 0f;

        if (Input.GetKey(KeyCode.W))
        {
            value = 0.5f;
        }
        else if (Input.GetKey(KeyCode.S))
        {
            value = -0.5f;
        }

        if (Input.GetMouseButton(1))
        {
            ThrowBox();
        }

        // Set indices 9, 10, 11 (upper leg control)
        continuousActions[9] = value;
        continuousActions[10] = value;
        continuousActions[11] = value;
    }

    private void ThrowBox()
    {
        Vector3 targetPos;
        do
        {
            Vector3 randomOffset = UnityEngine.Random.insideUnitSphere * maxBoxDistance;
            randomOffset.y = math.abs(randomOffset.y);
            targetPos = transform.position + randomOffset;
        }
        while (Vector3.Distance(targetPos, transform.position) < minBoxDistance);

        boxScript bScript = Instantiate(box, targetPos, Quaternion.identity, boxHolder).GetComponent<boxScript>();
        bScript.Init(transform);
    }


}
