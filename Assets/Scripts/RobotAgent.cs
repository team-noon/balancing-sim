using System;
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


    void Start()
    {
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
            SetReward(-1000f);
            EndEpisode();
        }

        AddReward(maxBalancePenalty * balancePenalty.Evaluate(Vector3.Angle(transform.up, Vector3.up) / 180));
    }

    ActionBuffers? lastBuffer = null;

    public override void OnActionReceived(ActionBuffers actions)
    {
        
        int i = -1;

        

        jointDriver.BodyParts[rightFoot].SetTargetRotation(actions.ContinuousActions[++i],0, actions.ContinuousActions[++i]);
        jointDriver.BodyParts[leftFoot].SetTargetRotation(actions.ContinuousActions[++i], 0, actions.ContinuousActions[++i]);

        jointDriver.BodyParts[rightLowerLeg].SetTargetRotation(actions.ContinuousActions[++i], 0, 0);
        jointDriver.BodyParts[leftLowerLeg].SetTargetRotation(actions.ContinuousActions[++i], 0, 0);

        jointDriver.BodyParts[rightUpperLeg].SetTargetRotation(actions.ContinuousActions[++i], actions.ContinuousActions[++i], actions.ContinuousActions[++i]);
        jointDriver.BodyParts[leftUpperLeg].SetTargetRotation(actions.ContinuousActions[++i], actions.ContinuousActions[++i], actions.ContinuousActions[++i]);

        jointDriver.BodyParts[rightUpperArm].SetTargetRotation(actions.ContinuousActions[++i],0, actions.ContinuousActions[++i]);
        jointDriver.BodyParts[leftUpperArm].SetTargetRotation(actions.ContinuousActions[++i], 0, actions.ContinuousActions[++i]);

        jointDriver.BodyParts[rightLowerArm].SetTargetRotation(actions.ContinuousActions[++i], 0, 0);
        jointDriver.BodyParts[leftLowerArm].SetTargetRotation(actions.ContinuousActions[++i], 0, 0);


        if (lastBuffer.HasValue)
        {
            int j = 0;

            while(j < lastBuffer.Value.ContinuousActions.Length)
            {
                AddReward(math.abs(actions.ContinuousActions[j] - lastBuffer.Value.ContinuousActions[j]) * largeMovementPenaltyMultiplier);
                j++;
            }
        }

        lastBuffer = actions;

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

        // Set indices 9, 10, 11 (upper leg control)
        continuousActions[9] = value;
        continuousActions[10] = value;
        continuousActions[11] = value; 
    }
}
