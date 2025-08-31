using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class RobotAgent : Agent
{



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


    void Start()
    {
        bodyBP = new BodyPart(body, this);
        headBP = new BodyPart(head, this);
        leftPalmBP = new BodyPart(leftPalm, this);
        rightPalmBP = new BodyPart(rightPalm, this);



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
        
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        base.OnActionReceived(actions);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // rotation and velocity of the body
        sensor.AddObservation(body.transform.rotation.eulerAngles);
        sensor.AddObservation(bodyBP.partRigidBody.linearVelocity);
        sensor.AddObservation(bodyBP.partRigidBody.angularVelocity);

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
        foreach(var entry in jointDriver.BodyParts)
        {
            entry.Value.Reset();
        }
        bodyBP.Reset();
        headBP.Reset();
        leftPalmBP.Reset();
        rightPalmBP.Reset();
    }
}
