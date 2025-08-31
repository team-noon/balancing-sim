using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using Unity.MLAgents;
using UnityEngine;

public class JointDriver : MonoBehaviour
{
    public float jointSpring;
    public float jointDampen;
    public float maxJointForce;

    public Dictionary<Transform, BodyPart> BodyParts = new Dictionary<Transform, BodyPart>();

    public void addBodyPart(Transform bodyPartTransform)
    {


        BodyPart newBodyPart = new BodyPart(bodyPartTransform, transform.GetComponent<Agent>(), this);

        BodyParts.Add(bodyPartTransform, newBodyPart);
    }
}

public class BodyPart
{
    public ConfigurableJoint Joint;
    public GroundContact groundContact;
    public Transform partTransform;
    public quaternion startRotation;
    public Vector3 startPosition;
    public Rigidbody partRigidBody;
    public JointDriver jointDriver;

    public BodyPart(Transform thisTransform, Agent thisAgent, JointDriver? jd = null)
    {
        if (jd)
        {
            jointDriver = jd;
            Joint = thisTransform.GetComponent<ConfigurableJoint>();

            Joint.slerpDrive = new JointDrive
            {
                positionDamper = jointDriver.jointDampen,
                positionSpring = jointDriver.jointSpring,
                maximumForce = jointDriver.maxJointForce
            };
        }


        partTransform = thisTransform;

        groundContact = thisTransform.GetComponent<GroundContact>();

        if (!groundContact)
        {
            groundContact = thisTransform.gameObject.AddComponent<GroundContact>();
        }

        groundContact.agent = thisAgent;

        startRotation = thisTransform.rotation;
        startPosition = thisTransform.position;



        partRigidBody = thisTransform.GetComponent<Rigidbody>();

    }

    public Vector3 currentEularJointRotation;
    public float currentXNormalizedRot;
    public float currentYNormalizedRot;
    public float currentZNormalizedRot;

    // straight up stolen from ml agents <3
    public void SetTargetRotation(float x, float y, float z)
    {
        x = (x + 1f) * 0.5f;
        y = (y + 1f) * 0.5f;
        z = (z + 1f) * 0.5f;

        var xRot = Mathf.Lerp(Joint.lowAngularXLimit.limit, Joint.highAngularXLimit.limit, x);
        var yRot = Mathf.Lerp(-Joint.angularYLimit.limit, Joint.angularYLimit.limit, y);
        var zRot = Mathf.Lerp(-Joint.angularZLimit.limit, Joint.angularZLimit.limit, z);

        currentXNormalizedRot = Mathf.InverseLerp(Joint.lowAngularXLimit.limit, Joint.highAngularXLimit.limit, xRot);
        currentYNormalizedRot = Mathf.InverseLerp(-Joint.angularYLimit.limit, Joint.angularYLimit.limit, yRot);
        currentZNormalizedRot = Mathf.InverseLerp(-Joint.angularZLimit.limit, Joint.angularZLimit.limit, zRot);

        Joint.targetRotation = Quaternion.Euler(xRot, yRot, zRot);
        currentEularJointRotation = new Vector3(xRot, yRot, zRot);
    }

    public void Reset()
    {
        partTransform.position = startPosition;
        partTransform.rotation = startRotation;

        partRigidBody.angularVelocity = Vector3.zero;
        partRigidBody.linearVelocity = Vector3.zero;

        if (groundContact)
        {
            groundContact.touchingGround = false;
        }
    }
    
}
