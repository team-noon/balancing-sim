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
    public Quaternion startRotation;
    public Vector3 startPosition;
    public Rigidbody partRigidBody;
    public JointDriver jointDriver;

    public BodyPart(Transform thisTransform, Agent thisAgent, JointDriver? jd = null)
    {
        if (jd)
        {
            jointDriver = jd;
            Joint = thisTransform.GetComponent<ConfigurableJoint>();

            Joint.angularXDrive = new JointDrive
            {
                positionSpring = jointDriver.jointSpring,
                positionDamper = jointDriver.jointDampen,
                maximumForce = jointDriver.maxJointForce
            };
            Joint.angularYZDrive = new JointDrive
            {
                positionSpring = jointDriver.jointSpring,
                positionDamper = jointDriver.jointDampen,
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

        startRotation = thisTransform.localRotation;
        startPosition = thisTransform.position;



        partRigidBody = thisTransform.GetComponent<Rigidbody>();

    }




    public void SetTargetRotation(float x, float y, float z)
    {
        float targetX = Mathf.Lerp(Joint.lowAngularXLimit.limit, Joint.highAngularXLimit.limit, (x + 1f) * 0.5f);
        float targetY = Mathf.Lerp(-Joint.angularYLimit.limit, Joint.angularYLimit.limit, (y + 1f) * 0.5f);
        float targetZ = Mathf.Lerp(-Joint.angularZLimit.limit, Joint.angularZLimit.limit, (z + 1f) * 0.5f);

        Quaternion localRotation = Quaternion.Euler(targetX, targetY, targetZ);

        // targetRotation is relative to the "rest rotation"
        Joint.targetRotation = Quaternion.Inverse(localRotation) * startRotation;
        Debug.Log($"{Joint.name} targetRotation = {Joint.targetRotation.eulerAngles} axisParalel = {(Joint.axis.normalized + Joint.secondaryAxis.normalized).magnitude == 2 || (Joint.axis.normalized + Joint.secondaryAxis.normalized).magnitude == 0}");
    }


    public void Reset()
    {
        partTransform.position = startPosition;
        partTransform.localRotation = startRotation;

        partRigidBody.angularVelocity = Vector3.zero;
        partRigidBody.linearVelocity = Vector3.zero;

        if (groundContact)
        {
            groundContact.touchingGround = false;
        }
    }
    
}


