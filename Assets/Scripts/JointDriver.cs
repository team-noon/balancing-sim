using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using Unity.MLAgents;
using UnityEngine;

public class JointDriver : MonoBehaviour
{

    public Dictionary<Transform, BodyPart> BodyParts = new Dictionary<Transform, BodyPart>();

    public void addBodyPart(Transform bodyPartTransform)
    {


        BodyPart newBodyPart = new BodyPart(bodyPartTransform);

        BodyParts.Add(bodyPartTransform, newBodyPart);
    }
}

public class BodyPart
{
    public ArticulationBody Joint;
    public GroundContact groundContact;
    public Quaternion spriteStartRotation;
    public Vector3 spriteStartPosition;
    public Transform partTransform;
    public Quaternion startRotation;
    public Vector3 startPosition;
    public ArticulationReducedSpace startPositionJoint;


    public BodyPart(Transform thisTransform)
    {

            Joint = thisTransform.GetComponent<ArticulationBody>();

        


        partTransform = thisTransform;

        foreach(Transform child in thisTransform)
        {
            groundContact = child.GetComponent<GroundContact>();
            if (groundContact)
            {
                break;
            }
        }

        if (!groundContact)
        {
            throw new System.Exception($"EVERY TRANSFORM NEEDS TO HAVE A CHILD WITH A GROUNDCONTACT, this error was thrown at {thisTransform.transform}");
        }

        spriteStartPosition = groundContact.transform.position;
        spriteStartRotation = groundContact.transform.rotation;


        startRotation = thisTransform.localRotation;
        startPosition = thisTransform.position;
        startPositionJoint = Joint.jointPosition;
        this.SetTargetRotation(0f, 0f, 0f);

        Joint.SetDriveForceLimit(ArticulationDriveAxis.X, 10);
        Joint.SetDriveForceLimit(ArticulationDriveAxis.Y, 10);
        Joint.SetDriveForceLimit(ArticulationDriveAxis.Z, 10);


    }




    public void SetTargetRotation(float x, float y, float z)
    {
        Joint.SetDriveTarget(ArticulationDriveAxis.X, Mathf.Lerp(Joint.xDrive.lowerLimit, Joint.xDrive.upperLimit, (x+1f)/2f));
        Joint.SetDriveTarget(ArticulationDriveAxis.Y, Mathf.Lerp(Joint.yDrive.lowerLimit, Joint.yDrive.upperLimit, (y+1f)/2f));
        Joint.SetDriveTarget(ArticulationDriveAxis.Z, Mathf.Lerp(Joint.zDrive.lowerLimit, Joint.zDrive.upperLimit, (z+1f)/2f));


    }


    public void Reset(bool isRoot = false)
    {
        // Reset drive targets (neutral position)


        // Reset velocities
        Joint.linearVelocity = Vector3.zero;
        Joint.angularVelocity = Vector3.zero;

        
        if (isRoot)
        {
            // Only the root articulation body can be teleported
            Joint.TeleportRoot(startPosition, startRotation);
        }

        else
        {

            SetTargetRotation(0f, 0f, 0f);

            Joint.jointPosition = startPositionJoint;

            if (Joint.dofCount == 1)
            {
                Joint.jointForce = new ArticulationReducedSpace(0f);
                Joint.jointVelocity = new ArticulationReducedSpace(0f);
            }
            else if (Joint.dofCount == 3)
            {
                Joint.jointForce = new ArticulationReducedSpace(0f, 0f, 0f);
                Joint.jointVelocity = new ArticulationReducedSpace(0f, 0f, 0f);
            }
            else
            {
                Joint.jointForce = new ArticulationReducedSpace();
                Joint.jointVelocity = new ArticulationReducedSpace();
            }
        }

        // Reset ground contact sensor state
        if (groundContact != null)
        {
            groundContact.touchingGround = false;
        }
    }

}


