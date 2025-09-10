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
    public Vector3 startingJointRotation;


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
        /*
    float xRange = Joint.xDrive.upperLimit - Joint.xDrive.lowerLimit;
    float yRange = Joint.yDrive.upperLimit - Joint.yDrive.lowerLimit;
    float zRange = Joint.zDrive.upperLimit - Joint.zDrive.lowerLimit;

        // Map 0 to the range [-1, 1] between lowerLimit and upperLimit
        float xRot = xRange != 0 ? Mathf.Clamp(((0 - Joint.xDrive.lowerLimit) / xRange) * 2f - 1f, -1f, 1f) : 0f;
        float yRot = yRange != 0 ? Mathf.Clamp(((0 - Joint.yDrive.lowerLimit) / yRange) * 2f - 1f, -1f, 1f) : 0f;
        float zRot = zRange != 0 ? Mathf.Clamp(((0 - Joint.zDrive.lowerLimit) / zRange) * 2f - 1f, -1f, 1f) : 0f;

startingJointRotation = new Vector3(xRot, yRot, zRot);
        Debug.Log($"Listen here you fat fuck: {startingJointRotation.x} {startingJointRotation.y} {startingJointRotation.z}");
this.SetTargetRotation(startingJointRotation.x, startingJointRotation.y, startingJointRotation.z); */

        startingJointRotation = new Vector3(0, 0, 0);
        SetTargetRotation(0, 0, 0);

        Joint.SetDriveForceLimit(ArticulationDriveAxis.X, 30);
        Joint.SetDriveForceLimit(ArticulationDriveAxis.Y, 30);
        Joint.SetDriveForceLimit(ArticulationDriveAxis.Z, 30);
}




    public void SetTargetRotation(float x, float y, float z)
    {
        // old approach, -1 is lower limit 1 is upper limit, 0 is the avarage of the two
        //Joint.SetDriveTarget(ArticulationDriveAxis.X, Mathf.Lerp(Joint.xDrive.lowerLimit, Joint.xDrive.upperLimit, (x + 1f) / 2f));
        //Joint.SetDriveTarget(ArticulationDriveAxis.Y, Mathf.Lerp(Joint.yDrive.lowerLimit, Joint.yDrive.upperLimit, (y+1f)/2f));
        //Joint.SetDriveTarget(ArticulationDriveAxis.Z, Mathf.Lerp(Joint.zDrive.lowerLimit, Joint.zDrive.upperLimit, (z+1f)/2f));


        // Maps -1 to lowerLimit, 0 to 0, 1 to upperLimit for each axis

        float MapToLimit(float value, float lowerLimit, float upperLimit)
        {
            if (value < 0)
            {
                // value in [-1, 0): map to [lowerLimit, 0]
                return Mathf.Lerp(lowerLimit, 0f, value + 1f);
            }
            else
            {
                // value in [0, 1]: map to [0, upperLimit]
                return Mathf.Lerp(0f, upperLimit, value);
            }
        }

        Joint.SetDriveTarget(ArticulationDriveAxis.X, MapToLimit(x, Joint.xDrive.lowerLimit, Joint.xDrive.upperLimit));
        Joint.SetDriveTarget(ArticulationDriveAxis.Y, MapToLimit(y, Joint.yDrive.lowerLimit, Joint.yDrive.upperLimit));
        Joint.SetDriveTarget(ArticulationDriveAxis.Z, MapToLimit(z, Joint.zDrive.lowerLimit, Joint.zDrive.upperLimit));
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

            SetTargetRotation(startingJointRotation.x, startingJointRotation.y, startingJointRotation.z);

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


