using Unity.MLAgents;
using UnityEngine;

// this is stolen from ml agents example script <3
[DisallowMultipleComponent]
public class GroundContact : MonoBehaviour
{
    public Agent agent;

    [Header("Ground Check")] 
    public bool agentDoneOnGroundContact = true; // Whether to reset agent on ground contact.
    public bool penalizeGroundContact = true; // Whether to penalize on contact.
    public float groundContactPenalty = -1; // Penalty amount (ex: -1).
    public Collider thisCollider;
    public bool touchingGround;
    const string k_Ground = "ground"; // Tag of ground object.

    /// <summary>
    /// Check for collision with ground, and optionally penalize agent.
    /// </summary>
    void OnCollisionStay(Collision col)
    {
        if (col.transform.CompareTag(k_Ground) && col.contacts[0].thisCollider == thisCollider)
        {
            Debug.Log($"collided {transform.name}");
            touchingGround = true;
            if (penalizeGroundContact)
            {
                agent.AddReward(groundContactPenalty);
            }

            if (agentDoneOnGroundContact)
            {

                agent.EndEpisode();
            }
        }
    }

    /// <summary>
    /// Check for end of ground collision and reset flag appropriately.
    /// </summary>
    void OnCollisionExit(Collision other)
    {
        Debug.Log(other.contacts.Length);
        if (other.transform.CompareTag(k_Ground) && other.contacts[0].thisCollider == thisCollider)
        {
            touchingGround = false;
        }
    }
}
