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
    public float groundContactPenaltyMultipier = 1;
    public Collider thisCollider;
    public bool touchingGround;
    const string k_Ground = "ground"; // Tag of ground object.


    /// <summary>
    /// Check for end of ground collision and reset flag appropriately.
    /// </summary>
    void OnCollisionExit(Collision other)
    {
        if (other.gameObject.layer == 7)
        {
            thisCollider.transform.GetComponent<GroundContact>().touchingGround = false;
        }

    }

    private void FixedUpdate()
    {
        if (touchingGround && penalizeGroundContact && !agentDoneOnGroundContact)
        {
            //Debug.Log($"Penalty applied to agent '{agent.name}' for ground contact: {groundContactPenalty * groundContactPenaltyMultipier}");
            agent.AddReward(groundContactPenalty * groundContactPenaltyMultipier);
        }
    }
}
