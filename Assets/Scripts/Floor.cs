using Unity.Mathematics;
using UnityEngine;

public class Floor : MonoBehaviour
{


    private void OnCollisionEnter(Collision collision)
    {
        GroundContact GC = collision.collider.GetComponent<GroundContact>();
        if (GC)
        {
            GC.touchingGround = true;

            if (GC.penalizeGroundContact)
            {
                GC.agent.AddReward(GC.groundContactPenalty);
                Debug.Log($"[Floor] Penalized agent '{GC.agent.name}' with {GC.groundContactPenalty} for ground contact.");
            }

            if (GC.agentDoneOnGroundContact)
            {
                GC.agent.SetReward(-100f);
                GC.agent.EndEpisode();
                Debug.Log($"[Floor] Agent '{GC.agent.name}' episode ended due to ground contact. Reward set to -100.");
            }
        }
    }
}
