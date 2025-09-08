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
            }

            if (GC.agentDoneOnGroundContact)
            {
                GC.agent.EndEpisode();
            }

            
        }
    }
}
