using System.Collections;
using UnityEngine;

public class boxScript : MonoBehaviour
{
    [SerializeField] float Size = 1;
    [SerializeField] float SizeDeviation = 0;
    [SerializeField] float AngleDeviation = 0;
    [SerializeField] float InitialVelocity = 1;
    [SerializeField] float VelocityDeviation = 0;
    [SerializeField] float Mass = 0.01f;
    [SerializeField] float MassDeviation = 0.002f;
    [SerializeField] float AngularVelocityDeviation = 0.2f;

    Transform target;

    Rigidbody rigid;

    public void Init(Transform _target)
    {
        target = _target;

        if (!rigid)
        {
            rigid = gameObject.GetComponent<Rigidbody>();
        }

        this.transform.localScale = new Vector3(
            Size + Random.Range(-SizeDeviation, SizeDeviation),
            Size + Random.Range(-SizeDeviation, SizeDeviation),
            Size + Random.Range(-SizeDeviation, SizeDeviation)
        );

        rigid.linearVelocity = Quaternion.AngleAxis(Random.Range(-AngleDeviation, AngleDeviation), Vector3.up) * (target.position - transform.position).normalized * (InitialVelocity + Random.Range(-VelocityDeviation, VelocityDeviation));

        rigid.mass = Mass + Random.Range(-MassDeviation, MassDeviation);

        rigid.angularVelocity = new Vector3(Random.Range(-AngularVelocityDeviation, AngularVelocityDeviation), Random.Range(-AngularVelocityDeviation, AngularVelocityDeviation), Random.Range(-AngularVelocityDeviation, AngularVelocityDeviation));
    }

    private void FixedUpdate()
    {
        if (Vector3.Distance(target.position, transform.position) > 100)
        {
            Destroy(gameObject);
            return;
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.layer == 7)
        {
            Destroy(gameObject);
            StopAllCoroutines();
            return;
        }
        else
        {
            StartCoroutine(DeleteIn(0.2f));
            return;
        }
    }

    IEnumerator DeleteIn(float secs)
    {
        yield return new WaitForSeconds(secs);
        Destroy(gameObject);

    }
}
