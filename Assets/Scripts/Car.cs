using UnityEngine;

public class Car : MonoBehaviour
{
    [Header("-----Car-----")]
    [SerializeField] protected Color gizmoColor = Color.blue;
    [SerializeField] private MeshRenderer[] meshRenderers;
    [SerializeField] private bool drawThrottleGizmo = true;
    [Space]
    [SerializeField] protected float throttle = 0;
    [SerializeField] [Tooltip("Positive is right\nNegative is left")] protected float stearing = 0;
    [Space]
    [SerializeField] protected float maxSpeed = 3f;
    [SerializeField] protected float speed = 0;
    [Space]
    [SerializeField] protected float turnspeed = 90f;
    [Tooltip("in degrees")] [SerializeField] protected float maxStearingError = 1f;
    
    protected virtual void OnDrawGizmos()
    {
        Gizmos.color = gizmoColor;

        if (drawThrottleGizmo)
        {
            Gizmos.DrawRay(transform.position, transform.forward * speed);
            Gizmos.DrawRay(transform.position + Vector3.up, transform.forward * throttle);
        }
    }

    protected virtual void Start()
    {
        Material material = new Material(Shader.Find("Standard"));
        material.color = gizmoColor;
        foreach (var renderer in meshRenderers)
        {
            renderer.material = material;
        }
    }

    protected virtual void Update()
    {
        //Stearing
        transform.Rotate(Vector3.up * stearing * Time.deltaTime * turnspeed);

        //Throttle
        speed += throttle * Time.deltaTime * maxSpeed;
        speed = Mathf.Clamp(speed, 0, maxSpeed);
        transform.Translate(Vector3.forward * speed * Time.deltaTime);
    }
}
