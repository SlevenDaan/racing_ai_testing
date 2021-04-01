using UnityEngine;

public class WaypointCar : Car
{
    [Header("-----WaypointCar-----")]
    [SerializeField] private bool drawWaypointGizmo = true;
    [Space]
    [SerializeField] protected Waypoints waypoints;
    [SerializeField] protected int currentWaypointIndex = 0;
    
    protected override void OnDrawGizmos()
    {
        Gizmos.color = gizmoColor;
        if (drawWaypointGizmo)
            Gizmos.DrawSphere(waypoints[currentWaypointIndex], 0.5f);

        base.OnDrawGizmos();
    }

    protected override void Update()
    {
        base.Update();

        //Waypoint
        if ((transform.position - waypoints[currentWaypointIndex]).magnitude <= waypoints.WaypointWith)
        {
            currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Count;
        }
    }
}
