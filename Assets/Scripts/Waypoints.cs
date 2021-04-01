using System.Linq;
using UnityEngine;

public class Waypoints : MonoBehaviour
{
    [SerializeField] private bool drawTrackGizmos = true;
    [SerializeField] private bool drawCheckpointGizmos = true;
    [Space]
    [SerializeField] private Transform[] waypoints = new Transform[0];
    [SerializeField] private float waypointWidth = 7f;

    public Vector3 this[int index] => GetLoopedWaypoint(index);
    public float WaypointWith => waypointWidth;

    public int Count => waypoints.Length;

    private Vector3 GetLoopedWaypoint(int index)
    {
        while (index < 0) { index += Count; }
        return waypoints[index % Count].position;
    }

    private void OnDrawGizmos()
    {
        waypoints = transform.GetComponentsInChildren<Transform>();
        waypoints = waypoints.Where(w => !w.gameObject.Equals(gameObject)).ToArray();

        for (int i = 0; i < waypoints.Length; i++)
        {
            Gizmos.color = Color.red;
            
            if(drawTrackGizmos)
                Gizmos.DrawLine(this[i], this[i + 1]);

            if (drawCheckpointGizmos)
            {
                Gizmos.DrawWireSphere(this[i], waypointWidth);
                Gizmos.DrawSphere(this[i], 0.5f);
            }
        }
    }
}
