using System;
using UnityEngine;

public class Car_PurePursuitPlus : WaypointCar
{
    private struct CollisionInfo
    {
        public float distanceToCollision;
        public float distanceOfRaycast;
    }

    [Header("-----Car_PurePursuitPlus-----")]
    [SerializeField] [Tooltip("The base distance for the AI too look ahead")] private float lookAheadConstant = 5;
    [SerializeField] [Tooltip("The multiplier per step of look ahead")] private float lookAheadMultiplier = 2;
    [SerializeField] [Tooltip("The amount of look ahead steps")] private float lookAheadSteps = 3;
    [SerializeField] [Range(1,2)] private float safety = 1f;
    [Space]
    [SerializeField] private LayerMask collisionMask;
    [SerializeField] private float collisionDetectionDistance = 3f;
    [SerializeField] private int collisionSegmentsPerSide = 9;
    [Space]
    [SerializeField] private float frontalCollisionOffset = 0.8f;
    [SerializeField] private float sideCollisionOffset = 0.4f;
    [Space]
    [SerializeField] private float preferedCollisionAvoidanceAngle = 10f;

    protected override void OnDrawGizmos()
    {
        base.OnDrawGizmos();

        float lookAheadDistance = lookAheadConstant;
        float gizmoSize = 0.5f;
        for (int i = 0; i < lookAheadSteps; i++)
        {
            Gizmos.DrawSphere(GetLookAheadPointOnTrack(ProjectPositionOntoTrack(), lookAheadDistance), gizmoSize);

            lookAheadDistance *= lookAheadMultiplier;
            gizmoSize -= 0.1f;
        }
    }

    protected override void Update()
    {
        CalculateStearing();
        CalculateThrottle();
        base.Update();
    }

    #region Helper Functions
    private Vector3 ProjectPositionOntoTrack()
    {
        return ProjectPointOntoLine(transform.position, waypoints[currentWaypointIndex - 1], waypoints[currentWaypointIndex]);
    }

    private Vector3 GetLookAheadPointOnTrack(Vector3 currentTrackPoint, float lookAheadDistance)
    {
        int waypointIndex = currentWaypointIndex;

        int safety = 10;

        while ((waypoints[waypointIndex] - currentTrackPoint).magnitude < lookAheadDistance)
        {
            lookAheadDistance -= (waypoints[waypointIndex] - currentTrackPoint).magnitude;
            currentTrackPoint = waypoints[waypointIndex];
            waypointIndex++;

            if (safety < 0) { return Vector3.zero; }
            safety--;
        }

        Vector3 direction = (waypoints[waypointIndex] - currentTrackPoint).normalized;
        return currentTrackPoint + direction * lookAheadDistance;
    }

    private Vector3 ProjectPointOntoLine(Vector3 point, Vector3 linePoint1, Vector3 linePoint2)
    {
        Vector3 waypointVector = linePoint2 - linePoint1;
        Vector3 positionVector = point - linePoint1;

        Vector3 projectedPoint = linePoint1 + Vector3.Dot(positionVector, waypointVector) / Vector3.Dot(waypointVector, waypointVector) * waypointVector;

        return projectedPoint;
    }
    #endregion

    #region Throttle
    private void CalculateThrottle()
    {
        float lookAheadDistance = lookAheadConstant;
        bool turnable = true;
        for (int i = 0; i < lookAheadSteps; i++)
        {
            lookAheadDistance *= lookAheadMultiplier;
            if (!IsTurnable(lookAheadDistance))
                turnable = false;
        }

        throttle = turnable ? 1f : -1f;
    }

    private bool IsTurnable(float lookAheadDistance)
    {
        Vector3 lookAheadPoint = GetLookAheadPointOnTrack(ProjectPositionOntoTrack(), lookAheadDistance);

        Vector3 lookAheadVector = lookAheadPoint - transform.position;
        Vector3 velocity = transform.forward * speed;

        float angleToLookAhead = Vector3.Angle(velocity, lookAheadVector);
        float distanceToLookAhead = lookAheadVector.magnitude;

        float timeToLookAhead = distanceToLookAhead / speed;
        float maxTurnableToWaypoint = timeToLookAhead * turnspeed;

        return maxTurnableToWaypoint >= angleToLookAhead*safety;
    }
    #endregion

    #region Stearing
    private void CalculateStearing()
    {
        //Pure Pursuit
        Vector3 positionOnTrack = ProjectPositionOntoTrack();
        Vector3 lookAheadPoint = GetLookAheadPointOnTrack(positionOnTrack, lookAheadConstant);

        Vector3 lookAheadVector = lookAheadPoint - transform.position;
        Vector3 velocity = transform.forward * speed;

        float signedAngleToLookAhead = Vector3.SignedAngle(velocity, lookAheadVector, Vector3.up);

        //Collision Avoidance
        float collisionStearingAngle = 0;
        Vector3 frontalPosition = transform.position + transform.forward * frontalCollisionOffset;
        Vector3 rightFrontalPosition = frontalPosition + transform.right * sideCollisionOffset;
        Vector3 leftFrontalPosition = frontalPosition - transform.right * sideCollisionOffset;

        if (GetCollision(frontalPosition, 0, collisionDetectionDistance, out CollisionInfo _, true))
        {
            collisionStearingAngle += preferedCollisionAvoidanceAngle;
        }

        float anglePerSegment = 90f / collisionSegmentsPerSide;

        for (float i = 0; i <= collisionSegmentsPerSide; i++)
        {
            if (GetCollision(rightFrontalPosition, anglePerSegment * i, collisionDetectionDistance, out CollisionInfo _, true))
            {
                collisionStearingAngle -= anglePerSegment;
            }
            if (GetCollision(leftFrontalPosition, -anglePerSegment * i, collisionDetectionDistance, out CollisionInfo _, true))
            {
                collisionStearingAngle += anglePerSegment;
            }
        }

        //Steering
        float stearingAngle = Mathf.Clamp((signedAngleToLookAhead + collisionStearingAngle) / maxStearingError, -1f, 1f);

        stearing = Mathf.Clamp(stearingAngle, -1f, 1f);
    }

    private bool GetCollision(Vector3 origin, float angleInDegrees, float distance, out CollisionInfo collisionInfo, bool drawDebug = true)
    {
        collisionInfo = new CollisionInfo();
        Vector3 angleVector = RotateVector(transform.forward, angleInDegrees);
        bool collidedWithCar = Physics.Raycast(origin, angleVector, out RaycastHit carRaycastHit, distance, collisionMask);

        if (drawDebug && collidedWithCar) { Debug.DrawRay(origin, angleVector.normalized * distance, Color.magenta); }

        if (collidedWithCar)
        {
            collisionInfo.distanceToCollision = carRaycastHit.distance;
            collisionInfo.distanceOfRaycast = distance;
        }

        return collidedWithCar;
    }

    private Vector3 RotateVector(Vector3 vector, float angleInDegrees)
    {
        float radians = -DegreesToRadians(angleInDegrees);

        return new Vector3(
            vector.x * Mathf.Cos(radians) - vector.z * Mathf.Sin(radians),
            0,
            vector.x * Mathf.Sin(radians) + vector.z * Mathf.Cos(radians)
        );
    }

    private float DegreesToRadians(float degrees)
    {
        return degrees * Convert.ToSingle(Math.PI) / 180f;
    }
    #endregion
}
