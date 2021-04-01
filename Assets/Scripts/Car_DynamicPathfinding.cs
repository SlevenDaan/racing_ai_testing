using System;
using UnityEngine;

public class Car_DynamicPathfinding : Car
{
    [Header("-----Car_DynamicPathfinding-----")]
    [SerializeField] private LayerMask collisionMask;
    [SerializeField] private float frontalCollisionDetectionDistance = 5f;
    [SerializeField] private float sideCollisionDetectionDistance = 3f;
    [Space]
    [SerializeField] private float frontalCollisionOffset = 0.8f;
    [SerializeField] private float sideCollisionOffset = 0.4f;
    [Space]
    [SerializeField] private float preferedCollisionAvoidanceAngle = 10f;

    protected override void OnDrawGizmos()
    {
        base.OnDrawGizmos();
    }

    protected override void Update()
    {
        CalculateStearing();
        CalculateThrottle();
        base.Update();
    }

    #region Throttle
    private void CalculateThrottle()
    {
        Vector3 frontalPosition = transform.position + transform.forward * frontalCollisionOffset;

        if (GetCollision(frontalPosition, 0, speed, out float distanceToCollision, false))
        {
            throttle = -1f * (distanceToCollision / speed);
        }
        else
        {
            throttle = 1f;
        }
    }
    #endregion

    #region Stearing
    private void CalculateStearing()
    {
        float stearingAngle = 0;
        Vector3 frontalPosition = transform.position + transform.forward * frontalCollisionOffset;
        Vector3 rightFrontalPosition = frontalPosition + transform.right * sideCollisionOffset;
        Vector3 leftFrontalPosition = frontalPosition - transform.right * sideCollisionOffset;

        //Frontal
        if (GetCollision(frontalPosition, 0, frontalCollisionDetectionDistance, out float _, true))
        {
            stearingAngle += preferedCollisionAvoidanceAngle;
        }

        float angleDifferenceFrontAndSide = frontalCollisionDetectionDistance - sideCollisionDetectionDistance;
        for (float i = 0; i <= 9f; i++)
        {
            float multiplier = (9f - i) / 9f;
            float distance = sideCollisionDetectionDistance + (multiplier * angleDifferenceFrontAndSide);

            if (GetCollision(rightFrontalPosition, 10 * i, distance, out float _, true))
            {
                stearingAngle -= 10;
            }
            if (GetCollision(leftFrontalPosition, -10 * i, distance, out float _, true))
            {
                stearingAngle += 10;
            }
        }

        stearing = Mathf.Clamp(stearingAngle/maxStearingError, -1f, 1f);
    }

    private bool GetCollision(Vector3 origin, float angleInDegrees, float distance, out float distanceToCollision, bool drawDebug = true)
    {
        distanceToCollision = 0f;
        Vector3 angleVector = RotateVector(transform.forward, angleInDegrees);
        bool collided = Physics.Raycast(origin, angleVector, out RaycastHit raycastHit, distance, collisionMask);

        if (collided)
        {
            distanceToCollision = raycastHit.distance;
            if (drawDebug) { Debug.DrawRay(origin, angleVector.normalized * distance, Color.cyan); }
        }

        return collided;
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
