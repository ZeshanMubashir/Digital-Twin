using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class NewShipMovement : MonoBehaviour
{
    public TextAsset csvFile; // CSV file to import
    public float moveSpeed = 5f; // Speed of ship movement
    private List<Vector3> waypoints; // List of waypoints for ship movement
    private int currentWaypointIndex = 0; // Index of current waypoint

    private void Start()
    {
        waypoints = LoadWaypointsFromCSV(csvFile);
    }

private void Update()
{
    if (currentWaypointIndex < waypoints.Count)
    {
        // Get the current waypoint's X and Z coordinates
        Vector3 targetPosition = new Vector3(waypoints[currentWaypointIndex].x, transform.position.y, waypoints[currentWaypointIndex].z);

        // Move the ship towards the current waypoint
        transform.position = Vector3.MoveTowards(transform.position, targetPosition, moveSpeed * Time.deltaTime);

        // Calculate the direction vector to the current waypoint
        Vector3 direction = targetPosition;
        direction.Normalize(); // Normalize the direction vector

        // Calculate the rotation angle
        float angle = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;
        Debug.Log("Direction: " + direction);
        Debug.Log("Angle: " + angle);
        // float angle = Random.Range(-20,20);

        Debug.Log(angle);

        // Rotate the ship towards the current waypoint
        Quaternion targetRotation = Quaternion.Euler(0f, angle, 0f);
        transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, 360f * Time.deltaTime);

        // Check if the ship is close enough to the current waypoint
        float distanceToWaypoint = Vector3.Distance(transform.position, targetPosition);
        if (distanceToWaypoint < 0.1f)
        {
            currentWaypointIndex++; // Move to the next waypoint
        }
    }
}



    private List<Vector3> LoadWaypointsFromCSV(TextAsset csv)
    {
        List<Vector3> waypoints = new List<Vector3>();

        using (StringReader reader = new StringReader(csv.text))
        {
            string line;
            while ((line = reader.ReadLine()) != null)
            {
                string[] values = line.Split(',');

                // Parse the X, Y, and Z coordinates from the CSV file
                float x, y, z;
                if (float.TryParse(values[0], out x) && float.TryParse(values[1], out y) && float.TryParse(values[2], out z))
                {
                    // Create a new waypoint using the parsed coordinates
                    Vector3 waypoint = new Vector3(x, y, z);
                    waypoints.Add(waypoint);
                }
            }
        }

        return waypoints;
    }
}
