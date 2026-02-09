using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class ShipMovement : MonoBehaviour
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
            // Move the ship towards the current waypoint
            Vector3 targetPosition = waypoints[currentWaypointIndex];
            transform.position = Vector3.MoveTowards(transform.position, targetPosition, moveSpeed * Time.deltaTime);

            // Check if the ship has reached the current waypoint
            if (transform.position == targetPosition)
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
