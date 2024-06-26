using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Collections;
using System.Collections.Generic;

public class CapsuleAgent : Agent
{
    
    Rigidbody Capsule_rb;
    // public GameObject platforme;
    private float moveSpeed = 5f;
    private float forceMagnitude = 700f;
    private Vector3 startingPosition;
    // private float platformWidth;
    // private float platformLength;

    public GameObject whiteCapsulePrefab;

    private RayPerceptionSensorComponent3D rayPerceptionSensor;

    

    private bool hasSpawnedWhiteCapsule = false;

    private GameObject whiteCapsule;

    // List to store the positions and radii of the holes
    List<Vector3> holePositions = new List<Vector3>();
    List<double> holeRadius = new List<double>();

    List<Vector3> targetAreas = new List<Vector3>();

    private    Vector3 centerHole;
    private    Vector3 upLeftHole;
    private    Vector3 upRightHole;
    private   Vector3 DownLeftHole;
    private   Vector3 DownRightHole;
    

    public override void Initialize()
    {
        Capsule_rb = GetComponent<Rigidbody>();
        


        centerHole = new Vector3(-0.004015564918518066f,1.2f,0.3569144010543823f);
        upLeftHole = new Vector3(4.225170135498047f,1.2f,-4.503231525421143f);
        upRightHole = new Vector3(-4.871123313903809f,1.2f,-5.046647071838379f);
        DownLeftHole = new Vector3(4.508690357208252f,1.2f,4.144154071807861f);
        DownRightHole = new Vector3(-5.012884140014648f,1.2f,4.049646377563477f);

        targetAreas.Add(new Vector3(4.190000057220459f,1.2f,0.23999999463558198f));
        targetAreas.Add(new Vector3(-0.07999999821186066f,1.2f,4.210000038146973f));
        targetAreas.Add(new Vector3(-4.829999923706055f,1.2f,0.029999999329447748f));
        targetAreas.Add(new Vector3(-0.7900000214576721f,1.2f,-4.559999942779541f));

        

        holePositions.Add(centerHole);
        holePositions.Add(upLeftHole);
        holePositions.Add(upRightHole);
        holePositions.Add(DownLeftHole);
        holePositions.Add(DownRightHole);

        double centerHoleRad = 61.81926;
        double upLeftHoleRad = 74.72504;
        double upRightHoleRad = 52.59515;
        double DownLeftHoleRad = 31.23195;
        double DownRightHoleRad = 25.4;

        holeRadius.Add(centerHoleRad);
        holeRadius.Add(upLeftHoleRad);
        holeRadius.Add(upRightHoleRad);
        holeRadius.Add(DownLeftHoleRad);
        holeRadius.Add(DownRightHoleRad);

        // platformWidth = platforme.transform.localScale.x;
        // platformLength = platforme.transform.localScale.z;

        // rayPerceptionSensor = GetComponent<RayPerceptionSensorComponent3D>();

        

        
        

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Add the position and velocity of the capsule as observations


        sensor.AddObservation(transform.position);
        sensor.AddObservation(Capsule_rb.velocity);
        for (int i = 0; i < holePositions.Count; i++)
        {
            sensor.AddObservation(holePositions[i]);
            sensor.AddObservation((float)holeRadius[i]);
        }


        // sensor.AddObservation(whiteCapsule.transform.position);

    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float moveX = actionBuffers.ContinuousActions[0];
        float moveZ = actionBuffers.ContinuousActions[1];
        Vector3 movement = new Vector3(moveX, 0f, moveZ);        
        movement = movement.normalized * moveSpeed * Time.deltaTime;
        transform.Translate(movement, Space.Self);


        // Calculate agent's distance to target areas and assign rewards
        float proximityThreshold = 2f;
        float proximityReward = 0f;
        foreach (Vector3 targetArea in targetAreas)
        {
            float distance = Vector3.Distance(transform.position, targetArea);
            if (distance <= proximityThreshold)
            {
                proximityReward += (proximityThreshold - distance) / 2*proximityThreshold;
            }
        }

        // Provide the proximity-based reward to the agent
        SetReward(proximityReward);


        float holePenaltyMultiplier = 0.1f;
        float holeProximityThreshold = 0.5f;

        float closestDistanceToHole = float.MaxValue;
        foreach (Vector3 hole in holePositions)
        {
            float distanceToHole = Vector3.Distance(transform.position, hole);
            if (distanceToHole < closestDistanceToHole)
            {
                closestDistanceToHole = distanceToHole;
            }
        }
        // Assign negative reward for proximity to holes
        float holePenalty = -holePenaltyMultiplier * (holeProximityThreshold - closestDistanceToHole);
        SetReward(holePenalty);


        // if (whiteCapsule != null)
        // {


        //     Collider agentCollider = GetComponent<Collider>();
        //     Collider whiteCapsuleCollider = whiteCapsule.GetComponent<Collider>();
        //     Vector3 closestPointOnAgent = agentCollider.ClosestPoint(whiteCapsuleCollider.transform.position);
        //     Vector3 closestPointOnWhiteCapsule = whiteCapsuleCollider.ClosestPoint(transform.position);
        //     float distance = Vector3.Distance(closestPointOnAgent, closestPointOnWhiteCapsule);
            
        //     // CapsuleCollider agentCollider = GetComponent<CapsuleCollider>();
        //     // Vector3 directionToWhiteCapsule = whiteCapsule.transform.position - transform.position;
        //     // directionToWhiteCapsule.y = 0f;
        //     // directionToWhiteCapsule = directionToWhiteCapsule.normalized;

        //     // bool isColliding = Physics.CheckSphere(transform.position, agentCollider.radius, LayerMask.GetMask("enemyCapsule"));



            
        //     if (distance < 0.05)
        //     {


        //         SetReward(0.3f);
        //         Vector3 direction = (closestPointOnWhiteCapsule - closestPointOnAgent).normalized;
        //         Rigidbody whiteCapsuleRigidbody = whiteCapsule.GetComponent<Rigidbody>();
        //         Rigidbody agentRigidbody = GetComponent<Rigidbody>();

        //         whiteCapsuleRigidbody.AddForce(direction * forceMagnitude, ForceMode.Impulse);
        //         agentRigidbody.AddForce(-direction * forceMagnitude, ForceMode.Impulse);


        //         // SetReward(0.3f);
        //         // Vector3 direction = directionToWhiteCapsule;
        //         // Rigidbody whiteCapsuleRigidbody = whiteCapsule.GetComponent<Rigidbody>();
        //         // Rigidbody agentRigidbody = GetComponent<Rigidbody>();

        //         // whiteCapsuleRigidbody.AddForce(direction * forceMagnitude, ForceMode.Impulse);
        //         // agentRigidbody.AddForce(-direction * forceMagnitude, ForceMode.Impulse);


        //     }
        // }

        // Vector3 whiteCapsulePos = whiteCapsule?.gameObject.transform.position ?? Vector3.zero;
        // if (whiteCapsulePos.y < -1f)
        // {
        //     SetReward(1f);
        //     Destroy(whiteCapsule);
        //     hasSpawnedWhiteCapsule = false;
        //     EndEpisode();
        // }
            // Check proximity to holes 




        // if (transform.position.y > 0f){
        //     SetReward(0.2f);
        // }
        if (transform.position.y < -1f)
        {
            SetReward(-0.8f);
            EndEpisode();
        }



    }

    public override void OnEpisodeBegin()
    {
       // Teleport agent back to starting position
        // transform.position = startingPosition;
        if (transform.position.y < -1f)
        {
            transform.position = targetAreas[Random.Range(0,targetAreas.Count)];
            Capsule_rb.velocity = Vector3.zero;
        }
        
        // transform.forward = Vector3.forward;

        
        

        // Zero out agent's velocity
        // Capsule_rb.velocity = Vector3.zero;
        // SpawnWhiteCapsule();
        
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
      // Use ZQSD keys to set the actions
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetKey(KeyCode.Q) ? -1f : 0f;
        continuousActionsOut[0] += Input.GetKey(KeyCode.D) ? 1f : 0f;
        continuousActionsOut[1] = Input.GetKey(KeyCode.S) ? -1f : 0f;
        continuousActionsOut[1] += Input.GetKey(KeyCode.Z) ? 1f : 0f;
        
    }

    // public void SpawnWhiteCapsule()
    // {

    //     if (hasSpawnedWhiteCapsule)
    //     {
    //         // If a white capsule has already been spawned, return without spawning a new one.
    //         return;
    //     }
    //     // Define the minimum and maximum positions where the white capsule can spawn
    //     float minX = -6f;
    //     float maxX = 6f;
    //     float minZ = -6f;
    //     float maxZ = 6f;

    //     // Generate a random position within the defined range
    //     Vector3 randomPosition = new Vector3(
    //         Random.Range(minX, maxX),
    //         1.2f,
    //         Random.Range(minZ, maxZ)
    //     );



    //     // Spawn the white capsule at the random position
    //     whiteCapsule = Instantiate(whiteCapsulePrefab, randomPosition, Quaternion.identity);
    //     hasSpawnedWhiteCapsule = true;
    // }


   
}

