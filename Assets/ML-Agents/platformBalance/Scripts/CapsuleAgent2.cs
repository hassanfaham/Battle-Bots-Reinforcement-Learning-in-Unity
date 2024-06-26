using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Collections;

public class CapsuleAgent2 : Agent
{
    
    Rigidbody Capsule_rb;
    // public GameObject platforme;
    private float moveSpeed = 5f;
    // private float forceMagnitude = 700f;
    private Vector3 startingPosition;
    // private float platformWidth;
    // private float platformLength;

    // public GameObject whiteCapsulePrefab;

    // private RayPerceptionSensorComponent3D rayPerceptionSensor;

    

    // private bool hasSpawnedWhiteCapsule = false;

    // private GameObject whiteCapsule;
    


    public override void Initialize()
    {
        Capsule_rb = GetComponent<Rigidbody>();
        startingPosition = transform.position;

        // platformWidth = platforme.transform.localScale.x;
        // platformLength = platforme.transform.localScale.z;

        // rayPerceptionSensor = GetComponent<RayPerceptionSensorComponent3D>();

        
        

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Add the position and velocity of the capsule as observations
        sensor.AddObservation(transform.position);
        sensor.AddObservation(Capsule_rb.velocity);
        // sensor.AddObservation(whiteCapsule.transform.position);

    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float moveX = actionBuffers.ContinuousActions[0];
        float moveZ = actionBuffers.ContinuousActions[1];
        Vector3 movement = new Vector3(moveX, 0f, moveZ);        
        movement = movement.normalized * moveSpeed * Time.deltaTime;
        transform.Translate(movement, Space.Self);



        // if (whiteCapsule != null)
        // {


        //     Collider agentCollider = GetComponent<Collider>();
        //     Collider whiteCapsuleCollider = whiteCapsule.GetComponent<Collider>();
        //     Vector3 closestPointOnAgent = agentCollider.ClosestPoint(whiteCapsuleCollider.transform.position);
        //     Vector3 closestPointOnWhiteCapsule = whiteCapsuleCollider.ClosestPoint(transform.position);
        //     float distance = Vector3.Distance(closestPointOnAgent, closestPointOnWhiteCapsule);


            
        //     if (distance < 0.05)
        //     {


        //         SetReward(0.4f);
        //         Vector3 direction = (closestPointOnWhiteCapsule - closestPointOnAgent).normalized;
        //         Rigidbody whiteCapsuleRigidbody = whiteCapsule.GetComponent<Rigidbody>();
        //         Rigidbody agentRigidbody = GetComponent<Rigidbody>();

        //         whiteCapsuleRigidbody.AddForce(direction * forceMagnitude, ForceMode.Impulse);
        //         agentRigidbody.AddForce(-direction * forceMagnitude, ForceMode.Impulse);

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

        if (transform.position.y > 0f){
            SetReward(0.02f);
        }
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
            transform.position = startingPosition;
            Capsule_rb.velocity = Vector3.zero;
        }
        
        // transform.forward = Vector3.forward;

        
        

        // Zero out agent's velocity
        // Capsule_rb.velocity = Vector3.zero;
        // SpawnWhiteCapsule();
        
    }

    // public override void Heuristic(in ActionBuffers actionsOut)
    // {
    // //   // Use ZQSD keys to set the actions
    // //     var continuousActionsOut = actionsOut.ContinuousActions;
    // //     continuousActionsOut[0] = Input.GetKey(KeyCode.Q) ? -1f : 0f;
    // //     continuousActionsOut[0] += Input.GetKey(KeyCode.D) ? 1f : 0f;
    // //     continuousActionsOut[1] = Input.GetKey(KeyCode.S) ? -1f : 0f;
    // //     continuousActionsOut[1] += Input.GetKey(KeyCode.Z) ? 1f : 0f;
        
    // }

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



