using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class Mouvement : MonoBehaviour
{
    private float moveSpeed = 8f;
    // private float jumpForce = 5f;
    
    private Rigidbody rb;

    
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();

        
    }

    void Update()
    {
        float verticalInput = Input.GetAxis("Vertical");
        float horizontalInput = Input.GetAxis("Horizontal");

        Vector3 movement = new Vector3(horizontalInput, 0f, verticalInput);

        movement = movement.normalized * moveSpeed * Time.deltaTime;
        transform.Translate(movement, Space.Self);
        // rb.AddForce(movement, ForceMode.VelocityChange);
        // if (Input.GetKeyDown(KeyCode.Space))
        // {
        //     Jump();
        // }
    }
    // private void Jump()
    // {
    //     if (Mathf.Abs(rb.velocity.y) < 0.01f)
    //     {
    //         rb.AddForce(Vector3.up * jumpForce, ForceMode.Impulse);
    //     }
    // }






        
}
