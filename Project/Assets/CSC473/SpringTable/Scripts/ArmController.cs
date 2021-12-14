using System.Collections;
using System.Collections.Generic;
using Unity.MLAgentsExamples;
using UnityEngine;

public class ArmController : MonoBehaviour
{
    public Material groundedMaterial;
    public Material unGroundedMaterial;
    MeshRenderer mr;

    Rigidbody rb;
    GroundContact groundContact;

    public SpringJoint staticSpring;
    public SpringJoint dynamicSpring;

    public float maxForce;

    Vector3 defaultLocalPos;
    Quaternion defaultLocalQuaternion;

    /*
    public bool fixX = false;
    public bool fixY = false;
    public bool fixZ = false;
    */

    public void Awake()
    {
        rb = GetComponent<Rigidbody>();
        mr = GetComponent<MeshRenderer>();
        groundContact = GetComponent<GroundContact>();

        Setup();
    }

    public void Setup()
    {
        defaultLocalPos = transform.localPosition;
        defaultLocalQuaternion = transform.localRotation;
    }

    public void Reset()
    {
        transform.localPosition = defaultLocalPos;
        transform.localRotation = defaultLocalQuaternion;

        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        if (groundContact)
        {
            groundContact.touchingGround = false;
        }

        UpdateSpringForce(0f);
    }

    private void FixedUpdate()
    {
        mr.material = groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;


        /*
        if(fixX)
        {
            transform.localPosition = new Vector3(defaultLocalPos.x, transform.localPosition.y, transform.localPosition.z);
        }

        if(fixY)
        {
            transform.localPosition = new Vector3(transform.localPosition.x, defaultLocalPos.y, transform.localPosition.z);
        }

        if (fixZ)
        {
            transform.localPosition = new Vector3(transform.localPosition.x, transform.localPosition.y, defaultLocalPos.z);
        }*/
    }

    /// <summary>
    /// Expects a value between 0.0 and 1.0
    /// 0.0 is zero force, 1.0 is max force
    /// </summary>
    public void UpdateSpringForce(float value)
    {
        dynamicSpring.spring = Mathf.Clamp01(value) * maxForce;
    }

    public Vector3 GetPosDiff()
    {
        return transform.localPosition - defaultLocalPos;
    }

    public Vector3 GetVelocity()
    {
        return rb.velocity;
    }

    public bool IsGrounded()
    {
        return groundContact.touchingGround;
    }
}
