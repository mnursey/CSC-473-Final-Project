using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgentsExamples;

public class SingleAxisTranslationJoint : MonoBehaviour
{
    public float translationDistance = 1f;

    public Material groundedMaterial;
    public Material unGroundedMaterial;
    MeshRenderer mr;

    Rigidbody rb;
    GroundContact groundContact;

    Vector3 defaultLocalPos;
    Quaternion defaultLocalQuaternion;

    public SingleAxis localAxisOfMovement;

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

        rb.velocity = transform.TransformDirection(Vector3.zero);
        rb.angularVelocity = transform.TransformDirection(Vector3.zero);

        if (groundContact)
        {
            groundContact.touchingGround = false;
        }
    }

    private void FixedUpdate()
    {
        mr.material = groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;

        rb.velocity = transform.TransformDirection(Vector3.zero);
        rb.angularVelocity = transform.TransformDirection(Vector3.zero);

        transform.localRotation = defaultLocalQuaternion;
    }

    public void UpdateTranslationPosition(float value)
    {
        switch(localAxisOfMovement)
        {
            case SingleAxis.X_AXIS:
                transform.localPosition = new Vector3(Mathf.Clamp(value, -1f, 1f) * translationDistance, defaultLocalPos.y, defaultLocalPos.z);

                break;

            case SingleAxis.Y_AXIS:
                transform.localPosition = new Vector3(defaultLocalPos.x , Mathf.Clamp(value, -1f, 1f) * translationDistance, defaultLocalPos.z);

                break;

            case SingleAxis.Z_AXIS:
                transform.localPosition = new Vector3(defaultLocalPos.x , defaultLocalPos.y, Mathf.Clamp(value, -1f, 1f) * translationDistance);

                break;
        }
    }

    public Vector3 GetPosDiff()
    {
        return transform.localPosition - defaultLocalPos;
    }

    public bool IsGrounded()
    {
        return groundContact.touchingGround;
    }
}

[System.Serializable]
public enum SingleAxis { X_AXIS, Y_AXIS, Z_AXIS };
