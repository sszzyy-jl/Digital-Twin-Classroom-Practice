using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class Rigid_Bunny : MonoBehaviour
{
    bool launched = false;
    float dt = 0.015f;
    Vector3 v = new Vector3(0, 0, 0);   // velocity
    Vector3 w = new Vector3(0, 0, 0);   // angular velocity

    public float mass;                                  // mass
    Matrix4x4 I_ref;                            // reference inertia

    float linear_decay = 0.999f;                // for velocity decay
    float angular_decay = 0.98f;
    float restitution = 0.5f;                 // for collision
    float friction = 0.2f;

    Mesh mesh;
    Vector3[] vertices;
    Vector3 x;
    Quaternion q;

    public Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);

    // Use this for initialization
    void Start()
    {
        mesh = GetComponent<MeshFilter>().mesh;
        vertices = mesh.vertices;

        float m = 1;
        mass = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * vertices[i].sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3, 3] = 1;
    }

    Matrix4x4 Get_Cross_Matrix(Vector3 a)
    {
        //Get the cross product matrix of vector a
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = 0;
        A[0, 1] = -a[2];
        A[0, 2] = a[1];
        A[1, 0] = a[2];
        A[1, 1] = 0;
        A[1, 2] = -a[0];
        A[2, 0] = -a[1];
        A[2, 1] = a[0];
        A[2, 2] = 0;
        A[3, 3] = 1;
        return A;
    }

    private Matrix4x4 Matrix_Subtract(Matrix4x4 a, Matrix4x4 b)
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                a[i, j] -= b[i, j];

        return a;
    }

    private Matrix4x4 Matrix_Mulitiply(Matrix4x4 a, float b)
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                a[i, j] *= b;
        return a;
    }

    private Quaternion Quaternion_Add(Quaternion a, Quaternion b)
    {
        a.x += b.x;
        a.y += b.y;
        a.z += b.z;
        a.w += b.w;
        return a;
    }

    // In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    void Collision_Impulse(Vector3 P, Vector3 N)
    {
        List<Vector3> CollisionPoints = new List<Vector3>();
        Matrix4x4 q_matrix = Matrix4x4.Rotate(q);

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 xi = transform.TransformPoint(vertices[i]);
            float d = Vector3.Dot(xi - P, N);
            if (d < 0.0f)
            {
                Vector3 Rri = q_matrix.MultiplyVector(vertices[i]);
                Vector3 vi = v + Vector3.Cross(w, Rri);
                float viDotN = Vector3.Dot(vi, N);
                if (viDotN < 0.0f)
                {
                    CollisionPoints.Add(vertices[i]);
                }
            }
        }

        if (CollisionPoints.Count == 0) return;

        Vector3 averageCollisionPoint = Vector3.zero;
        for (int i = 0; i < CollisionPoints.Count; i++)
        {
            averageCollisionPoint += CollisionPoints[i];
        }
        averageCollisionPoint /= CollisionPoints.Count;
        Vector3 R_length = q_matrix.MultiplyVector(averageCollisionPoint);
        Vector3 CollisionPointSpeed = v + Vector3.Cross(w, R_length);

        Vector3 CollisionPointSpeedN = N * Vector3.Dot(N, CollisionPointSpeed);
        Vector3 CollisionPointSpeedF = CollisionPointSpeed - CollisionPointSpeedN;
        Vector3 CollisionPointSpeedN_New = -restitution * CollisionPointSpeedN;
        float a = Math.Max(1.0f - friction * (1.0f + restitution) * CollisionPointSpeedN.magnitude / CollisionPointSpeedF.magnitude, 0.0f);
        Vector3 CollisionPointSpeedF_New = a * CollisionPointSpeedF;
        Vector3 CollisionPointSpeed_New = CollisionPointSpeedN_New + CollisionPointSpeedF_New;

        Matrix4x4 RriStar = Get_Cross_Matrix(R_length);
        Matrix4x4 IInverse = Matrix4x4.Inverse(q_matrix * I_ref * Matrix4x4.Transpose(q_matrix));
        Matrix4x4 K = Matrix_Subtract(Matrix_Mulitiply(Matrix4x4.identity, 1.0f / mass), RriStar * IInverse * RriStar);
        Vector3 J = K.inverse.MultiplyVector(CollisionPointSpeed_New - CollisionPointSpeed);

        v += 1.0f / mass * J;
        w += IInverse.MultiplyVector(Vector3.Cross(R_length, J));
    }

    // Update is called once per frame
    void Update()
    {
        //Game Control
        if (Input.GetKey("r"))
        {
            transform.position = new Vector3(0, 0.6f, 0);
            restitution = 0.5f;
            launched = false;
        }
        if (Input.GetKey("l"))
        {
            v = new Vector3(5, 2, 0);
            launched = true;
        }

        if (launched)
        {
            // Part I: Update velocities
            v += dt * gravity;
            v *= linear_decay;
            w *= angular_decay;
            if (Vector3.Magnitude(v) <= 0.05f) launched = false;

            // Part II: Collision Impulse
            Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
            Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

            // Part III: Update position & orientation
            Vector3 x0 = transform.position;
            Quaternion q0 = transform.rotation;
            x = x0 + dt * v;
            Vector3 dw = 0.5f * dt * w;
            Quaternion qw = new Quaternion(dw.x, dw.y, dw.z, 0.0f);
            q = Quaternion_Add(q0, qw * q0);

            // Part IV: Assign to the object
            transform.position = x;
            transform.rotation = q;
        }

    }
}
