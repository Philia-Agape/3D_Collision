using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;                 // for collision

	Vector3[] vertex;
	Matrix4x4 j; 
	//Mesh mesh = GetComponent<MeshFilter>().mesh;
	//Vector3[] vertices = mesh.vertices;

	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;
		vertex = vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[0, 3] = 0;
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[1, 3] = 0;
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
			I_ref[2, 3] = 0;
		}
		I_ref[3, 0] = 0;
		I_ref[3, 1] = 0;
		I_ref[3, 2] = 0;
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N, Vector3 r_i, Vector3 v, Vector3 w)
	{
		Vector3 v_i = Get_Cross_Matrix(w) * r_i;
		v_i += v;

		if (Vector3.Dot(v_i, N) > 0) return;

		//Mesh mesh = GetComponent<MeshFilter>().mesh;
		//Vector3[] vertices = mesh.vertices;
		Matrix4x4 R = Get_Cross_Matrix(r_i);
		Matrix4x4 K = Matrix4x4.identity * (1/M) - R * I_ref.inverse * R;

		//Compute v_i_new
		Vector3 v_i_N = Vector3.Dot(Vector3.Dot(v_i, N), N);
		Vector3 v_i_T = v_i - v_i_N;
		int a = max(0, 1-(1+restitution)* restitution * (v_i_N.sqrMagnitude)/(v_i_T.sqrMagnitude));
		v_i_N = -restitution * v_i_N;
		v_i_T = a * v_i_T;
		Vector3 v_i_new = v_i_N + v_i_T;
		//Impulse
		j = K.inverse * (v_i_new - v_i);
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
			return;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}
		Vector3 r = transform.position;

		// Part I: Update velocities

		v[0] -= 9.8 * dt;
		v *= linear_decay;
		w *= angular_decay;
		//w ?


		// Part II: Collision Impulse
		for (int i=0; i<vertex.Length; ++i) {
			//(x[0],x[1],x[2]) * 
			if (r[0] + vertex[i][0] <= 2)
			{
				Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0), r + vertex[i], v, w);
				v += j / mass;
				w += I_ref.inverse * Get_Cross_Matrix(r + vertex[i]) * j;
			}
			if (r[1] + vertex[i][1] <= 0.01)
			{
				Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0), r + vertex[i], v, w);
				v += j / mass;
				w += I_ref.inverse * Get_Cross_Matrix(r + vertex[i]) * j;
			}
		}
		// Part III: Update position & orientation
		//Update linear status
		//Vector3 x = transform.position;
		r += v * dt;
		//Update angular status
		Quaternion q = transform.rotation;
		q += w;

		// Part IV: Assign to the object
		transform.position = r;
		transform.rotation = q;
	}
}
