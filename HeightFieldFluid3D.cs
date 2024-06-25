
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class HeightFieldFluid3D : MonoBehaviour
{

    [Header("Simulation Parameters")]
    public int numberOfColumnsX;
    public int numberOfColumnsZ;
    public float initialHeight;
    public float spacing;
    public float wavespeed;
    public float alpha;
    public int iterations;
    float c;
    public float positionalDamping;
    public float velocityDamping;
    float pd;
    float vd;


    public float[,] h;
    float[,] v;
    public Vector2[,] hv;
    float[,] bodyHeigts;
    float[,] prevBodyHeigts;

    Vector3 aabbCenter;
    Vector3 aabbSize;


    [Header("Material Setting")]
    public Material material;
    Mesh mesh;
    MeshRenderer meshRenderer;


    //Initialization and simulation loop-----------------------------------------------------------
    void Start()
    {
        Debug.Log("optimal Wavespeed: " + 0.5f * spacing / 0.02f);

        initArrays();
        initAABB();
        initWaterSurface();
        initPlane();
    }
    void FixedUpdate()
    {    
        simulateCoupling();
        simulateSurface();
        simulateHorizontalVelocities();
        updatePlane();
    }
  

    //Init functions-------------------------------------------------------------------------------
    void initArrays()
    {
        v = new float[numberOfColumnsX, numberOfColumnsZ];
        hv = new Vector2[numberOfColumnsX, numberOfColumnsZ];
        h = new float[numberOfColumnsX, numberOfColumnsZ];
        bodyHeigts = new float[numberOfColumnsX, numberOfColumnsZ];
        prevBodyHeigts = new float[numberOfColumnsX, numberOfColumnsZ];
    }
    void initAABB()
    {
        float sizeX = (numberOfColumnsX) * spacing;
        float sizeZ = (numberOfColumnsZ) * spacing;
        aabbCenter = new Vector3(sizeX / 2, initialHeight, sizeZ / 2);
        aabbSize = new Vector3(sizeX, initialHeight * 2, sizeZ);
    }
    void initWaterSurface()
    {
        for (int i = 0; i < numberOfColumnsX; i++)
        {
            for (int j = 0; j < numberOfColumnsZ; j++)
            {
                h[i, j] = initialHeight;
            }
        }
    }


    //Simulates the surface of the water-----------------------------------------------------------
    void simulateSurface()
    {
        if (Time.fixedDeltaTime * wavespeed > spacing) Debug.LogWarning("Unstable");

        c = (wavespeed * wavespeed) / (spacing * spacing);
        pd = Mathf.Min(positionalDamping * Time.fixedDeltaTime, 1);
        vd = Mathf.Max(0, 1 - velocityDamping * Time.fixedDeltaTime);

        for (int i = 0; i < numberOfColumnsX; i++)
        {
            for (int j = 0; j < numberOfColumnsZ; j++)
            {
                float sumH = 0;
                float hij = h[i, j];
                
                sumH += i > 0 ? h[i - 1, j] : hij;
                sumH += i < numberOfColumnsX - 1 ? h[i + 1, j] : hij;
                sumH += j > 0 ? h[i, j - 1] : hij;
                sumH += j < numberOfColumnsZ - 1 ? h[i, j + 1] : hij;

                float a = (sumH - 4 * hij);
                v[i,j] += Time.fixedDeltaTime * c * a;
                h[i,j] += (0.25f * sumH - hij) * pd;
            }
        }

        for (int i = 0; i < numberOfColumnsX; i++)
        {
            for (int j = 0; j < numberOfColumnsZ; j++)
            {
                v[i,j] *= vd;
                h[i,j] += v[i,j] * Time.fixedDeltaTime;
            }
        }

    }
    void simulateCoupling()
    {
        copy2DArray(bodyHeigts, prevBodyHeigts);
        bodyHeigts = new float[numberOfColumnsX, numberOfColumnsZ];

        Collider[] colliders = Physics.OverlapBox(aabbCenter, aabbSize / 2);

        foreach (Collider collider in colliders)
        {
            Rigidbody rb = collider.attachedRigidbody;
            if(rb != null)
            {
                
                Bounds bounds = CalculateAABB(collider.gameObject);
                int minX = Mathf.FloorToInt(bounds.min.x / spacing);
                int maxX = Mathf.CeilToInt(bounds.max.x / spacing);
                int minZ = Mathf.FloorToInt(bounds.min.z / spacing);
                int maxZ = Mathf.CeilToInt(bounds.max.z / spacing);
                           
                minX = minX > 0 ? minX : 0;
                maxX = maxX < numberOfColumnsX ? maxX : numberOfColumnsX;
                minZ = minZ > 0 ? minZ : 0;
                maxZ = maxZ < numberOfColumnsZ ? maxZ : numberOfColumnsZ;

                for (int i = minX; i < maxX; i++)
                {
                    for (int j = minZ; j < maxZ; j++)
                    {
                        float bodyMin = 0f;
                        float bodyMax = 0f;

                        RaycastHit hit;
                        Ray ray;
                        Vector3 pos = new Vector3(i * spacing, 0, j * spacing);

                        ray = new Ray(pos, Vector3.up);
                        if (collider.Raycast(ray, out hit, 100.0f)) bodyMin = Mathf.Clamp(hit.point.y, 0, h[i, j]);
                        
                        ray = new Ray(pos + new Vector3(0, 50, 0), Vector3.down);
                        if (collider.Raycast(ray, out hit, 100.0f)) bodyMax = Mathf.Clamp(hit.point.y, 0, h[i, j]);
                                   
                        if(bodyMax!=0 && bodyMin!=0 && bodyMax > bodyMin)
                        {
                            float bodyHeight = bodyMax - bodyMin;
                            bodyHeigts[i, j] = bodyHeight;
                        }
                        
                    }
                }


            }
        }




        for(int iter = 0; iter < iterations; iter++)
        {
            for (int i = 0; i < numberOfColumnsX; i++)
            {
                for (int j = 0; j < numberOfColumnsZ; j++)
                {
                    float avg = 0f;
                    int num = 0;

                    if(i > 0) avg += bodyHeigts[i - 1, j]; num++;
                    if(i < numberOfColumnsX - 1) avg += bodyHeigts[i + 1, j]; num++;
                    if (j > 0) avg += bodyHeigts[i, j - 1]; num++;
                    if (j < numberOfColumnsZ - 1) avg += bodyHeigts[i, j + 1]; num++;

                    avg /= num;
                    bodyHeigts[i, j] = avg;

                }
            }

        }



        for (int i = 0; i < numberOfColumnsX; i++)
        {
            for (int j = 0; j < numberOfColumnsZ; j++)
            {
                float bodyChange = bodyHeigts[i, j] - prevBodyHeigts[i, j];
                h[i, j] += bodyChange * alpha;
            }
        }

    }
    void simulateHorizontalVelocities()
    {
        for (int i = 0; i < numberOfColumnsX; i++)
        {
            for (int j = 0; j < numberOfColumnsZ; j++)
            {
                float vi = v[i, j];

                float vx0 = i > 0 ? v[i - 1, j] : vi;
                float vx1 = i < numberOfColumnsX - 1 ? v[i + 1, j] : vi;
                float vz0 = j > 0 ? v[i, j - 1] : vi;
                float vz1 = j < numberOfColumnsZ - 1 ? v[i, j + 1] : vi;

                float vx = (vi - vx0) * spacing + (vx1 - vi) * spacing;
                float vz = (vi - vz0) * spacing + (vz1 - vi) * spacing;

                hv[i, j] = new Vector2(vx, vz);
               
            }
        }
    }

    
    //Displays the water surface as a plane--------------------------------------------------------
    void initPlane()
    {
        MeshFilter meshFilter = gameObject.AddComponent<MeshFilter>();
        meshRenderer = gameObject.AddComponent<MeshRenderer>();
        meshRenderer.material = material;
        mesh = new Mesh();
        meshFilter.mesh = mesh;


        Vector3[] vertices = new Vector3[numberOfColumnsX * numberOfColumnsZ];
        for (int i = 0; i < numberOfColumnsX; i++)
        {
            for (int j = 0; j < numberOfColumnsZ; j++)
            {
                vertices[i * numberOfColumnsZ + j] = new Vector3(i * spacing, h[i, j], j * spacing);
            }
        }

        int[] triangles = new int[(numberOfColumnsX - 1) * (numberOfColumnsZ - 1) * 6];
        int t = 0;
        for (int i = 0; i < numberOfColumnsX - 1; i++)
        {
            for (int j = 0; j < numberOfColumnsZ - 1; j++)
            {
                int topLeft = i * numberOfColumnsZ + j;
                int bottomLeft = (i + 1) * numberOfColumnsZ + j;
                int topRight = i * numberOfColumnsZ + j + 1;
                int bottomRight = (i + 1) * numberOfColumnsZ + j + 1;

                triangles[t++] = topLeft;
                triangles[t++] = topRight;
                triangles[t++] = bottomLeft;

                triangles[t++] = topRight;
                triangles[t++] = bottomRight;
                triangles[t++] = bottomLeft;
            }
        }

        mesh.Clear();
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();

    }
    void updatePlane()
    {
        Vector3[] vertices = new Vector3[numberOfColumnsX * numberOfColumnsZ];
        for (int i = 0; i < numberOfColumnsX; i++)
        {
            for (int j = 0; j < numberOfColumnsZ; j++)
            {
                vertices[i * numberOfColumnsZ + j] = new Vector3(i * spacing, h[i, j], j * spacing);
            }
        }

        mesh.vertices = vertices;
        mesh.RecalculateNormals();
    }


    //Helper functions-----------------------------------------------------------------------------
    void copy2DArray(float[,] source, float[,] destination)
    {
        int rows = source.GetLength(0);
        int columns = source.GetLength(1);

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < columns; j++)
            {
                destination[i, j] = source[i, j];
            }
        }
    }
    Bounds CalculateAABB(GameObject obj)
    {
        Renderer renderer = obj.GetComponent<Renderer>();
        Bounds bounds = renderer.bounds;
        return bounds;
    }
    

    //Debug Gizmos---------------------------------------------------------------------------------
    private void OnDrawGizmos()
    {
        if (h != null)
        {
            for (int i = 0; i < numberOfColumnsX; i++)
            {
                for(int j = 0 ; j < numberOfColumnsZ; j++)
                {
                    Vector3 start = new Vector3(i * spacing, h[i, j], j * spacing);
                    Vector3 end = new Vector3(i * spacing + hv[i, j].x, h[i, j], j * spacing + hv[i, j].y);
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(start, end);
                }   
            }
        }


        //Display the AABB of the object to visualize boundaries
        float sizeX = (numberOfColumnsX) * spacing;
        float sizeZ = (numberOfColumnsZ) * spacing;
        Vector3 aabbCenter = new Vector3(sizeX / 2, initialHeight, sizeZ / 2);
        Vector3 aabbSize = new Vector3(sizeX, initialHeight * 2, sizeZ);

        Gizmos.color = Color.blue;
        Gizmos.DrawWireCube(aabbCenter, aabbSize);


        /* 
        //Display the AABB of the object
         
        Gizmos.color = Color.yellow;

        Bounds bounds = CalculateAABB(obj);
        Gizmos.DrawWireCube(bounds.center, bounds.size);
        Vector3[] corners = new Vector3[8];
        corners[0] = new Vector3(bounds.min.x, bounds.min.y, bounds.min.z);
        corners[1] = new Vector3(bounds.max.x, bounds.min.y, bounds.min.z);
        corners[2] = new Vector3(bounds.min.x, bounds.max.y, bounds.min.z);
        corners[3] = new Vector3(bounds.max.x, bounds.max.y, bounds.min.z);
        corners[4] = new Vector3(bounds.min.x, bounds.min.y, bounds.max.z);
        corners[5] = new Vector3(bounds.max.x, bounds.min.y, bounds.max.z);
        corners[6] = new Vector3(bounds.min.x, bounds.max.y, bounds.max.z);
        corners[7] = new Vector3(bounds.max.x, bounds.max.y, bounds.max.z);

        Gizmos.color = Color.green;
        foreach (var corner in corners)
        {
            Gizmos.DrawSphere(corner, 0.05f);
        }
        */

    }

}
