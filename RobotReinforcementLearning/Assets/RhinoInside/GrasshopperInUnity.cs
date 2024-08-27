using System.Collections.Generic;
using UnityEngine;
using Rhino;
using RhinoInside.Unity;
//using System.Diagnostics;

//[ExecuteInEditMode]
//class named GrasshopperInUnity, inherits from MonoBehaviour,  represents a script component that can be attached to a GameObject in Unity.
public class GrasshopperInUnity : MonoBehaviour
{

    public GameObject geoPrefab;
    private List<GameObject> _gameObjects = new List<GameObject>();
    [SerializeField] // Serialize point1 variable
    public GameObject point1; // Reference to the first point GameObject
    [SerializeField] // Serialize point1 variable
    public GameObject point2; // Reference to the second point GameObject

    void FromGrasshopper(object sender, Rhino.Runtime.NamedParametersEventArgs args)
    {
        Rhino.Geometry.GeometryBase[] values; //declares an array variable named values to store the geometry data received from Grasshopper.
        if (args.TryGetGeometry("mesh", out values)) // attempts to retrieve geometry data named "mesh" from the event arguments (args). If successful, it assigns the retrieved data to the values array.
        {
            var meshFilters = GetComponentsInChildren<MeshFilter>();
            //This line retrieves all MeshFilter components attached to the child GameObjects of the current GameObject. 
            //MeshFilter components are responsible for rendering meshes in Unity.
            //starts a loop that iterates over each MeshFilter component found in the previous step.
            foreach (var meshFilter in meshFilters)
            {
            if (meshFilter.sharedMesh != null)
            {
                DestroyImmediate(meshFilter.sharedMesh);
                    //checks if the current MeshFilter component has a shared mesh assigned to it. 
                    //If it does, it destroys the mesh immediately. 
                    //This is done to clear any existing mesh data before updating with new geometry.
                }
            }

            if (values.Length != _gameObjects.Count) //checks if the number of geometry objects received from Grasshopper (values.Length) matches the number of GameObject instances stored in the _gameObjects list.
            {
            foreach(var gb in _gameObjects)
            {
                DestroyImmediate(gb); //if they are not equal, it indicates that the number of geometry objects has changed, so the script needs to create or destroy GameObjects accordingly.
                }

            _gameObjects.Clear();

            for(int i=0; i<values.Length; i++) //This loop iterates over each geometry object received from Grasshopper (values) 
                    //and creates a new GameObject instance based on the geoPrefab. Each instance is then added to the _gameObjects list for future reference.
                {
                GameObject instance = (GameObject) Instantiate(geoPrefab);
                instance.transform.SetParent(transform);
                _gameObjects.Add(instance);
            }
            }

            for (int i = 0; i < values.Length; i++)
            {
                //converts the Grasshopper mesh data to a Unity mesh using the ToHost() method and assigns it to the mesh property of the MeshFilter component.
                _gameObjects[i].GetComponent<MeshFilter>().mesh = (values[i] as Rhino.Geometry.Mesh).ToHost();
            }
        }
    }
    //tahe 3 parameteres sent them to grasshopper.
    public void ToGrasshopper(float x1, float y1, float z1, float x2, float y2, float z2)
    {
        using (var args = new Rhino.Runtime.NamedParametersEventArgs())
        {
            // Set named parameters representing the first set of coordinates
            args.Set("X1", x1);
            args.Set("Y1", y1);
            args.Set("Z1", z1);

            // Set named parameters representing the second set of coordinates
            args.Set("X2", x2);
            args.Set("Y2", y2);
            args.Set("Z2", z2);

            // Send the named parameters back to Grasshopper
            Rhino.Runtime.HostUtils.ExecuteNamedCallback("ToGrasshopper", args);
        }
    }

    // Start is called before the first frame update
    void Start()
  {
        OpenGH();
        if (!Startup.isLoaded)
        {
            Startup.Init();
        }

        Rhino.Runtime.HostUtils.RegisterNamedCallback("Unity:FromGrasshopper", FromGrasshopper);
    }

  // Update is called once per frame
  void Update()
  {
    // Get the positions of the two points
    Vector3 position1 = point1.transform.position;
    Vector3 position2 = point2.transform.position;

        // Extract x, y, and z coordinates
        float x1 = position1.x;
        float y1 = position1.y;
        float z1 = position1.z;
        float x2 = position2.x;
        float y2 = position2.y;
        float z2 = position2.z;

        // Send the positions of the two points to Grasshopper
        ToGrasshopper(x1, y1, z1, x2, y2, z2);
    }

    public void OpenGH()
    {
        string script = "!_-Grasshopper _W _S ENTER";
        Rhino.RhinoApp.RunScript(script, false);

        // Open Rhino
        //Process.Start("rhino");
    }
    
    /*public void SendRndSeedToGH()
    {
        var val = Random.Range(0, 100000f);
        using (var args = new Rhino.Runtime.NamedParametersEventArgs())
        {
            args.Set("randomSeed", val);
            ToGrasshopper(args);

        }
    }*/

}

