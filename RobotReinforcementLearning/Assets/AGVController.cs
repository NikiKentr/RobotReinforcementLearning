using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; //Imports geometry message types from ROS, such as Twist messages used for velocity commands.
using Unity.Robotics.UrdfImporter.Control;

namespace RosSharp.Control
{
    public enum ControlMode { Keyboard, ROS }; //Defines an enumeration called ControlMode with two possible values: Keyboard and ROS.

    public class AGVController : MonoBehaviour //inherits from monobehaviour to make a unity component
    {
        public GameObject wheel1;
        public GameObject wheel2;
        public ControlMode mode = ControlMode.ROS;//public field to set the control mode, in default it is ros

        private ArticulationBody wA1; //private field to hold the articulation body components of the wheels
        private ArticulationBody wA2;

        public float maxLinearSpeed = 2; //  m/s 
        public float maxRotationalSpeed = 1;//
        public float wheelRadius = 0.033f; //meters
        public float trackWidth = 0.288f; // meters Distance between tyres
        public float forceLimit = 10;
        public float damping = 10;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f; //to track the time of the last received command
         
        ROSConnection ros; //Private field to hold the ROS connection instance. ROSConnection is the type of variable ros. So the ROSConnection is a class by the UnityTCP package and handles the connection between unity and ros
        private RotationDirection direction;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        void Start()
        {
            wA1 = wheel1.GetComponent<ArticulationBody>();
            wA2 = wheel2.GetComponent<ArticulationBody>();
            SetParameters(wA1);
            SetParameters(wA2);
            ros = ROSConnection.GetOrCreateInstance(); //in the start method, initialize the ros variable. GetOrCreateInstance() is a static method of the ROSConnection class, retrieves an existing instance or creates a new one
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd); //subscribes to the ros topic named "cmd_vel", it is used to sent velocity commands to the robot
            //the ReceiveRosCmd method is specified here as a callback function that will be invoked whenever a message of type 'TwistMsg' is received on this topic
        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinear = (float)cmdVel.linear.x; //linear velocity of the robot along the x-axis, forward +x and backward movement -x (robot moving on a plane)
            // retrieves the linear velocity in the x-direction from the TwistMsg and stores it in rosLinear.
            rosAngular = (float)cmdVel.angular.z; //angularvelocity around z-axis, yaw rotation (turning left or right) +z counterclockwise (right), -z clockwise ratation (left)
            // retrieves the angular velocity around the z-axis from the TwistMsg and stores it in rosAngular.
            lastCmdReceived = Time.time;
        }

        void FixedUpdate()
        {
            if (mode == ControlMode.Keyboard) //If the control mode is set to Keyboard, call the KeyBoardUpdate method.
            {
                KeyBoardUpdate();
            }
            else if (mode == ControlMode.ROS) //If the control mode is set to ROS, call the ROSUpdate method.
            {
                ROSUpdate();
            }
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive; //Get the current drive settings of the articulation body. xDrive is a property of the ArticulationBody component. controls the motion along the x-axis of the join
            drive.forceLimit = forceLimit; //Set the force limit and damping.
            drive.damping = damping;
            joint.xDrive = drive; //Apply the updated drive settings to the articulation body.
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive; //Get the current drive settings of the articulation body.
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;//Applies the direction of the motion.
                // 2* because linear velocity contribution of each wheel needs to be accounted for in pairs.
                //dividing linearspeed by the wheelRadius converts the linear speed (m/s) to angular speed (radians per second)
                //Mathf.Rad2Deg is a constant in Unity that converts radians to degrees.
                //ArticulationDrive.targetVelocity is expected to be in degrees per second, we convert the angular velocity from radians per second to degrees per second.
            }
            else
            {
                drive.targetVelocity = wheelSpeed; //If a specific wheelSpeed is provided, it is set here
            }
            joint.xDrive = drive; //Apply the updated drive settings to the articulation body
        }

        private void KeyBoardUpdate() //handles keyboard input
        {
            float moveDirection = Input.GetAxis("Vertical"); //get the vertical axis input (forward/backward)
            float inputSpeed;
            float inputRotationSpeed;
            //based on the input direction, set the input speed
            if (moveDirection > 0)
            {
                inputSpeed = maxLinearSpeed;
            }
            else if (moveDirection < 0)
            {
                inputSpeed = maxLinearSpeed * -1;
            }
            else
            {
                inputSpeed = 0;
            }

            float turnDirction = Input.GetAxis("Horizontal"); //horizontal axis input (left/right)
            if (turnDirction > 0)
            {
                inputRotationSpeed = maxRotationalSpeed;
            }
            else if (turnDirction < 0)
            {
                inputRotationSpeed = maxRotationalSpeed * -1;
            }
            else
            {
                inputRotationSpeed = 0;
            }
            RobotInput(inputSpeed, inputRotationSpeed); // Call RobotInput to process the speed and rotation.
        }


        private void ROSUpdate()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f; // Set linear velocity to 0 if no command received within the timeout period.
                rosAngular = 0f; // Set angular velocity to 0 if no command received within the timeout period.
            }
            RobotInput(rosLinear, -rosAngular);  // Call RobotInput to process the linear and angular speeds from ROS
        }

        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (speed > maxLinearSpeed) 
            {
                speed = maxLinearSpeed; // Cap the linear speed to maximum linear speed.
            }
            if (rotSpeed > maxRotationalSpeed) // Cap the rotational speed to maximum rotational speed.
            {
                rotSpeed = maxRotationalSpeed;
            }
            float wheel1Rotation = (speed / wheelRadius); // Calculate wheel rotation speed for wheel 1.
            float wheel2Rotation = wheel1Rotation; // Initially set wheel 2 rotation speed same as wheel 1.
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius); // Calculate the difference in wheel speed due to rotation.
            if (rotSpeed != 0)
            {
                wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg; // Convert wheel 1 rotation speed to degrees.
                wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg; // Convert wheel 2 rotation speed to degrees.
            } 
            else
            {
                wheel1Rotation *= Mathf.Rad2Deg; 
                wheel2Rotation *= Mathf.Rad2Deg; 
            }
            SetSpeed(wA1, wheel1Rotation); // Set the speed for wheel 1.
            SetSpeed(wA2, wheel2Rotation); // Set the speed for wheel 2
        }
    }
}