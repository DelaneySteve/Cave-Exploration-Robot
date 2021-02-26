#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle nh;
geometry_msgs::Twist msg;
ros::Publisher pub("husky/cmd_vel", &msg);

//////////////////////////////////
#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>


//initialize limit switches
#define limitSwitch1 11
#define limitSwitch2 10
#define limitSwitch3 9

//Initialize Motors//
// Define the stepper motors and the pins the will use
AccelStepper stepper1(1, 2, 5); // (Type:driver, STEP, DIR)
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);

Servo gripperServo;  // create servo object to control a servo
Servo storageServo;

int stepper1Position, stepper2Position, stepper3Position, stepper4Position;

//these depend on the motor
const float theta1AngleToSteps = 44.444444;
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10;

//storage breadcrumb counter
int storedBreadcrumbs = 3;
int emptyFlag = 0;
//////////////////////////////////


//VARIABLES THAT NEED TO BE SET
int currentPosition[] = {0.9, 0, 0};
int StoragePositon[] = {0.9, 0, 0};
int PlacementPosition[] = {0.9, 0, 0};
int L0 = 0.65;
int L1 = 0.2;
int L2 = 0.7;
int NodesRemaining = 3;



//OTHER VARIABLES
String received = "";
int SignalStrength = 100;//***Needs to be replaced with actual signal strength data
int DistanceToTarget = 100;//***needs to be replaced with value from MATLAB
int xstart, ystart, zstart, xend, yend, zend;




void setup()
{
  //ROS Initialization
  nh.initNode();
  nh.advertise(pub);
  //To take messages in from xBee
  Serial.begin(9600);

  //////////////////////////////////
  ///Motor Setup///
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);

  // Stepper motors max speed
  stepper1.setMaxSpeed(4688);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4688);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4688);
  stepper3.setAcceleration(2000);

  //Servos Positioning
  gripperServo.attach(A0, 600, 2500);
  // initial servo value - open gripper
  gripperServo.write(45);
  //open = 45, closed = 100
  delay(1000);
  storageServo.attach(9);
  storageServo.write(90);  // set servo to no movement



  homing();


  //////////////////////////////////

}



void loop()
{
  //Read messages from the xBee
  received = Serial.read();


  //Code to check signal strength and store in SignalStrength variable


  //check if the storage is empty:
  if (emptyFlag == 1) {
    Serial.write("Storage is Empty")
  }

  //OPTION 1 - Move forward
  if (received == "start") {
    //Drive forward
    msg.linear.x = 0.25;
    pub.publish(&msg);
  }


  //OPTION 2 - Stop moving
  else if (received == "stop") {
    //Stop driving
    msg.linear.x = 0.0;
    pub.publish(&msg);
  }


  //OPTION 3 - Retrieve Previous Node
  else if (received == "retrieve") {

    //Drive backwards to collect a node
    DistanceToTarget = 100;
    while (DistanceToTarget > 0) {
      msg.linear.x = -0.25;
      pub.publish(&msg);
      DistanceToTarget = DistanceCheck();//Taken from matlab camera
    }

    //At node so stop driving
    msg.linear.x = 0.0;
    pub.publish(&msg);

    //Retrieve node and increase the number of nodes
    RetrieveNode();
    NodesRemaining++;
  }


  //OPTION 4 - Place Node
  else if (received == "placeNode" || SignalStrength == 0) {
    //Stop moving
    msg.linear.x = 0.0;
    pub.publish(&msg);
    //Place Node
    PlaceNode();
    //Decrement the number of nodes
    NodesRemaining--;
    //Start moving again
    msg.linear.x = 0.25;
    pub.publish(&msg);
    //Reset signal strength
    SignalStrength == 100;
  }


  //Publish whatever message to the ROS robot
  pub.publish(&msg);
  nh.spinOnce();
}



//MATLAB - Check distance to node, return 0 if close enough to retrieve
int DistanceCheck() {
  //Check Distance to object via interaction with MATLAB
}


//MATLAB - Return x y or z coordinate of node location
double GetNodeCoordinates(int option) {
  if (option == 1) {
    //return x coordinate from MATLAB
  } else if (option == 2) {
    //return y coordinate from MATLAB
  } else if (option == 3) {
    //return z coordinate from MATLAB
  }
}


//Path planning algorithm will update the global array "path" with the path to follow for all three joints
void PathPlanning(int startx, int starty, int startz, int endx, int endy, int endz) {
  //Path Planning Algorithm
  //Will take the 3 starting and 3 ending coordinates and edit the path array

}



//Function to retrieve a node
void RetrieveNode() {

  //Set end position as the location of the node
  xend = GetNodeCoordinates(1);
  yend = GetNodeCoordinates(2);
  zend = GetNodeCoordinates(3);

  Open();
  //Calculate path to move to node
  pick = PathPlanning(currentPosition[0], currentPosition[1], currentPosition[2], xend, yend, zend);

  //Code to control the scara to follow the path returned by PathPlanning
  ControlScara(pick);
  //Code to pickup node with claw
  Close;

  //Store the new current position
  currentPosition[0] = xend;
  currentPosition[1] = yend;
  currentPosition[2] = zend;
  //New end position is location of the storage
  xend =  StoragePositon[0];//(x position of storage dropp off)
  yend =  StoragePositon[1];//(y position of storage dropp off)
  zend =  StoragePositon[2];//(z position of storage dropp off)
  //Calculate path to storage
  store = PathPlanning(currentPosition[0], currentPosition[1], currentPosition[2], xend, yend, zend);

  //Code to control the scara to follow the path returned by PathPlanning
  ControlScara(store);
  //Code to let go of node with claw
  Open();

  //Return SCARA to rest position
  homing();

  //Store the new current position
  currentPosition[0] = xend;
  currentPosition[1] = yend;
  currentPosition[2] = zend;

  storedBreadcrumb++;
  //move node back into the storage container
  if (storedBreadcrumb < 3) {
    storageBackward();
  }

}


//Function to place a node
void PlaceNode() {
  //Set end position as the location of the storage
  xend =  StoragePositon[0];//(x position of storage dropp off)
  yend =  StoragePositon[1];//(y position of storage dropp off)
  zend =  StoragePositon[2];//(z position of storage dropp off)

  if (storedBreadcrumb < 3) {
    //Code to make sure next node is ready for pickup - control storage equipment
    storageForward();
  }

  Open();
  //Calculate path to storage
  storage = PathPlanning(currentPosition[0], currentPosition[1], currentPosition[2], xend, yend, zend);

  //Code to control the scara to follow the path returned by PathPlanning
  ControlScara(storage);
  //Code to pickup node with claw
  Close();

  //Store the new current position
  currentPosition[0] = xend;
  currentPosition[1] = yend;
  currentPosition[2] = zend;

  //End position becomes placement location
  xend =  PlacementPosition[0];
  yend =  PlacementPosition[1];
  zend =  PlacementPosition[2];

  //Calculate path to place node
  place = PathPlanning(currentPosition[0], currentPosition[1], currentPosition[2], xend, yend, zend);

  //Code to control the scara to follow the path returned by PathPlanning
  ControlScara(place);
  //Code to let go of node with claw
  Open();


  //Return SCARA to rest position
  homing();

  //Store the new current position
  currentPosition[0] = xend;
  currentPosition[1] = yend;
  currentPosition[2] = zend;

  //Lower breadcrumb count in storage
  storedBreadcrumbs--;
}

//////////////////////////////////

void ControlScara(q) {
  s = size(q[0]);
  for (int i = 0; i < s; i++) {
    //convert the joint angles to steps
    stepper1Position = q[0][i] * theta1AngleToSteps;
    stepper2Position = data[1][i] * theta2AngleToSteps;
    stepper3Position = data[2][i] * phiAngleToSteps;
    //move the stepper motors
    stepper1.moveTo(stepper1Position);
    stepper2.moveTo(stepper2Position);
    stepper3.moveTo(stepper3Position);
    delay(15);

  }
}

//opening the gripper
void Open {
  gripperServo.write(45); 
  delay(15);
}

//closing the gripper
void Close {
  gripperServo.write(45);
  delay(15);
}

//Moving the Breadcrumb Forward in Storage
void storageForward() {
  if (storedBreadcrumbs != 0) {
    storageServo.write(0); //coil turns left
    delay(1000); //turn for one full rotation
    storageServo.write(90); // stop turning
  }
  //check if breadcrumb storage is empty 
  else if (storedBreadcrumbs == 0) {
    emptyFlag = 1; //send flag 
  }
}

//Moving the Breadcrumb Backward in Storage
void storageBackward() {
  storageServo.write(180); //coil turns right
  delay(1000); //turn for one full rotation
  storageServo.write(90); // stop turning
}

//Returning SCARA to Rest Position on Robot
void homing() {
  // Home Stepper 3
  while (digitalRead(limitSwitch3) != 1) {
    stepper3.setSpeed(-1000);
    stepper3.runSpeed();
    stepper3.setCurrentPosition(-1662); // When limit switch pressed set position to 0 steps
  }
  delay(30);

  stepper3.moveTo(0);
  while (stepper3.currentPosition() != 0) {
    stepper3.run();
  }

  // Home Stepper 2
  while (digitalRead(limitSwitch2) != 1) {
    stepper2.setSpeed(-1200);
    stepper2.runSpeed();
    stepper2.setCurrentPosition(-5420); // When limit switch pressed set position to -5440 steps
  }
  delay(30);

  stepper2.moveTo(0);
  while (stepper2.currentPosition() != 0) {
    stepper2.run();
  }

  // Home Stepper 1
  while (digitalRead(limitSwitch1) != 1) {
    stepper1.setSpeed(-1100);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(-3955); // When limit switch pressed set position to 0 steps
  }
  delay(30);
  stepper1.moveTo(0);
  while (stepper1.currentPosition() != 0) {
    stepper1.run();
  }
}

//////////////////////////////////
