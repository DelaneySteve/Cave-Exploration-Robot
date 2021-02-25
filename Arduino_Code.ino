#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle nh;
geometry_msgs::Twist msg;
ros::Publisher pub("husky/cmd_vel", &msg);


//VARIABLES THAT NEED TO BE SET
int currentPosition[] = {0.9, 0, 0};
int StoragePositon[] = {0.9, 0, 0};
int RestPositon[] = {0.9, 0, 0};
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
} 



void loop()
{
//Read messages from the xBee
received = Serial.read();

//Code to check signal strength and store in SignalStrength variable


//OPTION 1 - Move forward
if (received == "start"){
//Drive forward
msg.linear.x=0.25;
pub.publish(&msg);}


//OPTION 2 - Stop moving
else if (received == "stop"){
//Stop driving
msg.linear.x=0.0;
pub.publish(&msg);
}


//OPTION 3 - Retrieve Previous Node 
else if (received == "retrieve"){
  
//Drive backwards to collect a node
DistanceToTarget = 100;
while (DistanceToTarget>0){
msg.linear.x=-0.25;
pub.publish(&msg);
DistanceToTarget = DistanceCheck();//Taken from matlab camera
}

//At node so stop driving
msg.linear.x=0.0;
pub.publish(&msg);

//Retrieve node and increase the number of nodes
RetrieveNode();
NodesRemaining++;
}


//OPTION 4 - Place Node 
else if (received == "placeNode" || SignalStrength == 0){
//Stop moving
msg.linear.x=0.0;
pub.publish(&msg);
//Place Node
PlaceNode();
//Decrement the number of nodes
NodesRemaining--;
//Start moving again
msg.linear.x=0.25;
pub.publish(&msg);
//Reset signal strength
SignalStrength == 100;
}


//Publish whatever message to the ROS robot 
pub.publish(&msg);
nh.spinOnce();
}



//MATLAB - Check distance to node, return 0 if close enough to retrieve
int DistanceCheck(){
  //Check Distance to object via interaction with MATLAB
}


//MATLAB - Return x y or z coordinate of node location
double GetNodeCoordinates(int option){
  if (option == 1){
    //return x coordinate from MATLAB
  } else if (option == 2){
    //return y coordinate from MATLAB
  } else if (option == 3){
    //return z coordinate from MATLAB
  }
}


//Path planning algorithm will update the global array "path" with the path to follow for all three joints
void PathPlanning(int startx, int starty, int startz, int endx, int endy, int endz){
//Path Planning Algorithm
//Will take the 3 starting and 3 ending coordinates and edit the path array
  
}



//Function to retrieve a node
void RetrieveNode(){

  //Set end position as the location of the node
  xend = GetNodeCoordinates(1);
  yend = GetNodeCoordinates(2);
  zend = GetNodeCoordinates(3);
  //Calculate path to move to node
  PathPlanning(currentPosition[0], currentPosition[1], currentPosition[2], xend, yend, zend);
  
  //Code to control the scara to follow the path returned by PathPlanning
  //Code to pickup node with claw

  //Store the new current position
  currentPosition[0] = xend;
  currentPosition[1] = yend;
  currentPosition[2] = zend;
  //New end position is location of the storage
  xend =  StoragePositon[0];//(x position of storage dropp off)
  yend =  StoragePositon[1];//(y position of storage dropp off)
  zend =  StoragePositon[2];//(z position of storage dropp off)
  //Calculate path to storage
  PathPlanning(currentPosition[0], currentPosition[1], currentPosition[2], xend, yend, zend);
  
  //Code to control the scara to follow the path returned by PathPlanning
  //Code to let go of node with claw
  
  //Store the new current position
  currentPosition[0] = xend;
  currentPosition[1] = yend;
  currentPosition[2] = zend;
  //New end position is the rest position 
  xend = RestPositon[0];
  yend = RestPositon[1];
  zend = RestPositon[2];
  //Calculate path to rest
  PathPlanning(currentPosition[0], currentPosition[1], currentPosition[2], xend, yend, zend);
  
  //Code to control the scara to follow the path returned by PathPlanning

  //Store the new current position
  currentPosition[0] = xend;
  currentPosition[1] = yend;
  currentPosition[2] = zend;
}




//Function to place a node
void PlaceNode(){
  //Set end position as the location of the storage
  xend =  StoragePositon[0];//(x position of storage dropp off)
  yend =  StoragePositon[1];//(y position of storage dropp off)
  zend =  StoragePositon[2];//(z position of storage dropp off)

  //Code to make sure next node is ready for pickup - control storage equipment

  //Calculate path to storage
  PathPlanning(currentPosition[0], currentPosition[1], currentPosition[2], xend, yend, zend);
  
  //Code to control the scara to follow the path returned by PathPlanning
  //Code to pickup node with claw

  //Store the new current position
  currentPosition[0] = xend;
  currentPosition[1] = yend;
  currentPosition[2] = zend;
  
  //End position becomes placement location
  xend =  PlacementPosition[0];
  yend =  PlacementPosition[1];
  zend =  PlacementPosition[2];
 
  //Calculate path to place node
  PathPlanning(currentPosition[0], currentPosition[1], currentPosition[2], xend, yend, zend);
  //Code to control the scara to follow the path returned by PathPlanning
  //Code to let go of node with claw

  //Store the new current position
  currentPosition[0] = xend;
  currentPosition[1] = yend;
  currentPosition[2] = zend;
  //End position becomes rest location
  xend = RestPositon[0];
  yend = RestPositon[1];
  zend = RestPositon[2];

  //Calculate path to rest position
  PathPlanning(currentPosition[0], currentPosition[1], currentPosition[2], xend, yend, zend);
  //Code to control the scara to follow the path returned by PathPlanning

  //Store the new current position
  currentPosition[0] = xend;
  currentPosition[1] = yend;
  currentPosition[2] = zend;
  
}
