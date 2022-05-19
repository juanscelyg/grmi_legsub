#include <mcp_can.h>
#include <SPI.h>
#include <ros.h> // ROS header 
#include <can_msgs/Frame.h> // CAN_frame

#define LED_D8 8

// ROS infraestructure
ros::NodeHandle  nh;
can_msgs::Frame frame_out_msg;
ros::Publisher pub_frame("/frame_out", &frame_out_msg);

// CAN infraestructure
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);


// Callbacks
void frame_pub(){
  long unsigned int rxId;
  unsigned char len;
  unsigned char rxBuf[8];
  if(CAN_MSGAVAIL == CAN.checkReceive()){
    CAN.readMsgBuf(&len, rxBuf); 
    unsigned long rxId = CAN.getCanId();
    //frame_out_msg.header.frame_id = "CAN";
    frame_out_msg.header.stamp = nh.now();
    frame_out_msg.id=rxId;
    frame_out_msg.dlc=len;
    memcpy ( &frame_out_msg.data, &rxBuf, sizeof(rxBuf) );
    pub_frame.publish(&frame_out_msg);
    }
  }

void frame_sub(const can_msgs::Frame& msg ) {
  unsigned char buf[8];
  memcpy ( &buf, &msg.data, sizeof(msg.data) );
  CAN.sendMsgBuf(msg.id, msg.is_extended, msg.dlc, buf);
  delay(1);
  frame_pub();
}

// Servers definition
ros::Subscriber<can_msgs::Frame> sub_frame("/frame_in", &frame_sub);

void setup() {
  pinMode(LED_D8, OUTPUT);
  digitalWrite(LED_D8, LOW);
  if (CAN_OK != CAN.begin(CAN_1000KBPS))
  {
    digitalWrite(LED_D8, HIGH);
  }
  nh.getHardware()->setBaud(115200);
  // node Init
  nh.initNode();
  // Servers init
  nh.subscribe(sub_frame);
  nh.advertise(pub_frame);

}

void loop() {
  frame_pub();
  nh.spinOnce();
  
}
