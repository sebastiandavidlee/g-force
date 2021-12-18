/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "HX711.h"
#include <LiquidCrystal.h>

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 6;
const int LOADCELL_SCK_PIN = 7;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

HX711 scale;

ros::NodeHandle  nh;

// message call back when we get message from ROS
void messageCb( const std_msgs::String& environ_msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
  
  lcd.setCursor(0, 0);
  lcd.print(environ_msg.data);
}

std_msgs::Float32 load_msg;
ros::Publisher pub_load("loadcell", &load_msg);
ros::Subscriber<std_msgs::String> sub("environment", &messageCb );


void setup()
{
  //begin loadcell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  //calibrate loadcell
  
  scale.set_scale(392.74f);// calibration with 500g weight(Dec.02.2021 by Jungpyo)
  scale.tare();

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  //lcd.print("Load cell:");
  
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.advertise(pub_load);
  nh.subscribe(sub);
}

void loop()
{
  load_msg.data = -scale.get_units();
  pub_load.publish( &load_msg );
  nh.spinOnce();


  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  
  lcd.print(load_msg.data);

}
