#include <ros.h>
#include <std_msgs/Float32.h>

// pins
const byte THROTTLE_PIN = 9;
const byte STEERING_PIN = 10;

// limits
const byte THROTTLE_MIN_PWM = 74;
const byte THROTTLE_MID_PWM = 95;
const byte THROTTLE_MAX_PWM = 109;

const byte STEERING_MIN_PWM = 55;
const byte STEERING_MID_PWM = 95;
const byte STEERING_MAX_PWM = 131;

// invert
const float THROTTLE_SCALE = -1.0;
const float STEERING_SCALE = -1.0;


ros::NodeHandle nh;

const byte effort_to_pwm(const float effort, const byte MIN_PWN, const byte MID_PWM, const byte MAX_PWM)
{
  if(effort >= 0)
  {
    return (byte)(MID_PWM + effort * (MAX_PWM - MID_PWM));
  }
  else
  {
    return (byte)(MID_PWM + effort * (MID_PWM - MIN_PWN));
  }
}

void throttle_callback(const std_msgs::Float32& throttle)
{
  analogWrite(THROTTLE_PIN, effort_to_pwm(throttle.data * THROTTLE_SCALE, THROTTLE_MIN_PWM, THROTTLE_MID_PWM, THROTTLE_MAX_PWM));
}

void steering_callback(const std_msgs::Float32& steering)
{
  analogWrite(STEERING_PIN, effort_to_pwm(steering.data * STEERING_SCALE, STEERING_MIN_PWM, STEERING_MID_PWM, STEERING_MAX_PWM));
}

ros::Subscriber<std_msgs::Float32> throttle_sub("control_effort/throttle", throttle_callback);
ros::Subscriber<std_msgs::Float32> steering_sub("control_effort/steering", steering_callback);

void setup()
{
  Serial.begin(57600);
  pinMode(THROTTLE_PIN, OUTPUT); // analog
  pinMode(STEERING_PIN, OUTPUT); // analog
  analogWrite(THROTTLE_PIN, THROTTLE_MID_PWM);
  analogWrite(STEERING_PIN, STEERING_MID_PWM);

  // ros
  nh.initNode();
  nh.subscribe(throttle_sub);
  nh.subscribe(steering_sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
