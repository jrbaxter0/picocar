const byte THROTTLE_PIN = A0;
const byte STEERING_PIN = A1;

const unsigned int MAX_ANALOG = pow(2, 10) - 1;
const byte MAX_PWM = pow(2, 8) - 1;
const float MAX_VOLTS = 5.0;
const float QUANTIZATION_INTERVAL = MAX_VOLTS / pow(2,10);

unsigned int throttle_min = MAX_ANALOG;
unsigned int throttle_max = 0;

unsigned int steering_min = MAX_ANALOG;
unsigned int steering_max = 0;

char data[128];

void setup()
{
  Serial.begin(115200);
  pinMode(THROTTLE_PIN, INPUT); // analog
  pinMode(STEERING_PIN, INPUT); // analog

}

const float analog_to_volts(const unsigned int analog)
{
  return analog * QUANTIZATION_INTERVAL;
}

const byte volts_to_pwm(const float volts)
{
  return (byte)(volts / MAX_VOLTS * MAX_PWM);
}

void loop()
{
  // measure current values
  unsigned int throttle= analogRead(THROTTLE_PIN);
  unsigned int steering = analogRead(STEERING_PIN);

  // update max and mins
  if(throttle > throttle_max)
    throttle_max = throttle;
  if(throttle < throttle_min)
    throttle_min = throttle;

  if(steering > steering_max)
    steering_max = steering;
  if(steering < steering_min)
    steering_min = steering;

  // output results
  sprintf(data, "Throttle: [%.3d, %.3d] mV, [%d, %d] pwm\n"
                "Steering: [%.3d, %.3d] mV, [%d, %d] pwm",
                 (int)(1000*analog_to_volts(throttle_min)), (int)(1000*analog_to_volts(throttle_max)),
                 volts_to_pwm(analog_to_volts(throttle_min)), volts_to_pwm(analog_to_volts(throttle_max)),
                 (int)(1000*analog_to_volts(steering_min)), (int)(1000*analog_to_volts(steering_max)),
                 volts_to_pwm(analog_to_volts(steering_min)), volts_to_pwm(analog_to_volts(steering_max)));

  Serial.println(data);
}
