#include <LIDARLite.h>

// Exponential decay constant.  Higher values lead to faster motor dropoff with range, 
// allowing better vibration strength 'resolution' at lower ranges.  Lower values
// spread this 'resolution' over a larger distance range, increasing max but weakening
// close-range distinguishability.
// Recommended values:  ~0.04 for outdoor use, ~0.1 for indoor use
float decay_const = 0.05;

// Pins used for the 'use LIDAR' button
int p_button_read = 5;
int p_button_write = 4;

// Pins & settings used for LIDAR control
// Could leave LIDAR in always-on, but this would eat battery
int p_lidar_trigger = 3;
int p_lidar_read = 2;
int lidar_min_range = 50;   // cm, this distance & closer  = max strength
int lidar_max_range = 1500; // cm, this distance & further = min strength

// Pin & settings for motor control
// Motor vibration strength will be scaled between max-min as
// LIDAR distance readings vary from 0~100ft
int p_motor_pwm = 11;    // must be a PWM-capable digital out pin
int motor_pwm_max = 240; // max 255, recommended 200+
int motor_pwm_min = 40;  // min 0, recommended 20-30

void setup() {
  // put your setup code here, to run once:
  pinMode(p_button_write, OUTPUT);
  pinMode(p_lidar_trigger, OUTPUT);
  pinMode(p_motor_pwm, OUTPUT);

  digitalWrite(p_button_write, LOW);

  pinMode(p_button_read, INPUT_PULLUP);
  pinMode(p_lidar_read, INPUT);
  Serial.begin(9600);
}

void loop() {
  if ( digitalRead(p_button_read) == HIGH ) {
    digitalWrite(p_lidar_trigger, LOW); // Signal LIDAR to give reading (continuous mode)
    trigger_pressed();
  }
  else {
    Serial.println("Power Off");
    digitalWrite(p_lidar_trigger, HIGH); // Signal LIDAR to stop taking readings (saves power)
    motor_off();
  }
  delay(100);
}


// Call when LIDAR returns too-short pulses, often when lenses are blocked
// If no pwm commands are sent here, the system will continue on previous strength
void lidar_err(int pulse) {
  Serial.print("Err ");
  Serial.println(pulse);
}


// Motor control output.  takes in a dist_reading in cm
// Convert a distance reading into a vibration strength where 
// closer = higher-strength.  Then send the proper motor command
void fire_motor(int dist_reading) {

  // Sanity checks
  if ( dist_reading < lidar_min_range )
  {
    Serial.println("Within minimum range");
    analogWrite(p_motor_pwm, motor_pwm_min);
    return;
  }
  else if ( dist_reading > lidar_max_range ) {
    Serial.println("Beyond maximum range");
    analogWrite(p_motor_pwm, motor_pwm_max);
    return;
  }
  else {
    // Convert reading to a % of distance between min-max
    // where 0% = ~min range
    float dist_perc = 1.0 * (dist_reading - lidar_min_range) / (lidar_max_range - lidar_min_range);
    
    // Motor strength (linear falloff)
    //int pwm_duty = 255 - int( dist_perc * (motor_pwm_max - motor_pwm_min) + motor_pwm_min);

    // motor strength (exponential decay)
    //int pwm_duty = int( (motor_pwm_max-motor_pwm_min) * pow(3, -100.0 * decay_const *dist_perc)) + motor_pwm_min;
    
    
    // Inverted exponential model:  Focus usable resolution on distant objects
    // Curve is intended to be tan(perc*1.3) to put ~50% of usable strengths in the last 25%
    // all else is scaling factors to obey pwm_min/max settings and pwm output values
    float perc_pwm_str = (tan(dist_perc*1.3)*70)/255.0;
    int pwm_duty = int(((motor_pwm_max-motor_pwm_min) * perc_pwm_str) + motor_pwm_min);

    Serial.println(dist_perc);
    Serial.println(dist_reading);
    Serial.println(pwm_duty);
    
    // Send motor command
    analogWrite(p_motor_pwm, pwm_duty);
  }
}

// Stop the motor when power button turned off
void motor_off() {
  analogWrite(p_motor_pwm, 0);
}

// While 'take readings' button pressed, eat LIDAR data & trigger motor commands
void trigger_pressed() {
  int pulse_width = pulseIn(p_lidar_read, HIGH);
  if (pulse_width >= 20) // Ignore erroneous signals
  {
    int cm_dist = pulse_width / 10; // 10usec pulse = 1cm reading
    fire_motor(cm_dist);
  }
  else {
    // Often triggered when there's no beam return (ie blocked lenses)
    lidar_err(pulse_width);
  }
}
