#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>



#define R_ENCount 300
#define L_ENCount 300

#define R_CHA 3 // encoder R I
#define R_CHB 9 // encoder R

#define L_CHA 2 // encoder L I
#define L_CHB 8 // encoder L

#define R_PWM 10
#define L_PWM 5

#define R_MOTORB 11
#define R_MOTORA 12

#define L_MOTORA 7
#define L_MOTORB 6



//boolean limitSwitchState = digitalRead(LIMIT_SWITCH_PIN);
boolean Direction_right, Direction_left = true;
volatile long right_wheel_pulse_count = 0, left_wheel_pulse_count = 0;
int servoState=0;
float interval = 50;
long previousMillis = 0;
long currentMillis = 0;

float rpm_right = 0, rpm_left = 0;
int pwm_right = 0, pwm_left = 0;

float ref_r, ref_l;

float Kpr = 1.15, Kir = 0.087, Kdr = 0.8;
float Kpl = 1.15, Kil = 0.087, Kdl = 0.8;

float error_r = 0, prev_error_r = 0, sum_error_r = 0;
float error_l = 0, prev_error_l = 0, sum_error_l = 0;

const float WHEEL_DIAMETER = 0.067; // meters
const float WHEEL_DISTANCE = 0.325;  // meters

float linear_vel_cmd = 0, angular_vel_cmd = 0;
float left_wheel_vel = 0, right_wheel_vel = 0;

ros::NodeHandle nh;

std_msgs::Float32 rpsr, rpsl;
ros::Publisher right_wheel_publish("right_wheel_rpm", &rpsr);
ros::Publisher left_wheel_publisher("left_wheel_rpm", &rpsl);



void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel_msg)
{
    // Get the linear and angular velocity commands
    linear_vel_cmd = cmd_vel_msg.linear.x;
    angular_vel_cmd = cmd_vel_msg.angular.z;

    // Calculate the left and right wheel velocities using the kinematics of a differential drive robot
    left_wheel_vel = (linear_vel_cmd - (angular_vel_cmd * WHEEL_DISTANCE)/2) ;
    right_wheel_vel = (linear_vel_cmd + (angular_vel_cmd * WHEEL_DISTANCE)/2);

    // Convert the left and right wheel velocities to RPss
    ref_l = (left_wheel_vel * 60 / (WHEEL_DIAMETER * PI));
    ref_r = (right_wheel_vel * 60 / (WHEEL_DIAMETER * PI));

    ref_l = constrain(ref_l, -320, 320);
    ref_r = constrain(ref_r, -320, 320);
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);





void motor_write_r(float pwm_mtr)
{
    if (pwm_mtr >= 0)
    {    
        analogWrite(R_PWM,pwm_mtr);
        digitalWrite(R_MOTORB, HIGH);
        digitalWrite(R_MOTORA, LOW);
    }
    else
    {    
        analogWrite(R_PWM,abs(pwm_mtr));
        digitalWrite(R_MOTORA, HIGH);
        digitalWrite(R_MOTORB, LOW);
    }
}

void motor_write_l(float pwm_mtr)
{
    if (pwm_mtr >= 0)
    {
        
        analogWrite(L_PWM,pwm_mtr);
        digitalWrite(L_MOTORA, HIGH);
        digitalWrite(L_MOTORB, LOW);
    }
    else
    {
        analogWrite(L_PWM,abs(pwm_mtr));
        digitalWrite(L_MOTORB, HIGH);
        digitalWrite(L_MOTORA, LOW);
    }
}

void right_wheel_pulse()
{
    int val = digitalRead(R_CHB);

    if (val == LOW)
    {
        Direction_right = true; // Reverse
    }
    else
    {
        Direction_right = false; // Forward
    }

    if (Direction_right)
    {
        right_wheel_pulse_count++;
    }
    else
    {
        right_wheel_pulse_count--;
    }
}

void left_wheel_pulse()
{
    int val = digitalRead(L_CHB);

    if (val == LOW)
    {
        Direction_left = true; // Reverse
    }
    else
    {
        Direction_left = false; // Forward
    }

    if (Direction_left)
    {
        left_wheel_pulse_count++;
    }
    else
    {
        left_wheel_pulse_count--;
    }
}

float PID_right(float &kpr, float &kdr, float &kir)
{
    error_r = ref_r - rpm_right;
    pwm_right = (Kpr * error_r) + kdr * (error_r - prev_error_r) + kir * sum_error_r;
    prev_error_r = error_r;
    sum_error_r = sum_error_r + error_r;
    pwm_right = constrain(pwm_right, -255, 255);
    return pwm_right;
}

float PID_left(float &kpl, float &kdl, float &kil)
{
    error_l = ref_l - rpm_left;
    pwm_left = (kpl * error_l) + kdl * (error_l - prev_error_l) + kil * sum_error_l;
    prev_error_l = error_l;
    sum_error_l = sum_error_l + error_l;
    pwm_left = constrain(pwm_left, -255, 255);
    return pwm_left;
}

void setup()
{
    
    attachInterrupt(digitalPinToInterrupt(R_CHA), right_wheel_pulse, RISING);
    attachInterrupt(digitalPinToInterrupt(L_CHA), left_wheel_pulse, RISING);

    pinMode(R_CHA, INPUT_PULLUP);
    pinMode(R_CHB, INPUT);

    pinMode(L_CHA, INPUT_PULLUP);
    pinMode(L_CHB, INPUT);

    pinMode(R_MOTORA, OUTPUT);
    pinMode(R_MOTORB, OUTPUT);

    pinMode(L_MOTORA, OUTPUT);
     pinMode(L_MOTORB, OUTPUT);

    nh.initNode();
    nh.subscribe(cmd_vel_sub);
   
    nh.advertise(right_wheel_publish);
    nh.advertise(left_wheel_publisher);

    Serial.begin(57600);

}


  

void loop()
{
    currentMillis = millis();

    if (currentMillis - previousMillis > interval)
    {
        previousMillis = currentMillis;

        rpm_right = (float)(right_wheel_pulse_count * 60 / (R_ENCount * interval * 0.001));
        rpm_left = (float)(left_wheel_pulse_count * 60 / (L_ENCount * interval * 0.001));

        right_wheel_pulse_count = 0;
        left_wheel_pulse_count = 0;
    }
    // ref_l = 100*(sin(currentMillis/1e3));
    // ref_r = 100*(sin(currentMillis/1e3));
    
    // Serial.print(left_wheel_pulse_count); 
    // Serial.print("\t");
    // Serial.println(right_wheel_pulse_count);

// Serial.print("\t");

//     Serial.print(rpm_right); 
//     Serial.print("\t");
//     Serial.println(ref_r);

    rpsr.data = rpm_right;
    rpsl.data = rpm_left;

    pwm_right = PID_right(Kpr, Kdr, Kir);
    pwm_left = PID_left(Kpl, Kdl, Kil);

    motor_write_r(pwm_right);
    motor_write_l(pwm_left);

// ref_l =100;
// ref_r=80;

    right_wheel_publish.publish(&rpsr);
    left_wheel_publisher.publish(&rpsl);


    nh.spinOnce();
    delay(10);
}