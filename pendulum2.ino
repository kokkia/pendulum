//倒立振子
#include <MsTimer2.h>
#include <MPU9250_asukiaaa.h>
#include "kal/kal.h"

//debug
#define DEBUG 0
#define MOTOR_DEBUG 0
//offset
#define GY_OFFSET 0.73186
//motor control gain
#define KP 90.0
#define KD 0.2
#define KDD 0.05

using namespace kal;

//9 axis sensor
MPU9250 mySensor;
uint8_t sensorId;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
float phi = 0.0;
float phi_acc;

int cnt=0;
//posture estimation
KalmanFilter<double> kphi(0.0,0.149*DEG2RAD*0.149*DEG2RAD,1.0);
HPF<double> hdphi(0.0,0.1);

//wave generator
wave sin_wave(0.0,PI/3,1.0);
//differentiator
Diff<double> wheel_dtheta(0.0,100.0);
Diff<double> wheel_d2theta(0.0,5.0);

//robotdata
RobotData ref;
RobotData state;
RobotData ref_bfr;

void timerFire() {
  //9軸センサ取得
  aX = mySensor.accelX();
  aY = mySensor.accelY();
  aZ = mySensor.accelZ();
//  gX = mySensor.gyroX();
  gY = mySensor.gyroY() + GY_OFFSET;
//  gZ = mySensor.gyroZ();
  phi += gY * Ts;
  phi_acc = atan2(aZ,-aX);//加速度センサからの姿勢推定
  kphi.update(gY*DEG2RAD,phi_acc);
  hdphi.update(gY*DEG2RAD);

//状態取得
   state.phi = kphi.x_est+2.25*DEG2RAD;
   state.dphi = gY*DEG2RAD;
   state.theta = wheel_angle;
   wheel_dtheta.update(state.theta);
   state.dtheta =wheel_dtheta.x;
   wheel_d2theta.update(state.dtheta);
   state.d2theta = wheel_d2theta.x;
//目標値計算
  sin_wave.update();
  //ref.d2theta = sin_wave.output;
//  ref.d2theta = -(+0.516 * state.theta + 1.2477 * state.dtheta + 1016.8614 * state.phi + 87.5655 *state.dphi);
    ref.d2theta = -(+0.516 * state.theta + 0.2477 * state.dtheta + 1016.8614 * state.phi + 87.5655 *state.dphi);  
  ref.dtheta = trape_integral(Ts,ref_bfr.d2theta,ref.d2theta,ref.dtheta);
  ref.theta = trape_integral(Ts,ref_bfr.dtheta,ref.dtheta,ref.theta);//@todo摩擦補償
  dead_zone_compensate(ref_bfr.dtheta,ref.dtheta,ref.theta,10.0*DEG2RAD);//不感帯補償
  //ref.theta = sin_wave.output;
//出力計算
  double u = KP*(ref.theta-state.theta) + KD * (ref.dtheta - state.dtheta) + KDD * (ref.d2theta - state.d2theta);
  if(abs(state.phi*RAD2DEG)>60.0){
    u = 0.0;
  }
  motor(u);
#if DEBUG
  Serial.print(String(state.phi*RAD2DEG));
  Serial.print(",");
  Serial.print(String(phi_acc*RAD2DEG));
  Serial.print(",");
  Serial.println(String(kphi.x_est*RAD2DEG));
  //Serial.println(enc_cnt/2.0); 
#endif
#if MOTOR_DEBUG
  Serial.print(ref.theta*RAD2DEG);
  Serial.print(",");
  Serial.println(state.theta*RAD2DEG);
#endif
  ref_bfr = ref;
}

void setup() {
  //serial通信設定
  while(!Serial);
  Serial.begin(115200);
  Serial.println("started");
   
  //9軸センサの設定
  Wire.begin();
  mySensor.setWire(&Wire);
  mySensor.beginAccel();//分散ほぼ0
  mySensor.beginGyro();//分散0.149^2,ave:-0.73186
  mySensor.beginMag();
  sensorId = mySensor.readId();

  //モータの設定
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;//キャリア周波数32kHzに変更
  //エンコーダの設定
  pinMode(TACHOA0_PIN, INPUT);
  pinMode(TACHOA1_PIN, INPUT);
  attachInterrupt(0, ENC_READ, CHANGE);//外部割込み
  attachInterrupt(1, ENC_READ, CHANGE);

  //timer2の設定
  MsTimer2::set((int)(Ts*1000), timerFire);
  MsTimer2::start();

  //init
 
}

void loop() {
  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  
  

}
