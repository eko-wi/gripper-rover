/*
   program untuk transporter robot
   board: ESP32BOT3
   output: motor 1 (kiri) 17-16, motor 2 (kanan) 13-12
   Format data swerve (X, Y, S, X2, bstate)
   angka swerve dipakai untuk memutar arah hadap robot sementara
   urutan pengiriman data:
   - magic byte
   - x value (kiri kanan) 0-255, center=128
   - y value (maju mundur) 0-255, center=128
   - s value (naik turun arm) 0-180
   - x2 value (swerve)
   - button (grip buka tutup) 0=stop, 1=tutup, 2=buka, 4=putar kanan 90, 8=putar kiri 90
   (tidak pakai) output gyro: -PI sampai +PI, putar kanan --> makin besar
   Kontroller di HP dengan app inventor:
   https://drive.google.com/file/d/1tef8k8jJbKRhRBvKcuJgtwJ6TbcYfy70/view?usp=drive_link
*/
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "BluetoothSerial.h"
#include<ESP32Servo.h>
#include<driver/mcpwm.h>
#include<EEPROM.h>
#include"classes.cpp"
//tombol:
#define T1 14
#define T2 15
#define T3 18
#define T4 19
//output motor:
//kiri=17-16, kanan=13-12
/*
  #define M1A 17
  #define M1B 16
  #define M2A 13
  #define M2B 12
*/
//kiri=13-12, kanan=17-16
#define M2A 16
#define M2B 17
#define M1A 12
#define M1B 13
//channel ledc:
#define PWMC1 15
#define PWMC2 14
#define PWMC3 13
#define PWMC4 12
//servo:
#define S_ANGKAT_PIN 23
#define S_GRIP_PIN 25
#define SGRIP_ANGLE_HOLD 90
#define SGRIP_ANGLE_OPEN 180
//kalau tidak ada perintah, langsung set servo grip ke 90 derajat (untuk servo 360)
#define GRIP_AUTORETURN 0
//gerakan ujung ke ujung dalam 500 ms
//#define SERVORATE 0.036
#define SERVORATE 0.36
//eeprom:
#define MAGIC 0xbd
#define ADR_DATA 0x02
#define ADR_KP 2
#define ADR_KI 10
#define ADR_KD 18
#define ADR_RF 24
//bluetooth:
#define BLUETOOTHNAME "TOPScience-Transport3"
//untuk mengaktifkan gerakan test motor di awal, uncomment baris berikut ini
//#define TESTMOTORS

//gyro:
//offset dicoba dengan example IMUzero
/*
  ....................  XAccel      YAccel        ZAccel      XGyro     YGyro     ZGyro
  [-3021,-3019] --> [-9,11]  [261,262] --> [-5,9]  [1141,1142] --> [16383,16400] [102,103] --> [-1,4]  [58,59] --> [0,4] [-3,-2] --> [-2,1]
  .................... [-3021,-3020] --> [-9,6] [261,262] --> [-4,9]  [1141,1142] --> [16384,16400] [102,103] --> [-2,4]  [58,59] --> [0,4] [-3,-2] --> [-2,1]
  .................... [-3021,-3020] --> [-14,6]  [261,262] --> [-4,9]  [1141,1141] --> [16384,16387] [102,103] --> [-3,4]  [58,59] --> [0,4] [-3,-2] --> [-3,1]
  -------------- done --------------
*/
//urutan: xgyro, ygyro, zgyro, xaccel, yaccel, zaccel
//int gyrooffsets[6] = {102, 58, -2, -3021, 261, 1141}; //robot 1
struct savedvars {
  float kp, ki, kd, bo;
  int powermax, powerslow, powerputar;
  int gx, gy, gz, grx, gry, grz;
  byte magic;
} vars, vars1;
int gyrooffsets[6] = {180, 74, 277, 399, -563, 1036}; //robot wificar adventure
MPU6050 mpu;
Quaternion q;
VectorFloat grav;
uint8_t packetsize, fifocount;
float ypr[3]; //yaw pitch roll
uint8_t fifobuffer[64];
float arahhadap = 0, deltaarah = 0, targetarah = 0, targetarah1 = 0, deltatargetarah = 0;
mcpwm_config_t conf;
Servo s_angkat, s_grip;
BluetoothSerial btserial;
int commandsource = 0; //0 = serial, 1 = btserial
long t = 0, tlastcommand = 0, tlastproses = 0, tlastcalc = 0;
int  motorenable = 0, adagyro = 0;
int xdata = 0, ydata = 0, sdata = 90,x2data=0, bdata = 0;
float rotatecompensationfactor = 0;
float rotatecompensation = 0;
float powermaju = 0, powerputar = 0, deltamotor = 0;
float powerkiri = 0, powerkanan = 0;
float lastpower = 0;
float boostfactor = 1;
enum states {
  AWAL, SETKP, SETKI, SETKD, SETF, SAVEEE, READEE, XDATA, YDATA, X2DATA, SDATA, BDATA, PMAX, PLOW, PPUTAR
} state;

PIDController depan(120, 1.5, 15000, 80, 50); //set motor value dalam persen, maks 100
Smoothfilter putarsmooth(2);
RateLimiter armpos(SERVORATE,&t);
inline void ledon() {
  digitalWrite(2, HIGH);
}
inline void ledoff() {
  digitalWrite(2, LOW);
}

void updatemotor() {
  //kiri
  if (powerkiri >= 0) {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, powerkiri);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  }
  else {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, -powerkiri);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
  }
  //kanan
  if (powerkanan >= 0) {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, powerkanan);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  }
  else {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, -powerkanan);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  }
}


inline void putar(int power) { //skala -100 sampai +100
  deltamotor = power;
}

void maju(float power) {
  float bb = 0;
  if (fabs(power) > fabs(lastpower)) {
    bb = (power - lastpower) * boostfactor;
  }
  powermaju = bb + power;
  lastpower = power;
}

void putardelta(float delta) {
  targetarah = targetarah + delta;
  if (targetarah >= M_PI) {
    targetarah -= (2 * M_PI);
  }
  else if (targetarah <= -M_PI) {
    targetarah += (2 * M_PI);
  }
}
int sign(int a) {
  if (a > 0) return 1;
  else return -1;
}
void prosesoutput() {
  long deltat = t - tlastproses;
  tlastproses = t;
  //dari ydata jadi maju mundur
  int yy = ydata - 128;
  int xx2 = x2data - 128;
  float x2 = (float)(xx2) * 0.78125;
  float y = (float)(yy) * 0.78125; //dari skala 128 jadi skala 100
  float z = max(fabs(y), fabs(x2)) * sign(yy);
  maju(z);
  //kalau maju, body akan terputar ke kiri, jadi tambahkan sedikit power putar ke kanan
  //rotatecompensation = y * rotatecompensationfactor;
  //dari xdata jadi sudut putar
  deltaarah = (float)(xdata - 128) * 2.34375e-5 * deltat; //128 = 3 rad/s
  deltatargetarah = (float)(xx2) * 0.0122718 * sign(yy); // pi/2 rad untuk 128
  putardelta(deltaarah);
}
void stopmovement() {
  powerkiri = 0;
  powerkanan = 0;
  powermaju = 0;
  updatemotor();
  depan.reset();
  targetarah = arahhadap;
  putarsmooth.setvalue(0);
}
void noswerve(){
  deltatargetarah=0;
}
void prosesbstate() {
  //bit 5: grip hold
  //bit 6: grip open
  if (bdata & (1 << 5)) {
    s_grip.write(SGRIP_ANGLE_HOLD);
  }
  else if (bdata & (1 << 6)) {
    s_grip.write(SGRIP_ANGLE_OPEN);
  }
  else {
    if (GRIP_AUTORETURN) s_grip.write(90);
  }
}
int readsettingsfromeeprom() {
  if (EEPROM.read(0) == MAGIC) {
    EEPROM.get(ADR_DATA, vars1);
    if (vars1.magic == MAGIC) {
      Serial.print("saved kp:"); Serial.print(vars1.kp);
      Serial.print(" ki:"); Serial.print(vars1.ki);
      Serial.print(" kd:"); Serial.print(vars1.kd);
      Serial.print(" b:"); Serial.print(vars1.bo);
      Serial.println();
      Serial.print("powermax:"); Serial.print(vars1.powermax);
      Serial.print(" slow:"); Serial.print(vars1.powerslow);
      Serial.print(" putar:"); Serial.print(vars1.powerputar);
      Serial.println();
      depan.setKP(vars1.kp);
      depan.setKI(vars1.ki);
      depan.setKD(vars1.kd);
      boostfactor = vars1.bo;
      return 1;
    }
    else {
      Serial.println("Tidak ada data tersimpan.");
    }
  }
  return 0;
}
void savesettingstoeeprom() {
  if (EEPROM.read(0) != MAGIC) {
    EEPROM.write(0, MAGIC);
  }
  vars1.kp = depan.getKP();
  vars1.ki = depan.getKI();
  vars1.kd = depan.getKD();
  vars1.bo = boostfactor;
  vars1.magic = MAGIC;
  EEPROM.put(ADR_DATA, vars1);
  EEPROM.commit();
}
//==============command processor==============
void prosesdata(Stream &S) {
  float f;
  char c;
  int j;
  switch (state) {
    case AWAL: //terima magic byte awal
      c = S.read();
      Serial.println(c);
      if (c == MAGIC) {
        state = XDATA;
        ledcWrite(PWMC1, 0);
      }
      else if (c == 'p') { //set kp
        state = SETKP;
      }
      else if (c == 'i') {
        state = SETKI;
      }
      else if (c == 'd') {
        state = SETKD;
      }
      else if (c == 'f') {
        state = SETF;
      }
      else if (c == 's') { //save kp ki kd ke eeprom
        savesettingstoeeprom();
        Serial.println("pid constants saved.");
      }
      else if (c == 'r') { //baca kp ki kd dll
        S.print(depan.getKP());
        S.print(',');
        S.print(depan.getKI());
        S.print(',');
        S.print(depan.getKD());
        S.print(',');
        S.print(boostfactor);
        S.println();
      }
      else if (c == 'P') { //power max, power slow, power putar
        state = PMAX;
      }
      else if (c == 'R') { //putar kanan 90
        putardelta(M_PI_2);
        noswerve();
      }
      else if (c == 'L') { //putar kiri 90
        putardelta(-M_PI_2);
        noswerve();
      }
      else if (c == '0') {
        motorenable = 0;
        stopmovement();
        ledoff();
      }
      else if (c == '1') {
        motorenable = 1;
        ledon();
      }
      else if (c == 'W') { //perintah WSAD
        maju(vars1.powermax);
        putar(0);
        noswerve();
      }
      else if (c == 'w') { //maju pelan
        maju(vars1.powerslow);
        putar(0);
        noswerve();
      }
      else if (c == 'S') { //mundur pelan
        maju(-vars1.powerslow);
        putar(0);
      }
      else if (c == 'X') { //mundur full power
        maju(-vars1.powermax);
        putar(0);
      }
      else if (c == 'A') { //putar kiri
        maju(0);
        putar(-vars1.powerputar);
        noswerve();
      }
      else if (c == 'D') { //putar kanan
        maju(0);
        putar(vars1.powerputar);
        noswerve();
      }
      break;
    case XDATA:
      xdata = S.read();
      state = YDATA;
      break;
    case YDATA:
      ydata = S.read();
      state = X2DATA;
      break;
    case X2DATA:
      x2data = S.read();
      state = SDATA;
      break;
    case SDATA:
      sdata = S.read();
      state = BDATA;
      break;
    case BDATA:
      bdata = S.read();
      state = AWAL;
      //proses hitungan di sini
      prosesoutput();
      ledcWrite(PWMC2, 255 - fabs(powermaju * 2.55));
      S.print("t:");
      S.print((int)(targetarah * 180 / M_PI));
      S.print(" a:");
      S.println((int)(arahhadap * 180 / M_PI));
      /*
        Serial.print("target:");
        Serial.print(targetarah);
        Serial.print("\tarah:");
        Serial.println(arahhadap);
      */
      ledcWrite(PWMC1, 255);
      //proses bstate
      prosesbstate();
      //gerakkan arm
      
      s_angkat.write(armpos.update(sdata));
      break;
    case SETKP: //set KP
      f = S.parseFloat();
      depan.setKP(f);
      state = AWAL;
      break;
    case SETKI: //set KI
      f = S.parseFloat();
      depan.setKI(f);
      state = AWAL;
      break;
    case SETKD: //set KD
      f = S.parseFloat();
      depan.setKD(f);
      state = AWAL;
      break;
    case SETF: //set rotate compensation factor
      f = S.parseFloat();
      rotatecompensationfactor = f;
      state = AWAL;
      break;
    case PMAX: //set power max, power  slow, power putar
      j = S.read();
      if (j > 0 && j < 100) vars1.powermax = j;
      state = PLOW;
      break;
    case PLOW:
      j = S.read();
      if (j > 0 && j < 100) vars1.powerslow = j;
      state = PPUTAR;
      break;
    case PPUTAR:
      j = S.read();
      if (j > 0 && j < 100) vars1.powerputar = j;
      state = AWAL;
  } //tutup switch(state)
}
//===============awal program==================

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  EEPROM.begin(512);
  Wire.begin(21, 22, 400000);
  btserial.begin(BLUETOOTHNAME);
  pinMode(2, OUTPUT);
  ledon();
  readsettingsfromeeprom();
  s_angkat.attach(S_ANGKAT_PIN);
  s_grip.attach(S_GRIP_PIN);
  s_angkat.write(0);
  s_grip.write(SGRIP_ANGLE_OPEN);
  armpos.jumpto(0);
  ledcSetup(PWMC1, 1000, 8);
  ledcSetup(PWMC2, 1000, 8);
  ledcSetup(PWMC3, 1000, 8);
  ledcSetup(PWMC4, 1000, 8);
  ledcAttachPin(14, PWMC1);
  ledcAttachPin(15, PWMC2);
  ledcAttachPin(18, PWMC3);
  ledcAttachPin(19, PWMC4);
  for (int i = 0; i < 255; i++) {
    int t1 = i * 2;
    int t2 = t1 - 40;
    int t3 = t1 - 80;
    int t4 = t1 - 120;
    t1 = constrain(t1, 0, 255);
    t2 = constrain(t2, 0, 255);
    t3 = constrain(t3, 0, 255);
    t4 = constrain(t4, 0, 255);
    ledcWrite(PWMC1, 256 - t1);
    ledcWrite(PWMC2, 256 - t2);
    ledcWrite(PWMC3, 256 - t3);
    ledcWrite(PWMC4, 256 - t4);
    delay(2);
  }
  ledcWrite(PWMC1, 256);
  ledcWrite(PWMC2, 256);
  ledcWrite(PWMC3, 256);
  ledcWrite(PWMC4, 256);
  if (mpu.testConnection()) {
    Serial.println("MPU6050 koneksi OK");
    //tone(12, 500); delay(100); noTone(12); delay(400);
  }
  else {
    Serial.println("MPU connection failed");
    ledcWrite(PWMC1, 255);
    ledcWrite(PWMC2, 255);
    ledcWrite(PWMC3, 255);
    ledcWrite(PWMC4, 0);
  }
  if (mpu.dmpInitialize() == 0) {
    Serial.println("DMP OK");
    mpu.setXGyroOffset(gyrooffsets[0]);
    mpu.setYGyroOffset(gyrooffsets[1]);
    mpu.setZGyroOffset(gyrooffsets[2]);
    mpu.setXAccelOffset(gyrooffsets[3]);
    mpu.setYAccelOffset(gyrooffsets[4]);
    mpu.setZAccelOffset(gyrooffsets[5]);
    mpu.setDMPEnabled(true);
    packetsize = mpu.dmpGetFIFOPacketSize();
    adagyro = 1;
  }
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, M1A); //hubungkan unit PWM 0, fungsi yang mana, ke pin mana
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, M1B); //kiri
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, M2A);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, M2B); //kanan
  conf.frequency = 1000;
  conf.cmpr_a = 0;
  conf.cmpr_b = 0;
  conf.duty_mode = MCPWM_DUTY_MODE_0; //active high
  conf.counter_mode = MCPWM_UP_COUNTER;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &conf);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &conf);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);

  //experimental features untuk PID
  depan.setEminInt(10 * M_PI / 180);


#ifdef TESTMOTORS
  Serial.println("Testing motors...");
  powerkiri = 80; powerkanan = 80;  updatemotor();
  delay(100);
  powerkiri = -80; powerkanan = -80;  updatemotor();
  delay(100);
  powerkiri = 0; powerkanan = 0;  updatemotor();
  delay(100);
#endif
  vars1.powermax = 100;
  vars1.powerslow = 24;
  vars1.powerputar = 30;
  Serial.println("Setup done");
  ledoff();
}

void loop() {
  t = millis();
  if (adagyro) {
    if (mpu.dmpGetCurrentFIFOPacket(fifobuffer)) {
      mpu.dmpGetQuaternion(&q, fifobuffer);
      mpu.dmpGetGravity(&grav, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &grav);
      arahhadap = ypr[0]; //dalam radian
      //Serial.println(arahhadap);
      //kalau dekat batas +- pi:
      targetarah1 = targetarah + deltatargetarah; //delta ini untuk mensimulasikan gerak swerve
      if (targetarah1 >= M_PI) {
        targetarah1 -= (2 * M_PI);
      }
      else if (targetarah1 <= -M_PI) {
        targetarah1 += (2 * M_PI);
      }
      
      if (targetarah1 > M_PI_2) {
        if (arahhadap < 0) arahhadap += 2 * M_PI;
      }
      else if (targetarah1 < -M_PI_2) {
        if (arahhadap > 0) arahhadap -= 2 * M_PI;
      }
      float deltat = t - tlastcalc;
      powerputar = depan.calc(arahhadap, targetarah1, deltat); //positif = putar kanan
      //kalau arah hadap < target (kurang ke kanan), selisih = negatif, output harus positif
      putar(-powerputar);
      ledcWrite(PWMC3, 255 - fabs(powerputar * 2.55));
      tlastcalc = t;
    }
  }
  else { //tidak ada gyro! apa yang harus diperbuat?
    float deltat = t - tlastcalc;
    if (deltat > 50) { //update 20 kali perdetik
      if (targetarah > arahhadap) { //putarkanan
        powerputar = vars1.powerputar;
      }
      else if (targetarah < arahhadap) {
        powerputar = -vars1.powerputar;
      }
      else {
        powerputar = 0;
      }
      targetarah = 0;
      arahhadap = 0;
      //kalau arah hadap < target (kurang ke kanan), selisih = negatif, output harus positif
      putar(-powerputar);
      ledcWrite(PWMC3, 255 - fabs(powerputar * 2.55));
      tlastcalc = t;
    }
  }
  while (Serial.available()) {
    tlastcommand = t;
    prosesdata(Serial);
  }
  if (btserial.hasClient()) {
    while (btserial.available()) {
      tlastcommand = t;
      prosesdata(btserial);
    }
  }

  if (t - tlastcommand > 120) {
    stopmovement();
  }
  //update motor di setiap loop
  if (motorenable) {
    //putar positif ke kanan, power kiri tambahkan, power kanan kurangi
    powerkiri = powermaju + deltamotor;
    powerkanan = powermaju - deltamotor;
    if (powerkiri > 100) powerkiri = 100;
    else if (powerkiri < -100) powerkiri = -100;
    if (powerkanan > 100) powerkanan = 100;
    else if (powerkanan < -100) powerkanan = -100;
    updatemotor();
    s_angkat.write(armpos.update(sdata));
  }
}
