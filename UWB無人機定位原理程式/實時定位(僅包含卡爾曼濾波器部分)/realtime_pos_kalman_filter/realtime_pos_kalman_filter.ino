#include "I2Cdev.h"
#include <MatrixMath.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#include <HardwareSerial.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define RXD2 16
#define TXD2 17
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // connect to UWB tag A
    while (!Serial); 

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(2, 1);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    // load and configure the DMP
    for(int i = 0; i < 4; i++){
      Serial.print(".");
      delay(1000);
    }
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    float resl = mpu.get_acce_resolution();
    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(17);
    mpu.setYGyroOffset(18);
    mpu.setZGyroOffset(-40);
    mpu.setXAccelOffset(-5089);
    mpu.setYAccelOffset(2137);
    mpu.setZAccelOffset(605); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

float rotate_mat[3][3] = {0};

void get_rotate_mat(float rotate_mat[3][3], Quaternion q){
  rotate_mat[0][0] = 1 - 2*q.y*q.y - 2*q.z*q.z;
  rotate_mat[0][1] = 2*q.x*q.y - 2*q.w*q.z;   // 2*q.w*q.z 
  rotate_mat[0][2] = 2*q.x*q.z + 2*q.w*q.y;
  rotate_mat[1][0] = 2*q.x*q.y + 2*q.w*q.z;
  rotate_mat[1][1] = 1 - 2*q.x*q.x - 2*q.z*q.z;
  rotate_mat[1][2] = 2*q.y*q.z - 2*q.w*q.x;
  rotate_mat[2][0] = 2*q.x*q.z - 2*q.w*q.y;
  rotate_mat[2][1] = 2*q.y*q.z + 2*q.w*q.x;
  rotate_mat[2][2] = 1 - 2*q.x*q.x - 2*q.y*q.y;
  
}
//將機體座標系加速度轉換為UWB定位座標系加速度
void rotate_body_to_world(float rotate_mat[][3], float *acc_body, mtx_type *acc_world){
  for(uint8_t i = 0; i < 3; i++){
    acc_world[i] = 0;
    for(uint8_t j = 0; j < 3; j++){
      acc_world[i] += rotate_mat[i][j]*acc_body[j];
    }
  }
}

float acc_world[3] = {0};
float acc_body[3] = {0};
float dt = 0.05;

mtx_type F[6][6] = {
     {1, 0, 0, dt, 0, 0},
     {0, 1, 0, 0, dt, 0},
     {0, 0, 1, 0, 0, dt},
     {0, 0, 0, 1, 0, 0 },
     {0, 0, 0, 0, 1, 0 },
     {0, 0, 0, 0, 0, 1 }
  };
  
mtx_type B[6][3] = {
  {0.5*dt*dt, 0, 0},
  {0, 0.5*dt*dt, 0},
  {0, 0, 0.5*dt*dt},
  {dt, 0, 0},
  {0, dt, 0},
  {0, 0, dt}
};


mtx_type H[3][6] = {{1, 0, 0, 0, 0, 0},
                    {0, 1, 0, 0, 0, 0},
                    {0, 0, 1, 0, 0, 0}};

mtx_type HT[6][3] = {{1, 0, 0},
                     {0, 1, 0},
                     {0, 0, 1},
                     {0, 0, 0},
                     {0, 0, 0},
                     {0, 0, 0}};

//Q為預測模型誤差的共變異數矩陣
#define Q_var 0.4
mtx_type Q[6][6] = {{Q_var, 0, 0, 0, 0, 0},
                    {0, Q_var, 0, 0, 0, 0},
                    {0, 0, Q_var, 0, 0, 0},
                    {0, 0, 0, Q_var, 0, 0},
                    {0, 0, 0, 0, Q_var, 0},
                    {0, 0, 0, 0, 0, Q_var}};

//R為觀測誤差共變異數矩，在此程式中為UWB三軸分別的定位誤差變異數
#define R_var 0.3                  
mtx_type R[3][3] = {{R_var, 0, 0},  //x軸
                    {0, R_var, 0},  //y軸
                    {0, 0, 0.7}};   //z軸


//卡爾曼濾波器運算
void kalman_filter(mtx_type *Z_k1, mtx_type *X_k, mtx_type *U_k1, mtx_type P_k[6][6]){

  mtx_type X_k1[6], P_k1[6][6];
  //calculate X_k1_ = F X_k + B U
  mtx_type FX[6][1], BU[6][1], X_k1_[6];
  Matrix.Multiply((mtx_type *)F, (mtx_type *)X_k, 6, 6, 1, (mtx_type *)FX);
  Matrix.Multiply((mtx_type *)B, (mtx_type *)U_k1, 6, 3, 1, (mtx_type *)BU);
  Matrix.Add((mtx_type*) FX, (mtx_type*) BU, 6, 1, (mtx_type*) X_k1_);
  
  
  //calculate P_k1_ = F P_k F.T + Q
  mtx_type FP_k[6][6], FP_kFT[6][6], FT[6][6], P_k1_[6][6];
  Matrix.Transpose((mtx_type *)F, 6, 6, (mtx_type *)FT);
  Matrix.Multiply((mtx_type *)F, (mtx_type *)P_k, 6, 6, 6, (mtx_type *)FP_k);
  Matrix.Multiply((mtx_type *)FP_k, (mtx_type *)FT, 6, 6, 6, (mtx_type *)FP_kFT);
  Matrix.Add((mtx_type*) FP_kFT, (mtx_type*) Q, 6, 6, (mtx_type*) P_k1_);

  //calculate K = P_k1_ HT(inv(HP_k1_HT + R))
  mtx_type K[6][3], P_k1_HT[6][3], HP_k1_[3][6], HP_k1_HT[3][3], C[3][3];
  Matrix.Multiply((mtx_type *)P_k1_, (mtx_type *)HT, 6, 6, 3, (mtx_type *)P_k1_HT);
  Matrix.Multiply((mtx_type *)H, (mtx_type *)P_k1_, 3, 6, 6, (mtx_type *)HP_k1_);
  Matrix.Multiply((mtx_type *)HP_k1_, (mtx_type *)HT, 3, 6, 3, (mtx_type *)HP_k1_HT);
  Matrix.Add((mtx_type*) HP_k1_HT, (mtx_type*) R, 3, 3, (mtx_type*) C);
  Matrix.Invert((mtx_type*)C, 3);
  Matrix.Multiply((mtx_type *)P_k1_HT, (mtx_type *)C, 6, 3, 3, (mtx_type *)K);

  //x_k1 = X_k1_ + K(Z_k - HX_k1_)
  mtx_type HX_k1_[3], D[3], KD[6];
  Matrix.Multiply((mtx_type *)H, (mtx_type *)X_k1_, 3, 6, 1, (mtx_type *)HX_k1_);
  Matrix.Subtract((mtx_type*) Z_k1, (mtx_type*) HX_k1_, 3, 1, (mtx_type*) D);
  Matrix.Multiply((mtx_type *)K, (mtx_type *)D, 6, 3, 1, (mtx_type *)KD);
  Matrix.Add((mtx_type*) X_k1_, (mtx_type*) KD, 6, 1, (mtx_type*) X_k1);
  
  //P_k1 = (I - KH)P_k1_
  mtx_type I6[6][6] = {{1, 0, 0, 0, 0, 0},{0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 1}};
  mtx_type KH[6][6], E[6][6];
  Matrix.Multiply((mtx_type *)K, (mtx_type *)H, 6, 3, 6, (mtx_type *)KH);
  Matrix.Subtract((mtx_type*) I6, (mtx_type*) KH, 6, 6, (mtx_type*) E);
  Matrix.Multiply((mtx_type *)E, (mtx_type *)P_k1_, 6, 6, 6, (mtx_type *)P_k1);

  
  //X_k = X_k1, P_k = P_k1
  Matrix.Copy((mtx_type *)X_k1, 6, 1, (mtx_type *)X_k);
  Matrix.Copy((mtx_type *)P_k1, 6, 6, (mtx_type *)P_k);
  //Matrix.Print((mtx_type*)K, 6, 3, "K");
  
}

//讀取UWB定位結果
void read_Z(float *Z){
  //get Z code
  if (Serial2.available()) {
    String received = Serial2.readStringUntil('\n');
    for (int i = 0; i < 3; i++) {
      String temp = splitString(received, ',', i);
      float position = temp.toFloat();
      Z[i] = position;
    }
  }
  return;
}

mtx_type X[6] = {0};  //[x, y, z, vx, vy, vz]

mtx_type P[6][6] = {0};  //狀態誤差共變異數矩陣
mtx_type Z[3] = {0.1, 0.1, 0.1}; //UWB定位結果
int first = 0;
void loop() {
    
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
          
          
          mpu.dmpGetQuaternion(&q, fifoBuffer);    //讀取無人機姿態四元數 
          mpu.dmpGetAccel(&aa, fifoBuffer);        //讀取加速度
          mpu.dmpGetGravity(&gravity, &q);         //讀取重力加速度 
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);  //得到去除重力加速度的無人機加速度
          //mpu.dmpConvertToWorldFrame(&aaWorld, &aaReal, &q);  //MPU6050內部的機體座標系與UWB定位座標系座標轉換  
          
          //前五筆資料不用
          if(first < 5){
            first++;
            if(first == 4){
              for(int i = 0; i < 6; i++){
                    P[i][i]= 0.2;
                }
              acc_prev[0] = float(2*9.086*aaWorld.x) / 16384;  
              acc_prev[1] = float(2*9.086*aaWorld.y) / 16384;
              acc_prev[2] = float(2*9.086*aaWorld.z) / 16384;
              read_Z(Z);
              for(int i = 0; i < 3; i++){
                X[i] = Z[i];  
              }  
              
             } 
          }
          else{

            
            acc_world[0] = float(2*9.806 * aaWorld.x )/ 16384;
            acc_world[1] = float(2*9.806 * aaWorld.y )/ 16384;
            acc_world[2] = float(2*9.806 * aaWorld.z )/ 16384;
            
            //read Z from UWB
            read_Z(Z);
            
            //讀轉換加速度值單位至m/s
            acc_body[0] = float(2*9.806 * aaReal.x )/ 16384;
            acc_body[1] = float(2*9.806 * aaReal.y )/ 16384;
            acc_body[2] = float(2*9.806 * aaReal.z )/ 16384;
            //座標轉換
            get_rotate_mat(rotate_mat, q);
            rotate_body_to_world(rotate_mat, acc_body, acc_world);
            //卡爾曼濾波器運算
            kalman_filter(Z, X, acc_world, P);
           
          }
          
    }
}

String splitString(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
