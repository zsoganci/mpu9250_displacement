/*
 * https://github.com/hideakitai/MPU9250/blob/master/MPU9250.h
 * https://www.arduino.cc/reference/en/libraries/mpu9250/
 * https://docs.arduino.cc/static/80c1edec7e8deb89b3430a69f2e44b5d/A000066-datasheet.pdf
 * https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
 */
#include "MPU9250.h"
#include "Math.h"
#define g 9.80665

MPU9250 mpu;
 
                   
float simpson(float arr[], int size, float ms);
bool checkChange (bool arr[], int size);

void setup() {

    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    mpu.selectFilter(QuatFilterSel::MADGWICK);

    mpu.setFilterIterations(1);
    mpu.setMagneticDeclination(5.52);    
    
    if (!mpu.setup(0x69)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
}

void loop() {
    
  uint32_t current_us1 = micros();
  static uint32_t prev_us1 = 0;
  static uint32_t prev_us2 = 0;
  static uint32_t prev_us3 = 0;
  static float accX[10]; // mpu.get will overwrite initial values
  static float velX[10]; // velocity along x axis
  static float accY[10]; // mpu.get will overwrite initial values
  static float velY[10]; // velocity along Y axis 
  static float velocityY = 0;
  static float velocityX = 0;
  

  // reading IMU for acc. 
  static int i = 0;
  while(i<10)                                // while loop to get all array elements in single fucntion runtime (equally-spaced data)
  {
    current_us1 = micros();
    /* w8 area 
    *
    *
    *
    *  w8 area
    */ 
    if(current_us1 - prev_us3 > 1000)
    {
    mpu.update();
    accX[i] = mpu.getAccX() * g;             // saving IMU values to array
    accY[i] = mpu.getAccY() * g;             // saving IMU values to array

    i++;                                      // incrementing 'i' to get all array elements
    prev_us3 = current_us1;               
    }

    // if statement is not true --> loop back upuntil all the array elements are obtained within 10us intervals
  }
  
  if(i == 10)                                // set 'i' to zero after reaching all the array elements 
  {
  i=0;
  }
  current_us1 = micros();
 
  /*
  if (current_us1 - prev_us1 > 1000000)  
  {
 
  prev_us1 = current_us1;
  }
  */

  if(current_us1 - prev_us2 > 10000)
  {
  
  // to print per 10ms
  // shifting array elemets to the right (to this to work arry size must vary :/?)
  /* updating array with new elements
  mpu.update();  
	float newAcc = mpu.getAccX()*g;
	for (int j = 0; j < 9; j++)
		accX[j] = accX[j + 1];

	accX[9] = newAcc;
  */
  // because dt is 10ms
  print_roll_pitch_yaw();

    if(current_us1 > 8000000) // calculate velocity after 8 sec
  { 
    static int k = 0;
    // for first 8 sec it wont calculate velocity (because at first acc valueas are not relevent)
    static bool changeX[9];
    static bool changeY[9];
     for(int kk = 0;kk < 9; kk++)   // COMPARE 10 elements of acc value if there is no BIG change then acc is zero
      {
        if( (accX[kk + 1] - accX[kk]) < 0.01*g)  // if changeX == 0 then there is no change
          changeX[kk] = false;  
          else
          changeX[kk] = true;

        if((accY[kk + 1] - accY[kk]) < 0.01*g)
          changeY[kk] = false;  
          else
          changeY[kk] = true;
      }
    

    if (checkChange(changeX,9) == false)  // if there is no BIG change in acc then acc must be zero
    {  
        for(int n = 0; n < 10; n++)
        accX[n]= 0;   
    }
    if (checkChange(changeY,9) == false)  // if there is no BIG change in acc then acc must be zero
    {  
        for(int n = 0; n < 10; n++)
        accY[n]= 0;   
    }

    mpu.update();
    if(mpu.getAccX() > 0.02 || mpu.getAccX() < -0.02)             // if acc values are not smaller than 0.02 then calculate velocity
    velocityX +=simpson(accX,10, 10.0);                           // 10ms 
    
    if(mpu.getAccY() > 0.02 || mpu.getAccY() < -0.02)
    velocityY +=simpson(accY,10, 10.0);                           // 10ms

    velX[k] = velocityX;                                          // after every 10ms velX/Y will be updated (but this func is taking at least 20ms to compute)
    velY[k] = velocityY;                                          // some limitation due CPU speed

    Serial.println(velX[k],8);
    Serial.println(velY[k],8);
    k++;
    if(k == 10)   // if k is 10 then there will be at least 10*10 = 100 ms time will be passed
    {
      static float displacementX =0;
      static float displacementY =0;
      displacementX += simpson(velX,10,100.0);
      displacementY += simpson(velY,10,100.0);
      Serial.print("displacementX: ");
      Serial.println(displacementX);
      Serial.print("displacementY: ");
      Serial.println(displacementY);     
      k=0;
    }
  }
  prev_us2 = current_us1;
  }  

}

//*****************************************************************************

void print_roll_pitch_yaw() {
  mpu.update();
    Serial.print("Yaw, Pitch, Roll, accX, accY, accZ: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.print(mpu.getRoll(), 2);
    Serial.print(", ");
    Serial.print(mpu.getAccX(), 2);
    Serial.print(", ");
    Serial.print(mpu.getAccY(), 2);
    Serial.print(", ");
    Serial.println(mpu.getAccZ(), 2);
}

float simpson(float arr[], int size, float ms)  {  // 
  double sum =0;
  float SIZE;  
  static int j = 1,jj =2;
  for(j = 1; j <= (size - 3) ; j = j + 2)     // calculateing sum of the multiple of four    // 
  {
    sum  += arr[j]*4;
  }

  for(jj = 2 ; jj <= (size - 2); jj = jj + 2)   // calculating sum of the multiple of two
  {
    sum  += arr[jj]*2;
  }
  
  if(j > size - 3)
  j = 1;
  
  if(jj > size - 2)
  jj = 2;  

  //adding first and last element to the sum
  sum  += arr[0] + arr[size-1];
  SIZE = float(size);
  sum = sum*ms*0.01/(SIZE*3.0) ;
  //difference between lower and upper limit is 10 ms and there is 10 samples (1/3 from formula) // 10ms interval 
  return  sum; // *0.00333333
}


bool checkChange (bool arr[], int size)  // if return 1 then there is change  ,if return 0 then there is no change
{ 
  for (int i =0; i < size - 1; i++ )
  {
    if(arr[i] != false)
      return true;
  }
  return false;
}
