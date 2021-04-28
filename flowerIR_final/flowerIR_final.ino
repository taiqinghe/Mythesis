/*
********************************************************************************
  STEP motor of stepper motor that make stepper motor running/stoping (High/Low)
  DIR motor of stepper motor that make stepper motor positive/negative (High/Low)

  stepping motor microstepping
  full step 0 0 0
  1/16 step 1 1 1
********************************************************************************
*/
#include <Servo.h>
#include <Wire.h>
#include <Stepper.h>
#define slave_address 0x04
Servo servo_0; //肩膀
Servo servo_1; //手肘
Servo servo_2; //手腕
Servo servo_3;//夾爪
Servo servo_4;//flowertable
const int servoPin0 = 6;
const int servoPin1 = 5;
const int servoPin2 = 4;
const int servoPin3 = 3;
const int servoPin4 = 2;
//stepper motor setting
Stepper mstepper(2048, 29, 25, 27, 23);
const int dirPin1 = 22;
const int stepPin1 = 24;    //under robotic arm and move it  
const int microStep = 26;   //stepper1 is microstep control
const int enableStep1 = 28;//enable stepper1;running/stoping (Low/High)

const int endStepPin1 = 30;//control stepper motor limited location

const int buttonPin = 7;
const int ledPin = 13;
unsigned long currentTime = millis();
unsigned long previousTime;
int mode = 0,slideback;
int stat[4];
int arm[4]={2100,1550,1920,1500}; //初始化
int insert[3]={1500,1500,1100};//插花姿態
int lined[3]={1524,1238,2160};//取花姿態
int gripper[3]={1500,1300,1450};//放鬆1500,夾緊1300,插花結束的鬆開
//int data[10]={27,50,76,101,127,153,179,205,231,255};//需要修改
int endStep1,endStep2,number = 0;

boolean buttonState =  LOW;//第一個狀態
boolean lastbuttonState = LOW;//第二個狀態
boolean ledState = LOW;//power start
boolean armState = false;
boolean servoZero = false;
boolean stepZero = false;

boolean servoInsert = false;
boolean servoLined = false;
boolean instat = false;
boolean instat2 = false;
boolean backstat = false;
boolean backstat2 = false;
boolean armSlide = false;
boolean armSlide2 = false;
boolean inSlide = false;
//i2c
int rpidata,x = 0,d = 0,count = 0,y = 0;
int dataMode = 0,value;
int coordinate[10]={0};//與data一樣
boolean rpibutton = false;
boolean ledState1 = false;
boolean ledState2 = false;

void debounce();
void servoMotor( int Schulter, int Ellbogen, int Hand, int Gripper );
void stepper(const int dirPin,const int stepPin,boolean dir,int steps);
/*****************************************************************/
void setup() {

  servo_0.attach(servoPin0);
  servo_1.attach(servoPin1);
  servo_2.attach(servoPin2,700,2200);
  servo_3.attach(servoPin3,500,2400);
  servo_4.attach(servoPin4,500,2400);
  servo_4.writeMicroseconds(1580);//1580
  mstepper.setSpeed(12);
  pinMode(endStepPin1,INPUT); // Endstop switch
  pinMode(stepPin1,OUTPUT); // Stepping motor1's step pin
  pinMode(dirPin1,OUTPUT);  // stepping motor1's dir pin
  pinMode(microStep,OUTPUT);
  pinMode(enableStep1,OUTPUT);
  
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  digitalWrite(ledPin, ledState);
  digitalWrite(microStep,HIGH);//LOW :full； HIGH: 1/16
  digitalWrite(enableStep1,LOW);
  
  Wire.begin(slave_address);  
  Wire.onRequest(requestData);//回傳給RPI 
  Wire.onReceive(receiveData);//接收RPI data
  
  Serial.begin(9600);    

//  for(int i = 0;i < 10;i++) {
//      data[i]=data[i]*51; //影像辨識出的位置與實際夾取位置插了51倍
//      //Serial.print("data[");Serial.print(i);Serial.print("]: ");Serial.println(data[i]);
//  }
  //初始化 肩膀arm0=2100,手肘arm1=2200,手腕arm2=1620
  for(int i = 0;i < 3;i++) stat[i] = arm[i];
  servoMotor(stat[0],stat[1],stat[2],gripper[0]);//初始化 夾花放鬆1500,夾花夾緊1300,取花放鬆1600  
  Serial.println("TaiQi robotic arm **Final3x**!!");
}
/*****************************************************************/
/*****************************************************************/
void loop() {
//  Serial.print("endStep 1 : ");Serial.println(endStep1);
//  Serial.print("endStep 2 : ");Serial.println(endStep2);
  endStep1 = digitalRead(endStepPin1); //end stop : open/close (1 and 0) 
  digitalWrite(microStep,LOW);
  if(stepZero == false && endStep1 == 1){
    Serial.println("Step Motor Return To Zero!!"); 
    Serial.print("endStep 1 : ");Serial.println(endStep1);
    stepper(dirPin1,stepPin1,HIGH,100);
  }         
  else {
    digitalWrite(enableStep1,HIGH);
    //Serial.println("Stepper Motor is Zero!!"); 
    stepZero = true;
  }
  //Servo Motor Return To Zero 
  if(servoZero == false) {
    Serial.println("Servo Motor Return To Zero!!"); 
    //初始化 肩膀arm0=2100,手肘arm1=2200,手腕arm2=1620
    for(int i = 0;i < 3;i++) stat[i] = arm[i];
    servoMotor(stat[0],stat[1],stat[2],gripper[0]);//初始化 夾花放鬆1500,夾花夾緊1300,取花放鬆1600     
    servoZero = true;
  }
/****************************************************/
/*初始化結束，工作開始*/
/****************************************************/ 
  number = 0;
  debounce();
//  Serial.print("ledState1 :");Serial.println(ledState1);
//  Serial.print("ledState2 :");Serial.println(ledState2);
/**i2c**
ledState1 ledState2
 0 0  初始點、中繼點
 0 1  接收資料
 1 0  將RPI數值重新更新
**/
  if(ledState1 == false && ledState2 == false){         
    dataMode = 0;//收到10ㄍ資料 等待工作結束
    if(rpibutton == true) {
       //重啟RPI更新
       Serial.println("wait 20s put new flowers");
      delay(20000);
      dataMode = 2;
      Serial.println("wait 20s put new flowers finished");
      Serial.print("dataMode = ");Serial.println(dataMode);
      rpibutton = false;
         
    }
    delay(100);
    x = 0;
  }
  if(ledState1 == false && ledState2 == true) {
    dataMode =  1;//接收資料
    if(x == 10) ledState2 = false;
  }
  
/**i2c**/
  while(ledState1) {  
    Serial.println("Working up"); //doing job    
    Serial.print("stat[0] PWM = ");Serial.println(stat[0]);
    Serial.print("stat[1] PWM = ");Serial.println(stat[1]);   
    Serial.print("stat[2] PWM = ");Serial.println(stat[2]);
    for(int i =0;i<10;i++){ 
      Serial.print("coordinate[");Serial.print(i);Serial.print("] = ");Serial.println(coordinate[i]);
    }
/****************************************************/
/*執行動作*/
/*case0: 最終結束位置，但夾爪鬆緊不定*/
/*case1: 插花開始(向下)*/
/*case2: 插花結束(向上)*/
/*case3: 取花開始(向下)*/
/*case4: 取花結束(向上)*/
/****************************************************/
    
    if(number < 10){
      if( coordinate[number] > 0 ){
        Serial.print("number : ");Serial.println(number);
        Serial.print("coordinate[");Serial.print(number);Serial.print("] = ");Serial.println(coordinate[number]);
        Serial.print("dataMode :");Serial.println(dataMode);
//        Serial.print("armSlide 1: ");Serial.println(armSlide);
//        Serial.print("armSlide2 1: ");Serial.println(armSlide2);
//        Serial.print("servoLined 1: ");Serial.println(servoLined);
//        Serial.print("servoInsert 1: ");Serial.println(servoInsert);
//        Serial.print("instat 1: ");Serial.println(instat);
//        Serial.print("instat2 1: ");Serial.println(instat2);
//        Serial.print("backstat 1: ");Serial.println(backstat);
//        Serial.print("backstat2 1: ");Serial.println(backstat2);

        if(number == 0 || inSlide == true) mode = 0;
        if(armSlide == true) mode = 5;//ARM滑軌移動去取花台          
        if(servoLined == true){ //取花
          digitalWrite(enableStep1,HIGH);
          Serial.println("servoLined yes ");
          mode = 3;
          if(instat2 == true) mode = 4;
          if(instat2 == true && backstat2 == true) mode = 6;//ARM滑軌移動去插花台                            
        }
        if( servoInsert == true){//插花
          digitalWrite(enableStep1,HIGH);
          Serial.println("servoInsert yes ");
          mode = 1;
          if( instat == true ) mode = 2;
          if(instat ==true && backstat ==true){
            number+=1;
            mode = 0;
            inSlide = true;
                         
          }
        }
      } else number+=1;
    } 
    else {
      ledState1 = false;
      //rpibutton = true;
    }
/****************************************************/
/****************************************************/
    switch(mode){
      case 0://初始化 肩膀arm0=2100,手肘arm1=2200,手腕arm2=1620
            Serial.println("mode = 0");
            digitalWrite(enableStep1,HIGH);
            armSlide = true;
            servoInsert = false;
            instat = false;
            instat2 = false;
            backstat = false;
            backstat2 = false;            
            for(int i = 0;i < 3;i++) stat[i] = arm[i];         
            servoMotor(stat[0],stat[1],stat[2],gripper[0]);
            Serial.println("mode = 0 servoInsert");
/***插花平台***/                      
              if(number < 10){
                if(coordinate[number] > 0){
                  if(number < 4){ 
                    if(number != 0) mstepper.step(-512);
                    servo_4.writeMicroseconds(1540); 
                  }
                  if(number == 4) {
                    mstepper.step(-256);
                    servo_4.writeMicroseconds(1520);                
                  }
                  if(number > 4 && number < 8){
                    mstepper.step(-512);
                    servo_4.writeMicroseconds(1520);                
                  }
                  if(number == 8){
                    mstepper.step(-256);
                    servo_4.writeMicroseconds(1500);
                  }
                  if(number == 9){
                    mstepper.step(-512);
                    servo_4.writeMicroseconds(1500);
                  }                  
                }
              }
              else {
                ledState1 =  false;//關閉程式
                rpibutton = true;
                for(int i = 1500; i <= 1580;i++){
                  servo_4.writeMicroseconds(i);
                }                
              }
/***插花平台end***/          
            break;
      case 1://插花狀態
            Serial.println("mode = 1");//開始向下插花
            stat[2]-=20;
            servoMotor(stat[0],stat[1],stat[2],gripper[1]);  
            for(int i = 0;i < 200;i++){ 
              stat[0]-=1,stat[1]-=1,stat[2]-=1; 
              servoMotor(stat[0],stat[1],stat[2],gripper[1]);              
              //stat[3]={1900,2000,1400} 
            }
            for(int i =0;i < 200;i++){
              stat[0]-=1,stat[2]-=2; 
              if(i%4 == 0) stat[1]+=1;
              servoMotor(stat[0],stat[1],stat[2],gripper[1]);               
 
            }
            Serial.println("flower arrangement : insert");
            delay(500);
            for(int i = 0;i < 50;i++){
              stat[0]-=2,stat[1]+=1,stat[2]-=2;
              servoMotor(stat[0],stat[1],stat[2],gripper[1]);                         
            }
            stat[0] = 1500,stat[1] = 1500,stat[2] = 1000;
            servoMotor(stat[0],stat[1],stat[2],gripper[1]);          
            for(int i = 0;i < 3;i++) stat[i] = insert[i];
            servoMotor(stat[0],stat[1],stat[2],gripper[1]); 
            delay(1000);           
            if(stat[0] == insert[0] && stat[1] == insert[1] && stat[2] == insert[2] ){
//              for(int i = 0;i < 200;i++){
//                if(i%2 == 0) stat[0]+=10;
//                else stat[0]-=10;
//                servoMotor(stat[0],stat[1],stat[2],gripper[1]);
//              }
              stat[3] = gripper[1];
              instat = true;
            }             
            break;
      case 2://初始化 插花結束回歸
            Serial.println("mode = 2");
            for(int i = 0;i < 150;i++){//夾具鬆開
              stat[3]+=1;
              delay(3);
              servoMotor(stat[0],stat[1],stat[2],stat[3]);//gripper[3]={1500,1350,1450};
            }
            for(int i = 0;i < 100;i++){
              stat[0]+=2,stat[2]+=3;
              servoMotor(stat[0],stat[1],stat[2],gripper[2]);             
            }
            for(int i = 0;i < 400;i++){
              stat[0]+=1,stat[2]+=1;
              servoMotor(stat[0],stat[1],stat[2],gripper[2]);            
            }
            for(int i = 0;i < 3;i++) stat[i] = arm[i];
            servoMotor(stat[0],stat[1],stat[2],gripper[0]);
            if(stat[0] == arm[0] && stat[1] == arm[1] && stat[2] == arm[2] ) {
              backstat = true;
            }                         
            break; 
      case 3://取花姿態 //初始化
            //Serial.println("mode = 3");//下去取花
           for( int i = 0;i < 4;i++){
              stat[0]-=24;
              if(i%2 == 1) { stat[1]-=26,stat[2]+=20;}
              servoMotor(stat[0],stat[1],stat[2],gripper[0]);
              if(i%2 == 1) delay(50); 
           }            
           for( int i = 0;i < 20;i++){
              stat[0]-=24;
              if(i%2 == 1) { stat[1]-=26,stat[2]+=20;}
              servoMotor(stat[0],stat[1],stat[2],gripper[0]);
              if(i%2 == 1) delay(50); 
           }
            delay(800);
            for(int i = 0;i < 3;i++) stat[i] = lined[i];
            servoMotor(stat[0],stat[1],stat[2],gripper[0]);//gripper[2]={1500,1300};//初始化 放鬆1500,夾緊1350
            delay(500);
            stat[3] = gripper[0];
            if( stat[0] == lined[0] && stat[1] == lined[1] && stat[2] == lined[2] ){
              stat[3] = gripper[0];
              instat2 = true;              
            }              
           break;
      case 4://取花起來的動作
            Serial.println("mode = 4");
            for(int i = 0;i <150;i++){
              stat[3]-=1;
              delay(3);
              servoMotor(stat[0],stat[1],stat[2],stat[3]);
            }
            stat[3] = gripper[1];
            delay(500);
            for( int i = 0;i < 14;i++){
              stat[0]+=24;
              if(i%2 == 0) stat[1]+=26,stat[2]-=20;
              servoMotor(stat[0],stat[1],stat[2],gripper[1]);
              if(i%2 == 0) delay(300);
            }
            digitalWrite(enableStep1,LOW);
            digitalWrite(microStep,HIGH);//LOW :full； HIGH: 1/16
            stepper(dirPin1,stepPin1,HIGH,coordinate[number]);//取花起始點
            delay(500);            
            for( int i = 0;i < 10;i++){
              stat[0]+=24;
              if(i%2 == 0) stat[1]+=26,stat[2]-=20;
              servoMotor(stat[0],stat[1],stat[2],gripper[1]);
              if(i%2 == 0) delay(300);
            }             
            if( stat[0] == arm[0] && stat[1] == arm[1] && stat[2] == arm[2] ){
              delay(500);
              stat[3] = gripper[0];
              backstat2 = true;              
            }                     
            break;
      case 5://滑軌移動去取花            
            digitalWrite(enableStep1,LOW);
            digitalWrite(microStep,LOW);//LOW :full； HIGH: 1/16
            stepper(dirPin1,stepPin1,LOW,993);//取花起始點
            delay(500);
            digitalWrite(microStep,HIGH);//LOW :full； HIGH: 1/16
            stepper(dirPin1,stepPin1,LOW,coordinate[number]);
            servoLined = true;
            digitalWrite(enableStep1,HIGH);
            break;
      case 6://滑軌回到插花平台
            //slideback =(int) round(993+coordinate[number]/16);
            digitalWrite(microStep,LOW);//LOW :full； HIGH: 1/16
            digitalWrite(enableStep1,LOW);
            endStep1 = digitalRead(endStepPin1); //end stop : open/close (1 and 0)
            Serial.print("endStep 1 : ");Serial.println(endStep1);           
            if(endStep1 == 1) stepper(dirPin1,stepPin1,HIGH,993);
            else {                          
              servoLined = false;
              servoInsert = true; 
            }
            break;                 
    } 
  }//led = high 
}//loop
/*****************************************************************/
/*****************************************************************/
void debounce() {//按鍵防彈跳
  unsigned long currentMillis = millis(); 
  unsigned long lastDebounceTime = 0;
  unsigned long debounceDelay = 200; //delay
  int reading = digitalRead(buttonPin);
  //Serial.print("buttonPin = ");Serial.println(reading);
  if( reading != lastbuttonState){
    lastDebounceTime = millis();
  }
  if(currentMillis - lastDebounceTime > debounceDelay){
    if(reading != buttonState){
      buttonState = reading;
      if( buttonState == HIGH) {
        ledState = !ledState;
        if(y%2 == 0){
          rpibutton = false;
          ledState1 = false;
          ledState2 = true;  
        }        
        if(y%2 == 1){
          ledState1 = true;
          ledState2 = false;
        }

        y+=1;     
      }
    }
  }
  digitalWrite(ledPin,ledState);
  lastbuttonState = reading;
} 
/*****************************************************************/
void servoMotor( int Schulter, int Ellbogen, int Hand, int Gripper ){
  //Serial.println("Servo Motor RUN!!");
   int i =800;
   servo_3.writeMicroseconds(Gripper);
   delayMicroseconds(i);
   servo_2.writeMicroseconds(Hand);
   delayMicroseconds(i);
   servo_1.writeMicroseconds(Ellbogen);
   delayMicroseconds(i);
   servo_0.writeMicroseconds(Schulter);
   delayMicroseconds(i);   
}
/*****************************************************************/
void stepper(const int dirPin,const int stepPin,boolean dir,int steps){
  digitalWrite(dirPin,dir); // Enables the motor to move in a particular direction
  for(int x = 0; x < steps; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500); 
  }
}
/*****************************************************************/
void receiveData(int count){//接收RPI data
  while( Wire.available() ){
    rpidata = Wire.read();
    if(x < 10){    
      Serial.print("RPI data received: ");Serial.println(rpidata);
      if(rpidata > 160)coordinate[x] =rpidata*42;
      else coordinate[x] =rpidata*50;
      x+=1;      
      delay(1000);
    }
  }  
}
/*******************************************************/
void requestData(){//回傳給RPI    

  switch(dataMode){
    case 0:     ///等連線建立 接收到0的訊號
      Wire.write(0);
    break;
    case 1:     //當建立後傳"1"訊息 請求資料接收
      Wire.write(1);
      break;
    case 2:     //接收完成後重製RPI內數值
      Wire.write(2);
      break;
    default:{
      Wire.write(-1);
      Serial.println("Error");
    }
  }
}
