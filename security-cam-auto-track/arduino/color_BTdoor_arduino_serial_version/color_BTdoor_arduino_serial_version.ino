#include <SoftwareSerial.h>
#include<Wire.h>
#include<Servo.h>
/*
1 2 3
4 5 6
7 8 9
*/
SoftwareSerial I2CBT(2,4);//定義PIN2及PIN4分別為RX及TX腳位
int LED1 = 7;
int LED2 = 8;

#define SLAVE_ADDR 0x04
#define left '4'
#define right '6'
#define up '2'
#define down '8'
#define FIRE '5'

#define A 'a'
#define B 'b'
#define C 'c'
#define D 'd'
#define E 'e'
#define F 'f'
#define G 'g'
#define H 'h'
#define I 'i'

#define J 'j'
#define K 'k'
#define L 'l'
#define M 'm'
#define N 'n'
#define O 'o'
#define P 'p'
#define Q 'q'
#define R 'r'





Servo servo_h;
Servo servo_v;
Servo servo_f;

static int pos_h=90;
static int pos_v=90;
static int pos_f=90;

byte cmmd[20];
int insize;

void setup()
{
	Serial.begin(9600);           // start serial for output
        I2CBT.begin(9600);      //bluetooth baud rate
        pinMode(7, OUTPUT);  //設定 pin7 為輸出，LED1就接在這（開門）
        pinMode(8, OUTPUT);  //設定 pin8 為輸出，LED2就接在這（關門）
	servo_h.attach(9);			//horizen at 9
	servo_v.attach(10);			//vertical at 10
        servo_f.attach(11);
	servo_h.write(pos_h);
	servo_v.write(pos_v);
        servo_f.write(pos_f);
        
}
void(* resetFunc) (void) = 0;//declare reset function at address 0
void loop()
{
        
        while(Serial.available())  receiveEvent();       
        while(I2CBT.available()) bluetooth(); 
 //        if (I2CBT.available()>0)  bluetooth();     
           
	  delay(100);
}

void receiveEvent()
{
	int x = Serial.read();    // receive byte as an integer
	Serial.println(x);         // print the integer

        if(x==A) pos_h =110;
        if(x==B) pos_h =105;
        if(x==C) pos_h =100;
        if(x==D) pos_h =95;
        if(x==E) pos_h =90;
        if(x==F) pos_h =85;
        if(x==G) pos_h =80;
        if(x==H) pos_h =75;
        if(x==I) pos_h =70;
        
        if(x==R) pos_v =90;
        if(x==Q) pos_v =86;
        if(x==P) pos_v =82;
        if(x==O) pos_v =78;
        if(x==N) pos_v =74;
        if(x==M) pos_v =70;
        if(x==L) pos_v =66;
        if(x==K) pos_v =62;
        if(x==J) pos_v =58;
        
        if(x==FIRE) fire();
        
// /* face tracking and keyboard command
        if(x==up && pos_v<=175) pos_v --;
	if(x==down && pos_v>=5) pos_v ++;
	if(x==left && pos_h>=5) pos_h ++;
	if(x==right && pos_h<=175) pos_h --;
//*/ 
        

 //       pos_h=x;
	servo_h.write(pos_h);
	servo_v.write(pos_v);
        
}

void bluetooth()
{
 while(1)
 {
/*
read message from bluetooth
*/
    if ((insize=(I2CBT.available()))>0){  //讀取藍牙訊息
       Serial.print("input size = "); 
       Serial.println(insize);
       for (int i=0; i<insize; i++){
         Serial.print(cmmd[i]=char(I2CBT.read()));
         Serial.print(" "); 
       }
       Serial.println("  "); 
    }  
///*    
      if (cmmd[0]==115) {               //ASCII CODE   passwd:"sstu"    
        if (cmmd[1]==116) {
          if (cmmd[2]==117) {
            digitalWrite(LED1, HIGH);
            cmmd[0]=96;               //取代"aabc"的暫存字元,才能off LED1
            delay(300);
            digitalWrite(LED1, LOW);   //熄滅LED1
            delay(300);
            Serial.print(" reseting ");
             resetFunc(); //call reset ;
             
            }
          }
         }//if
         
  else if (cmmd[0]==116) {              //ASCII CODE    passwd:"ttsu"          
            if (cmmd[1]==115) {
              if (cmmd[2]==117) {
               digitalWrite(LED2, HIGH);
               cmmd[0]=96;               //取代"ttsu"的暫存字元,才能off LED2   
               delay(300);
              digitalWrite(LED2, LOW);   //熄滅LED2 
              delay(300);
              Serial.print(" reseting ");
              resetFunc(); //call reset ;
              
            }
           }
          }//else if     
    
 }
 
}
//*
void fire()
{
  for(pos_f = 30; pos_f < 120; pos_f += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    servo_f.write(pos_f);              // tell servo to go to position in variable 'pos' 
    delay(2);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos_f = 120; pos_f>=31; pos_f-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    servo_f.write(pos_f);              // tell servo to go to position in variable 'pos' 
    delay(2);                       // waits 15ms for the servo to reach the position 
  } 
  
}
//*/
