#include<Wire.h>
#include<Servo.h>
/*
1 2 3
4 5 6
7 8 9
*/
#define SLAVE_ADDR 0x04
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

static int pos_h=90;
static int pos_v=90;

void setup()
{
	Wire.begin(SLAVE_ADDR);                // join i2c bus with address #4
	Wire.onReceive(receiveEvent); // register event

	Serial.begin(9600);           // start serial for output

	servo_h.attach(9);			//horizen at 9
	servo_v.attach(10);			//vertical at 10
	servo_h.write(pos_h);
	servo_v.write(pos_v);
}

void loop()
{
	delay(100);
}

void receiveEvent(int howMany)
{
	int x = Wire.read();    // receive byte as an integer
	Serial.println(x);         // print the integer

	if(x==A) pos_h =110;
        if(x==B) pos_h =105;
        if(x==C) pos_h =100;
        if(x==D) pos_h =95;
        if(x==E) pos_h =90;
        if(x==F) pos_h =85;
        if(x==G) pos_h =80;
        if(x==H) pos_h =75;
        if(x==I) pos_h =65;
        
        if(x==R) pos_v =90;
        if(x==Q) pos_v =86;
        if(x==P) pos_v =82;
        if(x==O) pos_v =78;
        if(x==N) pos_v =74;
        if(x==M) pos_v =70;
        if(x==L) pos_v =66;
        if(x==K) pos_v =62;
        if(x==J) pos_v =58;
        
	servo_h.write(pos_h);
	servo_v.write(pos_v);
}



