
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include <util/twi.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "mpu6050_reg.h"

#define PI 3.14159265

#define TWISendStart()		(TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN))
#define TWISendStop()		(TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN))
#define TWISendTransmit()	(TWCR = (1<<TWINT)|(1<<TWEN))
#define TWISendACK()		(TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA))
#define TWISendNACK()		(TWCR = (1<<TWINT)|(1<<TWEN))

#define TWI_FREQ 12 // TWI bit rate
#define NO_PRESCALING 0
#define TWI_STATUS	(TWSR & 0xF8) // Get TWI status
#define TXMAXBUFLEN 16 // Transmit buffer length
#define RXMAXBUFLEN 16 // Receive buffer length

#define USART_BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

volatile uint8_t TWITransmitBuffer[TXMAXBUFLEN]; // Global transmit buffer
volatile uint8_t TWIReceiveBuffer[RXMAXBUFLEN]; // Global receive buffer

volatile int TXBuffIndex; // Index of the transmit buffer. Is volatile, can change at any time.
volatile int RXBuffIndex; // Current index in the receive buffer

int TXBuffLen; // The total length of the transmit buffer
int RXBuffLen; // The total number of bytes to read (should be less than RXMAXBUFFLEN)

int16_t ACCEL_XOUT;
int16_t ACCEL_YOUT;
int16_t ACCEL_ZOUT;

int16_t ACCEL_XOUT_OFFSET;
int16_t ACCEL_YOUT_OFFSET;
int16_t ACCEL_ZOUT_OFFSET;

int16_t ACCEL_XOUT_VALUE;
int16_t ACCEL_YOUT_VALUE;
int16_t ACCEL_ZOUT_VALUE;

uint8_t ACCEL_XOUT_L;
uint8_t ACCEL_XOUT_H;
uint8_t ACCEL_YOUT_L;
uint8_t ACCEL_YOUT_H;
uint8_t ACCEL_ZOUT_L;
uint8_t ACCEL_ZOUT_H;

float GYRO_XOUT_OFFSET;
float GYRO_YOUT_OFFSET;
float GYRO_ZOUT_OFFSET;

uint8_t GYRO_XOUT_L;
uint8_t GYRO_XOUT_H;
uint8_t GYRO_YOUT_L;
uint8_t GYRO_YOUT_H;
uint8_t GYRO_ZOUT_L;
uint8_t GYRO_ZOUT_H;

int GYRO_XOUT;
int GYRO_YOUT;
int GYRO_ZOUT;

float dt = 0.05f;

float GYRO_XANGLE;
float GYRO_YANGLE;
float GYRO_ZANGLE;

#define POLE_OFF			0b00000000

#define POLE_POS_NORTH		0b00000011
#define POLE_POS_NORTHEAST	0b00000010
#define POLE_POS_EAST		0b00000110
#define POLE_POS_SOUTHEAST	0b00000100

#define POLE_POS_SOUTH		0b00001100
#define POLE_POS_SOUTHWEST	0b00001000
#define POLE_POS_WEST		0b00001001
#define POLE_POS_NORTHWEST	0b00000001

#define POLE_POS_NORTH2		0b00001100
#define POLE_POS_EAST2		0b00011000
#define POLE_POS_SOUTH2		0b10010000
#define POLE_POS_WEST2		0b10000100
		
typedef struct struct_compass{
	uint8_t orient;
	struct struct_compass* prev;
	struct struct_compass* next;
} struct_compass;

struct_compass *NODE_NEEDLE, NODE_NORTH, NODE_NORTHEAST, NODE_EAST, NODE_SOUTHEAST, NODE_SOUTH, NODE_SOUTHWEST, NODE_WEST, NODE_NORTHWEST, NODE_OFF;
struct_compass *NODE_NEEDLE2, NODE_NORTH2, NODE_EAST2, NODE_SOUTH2, NODE_WEST2;

int stepper_turning = 0;
int turnThreshold = 0;
bool turning = false;
	
float targetYAngle = 0.0;
float yProportional = 0.0;
float pidThreshold = 0.0;
int delayTime = 1;

bool timer2tick = false;

char accelChar[28];
char gyroChar[28];
char dgyroChar[28];

char c;
int accelxint, accelyint, accelzint;

double accelxf1, accelxf2, accelyf1, accelyf2, accelzf1, accelzf2;
int accelxi1, accelxi2, accelyi1, accelyi2, accelzi1, accelzi2;

double pitchf1, pitchf2, rollf1, rollf2, yawf1, yawf2;
int pitchi1, pitchi2, rolli1, rolli2, yawi1, yawi2;

double dpitchf1, dpitchf2, drollf1, drollf2, dyawf1, dyawf2;
int dpitchi1, dpitchi2, drolli1, drolli2, dyawi1, dyawi2;

int pwmMin = 0x07D0;
int pwmMax = 0x0F9F;
int pwmOff = 0x0000;
int pwmMotA = 2200;
int pwmMotB = 2200;

float rollAngle = 0.0;
float pitchAngle = 0.0;
float yawAngle = 0.0;

float prevrollAngle = 0.0;
float prevpitchAngle = 0.0;
float prevyawAngle = 0.0;

float deltarollAngle = 0.0;
float deltapitchAngle = 0.0;
float deltayawAngle = 0.0;

bool keepTurning = false;
int stepperMotorTurn = 0;
int stepperMotor2Turn = 0;

void USART_init(void){
	
	UBRR0H = (uint8_t)(BAUD_PRESCALER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALER);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00);
}

unsigned char USART_receive(void){
	
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
	
}

void USART_send(char data){
	
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
	
}

void USART_putstring(char* StringPtr){
	
	while(*StringPtr != 0x00){
		USART_send(*StringPtr);
	StringPtr++;}
}

char hc_05_bluetooth_receive_byte(void)
{
	return USART_receive();
}



unsigned char i2c_readAck(void)
{
	TWISendACK();
	while(!(TWCR & (1<<TWINT))){_delay_us(1);}
	return TWDR;
}

unsigned char i2c_readNak(void)
{
	TWISendNACK();
	while(!(TWCR & (1<<TWINT))){_delay_us(1);}
	return TWDR;
}

void i2c_start(unsigned char address)
{
	TWISendStart();
	while(!(TWCR & (1<<TWINT))){_delay_us(1);}
	
	TWDR = address;
	TWISendTransmit();
	while(!(TWCR & (1<<TWINT))){_delay_us(1);}
}

void i2c_write( unsigned char data )
{
	TWDR = data;
	TWISendTransmit();
	while(!(TWCR & (1<<TWINT))){_delay_us(1);}
}

void TWIInit()
{
	TWSR = NO_PRESCALING;
	TWBR = (F_CPU /(16 + TWI_FREQ*2));
	TWCR = (1<<TWEN);
}

uint8_t mpu6050_readBytes(uint8_t regAddr, uint8_t length) {
	uint8_t i = 0;
	
	if(length > 0) {
		
		i2c_start(MPU6050_ADDR|I2C_WRITE);
		_delay_us(1);
		i2c_write(regAddr); //request register
		_delay_us(1);
		i2c_start(MPU6050_ADDR|I2C_READ); //read data
		_delay_us(1);
		
		for(i=0; i<length; i++) {
			if(i==length-1)
			{
				TWIReceiveBuffer[i] = i2c_readNak();
				_delay_us(1);
			}
			else {
				TWIReceiveBuffer[i] = i2c_readAck();
				_delay_us(1);
			}
		}
		TWISendStop();
	}
	return TWIReceiveBuffer[0];
}

int8_t mpu6050_readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    int8_t count = 0;
    uint8_t b = 0;
	
    if(length > 0) {
		if ((count = mpu6050_readBytes(regAddr, 1)) != 0) {
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			b &= mask;
			b >>= (bitStart - length + 1);
		}
    }
    return b;
}

void mpu6050_writeBytes(uint8_t regAddr, uint8_t length, uint8_t* data) {
	if(length > 0) {
		//write data
		i2c_start(MPU6050_ADDR|I2C_WRITE);
		_delay_us(1);
		i2c_write(regAddr);
		_delay_us(1);
		for (uint8_t i = 0; i < length; i++) {
			i2c_write((uint8_t) data[i]);
			_delay_us(1);
		}
		TWISendStop();
	}
}

void mpu6050_writeData(uint8_t regAddr, uint8_t byteData) {
	i2c_start(MPU6050_ADDR | I2C_WRITE);
	_delay_us(1);
	i2c_write(regAddr);
	_delay_us(1);
	i2c_write((uint8_t) byteData);
	_delay_us(1);
	TWISendStop();
}

void mpu6050_testConnection(){
	mpu6050_readBytes(MPU6050_RA_WHO_AM_I, 1);
} 

void mpu6050_initialize(){
	
	mpu6050_writeData(MPU6050_RA_SMPLRT_DIV, 0x07);					//Sets sample rate to 8000/1+7 = 1000Hz
	mpu6050_writeData(MPU6050_RA_CONFIG, 0x00);						//Disable FSync, 256Hz DLPF
	mpu6050_writeData(MPU6050_RA_GYRO_CONFIG, 0x08);			//Disable gyro self tests, scale of 500 degrees/s
	mpu6050_writeData(MPU6050_RA_ACCEL_CONFIG, 0x00);				//Disable acceleration self tests, scale of +-2g, no DHPF
	mpu6050_writeData(MPU6050_RA_FF_THR, 0x00);						//Free fall threshold of |0mg|
	mpu6050_writeData(MPU6050_RA_FF_DUR, 0x00);						//Free fall duration limit of 0
	mpu6050_writeData(MPU6050_RA_MOT_THR, 0x00);					//Motion threshold of 0mg
	mpu6050_writeData(MPU6050_RA_MOT_DUR, 0x00);					//Motion duration of 0s
	mpu6050_writeData(MPU6050_RA_ZRMOT_THR, 0x00);					//Zero motion threshold
	mpu6050_writeData(MPU6050_RA_ZRMOT_DUR, 0x00);					//Zero motion duration threshold
	mpu6050_writeData(MPU6050_RA_FIFO_EN, 0x00);					//Disable sensor output to FIFO buffer
	
	//AUX I2C setup
	//Sets AUX I2C to single master control, plus other config
	mpu6050_writeData(MPU6050_RA_I2C_MST_CTRL, 0x00);
	
	//disable sleep mode
	mpu6050_writeData(MPU6050_RA_PWR_MGMT_1, 0x00);

	//Setup AUX I2C slaves
	mpu6050_writeData(MPU6050_RA_I2C_SLV0_ADDR, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV0_REG, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV0_CTRL, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV1_ADDR, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV1_REG, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV1_CTRL, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV2_ADDR, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV2_REG, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV2_CTRL, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV3_ADDR, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV3_REG, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV3_CTRL, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV4_ADDR, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV4_REG, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV4_DO, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV4_CTRL, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV4_DI, 0x00);
	
	//MPU6050_RA_I2C_MST_STATUS //Read-only
	//Setup INT pin and AUX I2C pass through
	mpu6050_writeData(MPU6050_RA_INT_PIN_CFG, 0x00);
	
	//Enable data ready interrupt
	mpu6050_writeData(MPU6050_RA_INT_ENABLE, 0x00);
	
	//Slave Output
	mpu6050_writeData(MPU6050_RA_I2C_SLV0_DO, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV1_DO, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV2_DO, 0x00);
	mpu6050_writeData(MPU6050_RA_I2C_SLV3_DO, 0x00);
	
	//More slave config
	mpu6050_writeData(MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
	
	//Reset sensor signal paths
	mpu6050_writeData(MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
	
	//Motion detection control
	mpu6050_writeData(MPU6050_RA_MOT_DETECT_CTRL, 0x00);
	
	//Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
	mpu6050_writeData(MPU6050_RA_USER_CTRL, 0x00);
	
	//Sets clock source to gyro reference w/ PLL
	mpu6050_writeData(MPU6050_RA_PWR_MGMT_1, 0x02);
	
	//Controls frequency of wake-ups in acceleration low power mode plus the sensor standby modes
	mpu6050_writeData(MPU6050_RA_PWR_MGMT_2, 0x00);
	
	//Data transfer to and from the FIFO buffer
	mpu6050_writeData(MPU6050_RA_FIFO_R_W, 0x00);
}

void mpu6050_getAccel(){
	ACCEL_XOUT_H = mpu6050_readBytes(MPU6050_RA_ACCEL_XOUT_H,1);
	ACCEL_XOUT_L = mpu6050_readBytes(MPU6050_RA_ACCEL_XOUT_L,1);
	ACCEL_YOUT_H = mpu6050_readBytes(MPU6050_RA_ACCEL_YOUT_H,1);
	ACCEL_YOUT_L = mpu6050_readBytes(MPU6050_RA_ACCEL_YOUT_L,1);
	ACCEL_ZOUT_H = mpu6050_readBytes(MPU6050_RA_ACCEL_ZOUT_H,1);
	ACCEL_ZOUT_L = mpu6050_readBytes(MPU6050_RA_ACCEL_ZOUT_L,1);

	ACCEL_XOUT = (((int16_t)ACCEL_XOUT_H << 8) | ACCEL_XOUT_L);
	ACCEL_YOUT = (((int16_t)ACCEL_YOUT_H << 8) | ACCEL_YOUT_L);
	ACCEL_ZOUT = (((int16_t)ACCEL_ZOUT_H << 8) | ACCEL_ZOUT_L);
}

void mpu6050_calibrateAccel(){
	
	int16_t ACCEL_XOUT_OFFSET_1000SUM = 0;
	int16_t ACCEL_YOUT_OFFSET_1000SUM = 0;
	int16_t ACCEL_ZOUT_OFFSET_1000SUM = 0;
	
	for (int i = 0; i < 1000; i++){
		
		mpu6050_getAccel();
		ACCEL_XOUT_OFFSET_1000SUM += ACCEL_XOUT;
		ACCEL_YOUT_OFFSET_1000SUM += ACCEL_YOUT;
		ACCEL_ZOUT_OFFSET_1000SUM += ACCEL_ZOUT - 16384;
		_delay_ms(1);
	}
	
	ACCEL_XOUT_OFFSET = (int16_t) floor(ACCEL_XOUT_OFFSET_1000SUM / 1000);
	ACCEL_YOUT_OFFSET = (int16_t) floor(ACCEL_YOUT_OFFSET_1000SUM / 1000);
	ACCEL_ZOUT_OFFSET = (int16_t) floor(ACCEL_ZOUT_OFFSET_1000SUM / 1000);
}

void mpu6050_readAccel(){
	mpu6050_getAccel();
	ACCEL_XOUT_VALUE = (ACCEL_XOUT-ACCEL_XOUT_OFFSET);///16384;
	ACCEL_YOUT_VALUE = (ACCEL_YOUT-ACCEL_YOUT_OFFSET);///16384;
	ACCEL_ZOUT_VALUE = (ACCEL_ZOUT-ACCEL_ZOUT_OFFSET);///16384;
}

void mpu6050_getGyroRates()
{
	GYRO_XOUT_H = mpu6050_readBytes(MPU6050_RA_GYRO_XOUT_H, 1);
	GYRO_XOUT_L = mpu6050_readBytes(MPU6050_RA_GYRO_XOUT_L, 1);
	GYRO_YOUT_H = mpu6050_readBytes(MPU6050_RA_GYRO_YOUT_H, 1);
	GYRO_YOUT_L = mpu6050_readBytes(MPU6050_RA_GYRO_YOUT_L, 1);
	GYRO_ZOUT_H = mpu6050_readBytes(MPU6050_RA_GYRO_ZOUT_H, 1);
	GYRO_ZOUT_L = mpu6050_readBytes(MPU6050_RA_GYRO_ZOUT_L, 1);

	GYRO_XOUT = (((int16_t)GYRO_XOUT_H << 8) | GYRO_XOUT_L);
	GYRO_YOUT = (((int16_t)GYRO_YOUT_H << 8) | GYRO_YOUT_L);
	GYRO_ZOUT = (((int16_t)GYRO_ZOUT_H << 8) | GYRO_ZOUT_L);
}

void mpu6050_calibrateGyros(){

	long GYRO_XOUT_OFFSET_1000SUM = 0;
	long GYRO_YOUT_OFFSET_1000SUM = 0;
	long GYRO_ZOUT_OFFSET_1000SUM = 0;
	
	for (int i = 0; i < 1000; i++){
		
		mpu6050_getGyroRates();

		GYRO_XOUT_OFFSET_1000SUM += GYRO_XOUT;
		GYRO_YOUT_OFFSET_1000SUM += GYRO_YOUT;
		GYRO_ZOUT_OFFSET_1000SUM += GYRO_ZOUT;

		_delay_ms(1);
	}

	GYRO_XOUT_OFFSET = (float) (GYRO_XOUT_OFFSET_1000SUM / 1000);
	GYRO_YOUT_OFFSET = (float) (GYRO_YOUT_OFFSET_1000SUM / 1000);
	GYRO_ZOUT_OFFSET = (float) (GYRO_ZOUT_OFFSET_1000SUM / 1000);
}

void mpu6050_readGyro(){
	mpu6050_getGyroRates();
	//GYRO SENSITIVITY AT 65.5 (500 degrees/s)
	GYRO_XOUT = (float) ((GYRO_XOUT-GYRO_XOUT_OFFSET) / 500);
	GYRO_YOUT = (float) ((GYRO_YOUT-GYRO_YOUT_OFFSET) / 500);
	GYRO_ZOUT = (float) ((GYRO_ZOUT-GYRO_ZOUT_OFFSET) / 500);

	GYRO_XANGLE += GYRO_XOUT * dt;
	GYRO_YANGLE += GYRO_YOUT * dt;
	GYRO_ZANGLE += GYRO_ZOUT * dt;
}

void stepper_turnMotor(){
	uint8_t orientation =  NODE_NEEDLE->orient;
	PORTC = orientation;
}

void stepper_stopMotor(){
	uint8_t orientation =  POLE_OFF;
	PORTC = orientation;
}

void stepper_turnMotor2(){
	uint8_t orientation =  NODE_NEEDLE2->orient;
	PORTD = orientation;
}

void stepper_stopMotor2(){
	uint8_t orientation =  POLE_OFF;
	PORTD = orientation;
}

void initializeRotors(){
	
	DDRB |= (1 << DDB1)|(1 << DDB2);
	// PB1 and PB2 is now an output

	TCCR1A |= (1 << COM1A1)|(1 << COM1B1);
	// set none-inverting mode

	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12)|(1 << WGM13) | (1 << CS11);
	// set Fast PWM mode using ICR1 as TOP prescaler 8
	
	ICR1 = 0x9C38;
	// set TOP to 16bit to 39992 20ms

	/*
	OCR1A = 0x07D0;
	// set PWM for 5% duty cycle @ 16bit (2000) 1 ms

	OCR1B = 0x0F9F;
	// set PWM for 10% duty cycle @ 16bit (3999) 2 ms
	*/
	
	OCR1A = pwmOff;
	// set PWM for 10% duty cycle @ 16bit (4000)

	OCR1B = pwmOff;
	// set PWM for 20% duty cycle @ 16bit (8000)

	_delay_ms(3000);
	
	OCR1A = pwmMax;
	OCR1B = pwmMax;
	
	_delay_ms(6000);
	
	OCR1A = pwmMin;
	OCR1B = pwmMin;
	
	
	_delay_ms(7000);
	
	OCR1A = pwmOff;
	OCR1B = pwmOff;
	
	_delay_ms(1);
	
	OCR1A = pwmMotA;
	OCR1B = pwmMotB;
	
}

void timer0_init()
{
    TCCR0A |= (1 << WGM01);
    OCR0A = 0xF9;
    TIMSK0 |= (1 << OCIE0A);
    TCCR0B |= (1 << CS01) | (1 << CS00);
}

void timer1_init()
{
	// set up timer with prescaler = 64 and CTC mode
	TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);
		
	// initialize counter
	TCNT1 = 0;
		
	// initialize compare value
	OCR1A = 24999;
		
	// enable compare interrupt
	TIMSK1 |= (1 << OCIE1A);
}

// this code sets up timer2 for a 1ms  @ 16Mhz Clock
void timer2_init()
{
	OCR2A = 0xF9;
	TCCR2A |= (1 << WGM21);
	TIMSK2 |= (1 << OCIE2A);
	TCCR2B |= (1 << CS21);
}

ISR(TIMER2_COMPA_vect)
{	
	timer2tick = true;
}

/*
ISR(TIMER1_COMPA_vect)
{
	// toggle led here
	timer0count++;
	
	if(timer0count > 10){
		PORTC ^= (1 << 0);
		timer0count = 0;
	}
}

ISR (TIMER0_COMPA_vect)
{
	//timer0tick = true;
}
*/

	
int main(void)
{
	sei();
	/*
	NODE_NORTH.orient = POLE_POS_NORTH;
	NODE_NORTH.prev = &NODE_WEST;
	NODE_NORTH.next = &NODE_EAST;

	NODE_EAST.orient = POLE_POS_EAST;
	NODE_EAST.prev = &NODE_NORTH;
	NODE_EAST.next = &NODE_SOUTH;

	NODE_SOUTH.orient = POLE_POS_SOUTH;
	NODE_SOUTH.prev = &NODE_EAST;
	NODE_SOUTH.next = &NODE_WEST;

	NODE_WEST.orient = POLE_POS_WEST;
	NODE_WEST.prev = &NODE_SOUTH;
	NODE_WEST.next = &NODE_NORTH;
		
	NODE_NEEDLE = &NODE_NORTH;

	NODE_NORTH2.orient = POLE_POS_NORTH2;
	NODE_NORTH2.prev = &NODE_WEST2;
	NODE_NORTH2.next = &NODE_EAST2;

	NODE_EAST2.orient = POLE_POS_EAST2;
	NODE_EAST2.prev = &NODE_NORTH2;
	NODE_EAST2.next = &NODE_SOUTH2;

	NODE_SOUTH2.orient = POLE_POS_SOUTH2;
	NODE_SOUTH2.prev = &NODE_EAST2;
	NODE_SOUTH2.next = &NODE_WEST2;

	NODE_WEST2.orient = POLE_POS_WEST2;
	NODE_WEST2.prev = &NODE_SOUTH2;
	NODE_WEST2.next = &NODE_NORTH2;
	
	NODE_NEEDLE2 = &NODE_NORTH2;
	*/
	
	USART_init();
	TWIInit();
	mpu6050_testConnection();
	mpu6050_initialize();
	mpu6050_calibrateAccel();
	mpu6050_calibrateGyros();
	
	
	// initialize timer
	//timer0_init();
	DDRB |= (1 << 3);
	PORTB = 0x00;
	
	DDRC = 0x0F;    // Enable output on all of the C pins
	PORTC = 0x00;            // Set them all to 0v
	
	DDRD = 0xFF;    // Enable output on all of the C pins
	PORTD = 0x00;            // Set them all to 0v
	
	
	PORTB |= (1<<3);
	_delay_ms(500);
	PORTB &= ~(1<<3);
	
	//initializeRotors();
	
	timer2_init();
	
	while(1)
	{
		
		if(timer2tick == true){
			
			stepperMotorTurn = 0;
			stepperMotor2Turn = 0;
			
			mpu6050_readAccel();
			mpu6050_readGyro();
			
			//accelxint = (int) ACCEL_XOUT_VALUE;
			//accelyint = (int) ACCEL_YOUT_VALUE;
			//accelzint = (int) ACCEL_ZOUT_VALUE;
			accelxf1=floor(fabs(ACCEL_XOUT_VALUE));
			accelxf2=fabs(ACCEL_XOUT_VALUE) - fabs(accelxf1);
			
			accelxi1 = (int)accelxf1;
			accelxi2 = (int)100*accelxf2;
			
			accelyf1=floor(fabs(ACCEL_YOUT_VALUE));
			accelyf2=fabs(ACCEL_YOUT_VALUE) - fabs(accelyf1);
			
			accelyi1 = (int)accelyf1;
			accelyi2 = (int)100*accelyf2;
			
			accelzf1=floor(fabs(ACCEL_ZOUT_VALUE));
			accelzf2=fabs(ACCEL_ZOUT_VALUE) - fabs(accelzf1);
			
			accelzi1 = (int)accelzf1;
			accelzi2 = (int)100*accelzf2;
			
			pitchAngle = GYRO_YANGLE;
			rollAngle = GYRO_XANGLE;
			yawAngle = GYRO_ZANGLE;
			
			pitchf1=floor(pitchAngle);
			pitchf2=pitchAngle - pitchf1;
			
			pitchi1 = (int)pitchf1;
			pitchi2 = (int)100*pitchf2;
			
			rollf1=floor(rollAngle);
			rollf2=rollAngle - rollf1;
			
			rolli1 = (int)rollf1;
			rolli2 = (int)100*rollf2;
			
			yawf1=floor(yawAngle);
			yawf2=yawAngle - yawf1;
			
			yawi1 = (int)yawf1;
			yawi2 = (int)100*yawf2;
			
			deltapitchAngle = prevpitchAngle - pitchAngle;
			deltarollAngle = prevrollAngle - rollAngle;
			deltayawAngle = prevyawAngle - yawAngle;
			
			dpitchf1=floor(deltapitchAngle);
			dpitchf2=deltapitchAngle - dpitchf1;
			
			dpitchi1 = (int)dpitchf1;
			dpitchi2 = (int)100*dpitchf2;
			
			drollf1=floor(deltarollAngle);
			drollf2=deltarollAngle - drollf1;
			
			drolli1 = (int)drollf1;
			drolli2 = (int)100*drollf2;
			
			dyawf1=floor(deltayawAngle);
			dyawf2=deltayawAngle - dyawf1;
			
			dyawi1 = (int)dyawf1;
			dyawi2 = (int)100*dyawf2;
			
			/*
			if(prevyawAngle < yawAngle && abs(dyawi2) > 10){
				
				stepperMotorTurn += 1;
				stepperMotor2Turn -= 1;
						
			} else if(prevyawAngle > yawAngle && abs(dyawi2) > 10) {
				
				stepperMotorTurn -= 1;
				stepperMotor2Turn += 1;
				
			}
			
			if(ACCEL_XOUT > 700){
				
				stepperMotorTurn += 1;
				stepperMotor2Turn += 1;
				PORTB |= (1<<3);
							
			} else if(ACCEL_XOUT < -700) {
				
				stepperMotorTurn -= 1;
				stepperMotor2Turn -= 1;
				PORTB |= (1<<3);
				
			} else {
				PORTB &= ~(1<<3);
			}
			
			
			if(stepperMotorTurn > 0){
				NODE_NEEDLE = NODE_NEEDLE->next;
				stepper_turnMotor();
			} else if(stepperMotorTurn < 0){
				NODE_NEEDLE = NODE_NEEDLE->prev;
				stepper_turnMotor();
			} else {
				stepper_stopMotor();
			}
			
			if(stepperMotor2Turn > 0){
				NODE_NEEDLE2 = NODE_NEEDLE2->next;
				stepper_turnMotor2();
			} else if(stepperMotor2Turn < 0){
				NODE_NEEDLE2 = NODE_NEEDLE2->prev;
				stepper_turnMotor2();
			} else {
				stepper_stopMotor2();
			}
			*/
						
			prevpitchAngle = pitchAngle;
			prevrollAngle = rollAngle;
			prevyawAngle = yawAngle;
			
			timer2tick = false;
		}
					
		if((UCSR0A & (1<<RXC0))){
							
			c = USART_receive();
							
			if(c == '0'){
				
				//sprintf(accelChar,"X:%d.%d Y:%d.%d Z:%d.%d...", accelxi1,accelxi2,accelyi1,accelyi2,accelzi1,accelzi2);
				sprintf(accelChar,"X:%d Y:%d Z:%d...", ACCEL_XOUT_VALUE,ACCEL_YOUT_VALUE,ACCEL_ZOUT_VALUE);
				USART_putstring(accelChar);
				
			} else if(c == '1'){
				
				sprintf(gyroChar,"pitch: %d.%d roll:%d.%d yaw:%d.%d...", pitchi1,pitchi2,rolli1,rolli2,yawi1,yawi2);
				USART_putstring(gyroChar);
				
			} else if(c == '2'){
			
				sprintf(dgyroChar,"dpitch: %d.%d droll:%d.%d dyaw:%d.%d...", dpitchi1,dpitchi2,drolli1,drolli2,dyawi1,dyawi2);
				USART_putstring(dgyroChar);
			
			} else if(c == '3'){
			
				sprintf(accelChar,"X:%d Y:%d Z:%d P:%d.%d R:%d.%d Y:%d.%d...", ACCEL_XOUT_VALUE,ACCEL_YOUT_VALUE,ACCEL_ZOUT_VALUE,pitchi1,pitchi2,rolli1,rolli2,yawi1,yawi2);
				USART_putstring(accelChar);
			
			}
			/*
			else if(c == 'u'){
				pwmMotA = pwmMotA + 100;
				pwmMotB = pwmMotB + 100;
				OCR1A = pwmMotA;
				OCR1B = pwmMotB;
			} else if(c == 'd'){
				pwmMotA = pwmMotA - 100;
				pwmMotB = pwmMotB - 100;
				OCR1A = pwmMotA;
				OCR1B = pwmMotB;
			}
			*/
			//USART_send(c);
		}
		
	}
}
