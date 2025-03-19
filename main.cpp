
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>
#include <util/twi.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "mpu6050_reg.h"

#define I2C_READ 1
#define I2C_WRITE 0

#define TWILIB_H_
#define BAUD 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUD * 16UL))) - 1)
#define TWI_FREQ 100000 // TWI bit rate
#define NO_PRESCALING 0
#define TWI_STATUS	(TWSR & 0xF8) // Get TWI status
#define TXMAXBUFLEN 14 // Transmit buffer length
#define RXMAXBUFLEN 14 // Receive buffer length

#define TWISendStart()		(TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE))
#define TWISendStop()		(TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE))
#define TWISendTransmit()	(TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE))
#define TWISendACK()		(TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA))
#define TWISendNACK()		(TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE))

volatile uint8_t TWITransmitBuffer[TXMAXBUFLEN]; // Global transmit buffer
volatile uint8_t TWIReceiveBuffer[RXMAXBUFLEN]; // Global receive buffer

volatile int TXBuffIndex; // Index of the transmit buffer. Is volatile, can change at any time.
volatile int RXBuffIndex; // Current index in the receive buffer

int TXBuffLen; // The total length of the transmit buffer
int RXBuffLen; // The total number of bytes to read (should be less than RXMAXBUFFLEN)

uint8_t default_addr = 104;	//imu device address 0x68

int GYRO_XOUT_OFFSET;
int GYRO_YOUT_OFFSET;
int GYRO_ZOUT_OFFSET;

long GYRO_XOUT_OFFSET_1000SUM = 0;
long GYRO_YOUT_OFFSET_1000SUM = 0;
long GYRO_ZOUT_OFFSET_1000SUM = 0;

long ACCEL_XOUT_1000SUM = 0;
long ACCEL_YOUT_1000SUM = 0;
long ACCEL_ZOUT_1000SUM = 0;

uint8_t GYRO_XOUT_L;
uint8_t GYRO_XOUT_H;
uint8_t GYRO_YOUT_L;
uint8_t GYRO_YOUT_H;
uint8_t GYRO_ZOUT_L;
uint8_t GYRO_ZOUT_H;

int ACCEL_XOUT;
int ACCEL_YOUT;
int ACCEL_ZOUT;

double ACCEL_ZOUT_Calibrated;

uint8_t ACCEL_XOUT_L;
uint8_t ACCEL_XOUT_H;
uint8_t ACCEL_YOUT_L;
uint8_t ACCEL_YOUT_H;
uint8_t ACCEL_ZOUT_L;
uint8_t ACCEL_ZOUT_H;

float ACCEL_XANGLE;
float ACCEL_YANGLE;
float ACCEL_ZANGLE;

float gyro_xsensitivity = 66.5f; //66.5 Dead on at last check
float gyro_ysensitivity = 66.5f; //72.7 Dead on at last check
float gyro_zsensitivity = 65.5f;

int GYRO_XOUT;
int GYRO_YOUT;
int GYRO_ZOUT;

float GYRO_XRATE;
float GYRO_YRATE;
float GYRO_ZRATE;

float dt = 0.05f;

float GYRO_XANGLE;
float GYRO_YANGLE;
float GYRO_ZANGLE;

float COMPLEMENTARY_XANGLE;
float COMPLEMENTARY_XANGLEPREV;
float COMPLEMENTARY_YANGLE;
float COMPLEMENTARY_YANGLEPREV;


#define BAUD 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUD * 16UL))) - 1)

//Declaration of our functions
void USART_init(void);
unsigned char USART_receive(void);
void USART_send( unsigned char data);
void USART_putstring(char* StringPtr);

void USART_init(void){
	
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00);
}

unsigned char USART_receive(void){
	
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
	
}

void USART_send( unsigned char data){
	
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
	
}

void USART_putstring(char* StringPtr){
	
	while(*StringPtr != 0x00){
		USART_send(*StringPtr);
	StringPtr++;}
}

/* MPU6050 MPU6050 MPU6050 MPU6050 MPU6050 MPU6050 I2C METHODS I2C METHODS MPU6050 MPU6050 MPU6050 MPU6050 MPU6050 MPU6050*/


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
	TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
	TWCR = (1<<TWIE)|(1<<TWEN);
}



uint8_t mpu6050_readBytes(uint8_t regAddr, uint8_t length) {
	uint8_t i = 0;
	
	if(length > 0) {
		
		i2c_start(MPU6050_ADDR|I2C_WRITE);
		i2c_write(regAddr); //request register
		
		i2c_start(MPU6050_ADDR|I2C_READ); //read data
		
		for(i=0; i<length; i++) {
			if(i==length-1)
			{TWIReceiveBuffer[i] = i2c_readNak();}
			else {TWIReceiveBuffer[i] = i2c_readAck();}
		}
		TWISendStop();
	}
	return TWIReceiveBuffer[0];
}
void mpu6050_writeBytes(uint8_t regAddr, uint8_t length, uint8_t* data) {
	if(length > 0) {
		//write data
		i2c_start(MPU6050_ADDR | I2C_WRITE);
		i2c_write(regAddr);
		for (uint8_t i = 0; i < length; i++) {
			i2c_write((uint8_t) data[i]);
		}
		TWISendStop();
	}
}

void mpu6050_writeData(uint8_t regAddr, uint8_t byteData) {
	i2c_start(MPU6050_ADDR | I2C_WRITE);
	i2c_write(regAddr);
	i2c_write((uint8_t) byteData);
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

ISR(TWI_vect)
{
	switch(TWI_STATUS)
	{
		case TW_START: // Start condition has been transmitted
		case TW_REP_START: // Repeated Start condition has been transmitted
		case TW_MT_SLA_NACK: // SLA+W transmitted, NACK received
		case TW_MT_SLA_ACK: // SLA+W transmitted, ACK received
		case TW_MT_DATA_NACK: // data transmitted, NACK received
		case TW_MT_DATA_ACK: // data transmitted, ACK received
		case TW_MR_SLA_NACK: // SLA+R transmitted and NACK received
		case TW_MR_SLA_ACK: // SLA+R transmitted and ACK received
		case TW_MR_DATA_NACK: // data received, NACK returned
		case TW_MR_DATA_ACK: // data received, ACK returned
		case TW_BUS_ERROR: // Illegal START/STOP, abort and return error
		case TW_SR_STOP: // Illegal START/STOP, abort and return error
		break;
	}
}

uint8_t mpu6050_calibrateGyros(){
	
	for (int i = 0; i < 50; i++){
		
		GYRO_XOUT_H = mpu6050_readBytes(MPU6050_RA_GYRO_XOUT_H, 1);
		GYRO_XOUT_L = mpu6050_readBytes(MPU6050_RA_GYRO_XOUT_L, 1);
		GYRO_YOUT_H = mpu6050_readBytes(MPU6050_RA_GYRO_YOUT_H, 1);
		GYRO_YOUT_L = mpu6050_readBytes(MPU6050_RA_GYRO_YOUT_L, 1);
		GYRO_ZOUT_H = mpu6050_readBytes(MPU6050_RA_GYRO_ZOUT_H, 1);
		GYRO_ZOUT_L = mpu6050_readBytes(MPU6050_RA_GYRO_ZOUT_L, 1);

		GYRO_XOUT_OFFSET_1000SUM += ((GYRO_XOUT_H << 8) | GYRO_XOUT_L);
		GYRO_YOUT_OFFSET_1000SUM += ((GYRO_YOUT_H << 8) | GYRO_YOUT_L);
		GYRO_ZOUT_OFFSET_1000SUM += ((GYRO_ZOUT_H << 8) | GYRO_ZOUT_L);

		_delay_ms(1);
	}

	GYRO_XOUT_OFFSET = (int) (GYRO_XOUT_OFFSET_1000SUM / 50);
	GYRO_YOUT_OFFSET = (int) (GYRO_YOUT_OFFSET_1000SUM / 50);
	GYRO_ZOUT_OFFSET = (int) (GYRO_ZOUT_OFFSET_1000SUM / 50);
	
	return 0;
}

void mpu6050_getGyroRates()
{
	GYRO_XOUT_H = mpu6050_readBytes(MPU6050_RA_GYRO_XOUT_H, 1);
	GYRO_XOUT_L = mpu6050_readBytes(MPU6050_RA_GYRO_XOUT_L, 1);
	GYRO_YOUT_H = mpu6050_readBytes(MPU6050_RA_GYRO_YOUT_H, 1);
	GYRO_YOUT_L = mpu6050_readBytes(MPU6050_RA_GYRO_YOUT_L, 1);
	GYRO_ZOUT_H = mpu6050_readBytes(MPU6050_RA_GYRO_ZOUT_H, 1);
	GYRO_ZOUT_L = mpu6050_readBytes(MPU6050_RA_GYRO_ZOUT_L, 1);

	GYRO_XOUT = ((GYRO_XOUT_H << 8) | GYRO_XOUT_L) - GYRO_XOUT_OFFSET;
	GYRO_YOUT = ((GYRO_YOUT_H << 8) | GYRO_YOUT_L) - GYRO_YOUT_OFFSET;
	GYRO_ZOUT = ((GYRO_ZOUT_H << 8) | GYRO_ZOUT_L) - GYRO_ZOUT_OFFSET;

	GYRO_XRATE = (float) (GYRO_XOUT / gyro_xsensitivity);
	GYRO_YRATE = (float) (GYRO_YOUT / gyro_ysensitivity);
	GYRO_ZRATE = (float) (GYRO_ZOUT / gyro_zsensitivity);

	GYRO_XANGLE += GYRO_XRATE * dt;
	GYRO_YANGLE += GYRO_YRATE * dt;
	GYRO_ZANGLE += GYRO_ZRATE * dt;
}

void mpu6050_getAccelValues()
{
	ACCEL_XOUT_1000SUM = 0;
	ACCEL_YOUT_1000SUM = 0;
	ACCEL_ZOUT_1000SUM = 0;
	
	for (int i = 0; i < 50; i++){
		ACCEL_XOUT_H = mpu6050_readBytes(MPU6050_RA_ACCEL_XOUT_H, 1);
		ACCEL_XOUT_L = mpu6050_readBytes(MPU6050_RA_ACCEL_XOUT_L, 1);
		
		ACCEL_YOUT_H = mpu6050_readBytes(MPU6050_RA_ACCEL_YOUT_H, 1);
		ACCEL_YOUT_L = mpu6050_readBytes(MPU6050_RA_ACCEL_YOUT_L, 1);
		
		ACCEL_ZOUT_H = mpu6050_readBytes(MPU6050_RA_ACCEL_ZOUT_H, 1);
		ACCEL_ZOUT_L = mpu6050_readBytes(MPU6050_RA_ACCEL_ZOUT_L, 1);

		ACCEL_XOUT_1000SUM += ((ACCEL_XOUT_H << 8) | ACCEL_XOUT_L);
		ACCEL_YOUT_1000SUM += ((ACCEL_YOUT_H << 8) | ACCEL_YOUT_L);
		ACCEL_ZOUT_1000SUM += ((ACCEL_ZOUT_H << 8) | ACCEL_ZOUT_L);
		
		_delay_ms(1);
	}

	ACCEL_XOUT = (int) (ACCEL_XOUT_1000SUM / 50);
	ACCEL_YOUT = (int) (ACCEL_YOUT_1000SUM / 50);
	ACCEL_ZOUT = (int) (ACCEL_ZOUT_1000SUM / 50);

	//ACCEL_ZOUT_Calibrated = (float) (ACCEL_ZOUT / 16384);
	//2g sensitivity scaling : 16384 counts/g
	//4g sensitivity scaling : 8192 counts/g
	//8g sensitivity scaling : 4096 counts/g
	//16g sensitivity scaling : 2048 counts/g

}

void mpu6050_getAccelAngles()
{
	ACCEL_XANGLE = (float)(57.295 * atan((float)ACCEL_YOUT / sqrt(pow((float)ACCEL_ZOUT, 2) + pow((float)ACCEL_XOUT, 2))));
	ACCEL_YANGLE = (float)(57.295 * atan((float)-ACCEL_XOUT / sqrt(pow((float)ACCEL_ZOUT, 2) + pow((float)ACCEL_YOUT, 2))));
}

void mpu6050_zeroSensors()
{
	float BUFFER_XANGLE = 0;
	float BUFFER_YANGLE = 0;
	int x = 0;

	for (x = 0; x < 100; x++)
	{
		mpu6050_getAccelValues();
		mpu6050_getAccelAngles();
		BUFFER_XANGLE += ACCEL_XANGLE;
		BUFFER_YANGLE += ACCEL_YANGLE;
		_delay_ms(1);
	}

	COMPLEMENTARY_XANGLE = (float)(BUFFER_XANGLE / 100.0);
	COMPLEMENTARY_YANGLE = (float)(BUFFER_YANGLE / 100.0);
	GYRO_XANGLE = (float)(BUFFER_XANGLE / 100.0);
	GYRO_YANGLE = (float)(BUFFER_YANGLE / 100.0);
}

void mpu6050_showOffsets()
{
	char StringX[20];
	sprintf(StringX, "x: %d",GYRO_XOUT_OFFSET);
	USART_putstring(StringX);
	
	char StringY[20];
	sprintf(StringY, " y: %d",GYRO_YOUT_OFFSET);
	USART_putstring(StringY);
	
	char StringZ[20];
	sprintf(StringZ, " z: %d",GYRO_ZOUT_OFFSET);
	USART_putstring(StringZ);
}

int main(void)
{	
	
	sei();
	TWIInit();
	
	//USART_init();
	
	//give time for boot up
	//_delay_ms(6000);
	
	//connection test
	mpu6050_testConnection();
	
	DDRD = 0xFF;    // Enable output on all of the B pins
	PORTD = 0x00;            // Set them all to 0v
		PORTD |=(1<<0);
		_delay_ms(250);
		PORTD &= ~(1 << 0);
		_delay_ms(500);
		
	//initialize
	mpu6050_initialize();
	
			PORTD |=(1<<0);
			_delay_ms(250);
			PORTD &= ~(1 << 0);
			_delay_ms(750);
	//initialize
	//if(mpu6050_calibrateGyros()){while(1){_delay_ms(1000);}}
	
	
	//mpu6050_getGyroRates();
	//mpu6050_zeroSensors();
	
	//mpu6050_getAccelValues();
	
	while(1)
	{
		//mpu6050_getAccelValues();
	}
	
	return 0;
}
