/* define CPU frequency in hz here if not defined in Makefile */
#define F_CPU 7372800UL

/* I2C clock in Hz */
#define SCL_CLOCK  400000L

#include <inttypes.h>
#include <compat/twi.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "Test_BNO055.h"

#define PI 3.14159265

char pitchChar[28];
char rollChar[28];
char yawChar[28];
char accelChar[28];
char pwmChar[28];

double pitchf1, pitchf2, rollf1, rollf2, yawf1, yawf2, dyawf1, dyawf2, ryawf1, ryawf2;
int pitchi1, pitchi2, rolli1, rolli2, yawi1, yawi2, dyawi1, dyawi2, ryawi1, ryawi2,pidrolli1,pidrolli2;

#define USART_BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

volatile uint8_t buffer[14];

int pwmMin = 920;
int pwmMinPlus = 960;
int pwmMax = 1840;
int pwmOff = 0;
volatile int pwmMotA = 1100;
volatile int pwmMotB = 1100;
volatile int targetpwmMotA = 1100;
volatile int targetpwmMotB = 1100;

float rollAngle = 0.0f;
float pitchAngle = 0.0f;
float yawAngle = 0.0f;

float prevrollAngle = 0.0f;
float prevpitchAngle = 0.0f;
float prevyawAngle = 0.0f;

double deltarollAngle = 0.0f;
double deltapitchAngle = 0.0f;
double deltayawAngle = 0.0f;

double targetrollAngle = 0.0f;
double targetpitchAngle = 0.0f;
double targetyawAngle = 0.0f;

double proportionalrollAngle = 0.0f;
double proportionalpitchAngle = 0.0f;
double proportionalyawAngle = 0.0f;

double integralrollAngle = 0.0f;
double integralpitchAngle = 0.0f;
double integralyawAngle = 0.0f;

double derivativerollAngle = 0.0f;
double derivativepitchAngle = 0.0f;
double derivativeyawAngle = 0.0f;

int derRollAng1 = 0;
int derPitchAng1 = 0;
int derYawAng1 = 0;

int derRollAng2 = 0;
int derPitchAng2 = 0;
int derYawAng2 = 0;

int proRollAng1 = 0;
int proPitchAng1 = 0;
int proYawAng1 = 0;

int proRollAng2 = 0;
int proPitchAng2 = 0;
int proYawAng2 = 0;

double pidRollThreshold = 0.0f;
double pidPitchThreshold = 0.0f;
double pidYawThreshold = 0.0f;

double tunepitchP = 1.0f;
double tunepitchI = 0.0f;
double tunepitchD = 1.0f;

double tunerollP = 1.0f;
double tunerollI = 0.0f;
double tunerollD = 1.0f;

float pidpitchAngle = 0.0f;
float pidrollAngle = 0.0f;
float pidyawAngle = 0.0f;

volatile int ovr = 0;
//volatile int pwm1 = 742;
//volatile int pwm2 = 738;

volatile int pwm1 = 738;
volatile int pwm2 = 736;

volatile int targetpwmServoA = 738;
volatile int targetpwmServoB = 736;

char c;

int RawDatax,RawDatay,RawDataPitch;
float SmoothDatax,SmoothDatay, SmoothDataPitch;
float prevSmoothy,prevSmoothx,derSmoothx,derSmoothy = 0;
float LPF_Beta = 0.15; // 0<ß<1

volatile int pidCount = 0;
volatile int armCount = 0;

int derSumCounter = 0;
int derSumLimit = 1;
double derRollArr[2];
double derRollQsum = 0;

int rollIntQSumCounter = 0;
int rollIntQSumLimit = 99;
double rollIntQArr[100];
double rollIntQsum = 0;

int yawIntQSumCounter = 0;
int yawIntQSumLimit = 3;
double yawIntQArr[4];
double yawIntQsum = 0;

bool derresetter = false;
bool intresetter = false;

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



/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */

}/* i2c_init */


/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;

}/* i2c_start */


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}/* i2c_start_wait */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start( address );

}/* i2c_rep_start */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));

}/* i2c_stop */


/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{	
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

}/* i2c_write */


/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    

    return TWDR;

}/* i2c_readAck */


/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	
    return TWDR;

}/* i2c_readNak */

void initializeRotors(){
	
	DDRB |= (1 << DDB1)|(1 << DDB2);
	// PB1 and PB2 is now an output

	TCCR1A |= (1 << COM1A1)|(1 << COM1B1);
	// set none-inverting mode

	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12)|(1 << WGM13) | (1 << CS11);
	// set Fast PWM mode using ICR1 as TOP prescaler 8
	
	ICR1 = 18400;

	OCR1A = pwmOff;
	// set PWM for 10% duty cycle @ 16bit (4000)

	OCR1B = pwmOff;
	// set PWM for 20% duty cycle @ 16bit (8000)

	_delay_ms(4000);
	
	OCR1A = pwmMax;
	OCR1B = pwmMax;
	
	_delay_ms(4000);
	
	OCR1A = pwmMin;
	OCR1B = pwmMin;
	
	
	_delay_ms(6000);
	
	OCR1A = pwmOff;
	OCR1B = pwmOff;
	
	_delay_ms(1);
	
	OCR1A = pwmMotA;
	OCR1B = pwmMotB;
}

void armRotors(){
		DDRB |= (1 << DDB1)|(1 << DDB2);
		// PB1 and PB2 is now an output

		TCCR1A |= (1 << COM1A1)|(1 << COM1B1);
		// set none-inverting mode

		TCCR1A |= (1 << WGM11);
		TCCR1B |= (1 << WGM12)|(1 << WGM13) | (1 << CS11);
		// set Fast PWM mode using ICR1 as TOP prescaler 8
		
		ICR1 = 18400;
		// set TOP to 16bit to 20000*0.92 20ms

		OCR1A = pwmOff;
		OCR1B = pwmOff;

		_delay_ms(1000);
		
		OCR1A = pwmMax;
		OCR1B = pwmMax;
		
		_delay_ms(1000);
		
		OCR1A = pwmMin;
		OCR1B = pwmMin;
		
		
		_delay_ms(2000);
		
		OCR1A = pwmOff;
		OCR1B = pwmOff;
		
		_delay_ms(1);
		
		OCR1A = pwmMotA;
		OCR1B = pwmMotB;
}

int format_print_left(double num){
	return (int) num;
}

int format_print_right(double num){
	double left;
	double right;
	
	left = (int) num;
	right = fabs(num-left);
	return (int) 100*right;
}

double sumArray(double value, double* sum, double* arr, int limit, int* counter, bool avg, bool* reset){
		
		double result = 0;
		
		if(*reset){
			*sum = 0;
			
			for(int i = 0; i <= limit; i++){
				arr[i] = 0;
			}
			
			 *counter = 0;
			 *reset = false;
		}
		
		if(*counter < limit){
			
			*sum = (double) *sum + value;
			arr[*counter] = value;
			
			if(avg){
				result = (double) *sum/(*counter+1);
			} else {
				result  = (double) *sum;
			}
			
			*counter = *counter + 1;
			
		} else {
			
			*sum = (double) *sum - arr[0];
			*sum = (double) *sum + value;
						
			for(int i = 0; i < limit; i++){
				arr[i] = arr[i+1];
			}
						
			arr[limit] = value;
						
			if(avg){
				result = (double) *sum/(*counter);
			} else {
				result  = (double) *sum;
			}
		}
		
		return (double) result;
}

void loop_PID(){
	
		deltapitchAngle = prevpitchAngle - pitchAngle;
		deltarollAngle = prevrollAngle - rollAngle;
		deltayawAngle = prevyawAngle - yawAngle;
		
		if(deltayawAngle > 180.0){
			deltayawAngle = (int) 360.0 - deltayawAngle;
		} else if(deltayawAngle < -300.0) {
			deltayawAngle = (int)   -360.0 - deltayawAngle;
		}
	
		proportionalpitchAngle = targetpitchAngle - pitchAngle;
		proportionalrollAngle = targetrollAngle - rollAngle;
		proportionalyawAngle = targetyawAngle - yawAngle;
		
		if(fabs(proportionalyawAngle) > 180.0){
			proportionalyawAngle = (180 - fabs(proportionalyawAngle + 180));
		} else if(proportionalyawAngle < 180.0) {
			proportionalyawAngle = -1*(180 - fabs(proportionalyawAngle + 180));
		}
		
		integralpitchAngle += proportionalpitchAngle;
		integralrollAngle += proportionalrollAngle;
		//integralyawAngle = sumArray(deltayawAngle, &yawIntQsum, yawIntQArr, yawIntQSumLimit, &yawIntQSumCounter, false, &intresetter);
		
		//integralyawAngle += deltayawAngle; ///(1 + exp((180 - fabs(proportionalyawAngle + 180))/3  - 6));//deltayawAngle/4;
		
		
		
			
			
			
		//derivativerollAngle = sumArray(deltarollAngle, &derRollQsum, derRollArr, derSumLimit, &derSumCounter, true, &derresetter);
		//integralrollAngle = (fabs(proportionalrollAngle)*sumArray(proportionalrollAngle, &rollIntQsum, rollIntQArr, rollIntQSumLimit, &rollIntQSumCounter, false, &intresetter))/500;
		 
		
		pidPitchThreshold = (int) proportionalpitchAngle/3;
		
		pidPitchThreshold = (pidPitchThreshold > 20.0) ? 20.0 : pidPitchThreshold;
		pidPitchThreshold = (pidPitchThreshold < -20.0) ? -20.0 : pidPitchThreshold;
		
		/*
		pwm1 = targetpwmServoA - pidPitchThreshold;
		pwm2 = targetpwmServoB + pidPitchThreshold;
		*/
		
		//pidRollThreshold = (proportionalrollAngle)*(1/(1+exp((-1*fabs(proportionalrollAngle/derivativerollAngle)+6))));// + integralrollAngle;// - derivativerollAngle;// *(1/(1+fabs(derivativerollAngle)));// - derivativerollAngle;// + integralrollAngle - derivativerollAngle;// + proportionalrollAngle*integralrollAngle/500;// + 10*derivativerollAngle + proportionalrollAngle*integralrollAngle/500;
		//pidRollThreshold = (proportionalrollAngle + derivativerollAngle)*(1/(1+exp(-3*fabs(proportionalrollAngle/derivativerollAngle)+6)));// + integralrollAngle;
		
		pidRollThreshold = proportionalrollAngle*8 + deltarollAngle*34 + integralrollAngle/128; //*(1/(1+exp((-1*fabs(proportionalrollAngle*2/(deltarollAngle+1))+12)))) ;
		
		
		
		//pidYawThreshold = (int) proportionalyawAngle/6; // proportionalyawAngle/6 + deltayawAngle;// integralyawAngle;
		
		
		
		//pidYawThreshold = integralyawAngle/(1 + exp(-1*fabs(2*deltayawAngle) + 6));
		
	
		/*
		pwm1 = (int) pwm1 - pidYawThreshold;
		pwm2 = (int) pwm2 - pidYawThreshold;
		
		pwm1 = targetpwmServoA - pidYawThreshold;
		pwm2 = targetpwmServoB - pidYawThreshold;
		*/
		
		pwmMotA = (int) targetpwmMotA - pidRollThreshold;
		pwmMotB = (int) targetpwmMotB + pidRollThreshold;
		
		pwmMotA = (pwmMotA > pwmMax) ? pwmMax : pwmMotA;
		pwmMotA = (pwmMotA < pwmMinPlus) ? pwmMinPlus : pwmMotA;
		
		pwmMotB = (pwmMotB > pwmMax) ? pwmMax : pwmMotB;
		pwmMotB = (pwmMotB < pwmMinPlus) ? pwmMinPlus : pwmMotB;
		
		OCR1A = pwmMotA;
		OCR1B = pwmMotB;
		
}

void initializeServos(){
		DDRD |= (1 << DDD6) | (1 << DDD5);
		// PD6 is now an output

		OCR2A = 184;

		TCCR2A |= (1 << WGM21);
		// Set to CTC Mode

		TIMSK2 |= (1 << OCIE2A);
		//Set interrupt on compare match

		TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20);
		// set prescaler to 8 and starts timer
		
		GTCCR = (1<<PSRASY) | (1<<PSRSYNC);
}

int main(void)
{
	sei();
	_delay_ms(1500);
	
	DDRC = 0x0F;    // Enable output on all of the C pins
	PORTC = 0x00;            // Set them all to 0v
	
	DDRD = 0xFF;    // Enable output on all of the C pins
	PORTD = 0x00;            // Set them all to 0v
	
	//AVR_Init();
	USART_init();
	i2c_init();
	
	//Init_SPI();
	//Init_nrf();
	
	unsigned char Euler_Raw_LSB;
	unsigned char Euler_Raw_MSB;
	
	unsigned char Linear_Raw_LSBx;
	unsigned char Linear_Raw_MSBx;
	unsigned char Linear_Raw_LSBy;
	unsigned char Linear_Raw_MSBy;

	float angle_scale = 1.0f/16.0f;

	initializeServos();
	initializeRotors();	
	armRotors();
				
				/*
				
				i2c_start_wait(BNO055_ADDRESS|I2C_WRITE);	//Set device address and read mode
				i2c_write(BNO055_OPR_MODE_ADDR);
				i2c_write(OPERATION_MODE_IMUPLUS);		//Set operation mode to IMU
				i2c_stop();
				
				*/
				
				//_delay_ms(10);
				
				
	//Endless Loop
	while(1)
	{
		
		
		i2c_start_wait(BNO055_ADDRESS|I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_H_LSB_ADDR);		//Access LSB of Heading Euler angle
		i2c_rep_start(BNO055_ADDRESS | I2C_READ);		//Set device address and read mode
		Euler_Raw_LSB = i2c_readNak();
		i2c_stop();

		i2c_start_wait(BNO055_ADDRESS|I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_H_MSB_ADDR);		//Access LSB of Heading Euler angle
		i2c_rep_start(BNO055_ADDRESS | I2C_READ);		//Set device address and read mode
		Euler_Raw_MSB = i2c_readNak();
		i2c_stop();

		int16_t Euler_H_Raw = (Euler_Raw_MSB << 8) | (Euler_Raw_LSB);

		yawAngle = (float)(Euler_H_Raw) * angle_scale;
		//yawAngle = fabs(yawAngle - 180.0);
		
		//itoa(Euler_H, String_Data, 10);			//Convert integer to string, radix=10
		//nRF_Put_String("Y: ");
		//nRF_Put_String(String_Data);
		
		i2c_start_wait(BNO055_ADDRESS|I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_R_LSB_ADDR);		//Access LSB of Roll Euler angle
		i2c_rep_start(BNO055_ADDRESS | I2C_READ);		//Set device address and read mode
		Euler_Raw_LSB = i2c_readNak();
		i2c_stop();

		i2c_start_wait(BNO055_ADDRESS|I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_R_MSB_ADDR);		//Access MSB of Roll Euler angle
		i2c_rep_start(BNO055_ADDRESS | I2C_READ);		//Set device address and read mode
		Euler_Raw_MSB = i2c_readNak();
		i2c_stop();

		int16_t Euler_R_Raw = (Euler_Raw_MSB << 8) | (Euler_Raw_LSB);

		rollAngle = (float)(Euler_R_Raw) * angle_scale;

		//itoa(Euler_R, String_Data, 10);  //convert integer to string, radix=10
		//nRF_Put_String(" R: ");
		//nRF_Put_String(String_Data);

		i2c_start_wait(BNO055_ADDRESS | I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_P_LSB_ADDR);		//Access LSB of Pitch Euler angle
		i2c_rep_start(BNO055_ADDRESS | I2C_READ);		//Set device address and read mode
		Euler_Raw_LSB = i2c_readNak();
		i2c_stop();

		i2c_start_wait(BNO055_ADDRESS | I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_EULER_P_MSB_ADDR);		//Access LSB of Pitch Euler angle
		i2c_rep_start(BNO055_ADDRESS | I2C_READ);		//Set device address and read mode
		Euler_Raw_MSB = i2c_readNak();
		i2c_stop();

		int16_t Euler_P_Raw = (Euler_Raw_MSB << 8) | (Euler_Raw_LSB);

		pitchAngle = (float)(Euler_P_Raw) * angle_scale;
		//SmoothDataPitch = SmoothDataPitch - (LPF_Beta * (SmoothDataPitch - pitchAngle));		
		//pitchAngle = SmoothDataPitch;

		i2c_start_wait(BNO055_ADDRESS|I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR);		//Access LSB of Heading Euler angle
		i2c_rep_start(BNO055_ADDRESS | I2C_READ);		//Set device address and read mode
		Linear_Raw_LSBx = i2c_readNak();
		i2c_stop();

		i2c_start_wait(BNO055_ADDRESS|I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR);		//Access LSB of Heading Euler angle
		i2c_rep_start(BNO055_ADDRESS | I2C_READ);		//Set device address and read mode
		Linear_Raw_MSBx = i2c_readNak();
		i2c_stop();

		int16_t Linear_X_Raw = (Linear_Raw_MSBx << 8) | (Linear_Raw_LSBx);
		//int16_t Linear_X_Raw = (Linear_Raw_MSBx << 8) | (Linear_Raw_LSBx);
		//int16_t Xforce = (Linear_Raw_MSBx << 8);
		
		i2c_start_wait(BNO055_ADDRESS|I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR);		//Access LSB of Heading Euler angle
		i2c_rep_start(BNO055_ADDRESS | I2C_READ);		//Set device address and read mode
		Linear_Raw_LSBy = i2c_readNak();
		i2c_stop();

		i2c_start_wait(BNO055_ADDRESS|I2C_WRITE);	//Set device address and read mode
		i2c_write(BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR);		//Access LSB of Heading Euler angle
		i2c_rep_start(BNO055_ADDRESS | I2C_READ);		//Set device address and read mode
		Linear_Raw_MSBy = i2c_readNak();
		i2c_stop();

		int16_t Linear_Y_Raw = (Linear_Raw_MSBy << 8) | (Linear_Raw_LSBy);
		//int16_t Linear_Y_Raw = (Linear_Raw_MSBy << 8) | (Linear_Raw_LSBy);
		//int16_t Yforce = (Linear_Raw_MSBy << 8);
		
		pitchi1 = format_print_left((double) pitchAngle);
		pitchi2 = format_print_right((double) pitchAngle);

		rolli1 = format_print_left((double) rollAngle);
		rolli2 = format_print_right((double) rollAngle);

		yawi1 = format_print_left((double) yawAngle);
		yawi2 = format_print_right((double) yawAngle);

		//itoa(Euler_P, String_Data, 10);  //convert integer to string, radix=10	
		//nRF_Put_String(" P: ");
		//nRF_Put_String(String_Data);
		//nRF_Put_String("\n");
		
		loop_PID();
			
		derPitchAng1 = format_print_left(deltapitchAngle);
		derRollAng1 = format_print_left(derivativerollAngle);
		derYawAng1 = format_print_left(deltayawAngle);
				
		derPitchAng2 = format_print_right(deltapitchAngle);
		derRollAng2 = format_print_right(derivativerollAngle);
		derYawAng2 = format_print_right(deltayawAngle);
		
		proPitchAng1 = format_print_left(proportionalpitchAngle);
		proRollAng1 = format_print_left(proportionalrollAngle);
		proYawAng1 = format_print_left(proportionalyawAngle);
				
		proPitchAng2 = format_print_right(proportionalpitchAngle);
		proRollAng2 = format_print_right(proportionalrollAngle);
		proYawAng2 = format_print_right(proportionalyawAngle);
						
		//int addRollder1 = format_print_left(derivativerollAngle);
		//int addRollder2 = format_print_right(derivativerollAngle);
						
		pidrolli1 = format_print_left(pidRollThreshold);
		pidrolli2 = format_print_right(pidRollThreshold);
				
		RawDatax = Linear_X_Raw;
		SmoothDatax = SmoothDatax - (LPF_Beta * (SmoothDatax - RawDatax));		
				
		int smoothedDatax1 = format_print_left(SmoothDatax);
		int smoothedDatax2 = format_print_right(SmoothDatax);
		
		RawDatay = Linear_Y_Raw;
		SmoothDatay = SmoothDatay - (LPF_Beta * (SmoothDatay - RawDatay));
		
		int smoothedDatay1 = format_print_left(SmoothDatay);
		int smoothedDatay2 = format_print_right(SmoothDatay);
		
		derSmoothx = SmoothDatax - prevSmoothx;
		derSmoothy = SmoothDatay - prevSmoothy;
		
		int derSmoothy1 = format_print_left(derSmoothy);
		int derSmoothy2 = format_print_right(derSmoothy);
				
		int intRollAng1 = format_print_left(integralrollAngle);
		int intRollAng2 = format_print_right(integralrollAngle);
		
		int intYawAng1 = format_print_left(integralyawAngle);
		int intYawAng2 = format_print_right(integralyawAngle);
		
		int intYaw = (int) yawAngle;
		
		
		if((UCSR0A & (1<<RXC0))){
			
			c = USART_receive();
			
			if(c == '0'){
				////sprintf(accelChar,"pitch:%d.%d roll:%d.%d yaw:%d.%d X:%d Y:%d pidR:%d.%d...", pitchi1,pitchi2,rolli1,rolli2,yawi1,yawi2,Linear_X_Raw,Linear_Y_Raw,pidrolli1,pidrolli2);
				//sprintf(accelChar,"%d %d.%d %d.%d ...", intYaw, derYawAng1,derYawAng2,intYawAng1,intYawAng2);
				USART_putstring(accelChar);
			} else if(c == '1'){
				//sprintf(accelChar,"%d.%d %d.%d %d.%d %d %d %d.%d ...", pitchi1,pitchi2,rolli1,rolli2,yawi1,yawi2,Linear_X_Raw,Linear_Y_Raw,pidrolli1,pidrolli2);
				USART_putstring(accelChar);
			} else if(c == '2'){
				//sprintf(accelChar,"%d.%d %d.%d %d.%d %d.%d %d.%d %d.%d %d.%d ...", pitchi1,pitchi2,rolli1,rolli2,yawi1,yawi2,proRollAng1,proRollAng2,derRollAng1,derRollAng2,intRollAng1,intRollAng2,pidrolli1,pidrolli2);
				USART_putstring(accelChar);
			} else if(c == '3'){
				//sprintf(accelChar,"%d.%d %d.%d ...", smoothedDatay1,smoothedDatay2,derSmoothy1,derSmoothy2);
				USART_putstring(accelChar);
			} else if(c == '4'){
				sprintf(accelChar,"%d %d %d %d...", pwmMotA,pwmMotB, pwm1, pwm2);
				USART_putstring(accelChar);
			} else if(c == 'u'){
				targetpwmMotA = targetpwmMotA + 50;
				targetpwmMotB = targetpwmMotB + 50;
			} else if(c == 'y'){
				targetpwmMotA = targetpwmMotA + 50;
			} else if(c == 'h'){
				targetpwmMotA = targetpwmMotA - 50;
			} else if(c == 'i'){
				targetpwmMotB = targetpwmMotB + 50;
			} else if(c == 'k'){
				targetpwmMotB = targetpwmMotB - 50;
			} else if(c == 'd'){
				targetpwmMotA = targetpwmMotA - 50;
				targetpwmMotB = targetpwmMotB - 50;
			} else if(c == 's'){
				targetpwmMotA = 0;
				targetpwmMotB = 0;
			} else if(c =='z') {
				pwm1 -= 1;
				pwm2 += 1;
			} else if(c == 'x'){
				pwm1 += 1;
				pwm2 -= 1;
			}
		}
		
		
		prevpitchAngle = pitchAngle;
		prevrollAngle = rollAngle;
		prevyawAngle = yawAngle;
		
		prevSmoothx = SmoothDatax;
		prevSmoothy = SmoothDatay;
		
		
		//armCount = 0;
		
		
	}
}

ISR (TIMER2_COMPA_vect)
{
	
	
	if(ovr > pwm1){
		PORTD |= (1<<5);
	}
	
	if(ovr > pwm2){
		PORTD |= (1<<6);
	}
		
	if(ovr > 799){
		PORTD &= ~(1<<5);
		PORTD &= ~(1<<6);
		ovr = 0;
		//pidCount++;
	}
	
	ovr++;
}
