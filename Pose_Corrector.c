#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
// Ultrasound setting
#define TRIG 6     //Trigger 신호 (출력 = PE6)
#define ECHO 7     //Echo 신호 (입력 = PE7)
#define SOUND_VELOCITY 340UL //소리 속도 (m/sec)
// flex setting
void init_adc();     //adc 초기화 함수
void show_led(unsigned short data);     //adc값에 따른 led 함수
volatile unsigned short read_adc;      //adc값     //16비트
volatile unsigned char adc_flag;       //adc 변환 확인 변수 
// MPU setting
#define FS_SEL 131
#define RAD2DEG 57.29578
void twi_write(unsigned char address,unsigned char data);
unsigned char twi_read(char addressr);
void get_raw_data();
void calibrate();
void MPU9250_init();
volatile double dt = 0.000;
volatile int temp;
volatile unsigned char a_x_l,a_x_h,a_y_l,a_y_h,a_z_l,a_z_h;
volatile unsigned char g_y_l,g_y_h;
volatile double bas_g_y;
volatile double a_x,a_y,a_z;
volatile double g_y;
volatile double angle_ay;
volatile double angle_gy;
volatile double las_angle_gy;
volatile double pitch;
volatile double alpha;
volatile int stand_flex,stand_mpu;

// flex funtions
ISR(ADC_vect){       //ADC interrupt function
    read_adc=ADCL;     //변환된 아날로그 값 저장, 반드시 Low부터 저장
    read_adc+=(ADCH<<8);
    adc_flag=1;     //인터럽트로 adc값 변환 완료
}

void init_adc(){      //adc 초기화 함수
    DDRF=0x00;     //F핀 입력핀 (PF0)
    ADMUX=0x40;  //0b01000000
    ADCSRA=0xef; //0b11101111
    sei();     //전역 인터럽트 활성화 
} 

// mpu functions
ISR(TIMER0_OVF_vect)   //0.002sS
{
   dt += 0.002;

   TCNT0 = 256-125;
}

void calibrate()   //초기값 읽기 
{
   int cal = 10;

   for(int i=0; i<cal; i++)   //평균 
   {
      get_raw_data();
   
      temp = (a_x_h<<8) | a_x_l;
      a_x += - temp - 16383;
      temp = (a_y_h<<8) | a_y_l;
      a_y += - temp;
      temp = (a_z_h<<8) | a_z_l;
      a_z += temp;
      temp = (g_y_h<<8) | g_y_l;
      g_y += temp;

      _delay_ms(100);
   }   
   
   a_x /= cal;
   a_y /= cal;
   a_z /= cal;
   g_y /= cal;

   bas_g_y = g_y;//초기 값으로 저장
}

void get_raw_data()
{
   a_x_h = twi_read(0x3B);      //x축 가속도
   _delay_us(10);
   a_x_l = twi_read(0x3C);
   _delay_us(10);
   a_y_h = twi_read(0x3D);      //y축 가속도 
   _delay_us(10);
   a_y_l = twi_read(0x3E);      
   _delay_us(10);
   a_z_h = twi_read(0x3F);      //z축 가속도 
   _delay_us(10);
   a_z_l = twi_read(0x40);      
   _delay_us(10);
   g_y_h = twi_read(0x45);      //y축 각속도 
   _delay_us(10);
   g_y_l = twi_read(0x46);      
   _delay_us(10);

}

void MPU9250_init()
{
   twi_write(0x6B, 0x03); //[PWR_MGMT_1] sleep 끔, Auto PLL
	_delay_ms(1);
   twi_write(0x1A, 0x05); //[CONFIG] DLPF 10Hz

   twi_write(0x6A, 0x20); //[USER_CTRL] ,master enable
   twi_write(0x24, 0x0D); //[I2C_MST_CTRL] ,내부 i2c clock 400kHz
   twi_write(0x67, 0x0F); //[I2C_MST_DELAY_CTRL]

   twi_write(0x2E, 0x0C); //[I2C_SLV3_ADDR] AK8963 write
    twi_write(0x2F, 0x0A); //[I2C_SLV3_REG]
    twi_write(0x30, 0x81); //[I2C_SLV3_CTRL]
    twi_write(0x66, 0x11); //[I2C_SLV3_DO]

    twi_write(0x34, 0x13); //[I2C_SLV4_CTRL] 모든 slave 19 (0b00010011) +1번 샘플

   twi_write(0x25, 0x8C); //[I2C_SLV0_ADDR] AK8963 read.
    twi_write(0x26, 0x03); //[I2C_SLV0_REG]
    twi_write(0x27, 0xD2); //[I2C_SLV0_CTRL]

   twi_write(0x28, 0x8C); //[I2C_SLV1_ADDR] AK8963 read.
    twi_write(0x29, 0x05); //I2C_SLV1_REG]
    twi_write(0x2A, 0xD2); //[I2C_SLV1_CTRL]

   twi_write(0x2B, 0x8C); //[I2C_SLV2_ADDR] AK8963 read.
    twi_write(0x2C, 0x07); //[I2C_SLV2_REG]
    twi_write(0x2D, 0xD2); //[I2C_SLV2_CTRL]
}

void twi_write(unsigned char address,unsigned char data)
{ 
   TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);   //START

   while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림 
   while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림  

   TWDR = 0b11010000;          //AD(1101000)+W(0) 
   TWCR = (1<<TWINT)|(1<<TWEN); //전송 

   while(!(TWCR & (1<<TWINT))); 
   while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

   TWDR = address;           //register address
   TWCR = (1<<TWINT)|(1<<TWEN); //전송

   while(!(TWCR & (1<<TWINT)));
   while((TWSR&0xF8) != 0x28);  //Data ACK 상태(28) 기다림 

   TWDR = data;              //data 
   TWCR = (1<<TWINT)|(1<<TWEN); //전송  

   while(!(TWCR & (1<<TWINT)));
   while((TWSR&0xF8) != 0x28);

   TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); //STOP
} 

unsigned char twi_read(char address)
{ 
   unsigned char data;

   TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);   //START

   while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림 
   while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림  

   TWDR = 0b11010000;          //AD(1101000)+W(0) 
   TWCR = (1<<TWINT)|(1<<TWEN); //전송 

   while(!(TWCR & (1<<TWINT))); 
   while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

   TWDR = address;           //register address
   TWCR = (1<<TWINT)|(1<<TWEN); //전송

   while(!(TWCR & (1<<TWINT)));
   while((TWSR&0xF8) != 0x28);  //Data ACK 상태(28) 기다림 

   TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);   //Repeat START

   while(!(TWCR & (1<<TWINT)));
   while((TWSR&0xF8) != 0x10);  //Repeat START 상태(08) 기다림

   TWDR = 0b11010001;          //AD(1101000)+R(1) 
   TWCR = (1<<TWINT)|(1<<TWEN); //전송 

   while(!(TWCR & (1<<TWINT)));
   while((TWSR&0xF8) != 0x40);  //SLA+R ACK 상태(40) 기다림 

   TWCR = (1<<TWINT)|(1<<TWEN); //전송

   while(!(TWCR & (1<<TWINT)));
   while((TWSR&0xF8) != 0x58);  //ACK 상태(58) 기다림 

   data = TWDR; 

   TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); //STOP

   return data; 
}

int main(void){
    unsigned int distance;     //거리 변수
	int i;
	// GPIO
    DDRA = 0xff; // mpu 출력핀
	DDRB=0x10;     //초음파 거리센서 출력핀
	DDRC = 0xff; // flex 출력핀
    
	// 초음파 init (GPIO)
    DDRE=((DDRE|(1<<TRIG)) & ~(1<<ECHO)); //TRIG = 출력 , ECHO = 입력 setting

	// flex init
    init_adc();

	// mpu init
   //TWI(I2C)
   TWCR = (1<<TWEN);
   TWBR = 12;          //400khz

   //TIMER0
   TCCR0 = (1<<CS02)|(1<<CS01);   //256 분주 
   TCNT0 = 256-125;            //125 번 => 0.002s
   TIMSK = (1<<TOIE0);

   MPU9250_init();
   calibrate();

   SREG = 0x80;

	// 전체 initialization 후
	// 종료 전까지 센서 working
   while(1){
   		// 초음파 거리 감지
        TCCR1B=0x03;     //Timer/Counter1 클록 4us(64분주)
        PORTE &= ~(1<<TRIG);     //Trig=LOW상태
        _delay_us(10);      //10us동안 유지
        PORTE |= (1<<TRIG);     //Trig=HIGH -> 거리 측정 명령 시작
        _delay_us(10);     //10us동안 유지
        PORTE &= ~(1<<TRIG);     //Trig=LOW -> 거리 측정 명령 끝

        while(!(PINE & (1<<ECHO)))     //Echo=HIGH가 될 때까지 대기
          ;
        TCNT1=0x0000;     //Timer/Counter1 값 초기화
        while(PINE & (1<<ECHO))     //Echo=LOW가 될 때까지 대기
         ;
        TCCR1B=0x00;     //Timer/Counter1 클록 정지(클록 입력 차단,CS11~CS10=000)

        distance = (unsigned int)(SOUND_VELOCITY * (TCNT1*4/2)/1000);     //거리=속도x시간, 거리 단위는 1mm


		//mpu 기울기 감지
      get_raw_data();
 
      las_angle_gy = pitch;//최근값 누적

      temp = (a_x_h<<8) | a_x_l;
      a_x = - temp;
      temp = (a_y_h<<8) | a_y_l;
      a_y = - temp;
      temp = (a_z_h<<8) | a_z_l;
      a_z = temp;
      temp = (g_y_h<<8) | g_y_l;
      g_y = temp;


      g_y = (g_y - bas_g_y)/FS_SEL;
      
      angle_ay = atan(a_x/sqrt(pow(a_y,2) + pow(a_z,2)))*RAD2DEG;

      angle_gy = g_y*dt + las_angle_gy;

      dt = 0.000;

      alpha = 0.96;
      pitch = alpha*angle_gy + (1.000 - alpha)*angle_ay;


	// flex 인터럽트 발생시
     if(adc_flag==1){     //인터럽트 발생했다면(adc값 변환했다면)
       adc_flag=0;     //초기화
     }
	

	// 전체 출력 모음
	stand_flex=20;
	stand_mpu=-60;

	if((distance<500) | (read_adc>stand_flex) | (pitch>stand_mpu)){
		if(distance<500){
			for(i=0;i<5;i++){
				PORTB = 0x10;
				_delay_ms(1);
			}
		}	
		if(read_adc>stand_flex){
			PORTC = 0x04;
			_delay_ms(5);
		}
		if(pitch>stand_mpu){
			PORTA = 0x04;
			_delay_ms(5);
		}
		if((distance>500) | (read_adc<stand_flex) | (pitch<stand_mpu)){
			if(distance>500){
				PORTB = 0x00;
			}
			if(read_adc<stand_flex){
				PORTC = 0x01;
			}
			if(pitch<stand_mpu){
				PORTA = 0x01;
			}
			_delay_ms(5);
		}
	}
	else{
		PORTB = 0x00;
		PORTC = 0x01;
		PORTA = 0x01;
		_delay_ms(5);
	}
	_delay_ms(100);
     }
}
