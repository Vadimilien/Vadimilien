#include <EEPROM.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

volatile char receivedString[1024];
volatile uint8_t receivedIndex = 0;
volatile bool stringComplete = false;

#define speed_25 111
#define speed_50 135
#define speed_75 160
#define speed_100 200

int velocity,infra,collagen,reverse,stop;
int alarm;
int recipe = 0;
int sstop = 0;
int IR_baypass = 0;
bool bit_test = false;
int task_motor = 0;
int bit_alarm = false;

#define rele_start A5
#define rele_cap A1
#define task 3
#define pin_start 4
#define lamp 2
#define back_forward A2
#define UP HIGH
#define DOWN LOW
#define led_colagen 5
#define IR_1 6
#define IR_2 7

void privod_start () {
  digitalWrite (pin_start, HIGH);
}

void privod_stop () {  
  digitalWrite (pin_start, LOW);
}

void privod_task (int n) {  
  analogWrite (task,n);
}

void privod_back () {  
  digitalWrite (back_forward, DOWN);
}

void privod_forward () {  
  digitalWrite (back_forward, UP);
}

void privod_rele_stop () {  
  digitalWrite (rele_cap, LOW);
  delay(200);
  digitalWrite (rele_start, LOW);
}

void privod_rele_start () {  
  digitalWrite (rele_cap, HIGH);
  delay(300);
  digitalWrite (rele_start, HIGH);
}

void setup() {
  stop=1;
  pinMode(back_forward, OUTPUT); 
  digitalWrite (back_forward, UP);
  pinMode(task, OUTPUT); 
  pinMode(rele_start, OUTPUT); 
  pinMode(rele_cap, OUTPUT);
  pinMode(pin_start, OUTPUT); 
  digitalWrite (pin_start, LOW);
  digitalWrite (rele_start, HIGH);
  delay(1000);
  digitalWrite (rele_cap, HIGH); 
  delay(500);
  USART_init();
  pinMode(lamp, OUTPUT); 
  digitalWrite (lamp, LOW);
  pinMode(led_colagen, OUTPUT); 
  digitalWrite (led_colagen, LOW);
  pinMode(IR_1, INPUT_PULLUP); 
  pinMode(IR_2, INPUT_PULLUP); 
  pinMode(13, INPUT_PULLUP); 
  IR_baypass = EEPROM.read(0);
  if (digitalRead(13) == LOW) { 
    privod_rele_start();
    privod_start();
    privod_forward();
    privod_task(150);
    digitalWrite (lamp, HIGH);
    analogWrite (led_colagen, 180);
    bit_test=true;
  }
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
}

void loop() {  }

void FUNCTON ()
{
  if (stop == 0 && alarm == 0) {
      privod_rele_start();
      privod_start();
      privod_task(task_motor);
    if (reverse == 0) {
      if (digitalRead(back_forward)) {
        privod_rele_stop();
        privod_stop();
        privod_task(0);
        delay(2000);
        digitalWrite(back_forward, LOW );
      }
    }
    if (reverse == 1) {
      if (!digitalRead(back_forward)) {
        privod_rele_stop();
        privod_stop();
        privod_task(0);
        delay(2000);
        digitalWrite(back_forward, HIGH);
      }
    }
  }

  if (stop == 1 || alarm == 1) {      
    privod_rele_stop();
    privod_stop();
    digitalWrite (lamp, LOW);
    analogWrite (task,0);   
    digitalWrite (pin_start, LOW);
  }

  if (infra > 0) {
    digitalWrite (lamp, HIGH);
  }
  else {
    digitalWrite (lamp, LOW);
  }

  if (collagen == 1) {
    analogWrite (led_colagen, 90);
  }
  else {
    analogWrite (led_colagen, 0);
  }

  if (IR_baypass != 0x3a) {
    static int cnt;
    if (digitalRead(IR_1) == LOW && digitalRead(IR_2) == LOW) {
      cnt = 0;
      if (bit_alarm == true) {
        bit_alarm = 0;
        alarm = 0;
        char outputUART[24];
        sprintf(outputUART, "<alarm_ok>");
        USART_TransmitString(outputUART);
        memset(outputUART, 0, sizeof(outputUART));
      }
    }
    else {
      if (cnt) {
        bit_alarm = 1;
        alarm = 1;
        char outputUART[24];
        sprintf(outputUART, "<alarm>");
        USART_TransmitString(outputUART);
        memset(outputUART, 0, sizeof(outputUART));
      }
      cnt++;
    }
  } 
}

void PARSING ()
{
  if (stringComplete) {
    stringComplete = false;
    int extractedVel, extractedInfra, extractedCol,extractedRev,extractedStop;
    if (sscanf(receivedString, "<velocity=%d,infra=%d,collagen=%d,reverse=%d,stop=%d",
        &extractedVel,&extractedInfra,&extractedCol,&extractedRev,&extractedStop) == 5) {
      velocity=extractedVel;
      infra=extractedInfra;
      collagen=extractedCol;
      reverse=extractedRev;
      stop=extractedStop;
      task_motor=velocity;

      if (task_motor > 10) {
        task_motor = map(task_motor, 0, 100, 80, 255);
      }
      if (task_motor < 10) {
        task_motor=0;
      }
      FUNCTON();
      memset(receivedString, 0, sizeof(receivedString));
    }
    int extractedIR,temp_ir;
    if (sscanf(receivedString,"<IR=%d",&extractedIR) ==1) {
      temp_ir=extractedIR;
      if(temp_ir == 1) {
        EEPROM.write(0, 0);
        IR_baypass=0;
      } if(temp_ir == 0) {
        EEPROM.write(0, 0x3a);
        IR_baypass=0x3a;
      }
    }
  }
}

void USART_Transmit(unsigned char data)
{
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

void USART_TransmitString(const char* str) {
  for (int i = 0; str[i] != '\0'; i++)
    USART_Transmit(str[i]);
}

void USART_init(void) {
  UBRR0H = (MYUBRR >> 8);
  UBRR0L = MYUBRR;
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

ISR (USART_RX_vect) {
  char receivedByte = UDR0;
  if (receivedByte == '>') {
    receivedString[receivedIndex] = '\0';  // Добавляем завершающий нулевой символ
    stringComplete = true;  // Устанавливаем флаг завершения строки
    receivedIndex = 0;  // Сбрасываем индекс
    PARSING();
  }
  else {
    receivedString[receivedIndex] = receivedByte;
    receivedIndex++;
  }
}
