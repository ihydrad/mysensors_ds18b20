#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <RF24.h>
#include <OneWire.h>

//#define DEBUG

#ifdef DEBUG
  #include <printf.h>
#endif

#define SBR(port, bit)        port |= (1<<bit)
#define CBR(port, bit)        port &= (~(1<<bit))
#define INV(port, bit)        port ^= (1<<bit)
#define SBRC(port, bit)      ((port & (1<<bit)) == 0)
#define SBRS(port, bit)      ((port & (1<<bit)) != 0)

#define TR_NUM               1
#define TR_DELMIN            30
#define DS_VCC               3
#define DS_DTA               4
#define ADCGND               A3
#define ADCBATPIN            A2
#define ADCVCC               A1
#define LED_DEBUG            7
#define DIV_ADC              2.15
#define PayloadSize          5
#define Channel              121
#define DS_POW_EN            digitalWrite(DS_VCC, 1)
#define DS_POW_DIS           digitalWrite(DS_VCC, 0) 


RF24            radio(9, 10); //4 for CE and 15 for CSN
OneWire         ds(DS_DTA);

uint8_t data[PayloadSize],
        tx_adrr[]="1Node",
        wdt_cnt = 7*TR_DELMIN,
        mcur;

void enWDT(void);
void disWDT();
void initNRF(void);
void sleep(void);
void wakeup(void);
void prepData(void);
void conf_ds18b20(void);

ISR(WDT_vect){
  wdt_cnt++;
}

void setup() {  
  mcur = MCUSR;
  MCUSR = 0;
  CLKPR=1<<CLKPCE;
  CLKPR=(0<<CLKPCE)|(0<<CLKPS3)|(0<<CLKPS2)|(1<<CLKPS1)|(1<<CLKPS0);  
  initNRF();
  //conf_ds18b20();
  analogReference(INTERNAL);
  ADCSRA = 0;
  ACSR = 0x80;  
  #ifdef DEBUG
    UCSR0A=0x02;
    UCSR0B=0x08;
    UCSR0C=0x06;
    UBRR0H=0x00;
    UBRR0L=0x0C;
    delay(1000);
    printf_begin();
    radio.printDetails();
    Serial.print("Last reset source: ");    
    Serial.println(MCUSR);
    MCUSR = 0x00;
    delay(1000);
  #endif  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep();
}

float read_ds18b20(){
  #ifdef DEBUG
    byte data[9];
  #else
    byte data[2];
  #endif
  DS_POW_EN; 
  ds.reset();
  delay(10);
  if(ds.reset()){
    ds.write(0xCC);
    ds.write(0x44);
    delay(200);
    //Читаем данные с датчика
    ds.reset();
    ds.write(0xCC);
    ds.write(0xBE);
    #ifdef DEBUG
      for(uint8_t i=0; i<9; i++)
        data[i] = ds.read();
      Serial.print("CFG DS = ");
      Serial.println(data[4], HEX);
    #else
      data[0] = ds.read();
      data[1] = ds.read();
    #endif
    DS_POW_DIS;
    int16_t raw = (data[1] << 8) | (data[0] & ~0x03);
    float temp = raw>>4;
    if(temp<0){
      temp--;
      if(raw & 0x08) temp -= 0.5L;
      if(raw & 0x04) temp -= 0.25L;
    }else{
      if(raw & 0x08) temp += 0.5L;
      if(raw & 0x04) temp += 0.25L;
    }
    return temp;
  } else return -127;
}

void conf_ds18b20(){
  DS_POW_EN;
  ds.reset();
  delay(100);
  if(ds.reset()){
    //Write Scratchpad [4Eh]
    ds.write(0xCC);  //skip rom
    ds.write(0x4E);  //write Scratchpad
    ds.write(120);  //Tl
    ds.write(120);  //Th
    ds.write(0x3f);  //Conf reg 10bit
    ds.reset();
    ds.write(0xCC);  //skip
    ds.write(0x48);  //copy to eeprom
    ds.reset();
  DS_POW_DIS;
  }
}

void initNRF(){
  //Конфигурируем SPI************************
  //SPI.setHwCs(true);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  //*****************************************
  radio.begin();                                    // Инициируем работу nRF24L01+
  radio.setChannel      (125);                      // Указываем канал приёма данных (от 0 до 127), 5 - значит приём данных осуществляется на частоте 2,405 ГГц (на одном канале может быть только 1 приёмник и до 6 передатчиков)
  radio.setDataRate     (RF24_2MBPS);               // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
  radio.setPALevel      (RF24_PA_MAX);              // Указываем мощность передатчика (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm)
  radio.disableDynamicPayloads();
  radio.setPayloadSize(5);
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setAddressWidth(5);
  radio.openWritingPipe (tx_adrr);
  radio.setCRCLength    (RF24_CRC_8);
  radio.stopListening();
}

void enWDT(){
  cli();
  wdt_reset();
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE); //WDT ISR 8sec
  WDTCSR = (1<<WDP3)| (1<<WDP0);
  WDTCSR |= bit (WDIE);
  sei();
  #ifdef DEBUG
    Serial.print(". ");
    delay(1000);
  #endif
}

void disWDT(){
  cli();
  wdt_reset();
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE); //WDT ISR 8sec
  WDTCSR = (1<<WDP3)| (1<<WDP0);
  WDTCSR  = 0;
  sei();
  #ifdef DEBUG
    Serial.print("WDT_DIS->");
    delay(1000);
  #endif
}

void sleep(){
  radio.powerDown();
  sleep_enable();
  enWDT();
  ADCSRA = 0;
  DDRB = 0XFF;
  DDRD = 0XFF;
  DDRC = 0XFF; 
  PORTB  = 0x00;
  //PORTD  = 0x00;
  PORTC  = 0x00;
  // turn off brown-out enable in software
  MCUCR = bit(6)|bit(5);
  MCUCR = bit(6);
  sleep_cpu();
}

void wakeup(){
  //disable sleep
  sleep_disable();
  disWDT();
  radio.powerUp();
  pinMode(ADCBATPIN, INPUT);
  pinMode(ADCVCC, OUTPUT);digitalWrite(ADCVCC, 1);
  pinMode(ADCGND, OUTPUT);digitalWrite(ADCGND, 0);
  ADCSRA = 0x87; //62Khz adc
}

void prepData(){
  data[0] = TR_NUM;
  if(mcur) {
    data[0] |= mcur<<4;
    mcur=0;   
  }
  int16_t temp = read_ds18b20()*100;
  int16_t bat = float(analogRead(ADCBATPIN))/DIV_ADC;
  data[1] = temp>>8;
  data[2] = temp&0xFF;  
  data[3] = bat>>8;  
  data[4] = bat&0xFF;

   #ifdef DEBUG    
      Serial.print("STATE:");
      Serial.println(data[0], HEX);
      Serial.print("ADCBAT:   ");
      Serial.println(bat/100.0F);
      Serial.print("Temp:     ");
      Serial.println(temp/100.0F);  
      
      for(uint8_t i = 0;i < 5; i++ ){
        Serial.print(data[i], HEX);
        Serial.print(';');
      }
        
      Serial.println();    
  #endif 
}

void loop() {
  if(wdt_cnt > (7*TR_DELMIN)){    // 7,4*8sec = 1min            
    wdt_cnt = 0;    
    wakeup();    
    SBR(DDRD, LED_DEBUG);
    SBR(PORTD, LED_DEBUG);
    prepData();
    radio.write(&data, PayloadSize);      
    CBR(DDRD, LED_DEBUG);
    CBR(PORTD, LED_DEBUG);
    #ifdef DEBUG
      delay(1000);
    #endif    
  }                                             
    sleep();                                                        
}
