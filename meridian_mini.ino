//Version 1.01 Meridian-mini - Test Generation!

#include <LiquidCrystal_I2C.h>
#include <MsTimer2.h>
#include <iarduino_RF433_Transmitter.h>
#include "Wire.h"

#define MAX5381 0x64
//const int SOURCE = A5;        // ARBITRARY = 1 OR AD9833 = 0
const int FSYNC = 10;
const int SDATA = 11;
const int SCLK = 13;
const float CRYSTAL = 24000000.0 ;
const int SINE = 0x2000;
const int SQUARE = 0x2020;
const int TRIANGLE = 0x2002;
unsigned long FREQ = 1000;


char text[5];
boolean nec_ok = 0;
byte  i, nec_state = 0, command, inv_command;
unsigned int address;
unsigned long nec_code;

#define PIN_nch 3 // генерация ВЧ? опционное
#define PIN_vch 9 // управление пропуском частоты ВЧ - управление подачей НЧ
#define CS 5 // управление регулятором уровня 5
#define UD 6 // управление регулятором уровня 6
#define PIN_ADC 14 // вход АЦП A0

#define PIN_1 16 // кнопка 1
#define PIN_2 15 // кнопка 3
#define PIN_3 17 // кнопка 3
#define PIN_4 7 // кнопка 4
#define PIN_5 8 // кнопка 5

#define rele 4 // порт реле
#define PIN_TX433 1 // pin вывода радиоканала 433МГц

int j;
int t=0;
float I=0;
int p=0; //уровень
int r=1; // номер режима
int rf=0;
int c=0;
int ch=0; //счетчик таймера - десятая секунды
int f=2; //режим частоты PWM
int adc=0;
int ind=0; //флаг отображения индикации каждую 1 сек
int rab=0; // режим работы таймера
int x2=0; //режим усилителя 0-без него  1- включить усилитель
long int nf=50; //частота НЧ

long int ss=0, sec=0, minut=0;

const int Sound=12; // 12 pin динамик

//----433gh
//int number; // число передачи
//char symbol; // символ команды передачи
//const int transmit_pin = PIN_TX433; // Пин подключения передатчика A2
iarduino_RF433_Transmitter radio(PIN_TX433);  // Создаём объект radio для работы с библиотекой iarduino_RF433, указывая номер вывода к которому подключён передатчик
int data[5]; //буфер данных:  0 байт- режим (1-3), 1 байт- Уровень, 2 байт - Сила тока(ma*10 и округлена), 3 байт- время мин, 4 байт - время сек

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display 0x3F для синегоj



void UpdateDDS(unsigned int data){
  unsigned int pointer = 0x8000;
  // Serial.print(data,HEX);Serial.print(" ");
  digitalWrite(FSYNC, LOW);     // AND NOW : WAIT 5 ns
  for (int i=0; i<16; i++){
   if ((data & pointer) > 0) { digitalWrite(SDATA, HIGH); }
      else { digitalWrite(SDATA, LOW); }
    digitalWrite(SCLK, LOW);
    digitalWrite(SCLK, HIGH);
    pointer = pointer >> 1 ;
  }
  digitalWrite(FSYNC, HIGH);
}

void UpdateFreq(long FREQ, int WAVE){

  long FTW = (FREQ * pow(2, 28)) / CRYSTAL;
  if (WAVE == SQUARE) FTW = FTW << 1;
  unsigned int MSB = (int)((FTW & 0xFFFC000) >> 14);
  unsigned int LSB = (int)(FTW & 0x3FFF);
  LSB |= 0x4000;
  MSB |= 0x4000;
  UpdateDDS(0x2100);
  UpdateDDS(LSB);
  UpdateDDS(MSB);
  UpdateDDS(0xC000);
  UpdateDDS(WAVE);
}


void up(int n)
{
  digitalWrite(UD, HIGH);
  delay(1);
  digitalWrite(CS, LOW);
  delay(1);
  for (c=0; c<n; c++)
  {
   digitalWrite(UD, LOW);
   delay(1);
   digitalWrite(UD, HIGH);
   delay(1);
  }
  digitalWrite(CS, HIGH);
}

void dn(int n)
{
  digitalWrite(UD, LOW);
  delay(1);
  digitalWrite(CS, LOW);
  delay(1);
  for (c=0; c<n; c++)
  {
   digitalWrite(UD, HIGH);
  delay(1);
   digitalWrite(UD, LOW);
   delay(1);
  }
  digitalWrite(CS, HIGH);
}


void remote_read() {
unsigned int timer_value;
  if(nec_state != 0){
    timer_value = TCNT1;                         // Store Timer1 value
    TCNT1 = 0;                                   // Reset Timer1
  }
  switch(nec_state){
   case 0 :                                      // Start receiving IR data (we're at the beginning of 9ms pulse)
    TCNT1  = 0;                                  // Reset Timer1
    TCCR1B = 2;                                  // Enable Timer1 module with 1/8 prescaler ( 2 ticks every 1 us)
    nec_state = 1;                               // Next state: end of 9ms pulse (start of 4.5ms space)
    i = 0;
    return;
   case 1 :                                      // End of 9ms pulse
    if((timer_value > 19000) || (timer_value < 17000)){         // Invalid interval ==> stop decoding and reset
      nec_state = 0;                             // Reset decoding process
      TCCR1B = 0;                                // Disable Timer1 module
    }
    else
      nec_state = 2;                             // Next state: end of 4.5ms space (start of 562µs pulse)
    return;
   case 2 :                                      // End of 4.5ms space
    if((timer_value > 10000) || (timer_value < 8000)){
      nec_state = 0;                             // Reset decoding process
      TCCR1B = 0;                                // Disable Timer1 module
    }
    else
      nec_state = 3;                             // Next state: end of 562µs pulse (start of 562µs or 1687µs space)
    return;
   case 3 :                                      // End of 562µs pulse
    if((timer_value > 1400) || (timer_value < 800)){           // Invalid interval ==> stop decoding and reset
      TCCR1B = 0;                                // Disable Timer1 module
      nec_state = 0;                             // Reset decoding process
    }
    else
      nec_state = 4;                             // Next state: end of 562µs or 1687µs space
    return;
   case 4 :                                      // End of 562µs or 1687µs space
    if((timer_value > 3600) || (timer_value < 800)){           // Time interval invalid ==> stop decoding
      TCCR1B = 0;                                // Disable Timer1 module
      nec_state = 0;                             // Reset decoding process
      return;
    }
    if( timer_value > 2000)                      // If space width > 1ms (short space)
      bitSet(nec_code, (31 - i));                // Write 1 to bit (31 - i)
    else                                         // If space width < 1ms (long space)
      bitClear(nec_code, (31 - i));              // Write 0 to bit (31 - i)
    i++;
    if(i > 31){                                  // If all bits are received
      nec_ok = 1;                                // Decoding process OK
      detachInterrupt(0);                        // Disable external interrupt (INT0)
      return;
    }
    nec_state = 3;                               // Next state: end of 562µs pulse (start of 562µs or 1687µs space)
  }
}

ISR(TIMER1_OVF_vect) {                           // Timer1 interrupt service routine (ISR)
  nec_state = 0;                                 // Reset decoding process
  TCCR1B = 0;                                    // Disable Timer1 module
}

void fun()
{
    nec_ok = 0;                                  // Reset decoding process
    nec_state = 0;
    TCCR1B = 0;                                  // Disable Timer1 module
    address = nec_code >> 16;
    command = nec_code >> 8;
    inv_command = nec_code;

   // lcd.print(command);
    attachInterrupt(0, remote_read, CHANGE);     // Enable external interrupt (INT0)
 }

void funwork()
{
   adc= analogRead(PIN_ADC); I=adc; //I=(3.2*adc)/20.48; //if (ch==0 || ch==12 || ch==25 || ch==50) I=(3.2*adc)/20.48;

 //-------------Таймер--------------
 if(rab==1)
 {
  ss=ss+1; if (ss>=72000) ss=0;
  if((ss%20)==0)
  { ind=1; sec=sec+1; if(sec>=60) sec=0;
  }  else ind=0;
  minut=floor(ss/1200);
 }

  // if(j==0) {j=1;digitalWrite(13, LOW);} else{j=0;digitalWrite(13, HIGH);}
  ch++; if(ch>=52) {ch=0;} //
   if(ch==0)
    {
      Serial.println(adc);// передать данные в порт

      digitalWrite(13, HIGH);
     //передача по 433МГц из буфера данных:  0 байт- режим (1-3), 1 байт- Уровень, 2 байт - Сила тока(ma*10 и округлена), 3 байт- время мин, 4 байт - время сек
    data[0] =r;
    data[1] = p;

    radio.write(&data, sizeof(data));                     // отправляем данные из массива data указывая сколько байт массива мы хотим отправить
    digitalWrite(13, LOW);

     Serial.println(p); if(ch==25) Serial.println(r+100); //выброс данных в порт о интенсивности и номере режима- 101, 102, 103
    }
  if (r==3 && rf==0) { if((ch%2)==0) digitalWrite(PIN_vch, LOW);   else digitalWrite(PIN_vch, HIGH);}
  if (r==2 && rf==0) { if(ch==0) digitalWrite(PIN_vch, LOW);   if(ch==26) digitalWrite(PIN_vch, HIGH);}
 // if (r==4&& rf==0) {if(ch==0) dn(p);}


}

void setup() {

//Подтянуть кнопки
 pinMode(PIN_1, INPUT);
 pinMode(PIN_2, INPUT);
 pinMode(PIN_3, INPUT);
 pinMode(PIN_4, INPUT);
 pinMode(PIN_5, INPUT);

 digitalWrite(PIN_1, HIGH);
 digitalWrite(PIN_2, HIGH);
 digitalWrite(PIN_3, HIGH);
 digitalWrite(PIN_4, HIGH);
 digitalWrite(PIN_5, HIGH);
//-----------------

pinMode(PIN_nch, OUTPUT);
pinMode(PIN_vch, OUTPUT);
digitalWrite(PIN_nch, LOW); // разблокировать автономный 555 генератор
digitalWrite(PIN_vch, HIGH); //открыть подачу ВЧ сигнала с 555

pinMode(rele, OUTPUT);
digitalWrite(rele, LOW); // установить реле в 0 - без режим удвоения сигнала

pinMode(CS, OUTPUT);           // назначить выводу порт уровня
digitalWrite(CS, HIGH);
pinMode(UD, OUTPUT);           // назначить выводу порт уровня
digitalWrite(UD, HIGH);
dn(64);


  //установка DDS по умолчанию 50Гц
 // pinMode(SOURCE, OUTPUT);
  pinMode(FSYNC, OUTPUT);
  pinMode(SDATA, OUTPUT);
  pinMode(SCLK, OUTPUT);
 // digitalWrite(SOURCE, LOW);
  digitalWrite(FSYNC, HIGH);
  digitalWrite(SDATA, LOW);
  digitalWrite(SCLK, HIGH);
  UpdateDDS(0x2100);
  UpdateDDS(0x50C7);
  UpdateDDS(0x4000);
  UpdateDDS(0xC000);
  UpdateDDS(0x2000);
  // SWITCH TO SOURCE = DDS
 // digitalWrite(SOURCE, LOW);
  // NOW THERE SHOULD BE A
  // 384 Hz SIGNAL AT THE OUTPUT
  delay(200);
   UpdateFreq(nf, SINE);


   radio.begin();                                        // Инициируем работу передатчика FS1000A (в качестве параметра можно указать скорость ЧИСЛО бит/сек, тогда можно не вызывать функцию setDataRate)
   radio.setDataRate     (i433_500BPS);                   // Указываем скорость передачи данных (i433_5KBPS, i433_4KBPS, i433_3KBPS, i433_2KBPS, i433_1KBPS, i433_500BPS, i433_100BPS), i433_1KBPS - 1кбит/сек
   radio.openWritingPipe (5); //5труба

 //pinMode(PIN_ADC, INPUT);


 lcd.init();                      // initialize the lcd
 lcd.backlight();
 lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Meridian-2019");
  delay(2000);

  lcd.clear();
  lcd.setCursor(2, 0);
      lcd.print("Meridian  ");
      lcd.setCursor(11, 0);
      lcd.print(p);

  Serial.begin(9600);

// Timer1 module configuration
  TCCR1A = 0;
  TCCR1B = 0;                                    // Disable Timer1 module
  TCNT1  = 0;                                    // Set Timer1 preload value to 0 (reset)
  TIMSK1 = 1;                                    // enable Timer1 overflow interrupt
  attachInterrupt(0, remote_read, CHANGE);       // Enable external interrupt (INT0)
  MsTimer2::set(25, funwork); // 25ms period*2 (так работает после функции tone)!!!!!
  MsTimer2::start();
  tone(Sound,1000,100);
 //tone(Sound,1047,300);
 //tone(Sound,1319,300);
 //tone(Sound,1568,300);
 //tone(Sound,2093,1000);

}

void loop() {

 if(r==1 && digitalRead(PIN_2)==1) {r=2; rf=0; fun_ind(); tone(Sound,1000,100);delay(200);}
 if(r==2 && digitalRead(PIN_1)==1) {r=1; rf=0; fun_ind(); tone(Sound,1000,100);delay(200);}
 if(r==2 && digitalRead(PIN_2)==1) {r=3; rf=0; fun_ind();tone(Sound,1000,100);delay(200); }
 if(r==3 && digitalRead(PIN_1)==1) {r=2; rf=0; fun_ind();tone(Sound,1000,100);delay(200);}

 if((digitalRead(PIN_2)==1) && (digitalRead(PIN_1)==1)) { //если нажаты обе кнопки <> режима -  включить/выключить усилитель
   if (x2==0) {tone(Sound,261,100); digitalWrite(rele, HIGH);  x2=1;} // установить реле в 1 - включить усилитель
    else {tone(Sound,524,100); digitalWrite(rele, LOW);  x2=0;} // установить реле в 0 - выключить усилитель
    fun_ind();
    delay(200);
    }



 if(digitalRead(PIN_3)==1)
  {
     tone(Sound,900,200);
      p=p+1;
      if(p>64)p=64; else up(1);
     // analogWrite(pwm, p);

    //  lcd.clear();
    fun_ind();
      lcd.setCursor(0, 1);
      lcd.print(p);
      delay(100);
    }

 if(digitalRead(PIN_4)==1)
  {
     tone(Sound,1000,200);
      p=p-1;
      if(p<0)p=0; else dn(1);

     fun_ind();
     // analogWrite(pwm, p);

      //lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print(p);
      delay(100);
    }

if(digitalRead(PIN_5)==1)
  {

    if (rab==0) {tone(Sound,261,100); dn(64); up(p); ss=0, sec=0, minut=0; rab=1;} //запустить работу с установленого уровня и отчет
    else {tone(Sound,524,100); dn(64); r=1; nf=50; ss=0, sec=0, minut=0; rab=0;} // обнулить таймер и перейти в режим 1 и установить уровень 0
    fun_ind();
    delay(300);
  }


 if(nec_ok)
 {

  fun(); //функция завершения обработки импульсов ИК


  // обработка переменной команды
  if (command == 194) //включить/выключить
  {

    if (rab==0) {tone(Sound,261,100); dn(64); up(p); ss=0, sec=0, minut=0; rab=1;} //запустить работу с установленого уровня и отчет
    else {tone(Sound,524,100); dn(64); r=1; nf=50; ss=0, sec=0, minut=0; rab=0;} // обнулить таймер и перейти в режим 1 и установить уровень 0
    fun_ind();
    delay(200);

     }

  if (command == 168) // ++уровень
  {
      tone(Sound,900,100);
      p=p+1;
      if(p>64)p=64; else up(1);
     // analogWrite(pwm, p);

    //  lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print(p);
     }

  if (command == 224) //  --уровень
  {
      tone(Sound,1000,100);
      p=p-1;
      if(p<0)p=0; else dn(1);
     // analogWrite(pwm, p);

      //lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print(p);
     }

   if (command == 104) // 0 уровень
  {
     tone(Sound,1000,100);
     p=0;
     dn(64);
    //  lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print(p);

     }

   if (command == 152) // 25 уровень
  {
      tone(Sound,1000,100);
      p=25;
      dn(64);
      up(25);

    //  lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print(p);
    }

   if (command == 176) // уровень 60 -макс
  {
       tone(Sound,1000,100);
       p=60;
      dn(64);
      up(60);
    //  lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print(p);

     }

  if (command == 48) // Режим 1 - Meridian
  {

      tone(Sound,1000,100);
      r=1;
      rf=0;
      if(nf>200) {nf=50; UpdateFreq(50, SINE);}

     }

  if (command == 24) //  Режим 2 - Interval
  {
      tone(Sound,1000,100);
      r=2;
      rf=0;
      if(nf>200) {nf=50; UpdateFreq(50, SINE);}
     }


 if (command == 122) //  Режим 3 - Pulsed
  {
      tone(Sound,1000,100);
      r=3;
      rf=0;
      if(nf>200) {nf=50; UpdateFreq(50, SINE);}
     }

  if (command == 16) //  Режим 4
  {
      tone(7,1000,100);
      r=4;
      rf=0;
     }

   if (command == 162) // Режим 20Гц
  {
      tone(Sound,1000,100);
      digitalWrite(PIN_nch, LOW); // разблокировать автономный 555 генератор
      f=1;
      nf=20;
      UpdateFreq(20, SINE);
      Serial.println(101);
     }
  if (command == 98) //  Режим 50Гц
  {
     tone(Sound,1000,100);
     digitalWrite(PIN_nch, LOW); // разблокировать автономный 555 генератор
     f=2;
     nf=50;
      UpdateFreq(50, SINE);
     Serial.println(102);
     }

  if (command == 226) //  Режим 77Гц
  {
     tone(Sound,1000,100);
     digitalWrite(PIN_nch, LOW); // разблокировать автономный 555 генератор
     f=3;
     nf=77;
      UpdateFreq(77, SINE);
     Serial.println(103);
     }

  if (command == 34) // Режим -1Гц  <<
  {
      tone(Sound,1000,100);
      nf=nf-1; if(nf<=5) nf=5;
      UpdateFreq(nf, SINE);
      //Serial.println(101);
     }
  if (command == 2) // Режим +1Гц  <<
  {
      tone(Sound,1000,100);
      nf=nf+1; if(nf>=100) nf=150;
      UpdateFreq(nf, SINE);
      //Serial.println(101);
     }

  if (command == 144) //  включить усилитель x2!
  {

    if (x2==0) {tone(Sound,261,100); digitalWrite(rele, HIGH);  x2=1;} // установить реле в 1 - включить усилитель
    else {tone(Sound,524,100); digitalWrite(rele, LOW);  x2=0;} // установить реле в 0 - выключить усилитель
    fun_ind();
    delay(200);

  }

   if (command == 82) //  Режим 300 кГц с усилителем... включить усилитель!
  {
     tone(Sound,1000,100);
     digitalWrite(PIN_nch, HIGH); // заблокировать автономный 555 генератор
     UpdateFreq(300000, SINE);
     nf=300000;
     r=9;
     //Serial.println(103);
     }

  if (command == 66) // Режим цифоры 7 - звук
  {
     tone(Sound,1047,300);
     tone(Sound,1319,300);
      tone(Sound,1568,300);
      tone(Sound,2093,1000);
     }

  // Обработка дополнительного кода вывода
   fun_ind();

}


//-------------------Вывод параметров------

      // вывод тока
   /*   lcd.setCursor(0, 1);
      lcd.print("I:");
      lcd.print(I);
      lcd.print(" ");
      lcd.print(nf);
      lcd.print("H");
    */
  if(r==9)
      {
      lcd.setCursor(0, 1);
      lcd.print("magnetic");
        }
      else
      {
      lcd.setCursor(0, 1);
      lcd.print(nf);
      lcd.print("Hz");
      }
 //вывод времени посекундно
  if((ind)==1)
    {
     lcd.setCursor(0, 1);
     lcd.print("                ");


     // lcd.setCursor(5, 1);
     // lcd.print("  ");
    //  lcd.print("      ");
      lcd.setCursor(11, 1);
      lcd.print(minut);
      lcd.print(":");
      lcd.print(sec);
    }

   if((minut>=45)&& (minut<=46)) { tone(Sound,2093,1000); rab=0;}

}


void fun_ind() // Обработка дополнительного кода вывода и индикация
{
  if(r==1)
  {

      lcd.clear();
      lcd.setCursor(2, 0);
      lcd.print("Meridian  ");
      lcd.setCursor(11, 0);
      lcd.print(p);

      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(11, 1);
      lcd.print(minut);
      lcd.print(":");
      lcd.print(sec);
       digitalWrite(PIN_vch, HIGH); // в этом режиме всегда разрешается подача сигнала от генератора

  }

  if(r==2)
  {

      lcd.clear();
      lcd.setCursor(2, 0);
      lcd.print("Interval  ");
      lcd.setCursor(11, 0);
      lcd.print(p);

      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(11, 1);
      lcd.print(minut);
      lcd.print(":");
      lcd.print(sec);

  }

  if(r==3)
  {

      lcd.clear();
      lcd.setCursor(2, 0);
      lcd.print("  Pulsed  ");
      lcd.setCursor(11, 0);
      lcd.print(p);

      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(11, 1);
      lcd.print(minut);
      lcd.print(":");
      lcd.print(sec);
  }

  if(r==9) // режим 300000 кГц
  {
      lcd.clear();
      lcd.setCursor(2, 0);
      lcd.print("  300kHz  ");
      lcd.setCursor(11, 0);
      lcd.print(p);

      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(11, 1);
      lcd.print(minut);
      lcd.print(":");
      lcd.print(sec);
  }

  if(f==1)
  {
    //  lcd.setCursor(8, 1);
    //  lcd.print("A");

  }

  if(f==2)
  {
    // lcd.setCursor(8, 1);
    // lcd.print("B");
  }

  if(f==3)
  {
    // lcd.setCursor(8, 1);
    // lcd.print("C");
  }

}
