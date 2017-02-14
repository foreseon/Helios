//Kütüphaneler
#include <Servo.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "pitches.h"
#include <SoftwareSerial.h>


//LCD tanımlama
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#define rxPin 10
#define txPin 11

SoftwareSerial soundSerial = SoftwareSerial(rxPin, txPin);
//servo init
  #define  servomaxx   180  
  #define  servomaxy   180  
  int  screenmaxx = 640;
  #define  screenmaxy  430     
  #define  servocenterx   120  
  #define  servocentery   40
  #define  servopinx   32
  #define  servopiny   33 
   
  #define distancex 2  
  #define distancey 1
   
  
  
  int posx = 0;
  int posy = 0;
  int incx = 45;  
  int incy = 45;  
  int valx = 0;       
  int valy = 0;
//---Motors---
// Motor 1
int dir1PinA = 9;
int dir2PinA = 8;
int speedPinA = 13; 

// Motor 2
int dir1PinB = 7;
int dir2PinB = 6;
int speedPinB = 12; 

// Motor 3
int dir1PinC = 1;
int dir2PinC = 0;
int speedPinC = 5; 

// Motor 4
int dir1PinD = 2;
int dir2PinD = 3;
int speedPinD = 4; 
 
bool motorTurnAut = true;
long interval_MotorTurnAut = 0;
unsigned long previousMillis_MotorTurnAut = 0;

bool motorTurnLeft = false;
long interval_MotorTurnLeft = 0;
unsigned long previousMillis_MotorTurnLeft = 0;

bool motorTurnRight = false;
long interval_MotorTurnRight = 0;
unsigned long previousMillis_MotorTurnRight = 0;

bool turnComplete = true;
  
  //-------FaceTracking Variables----- 
 
  int xDirection_FaceTracking = 1;
  int yDirection_FaceTracking = 0;
  bool xFlag_FaceTracking = false;
  bool faceTracking = false;
  bool followFace = false;
  bool faceOriantedX = false;

 //Timerlar için zaman tanımlamaları
  unsigned long previousMillis_FaceReset = 0;
   unsigned long previousMillis_FaceSearchAttack = 0;
  unsigned long previousMillis_FaceFind = 0;
  const long interval_FaceReset = 300;
  const long interval_FaceFind = 60;
  const long interval_FaceSearchAttack = 150;
  //Yuz takip değişkenler
  bool faceAttackTimerControl = true;
  bool faceFindTimerEnabled = false;
  int maxY_FaceTracking = 40;
  int minY_FaceTracking = 40; 
  short MSB = 0; 
  short LSB = 0;  
  bool faceDetected = false;
  int   MSBLSB = 0; 
 // -----functions------
 //----------------------
//---------LCD variables
String line1;
String line2;
int start1;
int start2;
//--------end LCD variables
 
  //----End FaceTracking Variables----
   
//---------------TEST------
 unsigned long previousMillis_TESTX = 0;
  const long interval_TESTX = 15000;
 bool testX_Enabled = true;

//------END TEST -----
  
  int currentY = maxY_FaceTracking;
  int xDirectionCounter = 0;
  
 bool testSwitch = true; 
Servo servox;
Servo servoy;
//-----Speaker------
const int speaker = 34;
//------------------
//-----------Sonic Sensors----
//1
const int trigPin1 = 22; 
const int echoPin1 = 23;
//2
const int trigPin2 = 24; 
const int echoPin2 = 25;
//3
const int trigPin3 = 26; 
const int echoPin3 = 27;
//4
const int trigPin4 = 28; 
const int echoPin4 = 29;
//5
const int trigPin5 = 50; 
const int echoPin5 = 51;
//6
const int trigPin6 = 48; 
const int echoPin6 = 49;
//7
const int trigPin7 = 46; 
const int echoPin7 = 47;
//8
const int trigPin8 = 30; 
const int echoPin8 = 31;

int trigState = LOW; 

int interval = 12; 
unsigned long previousMillis_SonicSensor1 = 0; 
unsigned long previousMillis_SonicSensor2 = 0;
unsigned long previousMillis_SonicSensor3 = 0;
unsigned long previousMillis_SonicSensor4 = 0;
unsigned long previousMillis_SonicSensor5 = 0;
unsigned long previousMillis_SonicSensor6 = 0;
unsigned long previousMillis_SonicSensor7 = 0;
unsigned long previousMillis_SonicSensor8 = 0;

//--------------------------



//------------CONSOLE--------
   byte MOTOR_FORWARD_BACKWARD;
   byte MOTOR_LEFT_RIGHT;
   byte Y_UP_DOWN;
   byte X_LEFT_RIGHT;

   int MOTOR_MOVE_DATA;
   int MOTOR_TURN_DATA;
   int Y_DATA;
   int X_DATA;
//----------------------------

  //---------------MANUEL
  bool manualControl = true;
//Bir kere çalıştırılacak setup()
void setup()
{
  //MOTOR PINS
  pinMode(dir1PinA,OUTPUT);
pinMode(dir2PinA,OUTPUT);
pinMode(speedPinA,OUTPUT);

pinMode(dir1PinB,OUTPUT);
pinMode(dir2PinB,OUTPUT);
pinMode(speedPinB,OUTPUT);


pinMode(dir1PinC,OUTPUT);
pinMode(dir2PinC,OUTPUT);
pinMode(speedPinC,OUTPUT);

pinMode(dir1PinD,OUTPUT);
pinMode(dir2PinD,OUTPUT);
pinMode(speedPinD,OUTPUT);
 //Serial2.begin(9600);
 //İletişim için kulanılıcak portların açılması
 pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
  Serial1.begin(4800);
  Serial2.begin(9600);
  Serial3.begin(9600);
  //----pinModes(OUTPUT -INPUT tanımlamaları)
  pinMode(servopinx,OUTPUT);   
  pinMode(servopiny,OUTPUT);   
  pinMode(speaker, OUTPUT); 
  
  pinMode(trigPin1,OUTPUT); 
  pinMode(echoPin1,INPUT);
  
  pinMode(trigPin2,OUTPUT); 
  pinMode(echoPin2,INPUT);

  pinMode(trigPin3,OUTPUT); 
  pinMode(echoPin3,INPUT);
  
  pinMode(trigPin4,OUTPUT); 
  pinMode(echoPin4,INPUT);

  pinMode(trigPin5,OUTPUT); 
  pinMode(echoPin5,INPUT);
  
  pinMode(trigPin6,OUTPUT); 
  pinMode(echoPin6,INPUT);
  
  pinMode(trigPin7,OUTPUT); 
  pinMode(echoPin7,INPUT);
  
  pinMode(trigPin8,OUTPUT); 
  pinMode(echoPin8,INPUT);
  
//Servoların takıldığı pinlerin belirtilmesi
  servoy.attach(servopiny); 
  servox.attach(servopinx);
  //PlayImperialMarch();
//StartFaceTracking();

 
 lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
   
/*Alttaki ilk while döngüsünde servoY ikincisinde servoX servoların açısı yavaş yavaş zaman içinde artılılarak merkezlenir, kameranın sarılması azaltılır)*/
  int i = servoy.read();
  while(i != servocentery){
  
  if(i < servocentery)
  i++;
  else
  i--;
servoy.write(i);
delay(5);
i = servoy.read();
 
}
i = servox.read();
 while(i != servocenterx){
  
  if(i < servocenterx)
  i++;
  else
  i--;
servox.write(i);
delay(5);
i = servox.read();
delay(15); 
}  

 

    soundSerial.begin(9600);

 LCD_Display("SISTEM",4,"BASLATILIYOR...",2);
  line1 = "SISTEM";
  line2 = "BASLATILIYOR...";
  


  LCD_Display("KOMUT ALMAYA",2,"HAZIR",4);
  line1 = "KOMUT ALMAYA";
  line2 = "HAZIR";
   
  
}

//Sürekli dönecek olan loop(while(true) gibi, built in fonksiyondur setup gibi
//---------------------------loop------
/* loop() ÇALIŞMA MANTIĞI: bool faceTracking, bool followFace düzenli olarak kontrol edilir, StartStopBluetooth() fonskiyonu düzenli çağırılır. faceTracking true ise 
TrackFace(): Yuz koordinatlarının alındığı, servoların ona göre düzenlendiği., FaceResetTimer(): bool faceDetected değişkenini düzenli olarak false eşitleyen fonskiyon- timer gibi çalışır, 
her çağırıldığında çalışmaz ama içersindeki zaman değişkenini düzenler zaman belirli bir sınırı açınca çalışır(ileride daha ayrıntılı açıklandı).

bool faceFindTimerEnabled == true ise FaceFindCOntrolTimer(): Kamera tarafından bir yüz bulunamamış ise yuz aramasını başlatan timer gibi çalışan fonksiyon(ÖRNEK: Her 200ms de bir yüz bulunmadıkça servoların konumunu değiştiryor).

bool followFace == true ise FollowFace(): Yuzun algılanıp, robotun tekerler aracılığı ile yüz doğru dönmesi, kamera ortalandığında( yüz robota göre merkezlendiğinde) ilerlemesini sağlayan fonksiyon.

StartStopBLuetooth(): HC06 bluetooth modulünden ses komutları için veri okuyan fonskiyon.

*/
int xDegree = 0;
bool select = true;
void XdegreeReader()
{
  if (Serial1.available() >= 2)
 {
 byte rawData2=  Serial1.read();
 byte rawData1=  Serial1.read();
 xDegree = word(rawData2,rawData1);
 String DATA = (String) xDegree;
 
  lcd.clear();
  lcd.setCursor(0,0);
    lcd.print(DATA);
 }
}
void loop() 
{    

    
  XdegreeReader(); 
  
  MotorTurnAutTimer();  
if(faceTracking)
{
  TrackFace();
  FaceResetTimer();
  if(faceFindTimerEnabled){
     FaceFindControlTimer();
  }
  //XTEST();
} 

if(followFace)
{
  FollowFace();
}
if(manualControl)
{
  ManualControl();
}


StartStopBluetooth();
 
//MotorControl();



}
//-----------------------end loop------

void ManualControl()
{
  if(Serial3.available() >= 4)
{
  delay(10);
  X_LEFT_RIGHT = Serial3.read();
  delay(10);
  Y_UP_DOWN = Serial3.read();
  delay(10);
  MOTOR_LEFT_RIGHT = Serial3.read();
  delay(10);
 MOTOR_FORWARD_BACKWARD = Serial3.read();
  
  
  
  
  

  X_DATA = word(00000000,X_LEFT_RIGHT);
  Y_DATA = word(00000000,Y_UP_DOWN);
  MOTOR_LEFT_RIGHT = word(00000000,MOTOR_LEFT_RIGHT);
  MOTOR_MOVE_DATA = word(00000000,MOTOR_FORWARD_BACKWARD);
 if(MOTOR_MOVE_DATA > 128)
  {
    GoBackward(125);
    if(MOTOR_MOVE_DATA > 178)
  {
    GoBackward(150);
  }
  if(MOTOR_MOVE_DATA > 208)
  {
     GoBackward(200);
  }
  if(MOTOR_MOVE_DATA == 255)
  {
      GoBackward(255);
  }
  }  
  else if (MOTOR_MOVE_DATA < 128)
  {
   GoForward(120);
    if(MOTOR_MOVE_DATA < 78)
  {
     GoForward(150);
  }
  if(MOTOR_MOVE_DATA > 50)
  {
     GoForward(200);
  }
  if(MOTOR_MOVE_DATA == 0)
  {
      GoForward(255);
  }
  }  






   if(MOTOR_LEFT_RIGHT > 128)
  {
    TurnRight(120);
    if(MOTOR_LEFT_RIGHT > 178)
  {
    TurnRight(150);
  }
  if(MOTOR_LEFT_RIGHT > 208)
  {
    TurnRight(200);
  }
  if(MOTOR_LEFT_RIGHT == 255)
  {
     TurnRight(255);
  }
  }  
  else if (MOTOR_LEFT_RIGHT < 128)
  {
   TurnLeft(120);
    if(MOTOR_LEFT_RIGHT < 78)
  {
    TurnLeft(150);
  }
  if(MOTOR_LEFT_RIGHT > 50)
  {
    TurnLeft(200);
  }
  if(MOTOR_LEFT_RIGHT == 0)
  {
     TurnLeft(255);
  }
  }  

  if(MOTOR_LEFT_RIGHT == 128 && MOTOR_FORWARD_BACKWARD == 128)
  {
    Stop();
  }
  
  
  if(X_DATA > 128)
  {
  servox.write(servox.read() - 3);
    if(X_DATA > 210)
  {
    servox.write(servox.read() - 2);
  }
  if(X_DATA == 255)
  {
    servox.write(servox.read() - 2);
  }
  }   
  else if (X_DATA < 128)
  {
    servox.write(servox.read() + 3 );
    
    if(X_DATA < 55)
  {
    servox.write(servox.read() + 2);
  }
  if(X_DATA == 0)
  {
    servox.write(servox.read() + 2);
  } 
  }  

    if(Y_DATA > 128)
  {
  servoy.write(servoy.read() + 3); 
  
  }
  else if (Y_DATA < 128)
  {
    servoy.write(servoy.read() - 3 );  
  }  
}
}
/*StartStopBLuetooth() 
 * AMACI: Bluetooth veri paketi şeklinde içerisinde Ses komutunu String olarak taşıyan paketlerin okunması, komutun anlaşılıp gerekli fonskiyonların başlatılması.
 * 
 * 
 * ÇALIŞMA MANTIĞI: 
 String voice tanımlanır.
 Serial1 portunda okunabilecek hazırda veri var ise, bütün veri( ses komutu) okunana kadar while döngüsüne girilir.
 while döngüsü içerisinde 10ms aralıklarla ses komutu harf harf okunur ve voice içerisinde kayıt edilir.
 '#' harfi okunduğunda ses komutunun sonuna geldiği anlaşılır(Komut sonunda '#' sembolü ile gönderilmelidir).

 Komutuz uzunlupu kontrol edilir, eğer 0 dan büyük ise UYARI:ClearBluetoothBuffer(): EN SON AÇIKLANACAK.
 voice değişkeni if elseler ile kontrol edilir, gerekli fonksiyonlar çağırılır.

 ClearBluetoothBuffer(): HC06 ses komutlarını alan bluetooth moduludur Serial1 portunu kullanır bu fonksiyon bu modulu etkilemez,
 yüz takibi ve bilgisayar ile iletişim için kullandığım 2 adet HC05 bluetooh modulum vardır bu fonksiyon robota bağlı olan yani alıcı olan HC05' in bufferını temizler.
 Bu temizleme işleminin sebebi ise bilgisayardaki HC05'in koordinatları sürekli olarak göndermesi, robota bağlı olan HC05 bu verileri alması ama okunmadığında bufferda tutmasıdır.
 Buffer dolduğunda arızalara yol açtığını gözlemlediğim için ClearBluetoothBuffer() fonksiyonunu yüz takibinin veya başka herhangi bir bilgisayar iletişimim başlayabileceği yerlerden önce çağırdım.
*/
String voice;
void StartStopBluetooth()
{
  while (soundSerial.available()){  //Eğer veri okuyabiliyorsa
  delay(10); //Biraz bekle
  char c = soundSerial.read(); //verileri okumaya başla
  if (c == '#') {break;} //döngüden çık ve kelimeyi tekrar al
  voice += c;
  }
  if (voice.length() > 0) {
    soundSerial.println(voice);

    ClearBluetoothBuffer2();
   if (voice == "*yüz izle" || voice == "*Yüz izle" || voice == "*Yüz İzle" || voice == "*yüz izle" || voice == "*100 izle") 
   {  tone(speaker,LA3,Q);
   CancelOperations();
    delay(1+Q); 
    StartFaceTracking();
    }
   else if (voice == "*yüz takip" || voice == "*Yüz takip" || voice == "*Yüz Takip" || voice == "*yüz Takip")
    {
    if(!faceTracking)
    CancelOperations();
    tone(speaker,LA3,Q); 
    delay(1+Q);
   StartFollowFace();
    }
    else if (voice == "*Manuel" || voice == "*manuel")
    {
    CancelOperations();
    tone(speaker,LA3,Q); 
    delay(1+Q);
    StartManualControl();
    }
   else if ( voice == "*iptal" || voice == "*iptal" || voice == "*İptal" || voice == "*İptal" || voice == "*iptal") 
   { 
    tone(speaker,LA3,Q); 
    delay(1+Q); 
    CancelOperations();
    }
    else if(voice == "*Imperial March" || voice == "imperial March" || voice == "imperial march" || voice == "Imperial march")
    {
      PlayImperialMarch();
    }
    else if(voice == "*Süper Mario" || voice == "süper Mario" || voice == "Süper mario" || voice == "süper mario" || voice == "*Super Mario" || voice == "super Mario" || voice == "Super mario" || voice == "super mario") 
    {
      PlayMario();
    }
    voice="";
  ClearBluetoothBuffer2();
 
}} //ses yok tekrar dinle
void StartManualControl()
{
  manualControl = true;
}
void StopManualControl()
{
  manualControl = false;
}
void StartFollowFace()
{
  ClearBluetoothBuffer2();
  
  if(!faceTracking)
  StartFaceTracking();
  
  followFace = true;
}
void StopFollowFace()
{
  followFace = false;
}

void CancelOperations()
{
  Stop();
  StopFaceTracking();
  StopManualControl();
  StopFollowFace();
   

}
//--------------FaceTracking-----------------
//ResetDetection fonksiyonu timer
/* FaceResetTimer()  
 *  AMACI:
 *  Bilgisayardan koordinat verileri geldiğinde, faceFindTimerEnabled = false, faceDetected = true olur.(faceFindTimerEnabled: true ise kamera arama moduno girer) (faceDetected = true ise şuan algılanmış ve takip edilen bir yüz vardır)
 *  Bu fonksiyon 300ms de bir Serial2 portunu kontrol eder eğer okunmaya hazır veri yok ise(bilgisayardan koordinat gelmiyorsa), gerekli değişimleri yapar böylece kamera yüz arama moduna girip etrafı tarayabilir.
 *  
 *  ÇALIŞMA MANTIĞI: 
 *  ÖNBİLGİ: millis(): mikroÇipe akım geldiği andan itibaren geçen süreyi veren builtIn fonksiyondur.
 *  FaceResetTimer düzenli olarak çağırılan bir fonksiyondur, bir timer gibi çalışır.
 *  Her çağırıldığında millis() ile geçen zamanı alır. 
 *  Eğer Serial2 portu boş ve geçen zamanda previousMillis_FaceReset (en yukarıda tanımlandı) değişkeninden fazla ise arama modu aktifleştirilir(faceFindTimerEnabled = true) ve yüz bulundu(faceDetected = false) yapılır. 
 *  previousMillis_FaceReset = currentMillis olarak eşitlenirki fonskyon bir sonraki çalışmasından önce belirli bir süre beklesin.
 *  else içerisine düşerse Yuz Bulunmuş demektir LCDye bilgi yazılır.
 *
*/
void FaceResetTimer(){  
    unsigned long currentMillis = millis();
  if (Serial2.available() <=1 && currentMillis - previousMillis_FaceReset >= interval_FaceReset) {
    faceFindTimerEnabled = true;
    faceDetected = false;
    faceAttackTimerControl = false;
     previousMillis_FaceReset = currentMillis;
  }
  else{   
 
//    LCD_Display("YUZ BULUNDU",2,"TAKIP EDILIYOR",0);   
     line1 = "YUZ BULUNDU";
  line2 = "TAKIP EDILIYOR";
   
 
  }
}
/* FaceFindControlTimer
 *  AMACI: Algınana bir yüz yok ise kameranın etrafı tarayıp yüzü bulmasını sağlamak.
 *  ÇALIŞMA MANTIĞI: 
 *  ÖNBİLGİ: millis(): mikroÇipe akım geldiği andan itibaren geçen süreyi veren builtIn fonksiyondur.
 *  FaceResetTimer düzenli olarak çağırılan bir fonksiyondur, bir timer gibi çalışır.
 *  Her çağırıldığında eğer yuz algılanmamış faceDetected = false ise millis() ile geçen zamanı alır.
 *  geçen zaman fonskiyonu en son çağırılmasındaki geçen zamandan belirli bir derecede büyük ise FindFace() fonksiyonu çağırılır.
*/
void GoBackward(int turnSpeed)
{
  analogWrite(speedPinA, turnSpeed);
digitalWrite(dir1PinA, LOW);
digitalWrite(dir2PinA, HIGH);

analogWrite(speedPinB, turnSpeed);
digitalWrite(dir1PinB, LOW);
digitalWrite(dir2PinB, HIGH);

analogWrite(speedPinC, turnSpeed);
digitalWrite(dir1PinC, LOW);
digitalWrite(dir2PinC, HIGH);

analogWrite(speedPinD, turnSpeed);
digitalWrite(dir1PinD, LOW);
digitalWrite(dir2PinD, HIGH);
}
void GoForward(int turnSpeed)
{
analogWrite(speedPinA, turnSpeed);
digitalWrite(dir1PinA, HIGH);
digitalWrite(dir2PinA, LOW);

analogWrite(speedPinB, turnSpeed);
digitalWrite(dir1PinB, HIGH);
digitalWrite(dir2PinB, LOW);

analogWrite(speedPinC, turnSpeed);
digitalWrite(dir1PinC, HIGH);
digitalWrite(dir2PinC, LOW);

analogWrite(speedPinD, turnSpeed);
digitalWrite(dir1PinD, HIGH);
digitalWrite(dir2PinD, LOW);
}

void TurnLeft(int turnSpeed)
{
analogWrite(speedPinA, turnSpeed);
digitalWrite(dir1PinA, HIGH);
digitalWrite(dir2PinA, LOW);


analogWrite(speedPinB, turnSpeed);
digitalWrite(dir1PinB, HIGH);
digitalWrite(dir2PinB, LOW);

analogWrite(speedPinC, turnSpeed);
digitalWrite(dir1PinC, LOW);
digitalWrite(dir2PinC, HIGH);


analogWrite(speedPinD, turnSpeed);
digitalWrite(dir1PinD, LOW);
digitalWrite(dir2PinD, HIGH);
}

void TurnRight(int turnSpeed)
{
analogWrite(speedPinA, turnSpeed);
digitalWrite(dir1PinA, LOW);
digitalWrite(dir2PinA, HIGH);

analogWrite(speedPinB, turnSpeed);
digitalWrite(dir1PinB, LOW);
digitalWrite(dir2PinB, HIGH);

analogWrite(speedPinC, turnSpeed);
digitalWrite(dir1PinC, HIGH);
digitalWrite(dir2PinC, LOW);

analogWrite(speedPinD, turnSpeed);
digitalWrite(dir1PinD, HIGH);
digitalWrite(dir2PinD, LOW);
}

void Stop()
{
   analogWrite(speedPinA, 0);
    analogWrite(speedPinB, 0);
    analogWrite(speedPinC, 0);
    analogWrite(speedPinD, 0);
}

void TurnRightTimer(int turnTime , int turnSpeed, int afterDelay)
{
    if(!motorTurnLeft && motorTurnAut)
  {
    
  if(turnComplete)
  {  
  turnComplete = false;
  interval_MotorTurnRight = turnTime;
  interval_MotorTurnAut = afterDelay;  
  TurnRight(turnSpeed);
  previousMillis_MotorTurnRight = millis();
  
  }
  
  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis_MotorTurnRight >= interval_MotorTurnRight) 
  {
    motorTurnRight = false;
    turnComplete = true;
    motorTurnAut = false;  
    Stop();
    faceOriantedX = false;
    previousMillis_MotorTurnRight = millis();    
    previousMillis_MotorTurnAut = millis();    
   }  
  }
  
}

void TurnLeftTimer(int turnTime, int turnSpeed, int afterDelay)
{
  if(!motorTurnRight && motorTurnAut)
  {
    
  if(turnComplete)
  {  
  turnComplete = false;
  interval_MotorTurnLeft = turnTime;
  interval_MotorTurnAut = afterDelay;  
  TurnLeft(turnSpeed);
  previousMillis_MotorTurnLeft = millis();
  
  }
  
  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis_MotorTurnLeft >= interval_MotorTurnLeft) 
  {
    motorTurnLeft = false;
    turnComplete = true;
    motorTurnAut = false;  
    Stop();
    faceOriantedX = false;
    previousMillis_MotorTurnLeft = millis();    
    previousMillis_MotorTurnAut = millis();    
   }  
  } 
}

void MotorTurnAutTimer()
{
  if(!motorTurnAut)
  {
   Stop();
    
  
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis_MotorTurnAut >= interval_MotorTurnAut) 
     {
       previousMillis_MotorTurnAut = currentMillis;
       motorTurnAut = true;
     } 
  }
  
   
}

void FaceFindControlTimer()
{
 if(!faceDetected)
{   
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_FaceFind >= interval_FaceFind) 
  {
     previousMillis_FaceFind = currentMillis;
     FindFace();
  } 
}
}




/*StartFaceTracking()
 * AMACI:
 * YuzTakip özelliği başlatılmadan önce gerekli değişkenlere gerekli verileri atamak.Bu fonksiyon bluetoothta "yüz takip" komutu geldiğinde çağırılır.
 * ÇALIŞMA MANTIĞI:
 * LCDde Alınan komut 700ms gösterilir. xDirection_FaceTracking = 1;   yDirection_FaceTracking = 0; (yüz bulunamadığında hangi tarafa doğru aramayı başlatacaklarını belirten değişkenler).
 * 
  
*/
void StartFaceTracking()
{
  
  LCD_Display("ALINAN KOMUT:",2,"YUZ TAKIP",4);
  line1 = "ALINAN KOMUT:";
  line2 = "YUZ TAKIP";
 delay(500);
  xDirection_FaceTracking = 0;
  yDirection_FaceTracking = 0;
  faceDetected = false; //yüz bulundu false yapılır
  xFlag_FaceTracking = false; 
  faceTracking = true;
  previousMillis_FaceReset = millis(); //şuanki zamana eşitlenir
  previousMillis_FaceFind = millis();//şuanki zamana eşitlenir
  faceFindTimerEnabled = false;
  faceAttackTimerControl = true;

}

/*StopFaceTracking()
 * AMACI:
 * "iptal" komutu geldiğinde yüz takibini durdurmak için çağırılır, gerekli değişkenleri değiştirir ve yüz takibini durdurur.
 * ÇALIŞMA MANTIĞI:
 * LCDde Alınan komut 700ms gösterilir. Bütün değişkenleri resetler. ResetCamLocation() fonksiyonunu çağırır kamerayı merkezler.
 * 
  
*/
void StopFaceTracking()
{
  
  xDirection_FaceTracking = 0;
  yDirection_FaceTracking = 0;
  faceDetected = false;
  xFlag_FaceTracking = false;
  faceTracking = false;
  previousMillis_FaceReset = millis();
  previousMillis_FaceFind = millis();
  faceFindTimerEnabled = false;
  faceAttackTimerControl = true;
 
//  LCD_Display("ALINAN KOMUT:",2,"IPTAL",5);
   line1 = "ALINAN KOMUT:";
  line2 = "IPTAL";
  ResetCamLocation();
  delay(500);
 // LCD_Display("KOMUT ALMAYA",2,"HAZIR",5);
   line1 = "KOMUT ALMAYA";
  line2 = "HAZIR";
}


/*
 * TrackFace()
 * AMACI: Yuz konumu içeren HC05 modulu içerisindeki paketleri okumak, servoları yüz takip etmeye yönelik yönetmek, yüz takibi ile ilgili değişkenleri yönetmek.
 * ÇALIŞMA MANTIĞI:
 * Serial2 portundaki veri sayısı 4'ü geçmiş ise çalışır.( koordinatlar byte byte yollandığı için 2 koordinat 4 byte denk geliyor)
 * previousMillis_FaceReset = millis() yaparak FaceReset fonskiyonun belirli bir zaman boyunca çalıştırılmasına gerek olmadığı belirtir.
 * faceFindTimerEnabled = false böylece yüz arama modu kapatılır.
 *  xFlag_FaceTracking = true yuz kayıp edilirse son bulunduğu tarafa doğru kameranı hızlıca dönmesini sağlayan değişken true yapılır.
 *  faceDetected = true
 *  ÖNBİLGİ: word() fonksiyonu gelen 2 byte birleştiren bir builtIn fonksiyondur, bytelar birleşince koordinat oluşur.
 *  gelen ilk iki byte x koordinatıdır, ardından gelen 2 byte y koordinatıdır.
 *  servoların şuanki yerleri okunur ve posx, posy oalrak kayıt edilir.
 *  eğer x koordinatı ekranın yatay yarı boyutundan küçük ise: 
 *  posx artırılır.
 *  eğer servox'in pozisyonu 160dan büyük ise robot bir miktar sağa döner ve kameranın işini kolaylaştırır.
 *  
 *  eğer gelen xkoordinatı ekranın yatay boyutunu yarısından büyük ise
 *  posx azaltılır.
 * eğer servox'in pozisyonu 20den küçük ise robot bir miktar sola döner ve kameranın işini kolaylaştırır.
 * 
 * eğer servox pozisyonu yüzü tam ortalamış ise faceOriantedX = true yapılır( bu ileride yüzü motorlar aracığı ile takip ederken kullanılıcak değişkendir.)
 * 
 * 
 * //aynı mantık iley koordinatları işlenir posy değeri değiştirilir ama motorlara dönme emri verilmez.
    if(valy < (screenmaxy/2 - incy)){
      if(posy >= 5)posy -= distancey; 

      
    }
   
    else if(valy > (screenmaxy/2 + incy)){
      if(posy <= 175)posy += distancey; 
    }

    //yeni posx ve posy servolara yazılır( yüz takibi sağlanır)
    servox.write(posx);
    servoy.write(posy);
*/
void TrackFace()
{ 
  if(Serial2.available() >= 4)
  { 
    previousMillis_FaceReset = millis(); // record last face track data time
    if(!faceAttackTimerControl){
      faceAttackTimerControl = true; 
       previousMillis_FaceSearchAttack = millis();
    }
    faceFindTimerEnabled = false;
    xFlag_FaceTracking = true;
    faceDetected = true;    
  
    MSB = Serial2.read();    
    LSB = Serial2.read();
  
    MSBLSB=word(MSB, LSB);
    valx = MSBLSB;  
  
   
    MSB = Serial2.read();    
    LSB = Serial2.read();
   MSBLSB=word(MSB, LSB);
   
    valy = MSBLSB;
    // read last servos positions
    posx = servox.read(); 
    posy = servoy.read();

     String x = String(valx);
      String y = String(valy);
//      LCD_Display("x:" + x,0,"y:" + y,0);
     
     
 if(valx < (screenmaxx/2 - incx)){
/*
motor1.setSpeed(motorSpeed -80);
    motor2.setSpeed(motorSpeed -80);
    motor3.setSpeed(motorSpeed -80);
    motor4.setSpeed(motorSpeed -80);
      if(servox.read() > 160)
      {
        motor1.run(FORWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(BACKWARD);   
      }
      else
      {
     motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);   
      }
      */
         posx += distancex;  
        xDirection_FaceTracking = 1; 
        faceOriantedX = false;     
    }
    
    else if(valx > screenmaxx/2 + incx){
/*
   motor1.setSpeed(motorSpeed -80);
    motor2.setSpeed(motorSpeed -80);
    motor3.setSpeed(motorSpeed -80);
    motor4.setSpeed(motorSpeed -80);
     if(servox.read() < 20){
     motor1.run(BACKWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(FORWARD);   
      }
      else{
     motor1.run(RELEASE);
    motor2.run(RELEASE);  
    motor3.run(RELEASE);
    motor4.run(RELEASE);   
      }   */
        xDirection_FaceTracking = 0;   
        faceOriantedX = false;  
         posx -=distancex ; 
    }
    else {
      faceOriantedX = true;
    }

 
    if(valy < (screenmaxy/2 - incy)){
      if(posy >= 5)posy -= distancey; 

      
    }
   
    else if(valy > (screenmaxy/2 + incy)){
      if(posy <= 175)posy += distancey; 
    }
    servox.write(posx);
    servoy.write(posy);
  }   
}
/*
 * FollowFace()
 * AMACI: kamera yüzü ortaladığında, motorlar aracılığı ile robotunda yüzü ortalamasını sağlamak, yüz ortalanınca yüze doğru servoy açısı 60 dan ufa olduğu sürece ilerlemek.
 * ÇALIŞMA MANTIĞI:
 * Kodlar içerisinde açıklandı
*/
void FollowFace(){

    if(motorTurnRight)
    {
      TurnRightTimer(4*(120 - posx), 255, 1500);
    }
      
  else if(motorTurnLeft)
  {
     TurnLeftTimer(4*(posx - 120) , 255 , 1500);
  }
     
      
  if(faceOriantedX && faceDetected){// yüz kamera tarafından ortalanmadan ve yüz bulunmuş olmadan çalışmaz
 int posx = servox.read();// posx ve posy içerisine servoların pozisyonları kayıt edilir.
 int posy = servoy.read();

 String testx = String(posx);
 String testy = String(posy);
 
if((posx >=110 && posx <= 130) && posy >= 20  && turnComplete){// eğer posx 110 dan büyük ve 130 dan küçük ise ve posy de 60 dereceden küçük( çok yukarı bakmıyor) ise yüz ortalandığı kabl edilir ve motorlara ilerle emri verilir.
  //motorhızları ayarlanır
  GoForward(150);
//motorlar ilerlemeye alınır
    
}
else if(posy >= 20)// eğer posy 60dan küçük ise yani robot takip ettiği yüze çok yakın değil ise
    {
       
  //SONRADAN
  //eğer posx 110dan küçük ise robot sağa döndürülür
  if(posx <= 110)
  {
  if(turnComplete)
  motorTurnRight = true;
  
  
  }
  
  //eğer posx130 dna büyük ise robot sola döndürülür. 
  else if( posx >= 130)
  {
    if(turnComplete)
    motorTurnLeft = true;
  
  }  
}
} 
else if(!faceDetected || turnComplete)
{
Stop();
}
  
  
}
/*
 * FindFace()
 * AMACI: Kamera yüzü kayıp ettiğinde yüzün son bulunduğu yere kameranın hızlıca dönerek yeniden yüzü yakalamasnı sağlamak ve eğer yüz buna rağmen bulunamaz ise kameranın etrafı taramasını sağlamak.
 * Çalışma Mantığı: 
 * xFlag_FaceTracking
 * 
*/
void FindFace(){
 
//  LCD_Display("YUZ ARANIYOR",2,"",0);
   line1 = "YUZ ARANIYOR";
  line2 = "";
  
if(xFlag_FaceTracking){ 
  if(servox.read() > 175 || servox.read() < 5 ){
    xFlag_FaceTracking = false;
  }
if(xDirection_FaceTracking == 1) servox.write(servox.read() + 3);
else if(xDirection_FaceTracking == 0) servox.write(servox.read() - 3);
}
else{  
   if(servox.read() > 175)
      {
        xDirection_FaceTracking = 0;
        xDirectionCounter++;        
      }
      else if(servox.read() < 5)
      {
        xDirection_FaceTracking = 1;    
         xDirectionCounter++;   
      }
   
if(xDirection_FaceTracking == 1) servox.write(servox.read() + 3);
else if(xDirection_FaceTracking == 0) servox.write(servox.read() - 3);
      servoy.write(currentY);
     
      if(xDirectionCounter == 2){
        currentY -= 10;
        if(currentY < minY_FaceTracking && (servoy.read() <! minY_FaceTracking || servoy.read() >! maxY_FaceTracking)) currentY = maxY_FaceTracking;
        xDirectionCounter = 0;
      }
   }
}

//---------------End Face Tracking------------------------
void ResetCamLocation(){
   int i = servoy.read();
  while(i != servocentery){
  
  if(i < servocentery)
  i++;
  else
  i--;
servoy.write(i);
delay(5);
i = servoy.read();
delay(15);  
}
i = servox.read();
 while(i != servocenterx){
  
  if(i < servocenterx)
  i++;
  else
  i--;
servox.write(i);
delay(5);
i = servox.read();
delay(15);  
}  
}
//-----SONICSENSORS-----
bool SonicSensors(int sensorNumber,long lDistance){
long duration;
long distance;
int trigPin;
int echoPin;

switch(sensorNumber){
  case 1:
  trigPin = trigPin1;
  echoPin = echoPin1;
  break;
  case 2:
  trigPin = trigPin2;
  echoPin = echoPin2;
  break;
  case 3:
  trigPin = trigPin3;
  echoPin = echoPin3;
  break;
  case 4:
  trigPin = trigPin4;
  echoPin = echoPin4;
  break;
  case 5:
  trigPin = trigPin5;
  echoPin = echoPin5;
  break;
  case 6:
  trigPin = trigPin6;
  echoPin = echoPin6;
  break;
  case 7:
  trigPin = trigPin7;
  echoPin = echoPin7;
  break;
  case 8:
  trigPin = trigPin8;
  echoPin = echoPin8;
  break;
}


  digitalWrite(trigPin, LOW);  
  delay(2); // Added t"his line
  digitalWrite(trigPin, HIGH);
  delay(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
long distance2;
digitalWrite(trigPin, LOW);  
  delay(2); // Added t"his line
  digitalWrite(trigPin, HIGH);
  delay(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance2 = (duration/2) / 29.1;

  
  
if(distance <= lDistance && distance2 <=lDistance && distance2 >= 5)
return true;
else
return false;
 
}
bool ObjectDetectedInFront()
{
  if(ObjectDetectedInRightFront() || ObjectDetectedInLeftFront())
      return true;
  return false;
}

bool ObjectDetectedInRightFront()
{
  if(SonicSensors(1,25))
      return true;
  return false;
}
bool ObjectDetectedInLeftFront()
{
  if(SonicSensors(2,25))
      return true;
  return false;
}

bool ObjectDetectedInRightSideOne()
{
   if(SonicSensors(8,25))
      return true;
   return false;  
}

bool ObjectDetectedInLeftSideOne()
{
   if(SonicSensors(3,25))
      return true;      
  return false;  
}
bool ObjectDetectedInRightSideTwo()
{
   if(SonicSensors(7,25))
      return true;
      
  return false; 

       
}

bool ObjectDetectedInLeftSideTwo()
{
   if(SonicSensors(4,25))
      return true;
  else
      return false;  
}
bool ObjectDetectedInBack()
{
  
}

//---------SPEAKER-----------
void PlayMario(){
  tone(speaker,660,100);
delay(150);
tone(speaker,660,100);
delay(300);
tone(speaker,660,100);
delay(300);
tone(speaker,510,100);
delay(100);
tone(speaker,660,100);
delay(300);
tone(speaker,770,100);
delay(550);
tone(speaker,380,100);
delay(575);

tone(speaker,510,100);
delay(450);
tone(speaker,380,100);
delay(400);
tone(speaker,320,100);
delay(500);
tone(speaker,440,100);
delay(300);
tone(speaker,480,80);
delay(330);
tone(speaker,450,100);
delay(150);
tone(speaker,430,100);
delay(300);
tone(speaker,380,100);
delay(200);
tone(speaker,660,80);
delay(200);
tone(speaker,760,50);
delay(150);
tone(speaker,860,100);
delay(300);
tone(speaker,700,80);
delay(150);
tone(speaker,760,50);
delay(350);
tone(speaker,660,80);
delay(300);
tone(speaker,520,80);
delay(150);
tone(speaker,580,80);
delay(150);
tone(speaker,480,80);
delay(500);

tone(speaker,510,100);
delay(450);
tone(speaker,380,100);
delay(400);
tone(speaker,320,100);
delay(500);
tone(speaker,440,100);
delay(300);
tone(speaker,480,80);
delay(330);
tone(speaker,450,100);
delay(150);
tone(speaker,430,100);
delay(300);
tone(speaker,380,100);
delay(200);
tone(speaker,660,80);
delay(200);
tone(speaker,760,50);
delay(150);
tone(speaker,860,100);
delay(300);
tone(speaker,700,80);
delay(150);
tone(speaker,760,50);
delay(350);
tone(speaker,660,80);
delay(300);
tone(speaker,520,80);
delay(150);
tone(speaker,580,80);
delay(150);
tone(speaker,480,80);
delay(500);

tone(speaker,500,100);
delay(300);

tone(speaker,760,100);
delay(100);
tone(speaker,720,100);
delay(150);
tone(speaker,680,100);
delay(150);
tone(speaker,620,150);
delay(300);

tone(speaker,650,150);
delay(300);
tone(speaker,380,100);
delay(150);
tone(speaker,430,100);
delay(150);

tone(speaker,500,100);
delay(300);
tone(speaker,430,100);
delay(150);
tone(speaker,500,100);
delay(100);
tone(speaker,570,100);
delay(220);

tone(speaker,500,100);
delay(300);

tone(speaker,760,100);
delay(100);
tone(speaker,720,100);
delay(150);
tone(speaker,680,100);
delay(150);
tone(speaker,620,150);
delay(300);

tone(speaker,650,200);
delay(300);

tone(speaker,1020,80);
delay(300);
tone(speaker,1020,80);
delay(150);
tone(speaker,1020,80);
delay(300);

tone(speaker,380,100);
delay(300);
tone(speaker,500,100);
delay(300);

tone(speaker,760,100);
delay(100);
tone(speaker,720,100);
delay(150);
tone(speaker,680,100);
delay(150);
tone(speaker,620,150);
delay(300);

tone(speaker,650,150);
delay(300);
tone(speaker,380,100);
delay(150);
tone(speaker,430,100);
delay(150);

tone(speaker,500,100);
delay(300);
tone(speaker,430,100);
delay(150);
tone(speaker,500,100);
delay(100);
tone(speaker,570,100);
delay(420);

tone(speaker,585,100);
delay(450);

tone(speaker,550,100);
delay(420);

tone(speaker,500,100);
delay(360);

tone(speaker,380,100);
delay(300);
tone(speaker,500,100);
delay(300);
tone(speaker,500,100);
delay(150);
tone(speaker,500,100);
delay(300);

tone(speaker,500,100);
delay(300);

tone(speaker,760,100);
delay(100);
tone(speaker,720,100);
delay(150);
tone(speaker,680,100);
delay(150);
tone(speaker,620,150);
delay(300);

tone(speaker,650,150);
delay(300);
tone(speaker,380,100);
delay(150);
tone(speaker,430,100);
delay(150);

tone(speaker,500,100);
delay(300);
tone(speaker,430,100);
delay(150);
tone(speaker,500,100);
delay(100);
tone(speaker,570,100);
delay(220);

tone(speaker,500,100);
delay(300);

tone(speaker,760,100);
delay(100);
tone(speaker,720,100);
delay(150);
tone(speaker,680,100);
delay(150);
tone(speaker,620,150);
delay(300);

tone(speaker,650,200);
delay(300);

tone(speaker,1020,80);
delay(300);
tone(speaker,1020,80);
delay(150);
tone(speaker,1020,80);
delay(300);

tone(speaker,380,100);
delay(300);
tone(speaker,500,100);
delay(300);

tone(speaker,760,100);
delay(100);
tone(speaker,720,100);
delay(150);
tone(speaker,680,100);
delay(150);
tone(speaker,620,150);
delay(300);

tone(speaker,650,150);
delay(300);
tone(speaker,380,100);
delay(150);
tone(speaker,430,100);
delay(150);

tone(speaker,500,100);
delay(300);
tone(speaker,430,100);
delay(150);
tone(speaker,500,100);
delay(100);
tone(speaker,570,100);
delay(420);

tone(speaker,585,100);
delay(450);

tone(speaker,550,100);
delay(420);

tone(speaker,500,100);
delay(360);

tone(speaker,380,100);
delay(300);
tone(speaker,500,100);
delay(300);
tone(speaker,500,100);
delay(150);
tone(speaker,500,100);
delay(300);

tone(speaker,500,60);
delay(150);
tone(speaker,500,80);
delay(300);
tone(speaker,500,60);
delay(350);
tone(speaker,500,80);
delay(150);
tone(speaker,580,80);
delay(350);
tone(speaker,660,80);
delay(150);
tone(speaker,500,80);
delay(300);
tone(speaker,430,80);
delay(150);
tone(speaker,380,80);
delay(600);

tone(speaker,500,60);
delay(150);
tone(speaker,500,80);
delay(300);
tone(speaker,500,60);
delay(350);
tone(speaker,500,80);
delay(150);
tone(speaker,580,80);
delay(150);
tone(speaker,660,80);
delay(550);

tone(speaker,870,80);
delay(325);
tone(speaker,760,80);
delay(600);

tone(speaker,500,60);
delay(150);
tone(speaker,500,80);
delay(300);
tone(speaker,500,60);
delay(350);
tone(speaker,500,80);
delay(150);
tone(speaker,580,80);
delay(350);
tone(speaker,660,80);
delay(150);
tone(speaker,500,80);
delay(300);
tone(speaker,430,80);
delay(150);
tone(speaker,380,80);
delay(600);

tone(speaker,660,100);
delay(150);
tone(speaker,660,100);
delay(300);
tone(speaker,660,100);
delay(300);
tone(speaker,510,100);
delay(100);
tone(speaker,660,100);
delay(300);
tone(speaker,770,100);
delay(550);
tone(speaker,380,100);
delay(575);
}
void PlayImperialMarch(){
   tone(speaker,LA3,Q); 
    delay(1+Q); 
    tone(speaker,LA3,Q);
    delay(1+Q);
    tone(speaker,LA3,Q);
    delay(1+Q);
    tone(speaker,F3,E+S);
    delay(1+E+S);
    tone(speaker,C4,S);
    delay(1+S);
    
    tone(speaker,LA3,Q);
    delay(1+Q);
    tone(speaker,F3,E+S);
    delay(1+E+S);
    tone(speaker,C4,S);
    delay(1+S);
    tone(speaker,LA3,H);
    delay(1+H);
    
    tone(speaker,E4,Q); 
    delay(1+Q); 
    tone(speaker,E4,Q);
    delay(1+Q);
    tone(speaker,E4,Q);
    delay(1+Q);
    tone(speaker,F4,E+S);
    delay(1+E+S);
    tone(speaker,C4,S);
    delay(1+S);
    
    tone(speaker,Ab3,Q);
    delay(1+Q);
    tone(speaker,F3,E+S);
    delay(1+E+S);
    tone(speaker,C4,S);
    delay(1+S);
    tone(speaker,LA3,H);
    delay(1+H);
    
    tone(speaker,LA4,Q);
    delay(1+Q);
    tone(speaker,LA3,E+S);
    delay(1+E+S);
    tone(speaker,LA3,S);
    delay(1+S);
    tone(speaker,LA4,Q);
    delay(1+Q);
    tone(speaker,Ab4,E+S);
    delay(1+E+S);
    tone(speaker,G4,S);
    delay(1+S);
    
    tone(speaker,Gb4,S);
    delay(1+S);
    tone(speaker,E4,S);
    delay(1+S);
    tone(speaker,F4,E);
    delay(1+E);
    delay(1+E);//PAUSE
    tone(speaker,Bb3,E);
    delay(1+E);
    tone(speaker,Eb4,Q);
    delay(1+Q);
    tone(speaker,D4,E+S);
    delay(1+E+S);
    tone(speaker,Db4,S);
    delay(1+S);
    
    tone(speaker,C4,S);
    delay(1+S);
    tone(speaker,B3,S);
    delay(1+S);
    tone(speaker,C4,E);
    delay(1+E);
    delay(1+E);//PAUSE QUASI FINE RIGA
    tone(speaker,F3,E);
    delay(1+E);
    tone(speaker,Ab3,Q);
    delay(1+Q);
    tone(speaker,F3,E+S);
    delay(1+E+S);
    tone(speaker,LA3,S);
    delay(1+S);
    
    tone(speaker,C4,Q);
    delay(1+Q);
     tone(speaker,LA3,E+S);
    delay(1+E+S);
    tone(speaker,C4,S);
    delay(1+S);
    tone(speaker,E4,H);
    delay(1+H);
    
     tone(speaker,LA4,Q);
    delay(1+Q);
    tone(speaker,LA3,E+S);
    delay(1+E+S);
    tone(speaker,LA3,S);
    delay(1+S);
    tone(speaker,LA4,Q);
    delay(1+Q);
    tone(speaker,Ab4,E+S);
    delay(1+E+S);
    tone(speaker,G4,S);
    delay(1+S);
    
    tone(speaker,Gb4,S);
    delay(1+S);
    tone(speaker,E4,S);
    delay(1+S);
    tone(speaker,F4,E);
    delay(1+E);
    delay(1+E);//PAUSE
    tone(speaker,Bb3,E);
    delay(1+E);
    tone(speaker,Eb4,Q);
    delay(1+Q);
    tone(speaker,D4,E+S);
    delay(1+E+S);
    tone(speaker,Db4,S);
    delay(1+S);
    
    tone(speaker,C4,S);
    delay(1+S);
    tone(speaker,B3,S);
    delay(1+S);
    tone(speaker,C4,E);
    delay(1+E);
    delay(1+E);//PAUSE QUASI FINE RIGA
    tone(speaker,F3,E);
    delay(1+E);
    tone(speaker,Ab3,Q);
    delay(1+Q);
    tone(speaker,F3,E+S);
    delay(1+E+S);
    tone(8,C4,S);
    delay(1+S);
    
    tone(speaker,LA3,Q);
    delay(1+Q);
     tone(speaker,F3,E+S);
    delay(1+E+S);
    tone(speaker,C4,S);
    delay(1+S);
    tone(speaker,LA3,H);
    delay(1+H);
    
    delay(2*H);
}

void LCD_Display(String l1,int s1,String l2,int s2){
 if(l1 != line1 && l2 != line2){  
  lcd.clear();
  lcd.setCursor(s1,0);
    lcd.print(l1);
  lcd.setCursor(s2,1);
  lcd.print(l2);  
  
 }
  }
  
  void ClearBluetoothBuffer2(){
     while(Serial2.available() > 0)
  Serial2.read();
  
  }
/*
 void MotorControl(){
  
  if(!SonicSensors(1,25) && !SonicSensors(2,25))//eğer ön taraf 1,2 boş ise
  {
    motor1.setSpeed(motorSpeed);
    motor2.setSpeed(motorSpeed - 10);
    motor3.setSpeed(motorSpeed);
    motor4.setSpeed(motorSpeed -10);

    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  }
  else{
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);   

        if(SonicSensors(1,25))
          {
            while(!SonicSensors(8,30) )
            {
             motor1.run(FORWARD);
             motor2.run(BACKWARD);
             motor3.run(FORWARD);
             motor4.run(BACKWARD);
            }

            while(SonicSensors(1,25))
            {
             motor1.run(FORWARD);
             motor2.run(BACKWARD);
             motor3.run(FORWARD);
             motor4.run(BACKWARD);
            }      
                  
          }
          else if(SonicSensors(2,25))
          {
            while(!SonicSensors(3,30))
            {
             motor1.run(BACKWARD);
             motor2.run(FORWARD);
             motor3.run(BACKWARD);
             motor4.run(FORWARD);
            }

                 while(SonicSensors(2,25))
            {
             motor1.run(FORWARD);
             motor2.run(BACKWARD);
             motor3.run(FORWARD);
             motor4.run(BACKWARD);
            }
          
            
          }
  }
 }*/






