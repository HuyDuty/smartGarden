#include <ESP8266WiFi.h>//thu vien ho tro ket noi wifi cho esp8266                 
#include <FirebaseESP8266.h>//thu vien ho tro ket noi firebase cho esp8266
#include <NTPClient.h>//thu vien ho tro su dung may khach NTP -> lay thoi gian
#include <WiFiUdp.h>//thu vien ho tro su dung giao thuc UDP 

#define FIREBASE_HOST "vuon-thong-minh-6868-default-rtdb.firebaseio.com/"//may chu firebase     
#define FIREBASE_AUTH "AIzaSyCs9ay2VqXCBAvpJDGPc9SHIRp_cG8_fvc"//ma xac thuc firebase         
#define WIFI_SSID "TP-LINK_7942"//id wifi                                  
#define WIFI_PASSWORD "02548524"//password wifi  
        
FirebaseData firebaseData;//khai bao doi tuong firebase
WiFiUDP ntpUDP;//doi tuong giao thuc UDP
NTPClient timeClient(ntpUDP, "pool.ntp.org");//doi tuong may khach NTPClient

String path = "/";//khai bao bien duong dan
unsigned long t = 0;//khai bao bien tam thoi gian
String dataSensor;
String dataControlDevice;
String dataMode;

void setup() {
  // Khoi tao cac cong noi tiep
  Serial.begin(9600);//su dung Rx(UART0)
  Serial1.begin(9600);//su dung Tx(UART1)
  while (!Serial);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);//khoi tao ket noi wifi 
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID); 
  while (WiFi.status() != WL_CONNECTED) {//doi den khi ket noi den wifi
    Serial.print(".");
    delay(500);
  }
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);//khoi tao ket noi den firebase  
  timeClient.begin();//khoi tao doi tuong NTPclient de lay thoi gian
  timeClient.setTimeOffset(25200);//cai dat mui gio phu hop = 7 * 60 *60
  Serial.println();
  Serial.print("Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());//in ra ip address cua esp8266 trong mang wifi
  Firebase.setString(firebaseData, path + "/CHEDO", "1");//khoi tao ban dau che do 1
}
void loop() { // run over and over
  String chuoiNhanDuoc = "";
  Serial.println("Du lieu nhan duoc tu STM32F103C8T6: ");
  while (Serial.available()){
    char kiTuNhanDuoc = (char)Serial.read();
    if(kiTuNhanDuoc == '!'){
      Serial.println(chuoiNhanDuoc);
      chuoiNhanDuoc = "";
      timeClient.update();//bat dau lay du lieu thoi gian    
      String formattedTime = timeClient.getFormattedTime() + "!";//dinh dang lai thoi gian gio - phut - giay 
      Serial.println(formattedTime + " is sended to STM32F103C8T6");//in ra ip address cua esp8266 trong mang wifi  
      delay(1000);
      Serial1.println(formattedTime);
    }
    else if(kiTuNhanDuoc == '\n'){
      Serial.println(chuoiNhanDuoc);
      dataSensor = chuoiNhanDuoc.substring(0, 9);
      
      //gui du lieu sensor len firebase
      Firebase.setString(firebaseData, path + "/DULIEUCAMBIEN", dataSensor); 
      if(Firebase.getString(firebaseData, path + "/CHEDO"))
        dataMode = firebaseData.stringData();   
      if(dataMode == "1" || dataMode == "3"){
        dataControlDevice = chuoiNhanDuoc.substring(10, 14);
        Firebase.setString(firebaseData, path + "/TINHIEUDONGCO", dataControlDevice);
        delay(1000);
        Serial.print("Du lieu gui cho STM32F103C8T6: ");
        Serial.println(dataMode);
        Serial1.println(dataMode);
      }
      else{
        if(Firebase.getString(firebaseData, path + "/TINHIEUDONGCO"))
          dataControlDevice = firebaseData.stringData();
        Serial.print("Du lieu gui cho STM32F103C8T6: ");
        dataControlDevice = dataControlDevice + "2";
        Serial.println(dataControlDevice);
        Serial1.println(dataControlDevice);
      }
      chuoiNhanDuoc = "";
    }
    else
      chuoiNhanDuoc += kiTuNhanDuoc;     
  }
  String formattedTime = timeClient.getFormattedTime();//dinh dang lai thoi gian gio - phut - giay 
  Serial.println(formattedTime);//in ra ip address cua esp8266 trong mang wifi 
//  Serial1.println(formattedTime);//in ra ip address cua esp8266 trong mang wifi
  Firebase.setString(firebaseData, path + "/THOIGIANCAPNHAT", formattedTime);
  delay(1000);
}
