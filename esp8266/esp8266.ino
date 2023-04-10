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
String chuoiNhanDuoc, timeNow, hours, minutes, dataSensor, dataControlDevice, dataMode;
String timeOutDevice[4], results[4], pumpSpeed, arrResults;
signed long int timeOut[4];//khai bao bien tam thoi gian

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
  timeClient.update();//cap nhap mui gio 
  Serial.println();
  Serial.print("Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());//in ra ip address cua esp8266 trong mang wifi
  Firebase.setString(firebaseData, path + "/CHEDO", "1");//khoi tao ban dau che do 1
  timeOut[0] = 2147483647;
  timeOut[1] = 2147483647;
  timeOut[2] = 2147483647;
  timeOut[3] = 2147483647;
}
void loop() { // run over and over
  Serial.println("=================================================================");
  timeNow = timeClient.getFormattedTime();//lay thoi gian gio:phut:giay 
  Serial.println(timeNow);//in ra thoi gian gio:phut:giay 
  Firebase.setString(firebaseData, path + "/THOIGIANCAPNHAT", timeNow);
  chuoiNhanDuoc = "";
  Serial.println("Du lieu nhan duoc tu STM32F103C8T6: ");
  while (Serial.available()){
    char kiTuNhanDuoc = (char)Serial.read();
    if(kiTuNhanDuoc == '!'){
      Serial.println(chuoiNhanDuoc);
      chuoiNhanDuoc = "";   
      timeNow = timeClient.getFormattedTime() + "!";//dinh dang lai thoi gian gio - phut - giay 
      Serial.println(timeNow + " is sended to STM32F103C8T6");//in ra ip address cua esp8266 trong mang wifi  
      delay(1000);
      Serial1.println(timeNow);
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
        if(dataControlDevice.substring(0, 1) != "0"){
          Firebase.setString(firebaseData, path + "/pumpSpeed", dataControlDevice.substring(0, 1));
          dataControlDevice = "1" + dataControlDevice.substring(1, 4);
        }
        else
          Firebase.setString(firebaseData, path + "/pumpSpeed", "1");
        Firebase.setString(firebaseData, path + "/TINHIEUDONGCO", dataControlDevice);
        delay(1000);
        Serial.print("Du lieu gui cho STM32F103C8T6: ");
        Serial.println(dataMode);
        Serial1.println(dataMode);
      }
      else{
        if(Firebase.getString(firebaseData, path + "/TINHIEUDONGCO"))
          dataControlDevice = firebaseData.stringData();
        if(Firebase.getString(firebaseData, path + "/pumpSpeed"))
          pumpSpeed = firebaseData.stringData();
        if(Firebase.getString(firebaseData, path + "/timeoutPump"))
          timeOutDevice[0] = firebaseData.stringData();
        if(Firebase.getString(firebaseData, path + "/timeoutFan"))
          timeOutDevice[1] = firebaseData.stringData();
        if(Firebase.getString(firebaseData, path + "/timeoutBulb"))
          timeOutDevice[2] = firebaseData.stringData();
        if(Firebase.getString(firebaseData, path + "/timeoutCanopy"))
          timeOutDevice[3] = firebaseData.stringData();

        results[0] = dataControlDevice.substring(0, 1) == "0" ? "0" : pumpSpeed;
        results[1] = dataControlDevice.substring(1, 2) == "0" ? "0" : "1";
        results[2] = dataControlDevice.substring(2, 3) == "0" ? "0" : "1";
        results[3] = dataControlDevice.substring(3, 4) == "0" ? "0" : "1";

        if(timeOutDevice[0] != "" && timeOutDevice[0] != "0"){
          Firebase.setString(firebaseData, path + "/timeoutPump", "0");
          timeOut[0] = timeOutDevice[0].toInt() * 1000 + (signed long int)(millis());
        }
        if(timeOutDevice[1] != "" && timeOutDevice[1] != "0"){
          Firebase.setString(firebaseData, path + "/timeoutFan", "0");
          timeOut[1] = timeOutDevice[1].toInt() * 1000 + (signed long int)(millis());
        }
        if(timeOutDevice[2] != "" && timeOutDevice[2] != "0"){
          Firebase.setString(firebaseData, path + "/timeoutBulb", "0");
          timeOut[2] = timeOutDevice[2].toInt() * 1000 + (signed long int)(millis());
        }
        if(timeOutDevice[3] != "" && timeOutDevice[3] != "0"){
          Firebase.setString(firebaseData, path + "/timeoutCanopy", "0");
          timeOut[3] = timeOutDevice[3].toInt() * 1000 + (signed long int)(millis());
        }
        Serial.println(String(millis()) + " " + String(timeOut[0]) + " " + String(timeOut[1]) + " " + String(timeOut[2]) + " " + String(timeOut[3]));        
        if((signed long int)(millis()) - timeOut[0] >= 0){
          results[0] = pumpSpeed;
          timeOut[0] = 2147483647;
        }
        if((signed long int)(millis()) - timeOut[1] >= 0){
          results[1] = "1";
          timeOut[1] = 2147483647;
        }
        if((signed long int)(millis()) - timeOut[2] >= 0){
          results[2] = "1";
          timeOut[2] = 2147483647;
        }
        if((signed long int)(millis()) - timeOut[3] >= 0){
          results[3] = "1";
          timeOut[3] = 2147483647;
        }
        Serial.println("Tin hieu dong co gui cho STM32F103C8T6: ");
        dataControlDevice = results[0] + results[1] + results[2] + results[3];
        Firebase.setString(firebaseData, path + "/TINHIEUDONGCO", dataControlDevice);
        Serial.println(dataControlDevice + "2");
        Serial1.println(dataControlDevice + "2");
      }
      chuoiNhanDuoc = "";
    }
    else
      chuoiNhanDuoc += kiTuNhanDuoc;     
  }
  delay(1000);
}
