  //------------Librerias-----------------//
#include <WiFi.h>
#include <Husarnet.h>
#include <AceButton.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "DFRobotDFPlayerMini.h"
#include <esp_task_wdt.h>
using namespace ace_button;
//-------------------------------------//


//-------------Variables---------------//
#define DEV_TYPE           0           // type "0" servidor, y "1" cliente.
#define NUM_NETWORKS       2                                                                               // Cantidad de redes a evaluar para conectarse.
#define BUTTON_PIN_BITMASK 0x200000000 // 2^(GPIO) in hex - GPIO = 33.
#define NUMPIXELS          16          //Designamos cuantos pixeles tenemos en nuestra cinta led RGB.

const int luminosidad     = 20  ;
const int freq_1          = 4000;
const int freq_2          = 2   ;
const int resolution      = 8   ;
const int LED_CONN        = 22  ; // Led que indica si los esp están conectados entre si.
const int LED_SLP         = 21  ; // Led que indica que la esp está por mimir.
const int MOT_CONN        = 32  ; // Pin motor que vibra.
const int LED_PIX         = 19  ; // Pin de datos NeoPixel.

const int BUTTON_PIN1 = 39  ; // categoría 1.
const int BUTTON_PIN2 = 36  ; // categoría 2.
const int BUTTON_PIN3 = 34  ; // ok.
const int BUTTON_PIN4 = 35  ; // cancelar.
const int BUTTON_PIN5 = 23  ; // último msg recivido.
const int BUTTON_SLP  = 33  ; // Botón que regular el Sleep mode.

AceButton btn_1(BUTTON_PIN1);
AceButton btn_2(BUTTON_PIN2);
AceButton btn_3(BUTTON_PIN3);
AceButton btn_4(BUTTON_PIN4);
AceButton btn_5(BUTTON_PIN5);
AceButton btn_6(BUTTON_SLP );

int estadoB_1     = 0   ;
int estadoB_2     = 0   ;
int estadoB_1_lim = 1   ;
int estadoB_2_lim = 1   ;
int tiempo_msg    = 0   ;
int port          = 8001;
int factor        = 1   ;
int index_pix     = 0   ;

bool ver_last_msg      = false;
bool habilitar_cat_1   = false;
bool habilitar_cat_2   = false;
bool habilitar_audio   = false;
bool estado_ok         = false;
bool estado_cancel     = false;
bool msg_confirmado    = false;
bool quiere_enviar_msj = false;
bool blink_listo       = false;
bool subida            = true ;

uint32_t rojo    ;
uint32_t verde   ;
uint32_t azul    ;
uint32_t apagado ;
uint32_t amarillo;
uint32_t morado  ;
uint32_t celeste ;
uint32_t blanco  ;

long   now             = 0        ;
String msg_luz         = "cero"   ;

char   c               = '0'      ;
char   msg_to_send     = '0'      ;
char   last_mesagge[2] = {'p','p'}; 

const char* hostName0        = "fc94:a4de:5f1c:4bec:305e:64f:cf91:2bb8";  
const char* hostName1        = "Esp1";  
const char* husarnetJoinCode = "fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/JoTcYL5PuLHpGXAdvtbcGm";

const char* ssidTab[NUM_NETWORKS] = 
{
  "VTR-3288395",
  "iPhone"     ,
};

const char* passwordTab[NUM_NETWORKS] = 
{
  "5psttrtPttcb",
  "12345678"    ,
};

HusarnetClient client;

#if DEV_TYPE == 0
HusarnetServer server(port);
#endif

Adafruit_NeoPixel strip(NUMPIXELS, LED_PIX, NEO_GRB + NEO_KHZ800);
//HardwareSerial mySoftwareSerial(1); //rxMP3=17 TXmp3=16
DFRobotDFPlayerMini myDFPlayer;
//-------------------------------------//


//--------Declaración funcoines--------//
void secuencias_motor();
void secuencias_luz();
void handleEvent(AceButton*, uint8_t, uint8_t);
void btn_all_check();
void last_msgs();
void time_vib();
void taskWifi( void * parameter );
void taskConnection( void * parameter );
void sel_light();
void neopix_set(int color, int efecto);
void audio(int num_audio);
void colorWipe(uint32_t color, int wait);
//-------------------------------------//


void setup() 
{
  esp_task_wdt_init(8, true);
  //esp_task_wdt_add(NULL); //add current thread to WDT watch
  Serial.begin(115200);
  pinMode(BUTTON_PIN1, INPUT );
  pinMode(BUTTON_PIN2, INPUT );
  pinMode(BUTTON_PIN3, INPUT );
  pinMode(BUTTON_PIN4, INPUT );
  pinMode(BUTTON_PIN5, INPUT );
  pinMode(BUTTON_SLP , INPUT );
  btn_1.setEventHandler(handleEvent);
  btn_2.setEventHandler(handleEvent);
  btn_3.setEventHandler(handleEvent);
  btn_4.setEventHandler(handleEvent);
  btn_5.setEventHandler(handleEvent);
  btn_6.setEventHandler(handleEvent);
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);
  ledcSetup(0, freq_1, resolution);
  ledcSetup(1, freq_2, resolution);
  ledcSetup(2, freq_1, resolution);
  ledcAttachPin(LED_CONN, 0);
  ledcAttachPin(LED_SLP , 2);
  ledcAttachPin(MOT_CONN, 1);
  ledcWrite(0, 0);
  ledcWrite(2, 0);
  xTaskCreate(
    taskWifi,          /* Task function.                       */
    "taskWifi",        /* String with name of task.            */
    10000,            /* Stack size in bytes.                  */
    NULL,             /* Parameter passed as input of the task */
    1,                /* Priority of the task.                 */
    NULL);            /* Task handle.                          */

  xTaskCreate(
    taskConnection,   /* Task function.                        */
    "taskConnection", /* String with name of task.             */
    10000,            /* Stack size in bytes.                  */
    NULL,             /* Parameter passed as input of the task */
    1,                /* Priority of the task.                 */
    NULL);            /* Task handle.                          */
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'
  Serial2.begin(9600);
  myDFPlayer.begin(Serial2);
  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  myDFPlayer.volume(30);  //Set volume value (0~30)
  Serial.println("Iniciado");
  neopix_set(4,0);
  audio(1); 
}

void loop() 
{
  btn_all_check();
  secuencias_motor();
  secuencias_luz();
  last_msgs();
}

//----------------------Funciones Wifi----------------------//
void taskWifi( void * parameter ) 
{
  while (1) 
  {
    for (int i = 0; i < NUM_NETWORKS; i++) 
    {
      //Serial.print("Connecting to ");
      //Serial.print(ssidTab[i]);
      WiFi.begin(ssidTab[i], passwordTab[i]);
      for (int j = 0; j < 10; j++) {
        if (WiFi.status() != WL_CONNECTED) 
        {
          delay(500);
          Serial.print(".");
        } else 
        {
          Serial.println("done");
          Serial.print("IP address: ");
          Serial.println(WiFi.localIP());
#if DEV_TYPE == 0
          Husarnet.join(husarnetJoinCode, hostName0);
#elif DEV_TYPE == 1
          Husarnet.join(husarnetJoinCode, hostName1);
#endif
          Husarnet.start();
          while (WiFi.status() == WL_CONNECTED) 
          {
            delay(500);
          }
        }
      }
    }
  }
}

void taskConnection( void * parameter ) 
{
  while (1) 
  {
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }
#if DEV_TYPE == 0

    server.begin();
    
    do 
    {
      delay(500);
      client = server.available();
      //Serial.println(client.status());
      client.setTimeout(3);
      Serial.println("Waiting for client");
    } while (client < 1);

#elif DEV_TYPE == 1

    while (client.connect(hostName0, port) == 0) 
    {
      delay(500);
      Serial.printf("Connecting to %s\r\n", hostName0);
    }

#endif

    Serial.printf("Client connected: %d\r\n", (int)client.connected());

    unsigned long lastMsg = millis();
    auto lastPing = 0;
    while (client.connected()) 
    {
      ledcWrite(0, 255);
      //ping RX
      if (millis() - lastMsg > 6000) 
      {
        //Serial.println("ping timeout");
        break;
      }

      //ping TX
      auto now = millis();
      if (now > lastPing + 4000) 
      {
        client.print('p');
        lastPing = now;
      }
      
      if(msg_confirmado)
      {
        client.print(msg_to_send);
        msg_confirmado = false   ;
        msg_to_send    = '0'     ; 
      }
      
      if (client.available()) {
        char c = client.read();
        time_vib(c) ;
        sel_light(c);
        Serial.printf("read: %c\r\n", c);
        habilitar_audio = true;
        if(c != 'p')
        {
          Serial.println("Tienes un mensaje de X ¿Quieres escucharlo?");
          audio(9);
          now = millis();
          while(millis()-now <= 4500)
          {
            neopix_set(4,1);
          }
          neopix_set(4,0);
        }
      }
    }
    
    client.stop();
    ledcWrite(0, 0);
    Serial.println("Client disconnected.");
  }
}
//----------------------------------------------------------//


//---------------Funciones para botones---------------------//
void handleEvent(AceButton* button, uint8_t eventType, //boton 1: UP/Last_msg
                 uint8_t buttonState) 
{
  uint8_t pin_button = button->getPin();
  //Serial.println(pin_button);
  switch (eventType) 
  {
    case AceButton::kEventReleased:
      //Serial.print("pressed: ");
      switch(pin_button)
      {
        case BUTTON_PIN1:
          estadoB_1 += 1; 
          if(estadoB_1 > 5 && habilitar_cat_1 == false) estadoB_1 = 0; 
          if(estadoB_1 > 5 && habilitar_cat_1 ==  true) estadoB_1 = estadoB_1_lim; 
          if(estadoB_1 != 0) habilitar_cat_1 = true; 
          habilitar_audio = true;
          //Serial.print("Estado B1: ");
          //Serial.println(estadoB_1);
          break;
       case BUTTON_PIN2:
          estadoB_2 += 1; 
          if(estadoB_2 > 6 && habilitar_cat_2 == false) estadoB_2 = 0; 
          if(estadoB_2 > 6 && habilitar_cat_2 ==  true) estadoB_2 = estadoB_2_lim; 
          if(estadoB_2 != 0) habilitar_cat_2 = true;
          //Serial.print("Estado B2: ");
          //Serial.println(estadoB_2);
          break;
        case BUTTON_PIN3:
          // FUNCION PARA DECIR QUE SE SELECCIONA "OK"
          estado_ok = !estado_ok;
          break;
        case BUTTON_PIN4:
          // Función para cancelar selección
          estado_cancel = !estado_cancel;
          break;
        case BUTTON_PIN5:
          // Función para ver last msg
          ver_last_msg = !ver_last_msg;
          //Serial.println(ver_last_msg);
          break;
        default:
          break;
      }
    case AceButton::kEventPressed:
      if(pin_button == BUTTON_SLP)
      {
        now = millis();
        audio(2); // audio a mimir
        neopix_set(2, 0);
        while(millis()-now <= 2000)
        {
          // mensajes que indica que el dispositivo se "apaga"
          ledcWrite(2, 255); 
          Serial.println("A mimir"); 
        }
        neopix_set(5, 0);
        ledcWrite(2, 0);
        esp_deep_sleep_start();
      }
      //Serial.print("released: ");
      //Serial.println(durationB);
      break;
  }
}
//----------------------------------------------------------//

// Función para seleccionar secuencia de motor a reproducir //
void secuencias_motor()
{
  if(habilitar_cat_2 == true)
  {
    Serial.println("¿Quieres enviar un mensaje a tu pareja?");
    audio(5);
    while(!estado_ok && !estado_cancel)
         {
          btn_all_check();
          if(estado_ok){
            quiere_enviar_msj = true;
            estadoB_2 = 1;
          }
          if(estado_cancel) quiere_enviar_msj = false;
          neopix_set(4,1);
         }
    estadoB_2_lim = 1    ;
    estado_cancel = false;
    estado_ok     = false;
    while(!estado_ok && !estado_cancel && quiere_enviar_msj)
    {
      btn_all_check();
      if(estadoB_2 == 0) estadoB_2 += 1;
      switch(estadoB_2)
      {
       case 2:
         Serial.println("vib1");
         msg_to_send = 'A';
         now = millis();
         while(millis()-now < 500)
         {
           ledcWrite(1, 150);
           neopix_set(6,1);
         }
         ledcWrite(1, 0);
         while(estadoB_2 == 2)
         {
          btn_all_check();
          if(estado_ok || estado_cancel) break;
          neopix_set(6,1);
         }
         break;
       case 3:
         Serial.println("vib2");
         msg_to_send = 'B';
         now = millis();
         while(millis()-now < 1000)
         {
           ledcWrite(1, 150);
           neopix_set(6,1);
         }
         ledcWrite(1, 0);
         while(estadoB_2 == 3)
         {
          btn_all_check();
          if(estado_ok || estado_cancel) break;
          neopix_set(6,1);
         }
         break;
       case 4:
         Serial.println("vib3");
         msg_to_send = 'C';
         now = millis();
         while(millis()-now < 1500)
         {
           ledcWrite(1, 150);
           neopix_set(6,1);
         }
         ledcWrite(1, 0);
         while(estadoB_2 == 4)
         {
          btn_all_check();
          if(estado_ok || estado_cancel) break;
          neopix_set(6,1);
         }
         break;
       case 5:
         Serial.println("vib4");
         msg_to_send = 'D';
         now = millis();
         while(millis()-now < 2000)
         {
           ledcWrite(1, 150);
           neopix_set(6,1);
         }
         ledcWrite(1, 0);
         while(estadoB_2 == 5)
         {
          btn_all_check();
          if(estado_ok || estado_cancel) break;
          neopix_set(6,1);
         }
         break;
       case 6:
         Serial.println("vib5");
         msg_to_send = 'E';
         now = millis();
         while(millis()-now < 2000)
         {
           ledcWrite(1, 150);
           neopix_set(6,1);
         }
         ledcWrite(1, 0);
         while(estadoB_2 == 6)
         {
          btn_all_check();
          if(estado_ok || estado_cancel) break;
          neopix_set(6,1);
         }
         break;
       case 1:
         // msj de confirmación acá
         audio(4);
         Serial.println("Selecciona una vibración.");
         estadoB_2_lim = 2;
         //
         while(estadoB_2 == 1)
         {
          btn_all_check();
          if(estado_ok || estado_cancel) break;
          neopix_set(4,1);
         }
         break;
       default:
         break;
      } 
    }
    // Mensaje de confirmación
    if(!estado_cancel && quiere_enviar_msj)
    {
      Serial.println("¿Estas seguro?");
      audio(6);
    }
    while(estado_ok && !estado_cancel && quiere_enviar_msj)
    {
      neopix_set(4,1);
      btn_all_check();
    }
    //
    if(!estado_ok && !estado_cancel && quiere_enviar_msj)
    {
      // mensaje de mensaje enviado
      Serial.println("Mensaje enviado.");
      audio(7);
      neopix_set(4, 2);
      msg_confirmado = true;
    }
    if(estado_cancel || !quiere_enviar_msj)
    {
      // mensaje de mensaje no enviado
      Serial.println("Mensaje no enviado.");
      audio(8);
    }
    neopix_set(4, 0);
    estado_cancel   = false;
    estado_ok       = false;
    habilitar_cat_2 = false;
    ver_last_msg    = false;
    habilitar_cat_1 = false;
    estadoB_2       = 0    ;
    estadoB_1       = 0    ;
  }
}
//----------------------------------------------------------//


// Función para seleccionar secuencia de luz a mostrar //
void secuencias_luz()
{
  if(habilitar_cat_1 == true)
  {
    Serial.println("¿Quieres enviar un mensaje a tu pareja?");
    audio(5);
    while(!estado_ok && !estado_cancel)
         {
          btn_all_check();
          if(estado_ok){
            quiere_enviar_msj = true;
            estadoB_1 = 1;
          }
          if(estado_cancel) quiere_enviar_msj = false;
          neopix_set(4,1);
         }
    estadoB_1_lim = 1    ;
    estado_cancel = false;
    estado_ok     = false;
    while(!estado_ok && !estado_cancel && quiere_enviar_msj)
    {
      btn_all_check();
      if(estadoB_1 == 0) estadoB_1 += 1;
      switch(estadoB_1)
      {
       case 2:
         Serial.println("luz verde");
         //neopix_set(0, 0);
         msg_to_send = '1';
         while(estadoB_1 == 2)
         {
          btn_all_check();
          if(estado_ok || estado_cancel) break;
          neopix_set(0, 1);
         }
         break;
       case 3:
         Serial.println("luz azul");
         msg_to_send = '2';
         //neopix_set(1, 0);
         while(estadoB_1 == 3)
         {
          btn_all_check();
          if(estado_ok || estado_cancel) break;
          neopix_set(1, 1);
         }
         break;
       case 4:
         Serial.println("luz amarilla");
         //neopix_set(3, 0);
         msg_to_send = '3';
         while(estadoB_1 == 4)
         {
          btn_all_check();
          if(estado_ok || estado_cancel) break;
          neopix_set(3, 1);
         }
         break;
       case 5:
         Serial.println("luz roja");
         //neopix_set(2, 0);
         msg_to_send = '4';
         while(estadoB_1 == 5)
         {
          btn_all_check();
          if(estado_ok || estado_cancel) break;
          neopix_set(2, 1);
         }
         break;
       case 1:
         estadoB_1_lim = 2;
         // msj de confirmación acá
         audio(3);
         Serial.println("Selecciona una Luz.");
         //
         while(estadoB_1 == 1)
         {
          btn_all_check();
          if(estado_ok || estado_cancel) break;
          neopix_set(4,1);
         }
         break;
       default:
         break;
      } 
    }
    // Mensaje de confirmación
    if(!estado_cancel && quiere_enviar_msj)
    {
      Serial.println("Listo, ¿seguro que quieres enviarlo?");
      audio(6);
    }
    while(estado_ok && !estado_cancel && quiere_enviar_msj)
    {
      neopix_set(4,1);
      btn_all_check();
    }
    //
    if(!estado_ok && !estado_cancel && quiere_enviar_msj)
    {
      // mensaje de mensaje enviado
      Serial.println("Tu mensaje ha sido enviado");
      audio(7);
      neopix_set(4, 2);
      //blink_listo    = false;
      msg_confirmado = true;
    }
    if(estado_cancel || !quiere_enviar_msj)
    {
      // mensaje de mensaje no enviado
      Serial.println("Mensaje no enviado.");
      audio(8);
    }
    neopix_set(4, 0);
    estado_cancel     = false;
    estado_ok         = false;
    habilitar_cat_1   = false;
    ver_last_msg      = false;
    habilitar_cat_2   = false;
    quiere_enviar_msj = false;
    estadoB_1         = 0    ;
    estadoB_2         = 0    ;
  }
}
//----------------------------------------------------------//



//--------------------Úlitimos msgs-------------------------//
void last_msgs()
{
  //Serial.println(ver_last_msg);
  if(ver_last_msg)
  {
    Serial.println("El último mensaje recivido es: ");
    neopix_set(last_mesagge[1] - '0', 0);
    now = millis();
    while(millis()-now <= 2500)
    {
      // funciones para reproducir los últimos msgs (luz+vib)
      if(millis()-now <= tiempo_msg) ledcWrite(1, 150);
      else ledcWrite(1, 0);
      // -- función para establecer secuencia de luz
      Serial.println(msg_luz);
    }
    ledcWrite(1, 0);
    neopix_set(4, 0);
    ver_last_msg = false;
  }
}

void time_vib(char B)
{
  switch(B)
  {
    case 'A':
      tiempo_msg      = 500;
      last_mesagge[0] = 'A';
      break;
    case 'B':
      tiempo_msg      = 1000;
      last_mesagge[0] = 'B' ;
      break;
    case 'C':
      tiempo_msg = 1500;
      last_mesagge[0] = 'C';
      break;
    case 'D':
      tiempo_msg      = 2000;
      last_mesagge[0] = 'D' ;
      break;
    case 'E':
      tiempo_msg      = 2500;
      last_mesagge[0] = 'E' ;
      break;
    default:
      break;
  }
}

void sel_light(char B)
{
  switch(B)
  {
    case '1':
      msg_luz         = "Verde";
      last_mesagge[1] = '0'    ;
      break;
    case '2':
      msg_luz         = "Azul";
      last_mesagge[1] = '1'   ;
      break;
    case '3':
      msg_luz         = "Amarillo";
      last_mesagge[1] = '3'       ;
      break;
    case '4':
      msg_luz         = "Rojo";
      last_mesagge[1] = '2'   ;
      break;
    default:
      break;
  }
}
//----------------------------------------------------------//


//--------------------Demo color neopixel-------------------//
void neopix_set(int color, int efecto)
{
  switch(color) 
  {   
    case 0:
      if(efecto == 0) colorWipe(strip.Color(  0, luminosidad,   0), 5);    // Green
      if(efecto == 1) colorCircle(strip.Color(  0, luminosidad,   0), 2);
      break;
    case 1:
      if(efecto == 0) colorWipe(strip.Color(  0,   0, luminosidad), 5);    // Blue
      if(efecto == 1) colorCircle(strip.Color(  0,   0, luminosidad), 2);
      break;
    case 2:
      if(efecto == 0) colorWipe(strip.Color(luminosidad,   0,   0), 5);    // Red 
      if(efecto == 1) colorCircle(strip.Color(luminosidad,   0,   0), 2);
      break;
    case 3:
      if(efecto == 0) colorWipe(strip.Color(luminosidad,   luminosidad,   0), 5);    // Yellow
      if(efecto == 1) colorCircle(strip.Color(luminosidad,   luminosidad,   0), 2);
      break;
    case 4:
      if(efecto == 0) colorWipe(strip.Color(luminosidad, luminosidad, luminosidad), 5); // White
      if(efecto == 1)
      {
        if(subida == true)
        {
          factor += 1;
          if(factor == 10) subida = false;
        }
        if(subida == false)
        {
          factor -= 1;
          if(factor == 1) subida = true;
        }
        colorWipe(strip.Color(luminosidad/factor, luminosidad/factor, luminosidad/factor), 3);
      }
      if(efecto == 2)
      {
        colorBlink(strip.Color(luminosidad, luminosidad, luminosidad));
      }
      break;
    case 5:
      if(efecto == 0) colorWipe(strip.Color(  0,   0,   0), 5);    // Black/off
      break;
    case 6:
      if(efecto == 0) colorWipe(strip.Color(luminosidad,luminosidad/2,0), 5);    // Brown
      if(efecto == 1)
      {
        if(subida == true)
        {
          factor += 1;
          if(factor == 10) subida = false;
        }
        if(subida == false)
        {
          factor -= 1;
          if(factor == 1) subida = true;
        }
        colorWipe(strip.Color(luminosidad/factor, luminosidad/(factor*2), 0), 3);
      }
      break;
  }
}

void colorWipe(uint32_t color, int wait) 
{  
  for(int i=0; i<strip.numPixels(); i++) // For each pixel in strip...
  {
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void colorCircle(uint32_t color, int wait) 
{  
  for(int i=0; i<strip.numPixels(); i++) // For each pixel in strip...
  {
    btn_all_check();
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    if(i == index_pix) strip.setPixelColor(i, strip.Color(0,0,0));
    if(i == index_pix+1) strip.setPixelColor(i, strip.Color(0,0,0));
    if(i == index_pix+2) strip.setPixelColor(i, strip.Color(0,0,0));
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
  index_pix += 1;
  if(index_pix == 19) index_pix = 0;
}

void colorBlink(uint32_t color) 
{
  if(!blink_listo)
  {
    colorWipe(color, 1);
    delay(100);
    colorWipe(strip.Color(0,0,0), 1);
    delay(100);
    colorWipe(color, 1);
    delay(100);
    colorWipe(strip.Color(0,0,0), 1);  
    //blink_listo = true; 
  }
}
//----------------------------------------------------------//



//------------------------Audio-----------------------------//
void audio(int num_audio)
{
  // num_audio = 1 - audio 0001 - Encendido 
  // num_audio = 2 - audio 0002 - Apagado
  // num_audio = 3 - audio 0003 - Seleccionar Luz
  // num_audio = 4 - audio 0004 - Seleccionar Vibración
  // num_audio = 5 - audio 0005 - ¿Quieres enviar un audio?
  // num_audio = 6 - audio 0006 - Listo, ¿Estas seguro?
  // num_audio = 7 - audio 0007 - Mensaje enviado
  // num_audio = 8 - audio 0008 - Mensajo no enviado
  myDFPlayer.pause();
  delay(100);
  myDFPlayer.play(num_audio);
  habilitar_audio = false;
}
//----------------------------------------------------------//



//--------------------Check botones-------------------------//
void btn_all_check()
{
  btn_1.check();
  btn_2.check();
  btn_3.check();
  btn_4.check();
  btn_5.check();
  btn_6.check();
}
//---------------------------------------------------------//
