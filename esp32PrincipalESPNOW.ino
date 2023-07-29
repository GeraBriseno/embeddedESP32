#include "Nextion.h"
#include "AiEsp32RotaryEncoder.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

Preferences preferences;

//VARIABLES SENSORES DE FLUJO//
int intervaloFlujo = 1000;
long milisActuales = 0;
long milisPrevios = 0;
float FactorDeCalibracion = 4.5;
volatile byte conteoPulsoHardener;
volatile byte conteoPulsoResina;
byte pulsoHardener1seg = 0;
byte pulsoResina1seg = 0;
float rateFlujoHardener = 0.0; 
float rateFlujoResina = 0.0;
int flujoMililitrosHardener = 0;
int flujoMililitrosResina = 0;
unsigned long totalMililitrosHardener = 0;
unsigned long totalMililitrosResina = 0;
float diferenciaFlujos = 0;

//VARIABLE AUXILIAR PARA EL ENCODER//
int ECS = 0;

//VARIABLES AUXILIARES PARA FUNCION BAJA PRESION//
int bajaPresion = 0;
long inicioBajaPresion = 0;

//VARIABLES AUXILIARES PARA MEDIR TIEMPO RECIRCULACION//
int segundosRecirculacion = 0;
long ultimoSegundo = 0;

//VARIABLES PARA LECTURA DE LOS SENSORES//
int sensorCount = 0;

int valorSensorNivel1 = 0;
int valorSensorNivel2 = 0;

float lecturaTemperatura1 = 0;
float promedioSensorTemperatura1 = 0;
float valorSensorTemperatura1 = 0;

float lecturaTemperatura2 = 0;
float promedioSensorTemperatura2 = 0;
float valorSensorTemperatura2 = 0;

float lecturaPresion1 = 0;
float promedioSensorPresion1 = 0;
float valorSensorPresion1 = 0;

float lecturaPresion2 = 0;
float promedioSensorPresion2 = 0;
float valorSensorPresion2 = 0;

//PINES ENCODER//
#define ROTARY_ENCODER_A_PIN 12
#define ROTARY_ENCODER_B_PIN 14
#define ROTARY_ENCODER_BUTTON_PIN 25
#define ROTARY_ENCODER_VCC_PIN 27

#define ROTARY_ENCODER_STEPS 4

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

//VARIABLE AUXILIAR PARA MOSTRAR DATOS EN LA PANTALLA NEXTION//
char result[8];

//PINES SENSORES//
const int pinSensorTemp1 = 13;
const int pinSensorTemp2 = 34;
const int pinSensorPresion1 = 32;
const int pinSensorPresion2 = 35;
const int pinSensorNivel1 = 33;
const int pinSensorNivel2 = 25;
const int pinSensorFlujoHardener = 36;  //podría cambiarse
const int pinSensorFlujoResina = 39;    //podría cambiarse

const int pinLed = 2;

//PINES ITV//
const int itvResina = 4;
const int itvHardener = 21; 
const int itvAire = 26;

//VARIABLES PARA CONTROLAR ESTADO Y OPERACION//
int estadoRecirculacion = 0;
int estadoInyeccion = 0;
int estadoPurga = 0;

int estadoPausaInyeccion = 0;
int estadoAirePurgas = 0;

int purgaSeleccionada = 0;
int paginaSeleccionada = 0;

int presionMinimaSeleccionada = 0;
int variableSeleccionadaRecirculacion = 0;   //3
int variableSeleccionadaInyeccion = 0;       //4
int variableSeleccionadaPurgas = 0;          //2

float presionMinima = 0;
float presionResinaRecirculacion = 0;
float presionHardenerRecirculacion = 0;
int tiempoRecirculacion = 5;
float presionResinaInyeccion = 0;
float presionHardenerInyeccion = 0;
float presionAireInyeccion = 0;
float presionAireInyeccionAplicada = 0;
float presionResinaPurgas = 0;
float presionHardenerPurgas = 0;

int presionRRMap = 0;
int presionHRMap = 0;
int presionRIMap = 0;
int presionHIMap = 0;
int presionAIMap = 0;
int presionAIAplicadaMap = 0;
int presionRPMap = 0;
int presionHPMap = 0;

//OBJETOS DE LA PANTALLA NEXTION//
NexPage pagPrincipal = NexPage(0, 0, "pPrincipal");
NexButton bP0Recirculacion = NexButton(0, 3, "bR");
NexButton bP0Inyeccion = NexButton(0, 5, "bI");
NexButton bP0Purgas = NexButton(0, 4, "bP");
NexText tP0Temperatura1 = NexText(0, 10, "t4");
NexText tP0Temperatura2 = NexText(0, 11, "t5");
NexText tP0NResina = NexText(0, 12, "t6");
NexText tP0NHardener = NexText(0, 13, "t7");
NexText tP0PresionMinimaSelect = NexText(0, 14, "t8");
NexText tP0PresionMinima = NexText(0, 15, "t9"); 

NexPage pagRecirculacion = NexPage(1, 0, "pRecircularIN");
NexDSButton bP1Iniciar = NexDSButton(1, 11, "bt0");
NexButton bP1Principal = NexButton(1, 2, "bPr");
NexButton bP1Inyeccion = NexButton(1, 5, "bI");
NexButton bP1Purgas = NexButton(1, 4, "bP");
NexText tP1TiempoSelect = NexText(1, 10, "t0");
NexText tP1ResinaSelect = NexText(1, 6, "t1");
NexText tP1HardenerSelect = NexText(1, 7, "t2");
NexText tP1Tiempo = NexText(1, 16, "t8");
NexText tP1Resina = NexText(1, 14, "t6");
NexText tP1Hardener = NexText(1, 15, "t7");
NexText tP1Presion1 = NexText(1, 17, "t9");
NexText tP1Presion2 = NexText(1, 18, "t10");
NexVariable varRecirculacion = NexVariable(1, 12, "recircularON");

NexPage pagInyeccion = NexPage(2, 0, "pInyeccionIN");
NexDSButton bP2Iniciar = NexDSButton(2, 12, "bt0");
NexDSButton bP2Pausa = NexDSButton(2, 13, "bt1");
NexButton bP2Principal = NexButton(2, 2, "bPr");
NexButton bP2Recirculacion = NexButton(2, 3, "bR");
NexButton bP2Purgas = NexButton(2, 4, "bP");
NexText tP2ResinaSelect = NexText(2, 6, "t1");
NexText tP2HardenerSelect = NexText(2, 7, "t2");
NexText tP2AireSelect = NexText(2, 11, "t3");
NexText tP2Resina = NexText(2, 17, "t9");
NexText tP2Hardener = NexText(2, 18, "t10");
NexText tP2Aire = NexText(2, 19, "t11");
NexText tP2Temperatura1 = NexText(2, 20, "t13");
NexText tP2Temperatura2 = NexText(2, 21, "t14");
NexText tP2NResina = NexText(2, 22, "t15");
NexText tP2NHardener = NexText(2, 23, "t16");
NexVariable varInyeccion = NexVariable(2, 14, "inyeccionON");

NexPage pagPurgas = NexPage(3, 0, "pPurgasIN");
NexDSButton bP3Iniciar = NexDSButton(3, 11, "bt3");
NexDSButton bP3Resina = NexDSButton(3, 9, "bt0");
NexDSButton bP3Hardener = NexDSButton(3, 10, "bt1");
NexDSButton bP3Aire = NexDSButton(3, 7, "bt2");
NexButton bP3Principal = NexButton(3, 2, "bPr");
NexButton bP3Recirculacion = NexButton(3, 3, "bR");
NexButton bP3Inyeccion = NexButton(3, 5, "bI");
NexText tP3ResinaSelect = NexText(3, 6, "t0");
NexText tP3HardenerSelect = NexText(3, 8, "t1");
NexText tP3Resina = NexText(3, 16, "t3");
NexText tP3Hardener = NexText(3, 17, "t4");
NexVariable varPurgas = NexVariable(3, 12, "purgasON");
NexVariable varSelectP = NexVariable(3, 14, "selectP");

NexTouch *nex_listen_list[] = {
    
    &bP0Recirculacion,
    &bP0Inyeccion,
    &bP0Purgas,
    &tP0PresionMinimaSelect,
    &bP1Iniciar,
    &bP1Principal,
    &bP1Inyeccion,
    &bP1Purgas,
    &tP1TiempoSelect,
    &tP1ResinaSelect,
    &tP1HardenerSelect,
    &bP2Iniciar,
    &bP2Pausa,
    &bP2Principal,
    &bP2Recirculacion,
    &bP2Purgas,
    &tP2ResinaSelect,
    &tP2HardenerSelect,
    &tP2AireSelect,
    &bP3Iniciar,
    &bP3Resina,
    &bP3Hardener,
    &bP3Aire,
    &bP3Principal,
    &bP3Recirculacion,
    &bP3Inyeccion,
    &tP3ResinaSelect,
    &tP3HardenerSelect,
    NULL
};

bool received = false;

String success;

//DIRECCION DE ESP A 
//uint8_t broadcastAddress[] = {0xCC,0xDB,0xA7,0x16,0x15,0x70};

//DIRECCION DE ESP B
//uint8_t broadcastAddress[] = {0xC8,0xF0,0x9E,0x9A,0xD6,0x28};

//DIRECCION DE ESP C
//uint8_t broadcastAddress[] = {0xC8,0xF0,0x9E,0x53,0x25,0xB4};

//DIRECCION DE ESP 2 REPETIDOR
uint8_t broadcastAddress[] = {0xC8,0xF0,0x9E,0x9C,0xB8,0x7C};

typedef struct struct_message {
    int id;
    int operacion;
    float valor;
    int confirmacion;
} struct_message;

struct_message mensajeESPNOWsaliente;
struct_message mensajeESPNOWentrante;
esp_now_peer_info_t peerInfo;

//mandarDatosESPNOW();

void mandarDatosESPNOW(){

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &mensajeESPNOWsaliente, sizeof(mensajeESPNOWsaliente));
  
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

//FUNCION SENSORES FLUJO//
void IRAM_ATTR pulseCounterHardener()
{
  conteoPulsoHardener++;
}
void IRAM_ATTR pulseCounterResina()
{
  conteoPulsoResina++;
}

//FUNCIONES//
void mandarDatosSerial(int operacion, int valor){
  
  StaticJsonDocument<200> doc;
  doc["operacion"] = operacion;
  doc["valor"] = valor;
  
  serializeJson(doc, Serial);
}

void rotary_onButtonClick()
{
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500)
  {
    return;
  }
  lastTimePressed = millis(); 
}

//FUNCION PRINCIAL DEL ENCODER//
void rotary_loop()
{
  //dont print anything unless value changed
  if (rotaryEncoder.encoderChanged() && ECS == 0)
  { 
    switch(paginaSeleccionada){
      case 0:
        if(presionMinimaSeleccionada == 1){
          presionMinima = rotaryEncoder.readEncoder();
          preferences.putFloat("presionMinima", presionMinima);
          tP0PresionMinima.setText(dtostrf(presionMinima/10, 2, 1, result));
          if(ECS == 0){
            mensajeESPNOWsaliente.operacion = 7;
            mensajeESPNOWsaliente.valor = presionMinima;
            mandarDatosESPNOW();
              }
          else{
            ECS = 0;
            }
          }
        break;
      case 1:
        switch(variableSeleccionadaRecirculacion){
          case 0:
            presionResinaRecirculacion = rotaryEncoder.readEncoder();
            preferences.putFloat("resinaR", presionResinaRecirculacion);
            tP1Resina.setText(dtostrf(presionResinaRecirculacion/10, 2, 1, result));
            //mandarDatosEthernet(10, presionResinaRecirculacion);
            //eCS == 0;
            if(ECS == 0){
              mensajeESPNOWsaliente.operacion = 10;
              mensajeESPNOWsaliente.valor = presionResinaRecirculacion;
              mandarDatosESPNOW();
              }
            else{
              ECS = 0;
              }
            if(estadoRecirculacion == 1){
              presionRRMap = map(presionResinaRecirculacion, 0, 65, 0, 255);
              analogWrite(itvResina, presionRRMap);
              }
            break;
          case 1:
            presionHardenerRecirculacion = rotaryEncoder.readEncoder();
            preferences.putFloat("hardenerR", presionHardenerRecirculacion);
            tP1Hardener.setText(dtostrf(presionHardenerRecirculacion/10, 2, 1, result));
            if(ECS == 0){
              mensajeESPNOWsaliente.operacion = 12;
              mensajeESPNOWsaliente.valor = presionHardenerRecirculacion;
              mandarDatosESPNOW();
              }
            else{
              ECS = 0;
              }
            if(estadoRecirculacion == 1){
              presionHRMap = map(presionHardenerRecirculacion, 0, 65, 0, 255);
              analogWrite(itvHardener, presionHRMap);
              }
            break;
          case 2:
            tiempoRecirculacion = rotaryEncoder.readEncoder();
            tP1Tiempo.setText(dtostrf(tiempoRecirculacion, 2, 0, result));
            if(ECS == 0){
              mensajeESPNOWsaliente.operacion = 45;
              mensajeESPNOWsaliente.valor = tiempoRecirculacion;
              mandarDatosESPNOW();
              }
            else{
              ECS = 0;
              }
            break;       
          }
          break;
        case 2:
          switch(variableSeleccionadaInyeccion){
            case 0:
              presionResinaInyeccion = rotaryEncoder.readEncoder();
              preferences.putFloat("resinaI", presionResinaInyeccion);
              tP2Resina.setText(dtostrf(presionResinaInyeccion/10, 2, 1, result));
              if(ECS == 0){
                mensajeESPNOWsaliente.operacion = 14;
                mensajeESPNOWsaliente.valor = presionResinaInyeccion;
                mandarDatosESPNOW();
                }
              else{
                ECS = 0;
                }
              //mandarDatosEthernet(14, presionResinaInyeccion);
              if(estadoInyeccion == 1 && estadoPausaInyeccion == 0){
                presionRIMap = map(presionResinaInyeccion, 0, 65, 0, 255);
                analogWrite(itvResina, presionRIMap);
              }
              break;
            case 1:
              presionHardenerInyeccion = rotaryEncoder.readEncoder();
              preferences.putFloat("hardenerI", presionHardenerInyeccion);
              tP2Hardener.setText(dtostrf(presionHardenerInyeccion/10, 2, 1, result));
              if(ECS == 0){
                mensajeESPNOWsaliente.operacion = 16;
                mensajeESPNOWsaliente.valor = presionHardenerInyeccion;
                mandarDatosESPNOW();
                }
              else{
                ECS = 0;
                }
              //mandarDatosEthernet(16, presionHardenerInyeccion);
              if(estadoInyeccion == 1 && estadoPausaInyeccion == 0){
                presionHIMap = map(presionHardenerInyeccion, 0, 65, 0, 255);
                analogWrite(itvHardener, presionHIMap);
              }
              break;
            case 2:
              presionAireInyeccion = rotaryEncoder.readEncoder();
              presionAireInyeccionAplicada = presionAireInyeccion;
              preferences.putFloat("aireI", presionAireInyeccion);
              preferences.putFloat("aireAplicadaI", presionAireInyeccionAplicada);
              tP2Aire.setText(dtostrf((presionAireInyeccion/10)*1.3, 2, 1, result));
              if(ECS == 0){
                mensajeESPNOWsaliente.operacion = 18;
                mensajeESPNOWsaliente.valor = presionAireInyeccion;
                mandarDatosESPNOW();
                }
              else{
                ECS = 0;
                }
              //mandarDatosEthernet(18, presionAireInyeccion);
              if(estadoInyeccion == 1){
                presionAIMap = map(presionAireInyeccion, 0, 65, 0, 255);
                analogWrite(itvAire, presionAIMap);
              }
              break;
            }
            break;
          case 3:
            switch(variableSeleccionadaPurgas){
              case 0:
                presionResinaPurgas = rotaryEncoder.readEncoder();
                preferences.putFloat("resinaP", presionResinaPurgas);
                tP3Resina.setText(dtostrf(presionResinaPurgas/10, 2, 1, result));
                if(ECS == 0){
                  mensajeESPNOWsaliente.operacion = 20;
                  mensajeESPNOWsaliente.valor = presionResinaPurgas;
                  mandarDatosESPNOW();
                  }
                else{
                  ECS = 0;
                  }
                //mandarDatosEthernet(20, presionResinaPurgas);
                if(estadoPurga == 1 && purgaSeleccionada == 1){
                  presionRPMap = map(presionResinaPurgas, 0, 65, 0, 255);
                  analogWrite(itvResina, presionRPMap);
                }
                break;
              case 1:
                presionHardenerPurgas = rotaryEncoder.readEncoder();
                preferences.putFloat("hardenerP", presionHardenerPurgas);
                tP3Hardener.setText(dtostrf(presionHardenerPurgas/10, 2, 1, result));
                if(ECS == 0){
                  mensajeESPNOWsaliente.operacion = 22;
                  mensajeESPNOWsaliente.valor = presionHardenerPurgas;
                  mandarDatosESPNOW();
                  }
                else{
                  ECS = 0;
                  }
                //mandarDatosEthernet(22, presionHardenerPurgas);
                if(estadoPurga == 1 && purgaSeleccionada == 2){
                  presionHPMap = map(presionHardenerPurgas, 0, 65, 0, 255);
                  analogWrite(itvHardener, presionHPMap);
                }
                break;
          }
        break;
      }
  }
  /*if (rotaryEncoder.isEncoderButtonClicked())
  {
    rotary_onButtonClick();
  }*/
}

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

//FUNCIONES//
void terminarTodasOperaciones(){
  analogWrite(itvResina, 0);
  analogWrite(itvHardener, 0);
  analogWrite(itvAire, 0);     
}

void botonPaginaPrincipal(void *ptr){
  paginaSeleccionada = 0;
  sensorCount = 0;
  presionMinimaSeleccionada = 0;
  variableSeleccionadaRecirculacion = 0;
  variableSeleccionadaInyeccion = 0;
  variableSeleccionadaPurgas = 0;
  tP0PresionMinima.setText(dtostrf(presionMinima/10, 2, 1, result));
  tiempoRecirculacion = 5;
  mensajeESPNOWsaliente.operacion = 24;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}

void botonPaginaRecirculacion(void *ptr){
  paginaSeleccionada = 1;
  sensorCount = 0;
  presionMinimaSeleccionada = 0;
  variableSeleccionadaRecirculacion = 0;
  variableSeleccionadaInyeccion = 0;
  variableSeleccionadaPurgas = 0;
  tP1Resina.setText(dtostrf(presionResinaRecirculacion/10, 2, 1, result));
  tP1Hardener.setText(dtostrf(presionHardenerRecirculacion/10, 2, 1, result));
  tP1Tiempo.setText(dtostrf(tiempoRecirculacion, 2, 0, result));
  rotaryEncoder.setEncoderValue(presionResinaRecirculacion);
  tiempoRecirculacion = 5;
  mensajeESPNOWsaliente.operacion = 25;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}

void botonPaginaInyeccion(void *ptr){
  paginaSeleccionada = 2;
  sensorCount = 0;
  presionMinimaSeleccionada = 0;
  variableSeleccionadaRecirculacion = 0;
  variableSeleccionadaInyeccion = 0;
  variableSeleccionadaPurgas = 0;
  tP2Resina.setText(dtostrf(presionResinaInyeccion/10, 2, 1, result));
  tP2Hardener.setText(dtostrf(presionHardenerInyeccion/10, 2, 1, result));
  tP2Aire.setText(dtostrf((presionAireInyeccion/10)*1.3, 2, 1, result));
  rotaryEncoder.setEncoderValue(presionResinaInyeccion);
  tiempoRecirculacion = 5;
  mensajeESPNOWsaliente.operacion = 26;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}

void botonPaginaPurgas(void *ptr){
  paginaSeleccionada = 3;
  sensorCount = 0;
  presionMinimaSeleccionada = 0;
  variableSeleccionadaRecirculacion = 0;
  variableSeleccionadaInyeccion = 0;
  variableSeleccionadaPurgas = 0;
  tP3Resina.setText(dtostrf(presionResinaPurgas/10, 2, 1, result));
  tP3Hardener.setText(dtostrf(presionHardenerPurgas/10, 2, 1, result));
  rotaryEncoder.setEncoderValue(presionResinaPurgas);
  tiempoRecirculacion = 5;
  mensajeESPNOWsaliente.operacion = 27;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}

void recirculacion(void *ptr){
  if(estadoRecirculacion == 0 && tiempoRecirculacion>0){
    presionRRMap = map(presionResinaRecirculacion, 0, 65, 0, 255);
    presionHRMap = map(presionHardenerRecirculacion, 0, 65, 0, 255);
    analogWrite(itvResina, presionRRMap);
    analogWrite(itvHardener, presionHRMap);
    Serial2.print("t8.pco=1024");
    Serial2.print("\xFF\xFF\xFF");
    estadoRecirculacion = 1;
    mandarDatosSerial(1, 1);
    mensajeESPNOWsaliente.operacion = 1;
    mensajeESPNOWsaliente.valor = 1;
    mandarDatosESPNOW();    
    }
  else if(estadoRecirculacion == 1){
    terminarTodasOperaciones();
    Serial2.print("t8.pco=0");
    Serial2.print("\xFF\xFF\xFF");
    estadoRecirculacion = 0;
    mandarDatosSerial(1, 0);
    mensajeESPNOWsaliente.operacion = 1;
    mensajeESPNOWsaliente.valor = 0;
    mandarDatosESPNOW();
    }        
}

void inyeccion(void *ptr){
  if(estadoInyeccion == 0){
    presionRIMap = map(presionResinaInyeccion, 0, 65, 0, 255);
    presionHIMap = map(presionHardenerInyeccion, 0, 65, 0, 255);
    presionAIMap = map(presionAireInyeccion, 0, 65, 0, 255);
    analogWrite(itvResina, presionRIMap);
    analogWrite(itvHardener, presionHIMap);
    analogWrite(itvAire, presionAIMap);
    estadoInyeccion = 1;
    mandarDatosSerial(2, 1);
    mensajeESPNOWsaliente.operacion = 2;
    mensajeESPNOWsaliente.valor = 1;
    mandarDatosESPNOW();
    }
  else {
    terminarTodasOperaciones();
    estadoInyeccion = 0;
    bajaPresion = 0;
    estadoPausaInyeccion = 0;
    mandarDatosSerial(2, 0);
    mandarDatosSerial(5, 0);
    mensajeESPNOWsaliente.operacion = 2;
    mensajeESPNOWsaliente.valor = 0;
    mandarDatosESPNOW();
    Serial2.print("vis t0,0");
    Serial2.print("\xFF\xFF\xFF");
    }        
}

void purga(void *ptr){
  if(estadoPurga == 0){
    if(purgaSeleccionada == 1){
      presionRPMap = map(presionResinaPurgas, 0, 65, 0, 255);
      analogWrite(itvResina, presionRPMap);
      mandarDatosSerial(3, 1);
      mensajeESPNOWsaliente.operacion = 3;
      mensajeESPNOWsaliente.valor = 1;
      mandarDatosESPNOW();
      }
    else if(purgaSeleccionada == 2){
      presionHPMap = map(presionHardenerPurgas, 0, 65, 0, 255);
      analogWrite(itvHardener, presionHPMap);
      mandarDatosSerial(4, 1);
      mensajeESPNOWsaliente.operacion = 4;
      mensajeESPNOWsaliente.valor = 1;
      mandarDatosESPNOW();
      }
    estadoPurga = 1;
    }
  else{
    terminarTodasOperaciones();
    estadoAirePurgas = 0;
    estadoPurga = 0;
    mandarDatosSerial(3, 0);
    mandarDatosSerial(4, 0);
    if(purgaSeleccionada == 1){
      mensajeESPNOWsaliente.operacion = 3;
      mensajeESPNOWsaliente.valor = 0;
      mandarDatosESPNOW();
      }
    else if(purgaSeleccionada == 2){
      mensajeESPNOWsaliente.operacion = 4;
      mensajeESPNOWsaliente.valor = 0;
      mandarDatosESPNOW();
      }
    }        
}

void purgaResinaSelect(void *ptr){
  purgaSeleccionada = 1;
  mensajeESPNOWsaliente.operacion = 36;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}

void purgaHardenerSelect(void *ptr){
  purgaSeleccionada = 2;
  mensajeESPNOWsaliente.operacion = 37;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();        
}

void pausaInyeccion(void *ptr){
  if(estadoPausaInyeccion == 0){
    estadoPausaInyeccion = 1;
    analogWrite(itvResina, 0);
    analogWrite(itvHardener, 0);
    mandarDatosSerial(2, 0);
    mensajeESPNOWsaliente.operacion = 5;
    mensajeESPNOWsaliente.valor = 0;
    mandarDatosESPNOW();
    }
  else{
    estadoPausaInyeccion = 0;
    presionRIMap = map(presionResinaInyeccion, 0, 65, 0, 255);
    presionHIMap = map(presionHardenerInyeccion, 0, 65, 0, 255);
    analogWrite(itvResina, presionRIMap);
    analogWrite(itvHardener, presionHIMap);
    mandarDatosSerial(2, 1);
    mensajeESPNOWsaliente.operacion = 5;
    mensajeESPNOWsaliente.valor = 1;
    mandarDatosESPNOW();    
    }
}

void airePurgas(void *ptr){
  if(estadoAirePurgas == 0){
    estadoAirePurgas = 1;
    analogWrite(itvAire, 200);
    mensajeESPNOWsaliente.operacion = 6;
    mensajeESPNOWsaliente.valor = 1;
    mandarDatosESPNOW(); 
    }
  else{
    estadoAirePurgas = 0;
    analogWrite(itvAire, 0);
    mensajeESPNOWsaliente.operacion = 6;
    mensajeESPNOWsaliente.valor = 0;
    mandarDatosESPNOW();
    }
}
void principalPresionMinimaSelect(void *ptr){
  presionMinimaSeleccionada = 1;
  rotaryEncoder.setEncoderValue(presionMinima);
  mensajeESPNOWsaliente.operacion = 9;
  mensajeESPNOWsaliente.valor = 0;
  mandarDatosESPNOW();  
}
void recirculacionResinaSelect(void *ptr){
  variableSeleccionadaRecirculacion = 0;
  rotaryEncoder.setEncoderValue(presionResinaRecirculacion);
  mensajeESPNOWsaliente.operacion = 28;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}
void recirculacionHardenerSelect(void *ptr){
  variableSeleccionadaRecirculacion = 1;
  rotaryEncoder.setEncoderValue(presionHardenerRecirculacion);
  mensajeESPNOWsaliente.operacion = 29;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}
void recirculacionTiempoSelect(void *ptr){
  variableSeleccionadaRecirculacion = 2;
  rotaryEncoder.setEncoderValue(tiempoRecirculacion);
  mensajeESPNOWsaliente.operacion = 30;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}

void inyeccionResinaSelect(void *ptr){
  variableSeleccionadaInyeccion = 0;
  rotaryEncoder.setEncoderValue(presionResinaInyeccion);
  mensajeESPNOWsaliente.operacion = 31;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}
void inyeccionHardenerSelect(void *ptr){
  variableSeleccionadaInyeccion = 1;
  rotaryEncoder.setEncoderValue(presionHardenerInyeccion);
  mensajeESPNOWsaliente.operacion = 32;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}
void inyeccionAireSelect(void *ptr){
  variableSeleccionadaInyeccion = 2;
  rotaryEncoder.setEncoderValue(presionAireInyeccion);
  mensajeESPNOWsaliente.operacion = 33;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}

void purgasResinaSelect(void *ptr){
  variableSeleccionadaPurgas = 0;
  rotaryEncoder.setEncoderValue(presionResinaPurgas);
  mensajeESPNOWsaliente.operacion = 34;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}
void purgasHardenerSelect(void *ptr){
  variableSeleccionadaPurgas = 1;
  rotaryEncoder.setEncoderValue(presionHardenerPurgas);
  mensajeESPNOWsaliente.operacion = 35;
  mensajeESPNOWsaliente.valor = 1;
  mandarDatosESPNOW();
}

/*void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&mensajeESPNOWentrante, incomingData, sizeof(mensajeESPNOWentrante));
  received = true;
  digitalWrite(pinLed, HIGH);
}*/

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){

    memcpy(&mensajeESPNOWentrante, incomingData, sizeof(mensajeESPNOWentrante));

    digitalWrite(pinLed, HIGH);
    
    int operacion = mensajeESPNOWentrante.operacion;
    int valor = mensajeESPNOWentrante.valor;               
  
    switch(operacion){
      case 1:
        if(valor == 1){
          if(estadoRecirculacion == 0){
            varRecirculacion.setValue(1);
            bP1Iniciar.setValue(1);
            bP1Iniciar.setText("Terminar");
            Serial2.print("tsw bPr,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bI,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bP,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("t8.pco=1024");
            Serial2.print("\xFF\xFF\xFF");               
            presionRRMap = map(presionResinaRecirculacion, 0, 65, 0, 255);
            presionHRMap = map(presionHardenerRecirculacion, 0, 65, 0, 255);
            analogWrite(itvResina, presionRRMap);
            analogWrite(itvHardener, presionHRMap);
            estadoRecirculacion = 1;
            mandarDatosSerial(1, 1);
            }
          }
        else{
          if(estadoRecirculacion == 1){
            varRecirculacion.setValue(0);
            Serial2.print("tsw bPr,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bI,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bP,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("t8.pco=0");
            Serial2.print("\xFF\xFF\xFF");
            bP1Iniciar.setValue(0);
            bP1Iniciar.setText("Iniciar");
            terminarTodasOperaciones();
            estadoRecirculacion = 0;
            mandarDatosSerial(1, 0);
            }
          }
        break;
      case 2:
        if(valor == 1){
          if(estadoInyeccion == 0){
            varInyeccion.setValue(1);
            bP2Iniciar.setValue(1);
            bP2Iniciar.setFont(0);
            bP2Iniciar.setText("Terminar");
            bP2Iniciar.setValue(1);
            bP2Iniciar.setFont(0);
            bP2Iniciar.setText("Terminar");
            Serial2.print("tsw bt1,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bPr,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bR,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bP,0");
            Serial2.print("\xFF\xFF\xFF");                 
            presionRIMap = map(presionResinaInyeccion, 0, 65, 0, 255);
            presionHIMap = map(presionHardenerInyeccion, 0, 65, 0, 255);
            presionAIMap = map(presionAireInyeccion, 0, 65, 0, 255);
            analogWrite(itvResina, presionRIMap);
            analogWrite(itvHardener, presionHIMap);
            analogWrite(itvAire, presionAIMap);
            estadoInyeccion = 1;
            mandarDatosSerial(2, 1);
            }
          }
        else{
          if(estadoInyeccion == 1){
            varInyeccion.setValue(0);
            bP2Iniciar.setValue(0);
            bP2Iniciar.setFont(3);
            bP2Iniciar.setText("Iniciar");
            bP2Iniciar.setValue(0);
            bP2Iniciar.setFont(3);
            bP2Iniciar.setText("Iniciar");
            Serial2.print("tsw bt1,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bPr,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bR,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bP,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("vis t0,0");
            Serial2.print("\xFF\xFF\xFF");
            bP2Pausa.setValue(0);
            terminarTodasOperaciones();
            estadoInyeccion = 0;
            estadoPausaInyeccion = 0;
            bajaPresion = 0;
            mandarDatosSerial(2, 0);
            mandarDatosSerial(5, 0);
            }
          }
        break;
      case 3:
        if(valor == 1){
          if(estadoPurga == 0){                
            presionRPMap = map(presionResinaPurgas, 0, 65, 0, 255);
            analogWrite(itvResina, presionRPMap);
            Serial2.print("tsw bt0,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bt1,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bt3,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bPr,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bR,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bI,0");
            Serial2.print("\xFF\xFF\xFF");
            bP3Iniciar.setValue(1);
            bP3Iniciar.setFont(0);
            bP3Iniciar.setText("Terminar");
            mandarDatosSerial(3, 1);
            estadoPurga = 1;
            purgaSeleccionada = 1;
            varPurgas.setValue(1);
            }
          }
        else{
          if(estadoPurga == 1){
            terminarTodasOperaciones();
            Serial2.print("tsw bt1,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bPr,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bR,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bI,1");
            Serial2.print("\xFF\xFF\xFF");
            bP3Iniciar.setFont(3);
            bP3Iniciar.setText("Iniciar");
            bP3Iniciar.setValue(0);
            bP3Aire.setValue(0);
            estadoAirePurgas = 0;
            estadoPurga = 0;
            varPurgas.setValue(0);
            mandarDatosSerial(3, 0);
            }
          }
        break;
      case 4:
        if(valor == 1){
          if(estadoPurga == 0){                 
            presionHPMap = map(presionHardenerPurgas, 0, 65, 0, 255);
            analogWrite(itvHardener, presionHPMap);
            Serial2.print("tsw bt0,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bt1,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bt3,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bPr,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bR,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bI,0");
            Serial2.print("\xFF\xFF\xFF");
            bP3Iniciar.setValue(1);
            bP3Iniciar.setFont(0);
            bP3Iniciar.setText("Terminar");
            mandarDatosSerial(4, 1);
            estadoPurga = 1;
            purgaSeleccionada = 2;
            varPurgas.setValue(1);
            }
          }
        else{
          if(estadoPurga == 1){
            terminarTodasOperaciones();
            Serial2.print("tsw bt0,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bPr,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bR,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bI,1");
            Serial2.print("\xFF\xFF\xFF");
            bP3Iniciar.setFont(3);
            bP3Iniciar.setText("Iniciar");
            bP3Iniciar.setValue(0);
            bP3Aire.setValue(0);
            estadoAirePurgas = 0;
            estadoPurga = 0;
            varPurgas.setValue(0);
            mandarDatosSerial(4, 0);
            }
          }
        break;
      case 5:
        if(valor == 0){
          if(estadoPausaInyeccion == 0){
            bP2Pausa.setValue(1);
            estadoPausaInyeccion = 1;
            analogWrite(itvResina, 0);
            analogWrite(itvHardener, 0);
            mandarDatosSerial(2, 0);
            }
          }
        else{
          if(estadoPausaInyeccion == 1){
            bP2Pausa.setValue(0);
            estadoPausaInyeccion = 0;
            presionRIMap = map(presionResinaInyeccion, 0, 65, 0, 255);
            presionHIMap = map(presionHardenerInyeccion, 0, 65, 0, 255);
            analogWrite(itvResina, presionRIMap);
            analogWrite(itvHardener, presionHIMap);
            mandarDatosSerial(2, 1);
            }
          }
        break;
      case 6:
        if(valor == 1){
          if(estadoAirePurgas == 0){
            estadoAirePurgas = 1;
            bP3Aire.setValue(1);
            analogWrite(itvAire, 200);
            }
          }
        else{
          if(estadoAirePurgas == 1){
            estadoAirePurgas = 0;
            bP3Aire.setValue(0);
            analogWrite(itvAire, 0);
            }
          }
        break;
      case 7:
        ECS = 1;
        presionMinima = valor;
        rotaryEncoder.setEncoderValue(presionMinima);
        preferences.putFloat("presionMinima", presionMinima);
        tP0PresionMinima.setText(dtostrf(presionMinima/10, 2, 1, result));
        break;
      case 9:
        if(paginaSeleccionada == 0){
          Serial2.print("t8.bco=33800");
          Serial2.print("\xFF\xFF\xFF");
          presionMinimaSeleccionada = 1;
          }
        break;
      case 10:
        ECS = 1;
        presionResinaRecirculacion = valor;
        rotaryEncoder.setEncoderValue(presionResinaRecirculacion);
        preferences.putFloat("resinaR", presionResinaRecirculacion);
        tP1Resina.setText(dtostrf(presionResinaRecirculacion/10, 2, 1, result));
        break;
      case 11:
        ECS = 1;
        presionResinaRecirculacion = valor;
        rotaryEncoder.setEncoderValue(presionResinaRecirculacion);
        preferences.putFloat("resinaR", presionResinaRecirculacion);
        tP1Resina.setText(dtostrf(presionResinaRecirculacion/10, 2, 1, result));
        presionRRMap = map(presionResinaRecirculacion, 0, 65, 0, 255);
        if(estadoRecirculacion == 1){
          analogWrite(itvResina, presionRRMap);
        }
        break;
      case 12:
        ECS = 1;
        presionHardenerRecirculacion = valor;
        rotaryEncoder.setEncoderValue(presionHardenerRecirculacion);
        preferences.putFloat("hardenerR", presionHardenerRecirculacion);
        tP1Hardener.setText(dtostrf(presionHardenerRecirculacion/10, 2, 1, result));
        break;
      case 13:
        ECS = 1;
        presionHardenerRecirculacion = valor;
        rotaryEncoder.setEncoderValue(presionHardenerRecirculacion);
        preferences.putFloat("hardenerR", presionHardenerRecirculacion);
        tP1Hardener.setText(dtostrf(presionHardenerRecirculacion/10, 2, 1, result));
        presionHRMap = map(presionHardenerRecirculacion, 0, 65, 0, 255);
        if(estadoRecirculacion == 1){
          analogWrite(itvHardener, presionHRMap);
        }
        break;
      case 14:
        ECS = 1;
        presionResinaInyeccion = valor;
        rotaryEncoder.setEncoderValue(presionResinaInyeccion);
        preferences.putFloat("resinaI", presionResinaInyeccion);
        tP2Resina.setText(dtostrf(presionResinaInyeccion/10, 2, 1, result));
        break;
      case 15:
        ECS = 1;
        presionResinaInyeccion = valor;
        rotaryEncoder.setEncoderValue(presionResinaInyeccion);
        preferences.putFloat("resinaI", presionResinaInyeccion);
        tP2Resina.setText(dtostrf(presionResinaInyeccion/10, 2, 1, result));
        presionRIMap = map(presionResinaInyeccion, 0, 65, 0, 255);
        if(estadoInyeccion == 1){
          analogWrite(itvResina, presionRIMap);
        }
        break;
      case 16:
        ECS = 1;
        presionHardenerInyeccion = valor;
        rotaryEncoder.setEncoderValue(presionHardenerInyeccion);
        preferences.putFloat("hardenerI", presionHardenerInyeccion);
        tP2Hardener.setText(dtostrf(presionHardenerInyeccion/10, 2, 1, result));
        break;
      case 17:
        ECS = 1;
        presionHardenerInyeccion = valor;
        rotaryEncoder.setEncoderValue(presionHardenerInyeccion);
        preferences.putFloat("hardenerI", presionHardenerInyeccion);
        tP2Hardener.setText(dtostrf(presionHardenerInyeccion/10, 2, 1, result));
        presionHIMap = map(presionHardenerInyeccion, 0, 65, 0, 255);
        if(estadoInyeccion == 1){
          analogWrite(itvHardener, presionHIMap);
        }
        break;
      case 18:
        ECS = 1;
        presionAireInyeccion = valor;
        rotaryEncoder.setEncoderValue(presionAireInyeccion);
        preferences.putFloat("aireI", presionAireInyeccion);
        tP2Aire.setText(dtostrf((presionAireInyeccion/10)*1.3, 2, 1, result));
        break;
      case 19:
        ECS = 1;
        presionAireInyeccion = valor;
        rotaryEncoder.setEncoderValue(presionAireInyeccion);
        preferences.putFloat("aireI", presionAireInyeccion);
        tP2Aire.setText(dtostrf((presionAireInyeccion/10)*1.3, 2, 1, result));
        presionAIMap = map(presionAireInyeccion, 0, 65, 0, 255);
        if(estadoInyeccion == 1){
          analogWrite(itvAire, presionAIMap);
        }
        break;
      case 20:
        ECS = 1;
        presionResinaPurgas = valor;
        rotaryEncoder.setEncoderValue(presionResinaPurgas);
        preferences.putFloat("resinaP", presionResinaPurgas);
        tP3Resina.setText(dtostrf(presionResinaPurgas/10, 2, 1, result));
        break;
      case 21:
        ECS = 1;
        presionResinaPurgas = valor;
        rotaryEncoder.setEncoderValue(presionResinaPurgas);
        preferences.putFloat("resinaP", presionResinaPurgas);
        tP3Resina.setText(dtostrf(presionResinaPurgas/10, 2, 1, result));
        presionRPMap = map(presionResinaPurgas, 0, 65, 0, 255);
        if(estadoPurga == 1 && purgaSeleccionada == 1){
          analogWrite(itvResina, presionRPMap);
        }
        break;
      case 22:
        ECS = 1;
        presionHardenerPurgas = valor;
        rotaryEncoder.setEncoderValue(presionHardenerPurgas);
        preferences.putFloat("hardenerP", presionHardenerPurgas);
        tP3Hardener.setText(dtostrf(presionHardenerPurgas/10, 2, 1, result));
        break;
      case 23:
        ECS = 1;
        presionHardenerPurgas = valor;
        rotaryEncoder.setEncoderValue(presionHardenerPurgas);
        preferences.putFloat("hardenerP", presionHardenerPurgas);
        tP3Hardener.setText(dtostrf(presionHardenerPurgas/10, 2, 1, result));
        presionHPMap = map(presionHardenerPurgas, 0, 65, 0, 255);
        if(estadoPurga == 1 && purgaSeleccionada == 2){
          analogWrite(itvHardener, presionHPMap);
        }
        break;
      case 24:
        paginaSeleccionada = 0;
        presionMinimaSeleccionada = 0;
        tP0PresionMinima.setText(dtostrf(presionMinima/10, 2, 1, result));
        variableSeleccionadaRecirculacion = 0;
        variableSeleccionadaInyeccion = 0;
        variableSeleccionadaPurgas = 0;
        tiempoRecirculacion = 5;
        pagPrincipal.show();
        break;
      case 25:
        paginaSeleccionada = 1;
        presionMinimaSeleccionada = 0;
        variableSeleccionadaRecirculacion = 0;
        variableSeleccionadaInyeccion = 0;
        variableSeleccionadaPurgas = 0;
        pagRecirculacion.show();
        tP1Resina.setText(dtostrf(presionResinaRecirculacion/10, 2, 1, result));
        tP1Hardener.setText(dtostrf(presionHardenerRecirculacion/10, 2, 1, result));
        tP1Tiempo.setText(dtostrf(tiempoRecirculacion, 2, 0, result));
        rotaryEncoder.setEncoderValue(presionResinaRecirculacion);
        tiempoRecirculacion = 5;
        break;
      case 26:
        paginaSeleccionada = 2;
        presionMinimaSeleccionada = 0;
        variableSeleccionadaRecirculacion = 0;
        variableSeleccionadaInyeccion = 0;
        variableSeleccionadaPurgas = 0;
        pagInyeccion.show();
        tP2Resina.setText(dtostrf(presionResinaInyeccion/10, 2, 1, result));
        tP2Hardener.setText(dtostrf(presionHardenerInyeccion/10, 2, 1, result));
        tP2Aire.setText(dtostrf((presionAireInyeccion/10)*1.3, 2, 1, result));
        rotaryEncoder.setEncoderValue(presionResinaInyeccion);
        tiempoRecirculacion = 5;
        break;
      case 27:
        paginaSeleccionada = 3;
        presionMinimaSeleccionada = 0;
        variableSeleccionadaRecirculacion = 0;
        variableSeleccionadaInyeccion = 0;
        variableSeleccionadaPurgas = 0;
        pagPurgas.show();
        tP3Resina.setText(dtostrf(presionResinaPurgas/10, 2, 1, result));
        tP3Hardener.setText(dtostrf(presionHardenerPurgas/10, 2, 1, result));
        rotaryEncoder.setEncoderValue(presionResinaPurgas);
        tiempoRecirculacion = 5;
        break;
      case 28:
        variableSeleccionadaRecirculacion = 0;
        rotaryEncoder.setEncoderValue(presionResinaRecirculacion);
        tP1ResinaSelect.Set_background_color_bco(33800);
        tP1HardenerSelect.Set_background_color_bco(17424);
        tP1TiempoSelect.Set_background_color_bco(17424);
        break;
      case 29:
        variableSeleccionadaRecirculacion = 1;
        rotaryEncoder.setEncoderValue(presionHardenerRecirculacion);
        tP1ResinaSelect.Set_background_color_bco(17424);
        tP1HardenerSelect.Set_background_color_bco(33800);
        tP1TiempoSelect.Set_background_color_bco(17424);
        break;
      case 30:
        variableSeleccionadaRecirculacion = 2;
        rotaryEncoder.setEncoderValue(tiempoRecirculacion);
        tP1ResinaSelect.Set_background_color_bco(17424);
        tP1HardenerSelect.Set_background_color_bco(17424);
        tP1TiempoSelect.Set_background_color_bco(33800);
        break;
      case 31:
        variableSeleccionadaInyeccion = 0;
        rotaryEncoder.setEncoderValue(presionResinaInyeccion);
        tP2ResinaSelect.Set_background_color_bco(33800);
        tP2HardenerSelect.Set_background_color_bco(17424);
        tP2AireSelect.Set_background_color_bco(17424);
        break;
      case 32:
        variableSeleccionadaInyeccion = 1;
        rotaryEncoder.setEncoderValue(presionHardenerInyeccion);
        tP2ResinaSelect.Set_background_color_bco(17424);
        tP2HardenerSelect.Set_background_color_bco(33800);
        tP2AireSelect.Set_background_color_bco(17424);
        break;
      case 33:
        variableSeleccionadaInyeccion = 2;
        rotaryEncoder.setEncoderValue(presionAireInyeccion);
        tP2ResinaSelect.Set_background_color_bco(17424);
        tP2HardenerSelect.Set_background_color_bco(17424);
        tP2AireSelect.Set_background_color_bco(33800);
        break;
      case 34:
        variableSeleccionadaPurgas = 0;
        rotaryEncoder.setEncoderValue(presionResinaPurgas);
        tP3ResinaSelect.Set_background_color_bco(33800);
        tP3HardenerSelect.Set_background_color_bco(17424);
        break;
      case 35:
        variableSeleccionadaPurgas = 1;
        rotaryEncoder.setEncoderValue(presionHardenerPurgas);
        tP3ResinaSelect.Set_background_color_bco(17424);
        tP3HardenerSelect.Set_background_color_bco(33800);
        break;
      case 36:
        purgaSeleccionada = 1;
        bP3Resina.setValue(1);
        bP3Hardener.setValue(0);
        varSelectP.setValue(1);
        Serial2.print("tsw bt0,0");
        Serial2.print("\xFF\xFF\xFF");
        Serial2.print("tsw bt1,1");
        Serial2.print("\xFF\xFF\xFF");
        break;
      case 37:
        purgaSeleccionada = 2;
        bP3Resina.setValue(0);
        bP3Hardener.setValue(1);
        varSelectP.setValue(2);
        Serial2.print("tsw bt0,1");
        Serial2.print("\xFF\xFF\xFF");
        Serial2.print("tsw bt1,0");
        Serial2.print("\xFF\xFF\xFF");
        break;
      case 45:
        ECS = 1;
        tiempoRecirculacion = valor;
        tP1Tiempo.setText(dtostrf(tiempoRecirculacion, 2, 0, result));
        if(variableSeleccionadaRecirculacion == 2){
          rotaryEncoder.setEncoderValue(tiempoRecirculacion);
          }
        break;                          
    }
  
}



void setup() {

  mensajeESPNOWsaliente.id = 2;

  pinMode(pinLed, OUTPUT);
  
  pinMode(pinSensorTemp1, INPUT);  
  pinMode(pinSensorTemp2, INPUT);
  pinMode(pinSensorPresion1, INPUT);
  pinMode(pinSensorPresion2, INPUT);
  pinMode(pinSensorNivel1, INPUT_PULLUP);
  pinMode(pinSensorNivel2, INPUT_PULLUP);
  pinMode(pinSensorFlujoHardener, INPUT_PULLUP);
  pinMode(pinSensorFlujoResina, INPUT_PULLUP);
  
  pinMode(itvResina, OUTPUT);
  pinMode(itvHardener, OUTPUT);
  pinMode(itvAire, OUTPUT);

  Serial.begin(115200);
  Serial2.begin(115200);

  attachInterrupt(digitalPinToInterrupt(pinSensorFlujoHardener), pulseCounterHardener, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinSensorFlujoResina), pulseCounterResina, FALLING);
  
  nexInit();
  
  WiFi.mode(WIFI_STA);

  esp_err_t resultLRProtocol = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
         
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);

  bP1Iniciar.attachPop(recirculacion, &bP1Iniciar);
  bP2Iniciar.attachPop(inyeccion, &bP2Iniciar);
  bP3Iniciar.attachPop(purga, &bP3Iniciar);
  
  bP0Recirculacion.attachPush(botonPaginaRecirculacion, &bP0Recirculacion);
  bP0Inyeccion.attachPush(botonPaginaInyeccion, &bP0Inyeccion);
  bP0Purgas.attachPush(botonPaginaPurgas, &bP0Purgas);
  tP0PresionMinimaSelect.attachPush(principalPresionMinimaSelect, &tP0PresionMinimaSelect);
  
  bP1Principal.attachPush(botonPaginaPrincipal, &bP1Principal);
  bP1Inyeccion.attachPush(botonPaginaInyeccion, &bP1Inyeccion);
  bP1Purgas.attachPush(botonPaginaPurgas, &bP1Purgas);
  tP1TiempoSelect.attachPush(recirculacionTiempoSelect, &tP1TiempoSelect);
  tP1ResinaSelect.attachPush(recirculacionResinaSelect, &tP1ResinaSelect);
  tP1HardenerSelect.attachPush(recirculacionHardenerSelect, &tP1ResinaSelect);
  
  bP2Principal.attachPush(botonPaginaPrincipal, &bP2Principal);
  bP2Recirculacion.attachPush(botonPaginaRecirculacion, &bP2Recirculacion);
  bP2Purgas.attachPush(botonPaginaPurgas, &bP2Purgas);
  bP2Pausa.attachPop(pausaInyeccion, &bP2Pausa);
  tP2ResinaSelect.attachPush(inyeccionResinaSelect, &tP2ResinaSelect);
  tP2HardenerSelect.attachPush(inyeccionHardenerSelect, &tP2HardenerSelect);
  tP2AireSelect.attachPush(inyeccionAireSelect, &tP2AireSelect);
  
  bP3Principal.attachPush(botonPaginaPrincipal, &bP3Principal);
  bP3Recirculacion.attachPush(botonPaginaRecirculacion, &bP3Recirculacion);
  bP3Inyeccion.attachPush(botonPaginaInyeccion, &bP3Inyeccion);

  bP3Resina.attachPop(purgaResinaSelect, &bP3Resina);
  bP3Hardener.attachPop(purgaHardenerSelect, &bP3Hardener);
  bP3Aire.attachPop(airePurgas, &bP3Aire);

  tP3ResinaSelect.attachPush(purgasResinaSelect, &tP3ResinaSelect);
  tP3HardenerSelect.attachPush(purgasHardenerSelect, &tP3HardenerSelect);

  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);

  bool circleValues = false;
  rotaryEncoder.setBoundaries(0, 65, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  rotaryEncoder.disableAcceleration();

  preferences.begin("presionesX", false);

  presionResinaRecirculacion = preferences.getFloat("resinaR", 0);
  presionHardenerRecirculacion = preferences.getFloat("hardenerR", 0);
  presionResinaInyeccion = preferences.getFloat("resinaI", 0);
  presionHardenerInyeccion = preferences.getFloat("hardenerI", 0);
  presionAireInyeccion = preferences.getFloat("aireI", 0);
  presionAireInyeccionAplicada = preferences.getFloat("aireAplicadaI", 0);
  presionResinaPurgas = preferences.getFloat("resinaP", 0);
  presionHardenerPurgas = preferences.getFloat("hardenerP", 0);
  presionMinima = preferences.getFloat("presionMinima", 0);
  
}

void loop() {

  digitalWrite(pinLed, LOW);
  
  ECS = 0;          

//LECTURA Y MUESTRA DE SENSORES//
  if(paginaSeleccionada == 0){      
      if(sensorCount < 1000){
        lecturaTemperatura1 += analogRead(pinSensorTemp1);
        lecturaTemperatura2 += analogRead(pinSensorTemp2);
        sensorCount += 1;
        }
      if(sensorCount == 1000){
        tP0PresionMinima.setText(dtostrf(presionMinima/10, 2, 1, result));
        mensajeESPNOWsaliente.operacion = 7;
        mensajeESPNOWsaliente.valor = presionMinima;
        mandarDatosESPNOW();
        valorSensorNivel1 = digitalRead(pinSensorNivel1);
        valorSensorNivel2 = digitalRead(pinSensorNivel2);
        if(valorSensorNivel1 == 0){
          tP0NResina.setText("X");
          mensajeESPNOWsaliente.operacion = 42;
          mensajeESPNOWsaliente.valor = 1;
          mandarDatosESPNOW();
          Serial2.print("t6.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP0NResina.setText("N");
          mensajeESPNOWsaliente.operacion = 42;
          mensajeESPNOWsaliente.valor = 0;
          mandarDatosESPNOW();
          Serial2.print("t6.pco=1024");
          Serial2.print("\xFF\xFF\xFF");
          }
        if(valorSensorNivel2 == 0){
          tP0NHardener.setText("X");
          mensajeESPNOWsaliente.operacion = 43;
          mensajeESPNOWsaliente.valor = 1;
          mandarDatosESPNOW();
          Serial2.print("t7.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP0NHardener.setText("N");
          mensajeESPNOWsaliente.operacion = 43;
          mensajeESPNOWsaliente.valor = 0;
          mandarDatosESPNOW();
          Serial2.print("t7.pco=1024");
          Serial2.print("\xFF\xFF\xFF");
          }  
        promedioSensorTemperatura1 = lecturaTemperatura1/1000;
        promedioSensorTemperatura2 = lecturaTemperatura2/1000;
        valorSensorTemperatura1 = map(promedioSensorTemperatura1, 0, 4095, -40, 140);
        valorSensorTemperatura2 = map(promedioSensorTemperatura2, 0, 4095, -40, 140);
        tP0Temperatura1.setText(dtostrf(valorSensorTemperatura1, 2, 0, result));
        tP0Temperatura2.setText(dtostrf(valorSensorTemperatura2, 2, 0, result));

        mensajeESPNOWsaliente.operacion = 38;
        mensajeESPNOWsaliente.valor = valorSensorTemperatura1;
        mandarDatosESPNOW();
        //POSIBLE PROBLEMA
        mensajeESPNOWsaliente.operacion = 39;
        mensajeESPNOWsaliente.valor = valorSensorTemperatura2;
        mandarDatosESPNOW();
        
        lecturaTemperatura1 = 0;
        lecturaTemperatura2 = 0;
        promedioSensorTemperatura1 = 0;
        promedioSensorTemperatura2 = 0;
        sensorCount = 0;
        }     
    }

  if(paginaSeleccionada == 1){
      if(sensorCount < 1000){
        lecturaPresion1 += analogRead(pinSensorPresion1);
        lecturaPresion2 += analogRead(pinSensorPresion2);
        sensorCount += 1;
        }
      if(sensorCount == 1000){
        promedioSensorPresion1 = lecturaPresion1/1000;
        promedioSensorPresion2 = lecturaPresion2/1000;
        valorSensorPresion1 = map(promedioSensorPresion1, 660, 4095, 0, 115);
        valorSensorPresion2 = map(promedioSensorPresion2, 660, 4095, 0, 115);
        tP1Presion1.setText(dtostrf(valorSensorPresion1/10, 2, 1, result));
        tP1Presion2.setText(dtostrf(valorSensorPresion2/10, 2, 1, result));

        mensajeESPNOWsaliente.operacion = 40;
        mensajeESPNOWsaliente.valor = valorSensorPresion1;
        mandarDatosESPNOW();       
        //POSIBLE PROBLEMA
        mensajeESPNOWsaliente.operacion = 41;
        mensajeESPNOWsaliente.valor = valorSensorPresion2;
        mandarDatosESPNOW();
        
        sensorCount = 0;
        lecturaPresion1 = 0;
        promedioSensorPresion1 = 0;
        lecturaPresion2 = 0;
        promedioSensorPresion2 = 0;
        }      
    }

  if(paginaSeleccionada == 2){    
    if(sensorCount < 1000){
      lecturaTemperatura1 += analogRead(pinSensorTemp1);
      lecturaTemperatura2 += analogRead(pinSensorTemp2);
      lecturaPresion1 += analogRead(pinSensorPresion1);
      lecturaPresion2 += analogRead(pinSensorPresion2);
      sensorCount += 1;
      }
    if(sensorCount == 1000){
        valorSensorNivel1 = digitalRead(pinSensorNivel1);
        valorSensorNivel2 = digitalRead(pinSensorNivel2);
        if(valorSensorNivel1 == 0){
          tP2NResina.setText("X");
          mensajeESPNOWsaliente.operacion = 42;
          mensajeESPNOWsaliente.valor = 1;
          mandarDatosESPNOW();
          Serial2.print("t15.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP2NResina.setText("N");
          mensajeESPNOWsaliente.operacion = 42;
          mensajeESPNOWsaliente.valor = 0;
          mandarDatosESPNOW();
          Serial2.print("t15.pco=1024");
          Serial2.print("\xFF\xFF\xFF");
          }
        if(valorSensorNivel2 == 0){
          tP2NHardener.setText("X");
          mensajeESPNOWsaliente.operacion = 43;
          mensajeESPNOWsaliente.valor = 1;
          mandarDatosESPNOW();
          Serial2.print("t16.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP2NHardener.setText("N");
          mensajeESPNOWsaliente.operacion = 43;
          mensajeESPNOWsaliente.valor = 0;
          mandarDatosESPNOW();
          Serial2.print("t16.pco=1024");
          Serial2.print("\xFF\xFF\xFF");
          }
      promedioSensorTemperatura1 = lecturaTemperatura1/1000;
      promedioSensorTemperatura2 = lecturaTemperatura2/1000;
      promedioSensorPresion1 = lecturaPresion1/1000;
      promedioSensorPresion2 = lecturaPresion2/1000;
      valorSensorTemperatura1 = map(promedioSensorTemperatura1, 0, 4095, -40, 140);
      valorSensorTemperatura2 = map(promedioSensorTemperatura2, 0, 4095, -40, 140);
      valorSensorPresion1 = map(promedioSensorPresion1, 660, 3800, 0, 108);
      valorSensorPresion2 = map(promedioSensorPresion2, 660, 3800, 0, 108);
      tP2Temperatura1.setText(dtostrf(valorSensorTemperatura1, 2, 0, result));
      tP2Temperatura2.setText(dtostrf(valorSensorTemperatura2, 2, 0, result));

      mensajeESPNOWsaliente.operacion = 38;
      mensajeESPNOWsaliente.valor = valorSensorTemperatura1;
      mandarDatosESPNOW();
      //POSIBLE PROBLEMA
      mensajeESPNOWsaliente.operacion = 39;
      mensajeESPNOWsaliente.valor = valorSensorTemperatura2;
      mandarDatosESPNOW();
      
      sensorCount = 0;
      lecturaTemperatura1 = 0;
      lecturaTemperatura2 = 0;
      lecturaPresion1 = 0;
      lecturaPresion2 = 0;
      promedioSensorTemperatura1 = 0;
      promedioSensorTemperatura2 = 0;
      promedioSensorPresion1 = 0;
      promedioSensorPresion2 = 0;
      //
      
      if(estadoInyeccion == 1){
        
        //COMPARACION FLUJOS INICIO//
        milisActuales = millis();
        if(milisActuales - milisPrevios > intervaloFlujo){
          pulsoHardener1seg = conteoPulsoHardener;
          pulsoResina1seg = conteoPulsoResina;

          conteoPulsoHardener = 0;
          conteoPulsoResina = 0;

          rateFlujoHardener = ((1000.0 / (millis() - milisPrevios)) * pulsoHardener1seg) / FactorDeCalibracion;
          rateFlujoResina = ((1000.0 / (millis() - milisPrevios)) * pulsoResina1seg) / FactorDeCalibracion;
          milisPrevios = millis();

          flujoMililitrosHardener = (rateFlujoHardener/60) * 1000;
          flujoMililitrosResina = (rateFlujoResina/60) * 1000;

          totalMililitrosHardener += flujoMililitrosHardener;
          totalMililitrosResina += flujoMililitrosResina;

          diferenciaFlujos = abs(flujoMililitrosHardener - flujoMililitrosResina);
          
          Serial.print("Diferencia de flujos = ");
          Serial.println(diferenciaFlujos);

          // Print the flow rate for this second in litres / minute
          Serial.print("Flujo Hardener: ");
          Serial.print(int(rateFlujoHardener));  // Print the integer part of the variable
          Serial.print("L/min");
          Serial.print("\t");       // Print tab space

          Serial.print("Flujo Resina: ");
          Serial.print(int(rateFlujoResina));  // Print the integer part of the variable
          Serial.print("L/min");
          Serial.print("\t");
      
          // Print the cumulative total of litres flowed since starting
          Serial.print("Output Liquid Quantity: ");
          Serial.print(totalMililitrosHardener);
          Serial.print("mL / ");
          Serial.print(totalMililitrosHardener / 1000);
          Serial.println("L");

          Serial.print("Output Liquid Quantity: ");
          Serial.print(totalMililitrosResina);
          Serial.print("mL / ");
          Serial.print(totalMililitrosResina / 1000);
          Serial.println("L");
          
          }
        //COMPARACION FLUJOS FINAL//
                  
        if(bajaPresion == 1){
          if(millis()>inicioBajaPresion+10000){
            Serial.println("DIE ENDE");
            bajaPresion = 0;
            inicioBajaPresion = 0;

            mensajeESPNOWsaliente.operacion = 2;
            mensajeESPNOWsaliente.valor = 0;
            mandarDatosESPNOW();
            //POSIBLE PROBLEMA
            mensajeESPNOWsaliente.operacion = 8;
            mensajeESPNOWsaliente.valor = 0;
            mandarDatosESPNOW();
            
            mandarDatosSerial(2, 0);
            mandarDatosSerial(5, 0);
            varInyeccion.setValue(0);
            bP2Iniciar.setValue(0);
            bP2Iniciar.setFont(3);
            bP2Iniciar.setText("Iniciar");
            bP2Iniciar.setValue(0);
            bP2Iniciar.setFont(3);
            bP2Iniciar.setText("Iniciar");
            Serial2.print("tsw bt1,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bPr,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bR,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bP,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("vis t0,0");
            Serial2.print("\xFF\xFF\xFF");
            bP2Pausa.setValue(0);
            terminarTodasOperaciones();
            estadoInyeccion = 0;
            estadoPausaInyeccion = 0;
            mandarDatosSerial(2, 0);
            return;
            }
          }
        if(valorSensorPresion1 < presionMinima && bajaPresion == 0){
          bajaPresion = 1;
          inicioBajaPresion = millis();
          mandarDatosSerial(5, 1);
          mensajeESPNOWsaliente.operacion = 8;
          mensajeESPNOWsaliente.valor = 1;
          mandarDatosESPNOW();
          Serial2.print("vis t0,1");
          Serial2.print("\xFF\xFF\xFF");
          }
        }
      }    
    }
    
//CALCULAR Y MOSTRAR TIEMPO RECIRCULACION//    
  if(estadoRecirculacion == 1){
    if(millis() > ultimoSegundo+1000){
      ultimoSegundo = millis();
      segundosRecirculacion += 1;
      if(segundosRecirculacion == 60){
        tiempoRecirculacion -= 1;
        tP1Tiempo.setText(dtostrf(tiempoRecirculacion, 2, 0, result));
        mensajeESPNOWsaliente.operacion = 45;
        mensajeESPNOWsaliente.valor = tiempoRecirculacion;
        mandarDatosESPNOW();
        segundosRecirculacion = 0;
        } 
      }
    if(tiempoRecirculacion == 0){
      mensajeESPNOWsaliente.operacion = 1;
      mensajeESPNOWsaliente.valor = 0;
      mandarDatosESPNOW();
      varRecirculacion.setValue(0);
      Serial2.print("tsw bPr,1");
      Serial2.print("\xFF\xFF\xFF");
      Serial2.print("tsw bI,1");
      Serial2.print("\xFF\xFF\xFF");
      Serial2.print("tsw bP,1");
      Serial2.print("\xFF\xFF\xFF");
      Serial2.print("t8.pco=0");
      Serial2.print("\xFF\xFF\xFF");
      bP1Iniciar.setValue(0);
      bP1Iniciar.setText("Iniciar");
      terminarTodasOperaciones();
      estadoRecirculacion = 0;
      }
    }
      
  nexLoop(nex_listen_list);
  rotary_loop();
  
  received = false;
  
}
