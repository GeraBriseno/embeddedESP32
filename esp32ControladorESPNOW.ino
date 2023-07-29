#include "Nextion.h"
#include "AiEsp32RotaryEncoder.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>

Preferences preferences;

//VARIABLES AUXILIARES ENCODER//
int ECS = 0;
float valorFloat = 0;

//VARIABLES AUXILIARES PARA LOS BOTONES//
int estadoBoton1;
int estadoBoton2;

int ultimoEstadoBoton1 = LOW;
int ultimoEstadoBoton2 = LOW;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

//PINES BOTONES, ENCODER Y BUZZER//
const int pinBoton1 = 33;
const int pinBoton2 = 32;

const int pinBuzzer = 4;

#define ROTARY_ENCODER_A_PIN 12
#define ROTARY_ENCODER_B_PIN 14
#define ROTARY_ENCODER_BUTTON_PIN 25
#define ROTARY_ENCODER_VCC_PIN 27

#define ROTARY_ENCODER_STEPS 4

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

//VARIABLE AUXILIAR PARA MOSTRAR DATOS EN LA PANTALLA NEXTION//
char result[8];

//VARIABLES PARA CONTROLAR OPERACION Y ESTADO//
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
float presionResinaPurgas = 0;
float presionHardenerPurgas = 0;

int presionRRMap = 0;
int presionHRMap = 0;
int presionRIMap = 0;
int presionHIMap = 0;
int presionAIMap = 0;
int presionRPMap = 0;
int presionHPMap = 0;

//OBJETOS PANTALLA NEXTION//
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

//FUNCIONES//
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

//FUNCION PRINCIPAL ENCODER//
void rotary_loop()
{
  //dont print anything unless value changed
  if (rotaryEncoder.encoderChanged())
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
            if(ECS == 0){
              mensajeESPNOWsaliente.operacion = 10;
              mensajeESPNOWsaliente.valor = presionResinaRecirculacion;
              mandarDatosESPNOW();
              }
            else{
              ECS = 0;
              }
            //mandarDatosEthernet(10, presionResinaRecirculacion);
            if(estadoRecirculacion == 1){
              mensajeESPNOWsaliente.operacion = 11;
              mensajeESPNOWsaliente.valor = presionResinaRecirculacion;
              mandarDatosESPNOW();
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
            //mandarDatosEthernet(12, presionHardenerRecirculacion);
            if(estadoRecirculacion == 1){
              mensajeESPNOWsaliente.operacion = 13;
              mensajeESPNOWsaliente.valor = presionHardenerRecirculacion;
              mandarDatosESPNOW();
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
                mensajeESPNOWsaliente.operacion = 15;
                mensajeESPNOWsaliente.valor = presionResinaInyeccion;
                mandarDatosESPNOW();
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
                mensajeESPNOWsaliente.operacion = 17;
                mensajeESPNOWsaliente.valor = presionHardenerInyeccion;
                mandarDatosESPNOW();
              }
              break;
            case 2:
              presionAireInyeccion = rotaryEncoder.readEncoder();
              preferences.putFloat("aireI", presionAireInyeccion);
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
                mensajeESPNOWsaliente.operacion = 19;
                mensajeESPNOWsaliente.valor = presionAireInyeccion;
                mandarDatosESPNOW();
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
                  mensajeESPNOWsaliente.operacion = 21;
                  mensajeESPNOWsaliente.valor = presionResinaPurgas;
                  mandarDatosESPNOW();
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
                  mensajeESPNOWsaliente.operacion = 23;
                  mensajeESPNOWsaliente.valor = presionHardenerPurgas;
                  mandarDatosESPNOW();
                }
                break;
          }
        break;
      }
  }
}

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

//FUNCIONES//
void botonPaginaPrincipal(void *ptr){
  paginaSeleccionada = 0;
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
    estadoRecirculacion = 1;
    Serial2.print("t8.pco=1024");
    Serial2.print("\xFF\xFF\xFF");
    mensajeESPNOWsaliente.operacion = 1;
    mensajeESPNOWsaliente.valor = 1;
    mandarDatosESPNOW();    
    }
  else if(estadoRecirculacion == 1){
    estadoRecirculacion = 0;
    Serial2.print("t8.pco=0");
    Serial2.print("\xFF\xFF\xFF");
    mensajeESPNOWsaliente.operacion = 1;
    mensajeESPNOWsaliente.valor = 0;
    mandarDatosESPNOW(); 
    }        
}

void inyeccion(void *ptr){
  if(estadoInyeccion == 0){
    estadoInyeccion = 1;
    bP2Iniciar.setFont(0);
    bP2Iniciar.setText("Terminar");
    mensajeESPNOWsaliente.operacion = 2;
    mensajeESPNOWsaliente.valor = 1;
    mandarDatosESPNOW(); 
    }
  else{
    estadoInyeccion = 0;
    bP2Iniciar.setFont(3);
    bP2Iniciar.setText("Iniciar");
    estadoPausaInyeccion = 0;
    mensajeESPNOWsaliente.operacion = 2;
    mensajeESPNOWsaliente.valor = 0;
    mandarDatosESPNOW(); 
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
    digitalWrite(pinBuzzer, LOW);
    }        
}

void purga(void *ptr){
  if(estadoPurga == 0){
    if(purgaSeleccionada == 1){
      mensajeESPNOWsaliente.operacion = 3;
      mensajeESPNOWsaliente.valor = 1;
      mandarDatosESPNOW(); 
      }
    else if(purgaSeleccionada == 2){
      mensajeESPNOWsaliente.operacion = 4;
      mensajeESPNOWsaliente.valor = 1;
      mandarDatosESPNOW(); 
      }
    estadoPurga = 1;
    }
  else{
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
    estadoAirePurgas = 0;
    estadoPurga = 0;
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
    mensajeESPNOWsaliente.operacion = 5;
    mensajeESPNOWsaliente.valor = 0;
    mandarDatosESPNOW(); 
    }
  else{
    estadoPausaInyeccion = 0;
    mensajeESPNOWsaliente.operacion = 5;
    mensajeESPNOWsaliente.valor = 1;
    mandarDatosESPNOW();      
    }
}

void airePurgas(void *ptr){
  if(estadoAirePurgas == 0){
    estadoAirePurgas = 1;
    mensajeESPNOWsaliente.operacion = 6;
    mensajeESPNOWsaliente.valor = 1;
    mandarDatosESPNOW(); 
    }
  else{
    estadoAirePurgas = 0;
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

// Callback when data is received
/*void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&mensajeESPNOWentrante, incomingData, sizeof(mensajeESPNOWentrante));
  received = true;
}*/

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){

    memcpy(&mensajeESPNOWentrante, incomingData, sizeof(mensajeESPNOWentrante));
    
    int operacion = mensajeESPNOWentrante.operacion;
    int valor = mensajeESPNOWentrante.valor;               
    
    switch(operacion){
    case 1:
      if(valor == 1){
        if(estadoRecirculacion == 0){
          varRecirculacion.setValue(1);
          bP1Iniciar.setValue(1);
          bP1Iniciar.setText("Terminar");
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
          estadoRecirculacion = 1;
          }
        }
      else{
        if(estadoRecirculacion == 1){
          varRecirculacion.setValue(0);
          bP1Iniciar.setValue(0);
          bP1Iniciar.setText("Iniciar");
          bP1Iniciar.setValue(0);
          bP1Iniciar.setText("Iniciar");
          Serial2.print("tsw bPr,1");
          Serial2.print("\xFF\xFF\xFF");
          Serial2.print("tsw bI,1");
          Serial2.print("\xFF\xFF\xFF");
          Serial2.print("tsw bP,1");
          Serial2.print("\xFF\xFF\xFF");
          Serial2.print("t8.pco=0");
          Serial2.print("\xFF\xFF\xFF");
          estadoRecirculacion = 0;
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
          estadoInyeccion = 1;
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
          digitalWrite(pinBuzzer, LOW);
          bP2Pausa.setValue(0);
          estadoInyeccion = 0;
          estadoPausaInyeccion = 0;
          }
        }
      break;
    case 3:
      if(valor == 1){
        if(estadoPurga == 0){                
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
          estadoPurga = 1;
          purgaSeleccionada = 1;
          varPurgas.setValue(1);
          }
        }
      else{
        if(estadoPurga == 1){
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
          }
        }
      break;
    case 4:
      if(valor == 1){
        if(estadoPurga == 0){                 
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
          estadoPurga = 1;
          purgaSeleccionada = 2;
          varPurgas.setValue(1);
          }
        }
      else{
        if(estadoPurga == 1){
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
          }
        }
      break;
    case 5:
      if(valor == 0){
        if(estadoPausaInyeccion == 0){
          bP2Pausa.setValue(1);
          estadoPausaInyeccion = 1;
          }
        }
      else{
        if(estadoPausaInyeccion == 1){
          bP2Pausa.setValue(0);
          estadoPausaInyeccion = 0;
          }
        }
      break;
    case 6:
      if(valor == 1){
        if(estadoAirePurgas == 0){
          estadoAirePurgas = 1;
          bP3Aire.setValue(1);
          }
        }
      else{
        if(estadoAirePurgas == 1){
          estadoAirePurgas = 0;
          bP3Aire.setValue(0);
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
    case 8:
      if(valor == 1){
        Serial2.print("vis t0,1");
        Serial2.print("\xFF\xFF\xFF");
        digitalWrite(pinBuzzer, HIGH);
        }
      else{
        Serial2.print("vis t0,0");
        Serial2.print("\xFF\xFF\xFF");
        digitalWrite(pinBuzzer, LOW);
        }
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
    /*case 11:
      presionResinaRecirculacion = valor;
      rotaryEncoder.setEncoderValue(presionResinaRecirculacion);
      preferences.putFloat("resinaR", presionResinaRecirculacion);
      tP1Resina.setText(dtostrf(presionResinaRecirculacion/10, 2, 1, result));
      break;
    */
    case 12:
      ECS = 1;
      presionHardenerRecirculacion = valor;
      rotaryEncoder.setEncoderValue(presionHardenerRecirculacion);
      preferences.putFloat("hardenerR", presionHardenerRecirculacion);
      tP1Hardener.setText(dtostrf(presionHardenerRecirculacion/10, 2, 1, result));
      break;
    /*case 13:
      presionHardenerRecirculacion = valor;
      rotaryEncoder.setEncoderValue(presionHardenerRecirculacion);
      preferences.putFloat("hardenerR", presionHardenerRecirculacion);
      tP1Hardener.setText(dtostrf(presionHardenerRecirculacion/10, 2, 1, result));
      break;
    */
    case 14:
      presionResinaInyeccion = valor;
      rotaryEncoder.setEncoderValue(presionResinaInyeccion);
      preferences.putFloat("resinaI", presionResinaInyeccion);
      tP2Resina.setText(dtostrf(presionResinaInyeccion/10, 2, 1, result));
      break;
    /*case 15:
      presionResinaInyeccion = valor;
      rotaryEncoder.setEncoderValue(presionResinaInyeccion);
      preferences.putFloat("resinaI", presionResinaInyeccion);
      tP2Resina.setText(dtostrf(presionResinaInyeccion/10, 2, 1, result));
      break;
    */
    case 16:
      ECS = 1;
      presionHardenerInyeccion = valor;
      rotaryEncoder.setEncoderValue(presionHardenerInyeccion);
      preferences.putFloat("hardenerI", presionHardenerInyeccion);
      tP2Hardener.setText(dtostrf(presionHardenerInyeccion/10, 2, 1, result));
      break;
    /*case 17:
      presionHardenerInyeccion = valor;
      rotaryEncoder.setEncoderValue(presionHardenerInyeccion);
      preferences.putFloat("hardenerI", presionHardenerInyeccion);
      tP2Hardener.setText(dtostrf(presionHardenerInyeccion/10, 2, 1, result));
      break;
    */
    case 18:
      ECS = 1;
      presionAireInyeccion = valor;
      rotaryEncoder.setEncoderValue(presionAireInyeccion);
      preferences.putFloat("aireI", presionAireInyeccion);
      tP2Aire.setText(dtostrf((presionAireInyeccion/10)*1.3, 2, 1, result));
      break;
    /*case 19:
      presionAireInyeccion = valor;
      rotaryEncoder.setEncoderValue(presionAireInyeccion);
      preferences.putFloat("aireI", presionAireInyeccion);
      tP2Aire.setText(dtostrf(presionAireInyeccion/10, 2, 1, result));
      break;
    */
    case 20:
      ECS = 1;
      presionResinaPurgas = valor;
      rotaryEncoder.setEncoderValue(presionResinaPurgas);
      preferences.putFloat("resinaP", presionResinaPurgas);
      tP3Resina.setText(dtostrf(presionResinaPurgas/10, 2, 1, result));
      break;
    /*case 21:
      presionResinaPurgas = valor;
      rotaryEncoder.setEncoderValue(presionResinaPurgas);
      preferences.putFloat("resinaP", presionResinaPurgas);
      tP3Resina.setText(dtostrf(presionResinaPurgas/10, 2, 1, result));
      break;
    */
    case 22:
      ECS = 1;
      presionHardenerPurgas = valor;
      rotaryEncoder.setEncoderValue(presionHardenerPurgas);
      preferences.putFloat("hardenerP", presionHardenerPurgas);
      tP3Hardener.setText(dtostrf(presionHardenerPurgas/10, 2, 1, result));
      break;
    /*case 23:
      presionHardenerPurgas = valor;
      rotaryEncoder.setEncoderValue(presionHardenerPurgas);
      preferences.putFloat("hardenerP", presionHardenerPurgas);
      tP3Hardener.setText(dtostrf(presionHardenerPurgas/10, 2, 1, result));
      break;
    */
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
    case 38:
      switch(paginaSeleccionada){
        case 0:
          tP0Temperatura1.setText(dtostrf(valor, 2, 0, result));
          break;
        case 2:
          tP2Temperatura1.setText(dtostrf(valor, 2, 0, result));
          break;             
        }
        break;
    case 39:
      switch(paginaSeleccionada){
        case 0:
          tP0Temperatura2.setText(dtostrf(valor, 2, 0, result));
          break;
        case 2:
          tP2Temperatura2.setText(dtostrf(valor, 2, 0, result));
          break;             
        }
        break;
    case 40:
      valorFloat = valor;
      if(paginaSeleccionada == 1){
        tP1Presion1.setText(dtostrf(valorFloat/10, 2, 1, result));
        }
        break;
    case 41:
      valorFloat = valor;
      if(paginaSeleccionada == 1){
        tP1Presion2.setText(dtostrf(valorFloat/10, 2, 1, result));
        }
        break;
    case 42:
      if(paginaSeleccionada == 0){
        if(valor == 1){
          tP0NResina.setText("X");
          Serial2.print("t6.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP0NResina.setText("N");
          Serial2.print("t6.pco=1024");
          Serial2.print("\xFF\xFF\xFF");
          }
        }
      else if(paginaSeleccionada == 2){
        if(valor == 1){
          tP2NResina.setText("X");
          Serial2.print("t15.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP2NResina.setText("N");
          Serial2.print("t15.pco=1024");
          Serial2.print("\xFF\xFF\xFF");
          }
        }
        break;
    case 43:
      if(paginaSeleccionada == 0){
        if(valor == 1){
          tP0NHardener.setText("X");
          Serial2.print("t7.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP0NHardener.setText("N");
          Serial2.print("t7.pco=1024");
          Serial2.print("\xFF\xFF\xFF");
          }
        }
      else if(paginaSeleccionada == 2){
        if(valor == 1){
          tP2NHardener.setText("X");
          Serial2.print("t16.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP2NHardener.setText("N");
          Serial2.print("t16.pco=1024");
          Serial2.print("\xFF\xFF\xFF");
          }
        }
        break;
    case 44:
      valorFloat = valor;
      if(paginaSeleccionada == 2){
        tP2Aire.setText(dtostrf((valorFloat/10)*1.3, 2, 1, result));
        preferences.putFloat("aireI", valorFloat);
        }
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
  
  mensajeESPNOWsaliente.id = 1;
  
  pinMode(pinBoton1, INPUT_PULLUP);
  pinMode(pinBoton2, INPUT_PULLUP);
  pinMode(pinBuzzer, OUTPUT);
  
  Serial.begin(115200);
  Serial2.begin(115200);
  
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
  presionResinaPurgas = preferences.getFloat("resinaP", 0);
  presionHardenerPurgas = preferences.getFloat("hardenerP", 0);
  presionMinima = preferences.getFloat("presionMinima", 0);
  
}

void loop() {

  ECS = 0;         
    
  nexLoop(nex_listen_list);
  rotary_loop();

//LECTURA ESTADO BOTONES//
  int lectura1 = !digitalRead(pinBoton1);
  int lectura2 = !digitalRead(pinBoton2);

  if(paginaSeleccionada == 1){
    
    if (lectura1 != ultimoEstadoBoton1 || lectura2 != ultimoEstadoBoton2){
      lastDebounceTime = millis();
      }

    if ((millis() - lastDebounceTime) > debounceDelay){
      
      if(lectura1 != estadoBoton1){
        estadoBoton1 = lectura1;
        if(lectura1 == HIGH){
          if(estadoRecirculacion == 0 && tiempoRecirculacion>0){
            estadoRecirculacion = 1;
            Serial2.print("t8.pco=1024");
            Serial2.print("\xFF\xFF\xFF");
            mensajeESPNOWsaliente.operacion = 1;
            mensajeESPNOWsaliente.valor = 1;
            mandarDatosESPNOW(); 
            varRecirculacion.setValue(1);
            bP1Iniciar.setValue(1);
            bP1Iniciar.setText("Terminar");
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
            }
          }
        }
      if(lectura2 != estadoBoton2){
        estadoBoton2 = lectura2;
        if(lectura2 == HIGH){
          if(estadoRecirculacion == 1){
            estadoRecirculacion = 0;
            Serial2.print("t8.pco=0");
            Serial2.print("\xFF\xFF\xFF");
            mensajeESPNOWsaliente.operacion = 1;
            mensajeESPNOWsaliente.valor = 0;
            mandarDatosESPNOW(); 
            varRecirculacion.setValue(0);
            bP1Iniciar.setValue(0);
            bP1Iniciar.setText("Iniciar");
            bP1Iniciar.setValue(0);
            bP1Iniciar.setText("Iniciar");
            Serial2.print("tsw bPr,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bI,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bP,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("t8.pco=0");
            Serial2.print("\xFF\xFF\xFF");
            }
          }
        }
      
      }
      
    }
    
  if(paginaSeleccionada == 2){
    if (lectura1 != ultimoEstadoBoton1 || lectura2 != ultimoEstadoBoton2){
      lastDebounceTime = millis();
      }
    if ((millis() - lastDebounceTime) > debounceDelay){
      if(lectura1 != estadoBoton1){
        estadoBoton1 = lectura1;
        if(lectura1 == HIGH){
          if(estadoInyeccion == 0){
            estadoInyeccion = 1;
            bP2Iniciar.setValue(1);
            bP2Iniciar.setValue(1);
            bP2Iniciar.setFont(0);
            bP2Iniciar.setFont(0);
            bP2Iniciar.setText("Terminar");
            bP2Iniciar.setText("Terminar");
            Serial2.print("tsw bt1,1");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bPr,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bR,0");
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("tsw bP,0");
            Serial2.print("\xFF\xFF\xFF");
            mensajeESPNOWsaliente.operacion = 2;
            mensajeESPNOWsaliente.valor = 1;
            mandarDatosESPNOW(); 
            }
          else{
            estadoInyeccion = 0;
            bP2Iniciar.setValue(0);
            bP2Iniciar.setValue(0);
            bP2Iniciar.setFont(3);
            bP2Iniciar.setFont(3);
            bP2Iniciar.setText("Iniciar");
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
            digitalWrite(pinBuzzer, LOW);
            bP2Pausa.setValue(0);
            bP2Pausa.setValue(0);
            estadoPausaInyeccion = 0;
            mensajeESPNOWsaliente.operacion = 2;
            mensajeESPNOWsaliente.valor = 0;
            mandarDatosESPNOW(); 
            }
          }            
        }
      if(lectura2 != estadoBoton2){
        estadoBoton2 = lectura2;
        if(lectura2 == HIGH){
          if(estadoPausaInyeccion == 0 && estadoInyeccion == 1){
            estadoPausaInyeccion = 1;
            bP2Pausa.setValue(1);
            bP2Pausa.setValue(1);
            mensajeESPNOWsaliente.operacion = 5;
            mensajeESPNOWsaliente.valor = 0;
            mandarDatosESPNOW(); 
            }
          else if(estadoPausaInyeccion == 1 && estadoInyeccion == 1){
            estadoPausaInyeccion = 0;
            bP2Pausa.setValue(0);
            bP2Pausa.setValue(0);
            mensajeESPNOWsaliente.operacion = 5;
            mensajeESPNOWsaliente.valor = 1;
            mandarDatosESPNOW();      
            }
          }
        }
      }   
    }
 
  if(paginaSeleccionada == 3){
    if (lectura1 != ultimoEstadoBoton1 || lectura2 != ultimoEstadoBoton2){
      lastDebounceTime = millis();
      }
    if ((millis() - lastDebounceTime) > debounceDelay){
      if(lectura1 != estadoBoton1){
        estadoBoton1 = lectura1;
        if(lectura1 == HIGH){
          if(estadoPurga == 0){
            if(purgaSeleccionada == 1){
              mensajeESPNOWsaliente.operacion = 3;
              mensajeESPNOWsaliente.valor = 1;
              mandarDatosESPNOW(); 
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
              estadoPurga = 1;
              varPurgas.setValue(1);
              }
            else if(purgaSeleccionada == 2){
              mensajeESPNOWsaliente.operacion = 4;
              mensajeESPNOWsaliente.valor = 1;
              mandarDatosESPNOW(); 
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
              estadoPurga = 1;
              varPurgas.setValue(1);
              }
            estadoPurga = 1;
            }
          }            
        }
      if(lectura2 != estadoBoton2){
        estadoBoton2 = lectura2;
        if(lectura2 == HIGH){
          if(estadoPurga == 1){
            if(purgaSeleccionada == 1){
              mensajeESPNOWsaliente.operacion = 3;
              mensajeESPNOWsaliente.valor = 0;
              mandarDatosESPNOW(); 
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
              }
            else if(purgaSeleccionada == 2){
              mensajeESPNOWsaliente.operacion = 4;
              mensajeESPNOWsaliente.valor = 0;
              mandarDatosESPNOW(); 
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
              }
            estadoAirePurgas = 0;
            estadoPurga = 0;
            }
          }
        }
      }   
    }    
    
  ultimoEstadoBoton1 = lectura1;
  ultimoEstadoBoton2 = lectura2;

  received = false;
}
