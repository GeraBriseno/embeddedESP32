#include "Nextion.h"
#include "AiEsp32RotaryEncoder.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PID_v1.h>

Preferences preferences;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 2);

EthernetServer server(80);
EthernetClient client;

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


//FUNCIONES//
void mandarDatosSerial(int operacion, int valor){
  
  StaticJsonDocument<200> doc;
  doc["operacion"] = operacion;
  doc["valor"] = valor;
  
  serializeJson(doc, Serial);
}

void mandarDatosEthernet(int operacion, float valor){
  String x = "";

  x += "{'operacion':";
  x += String(operacion);
  x += ",'valor':";
  x += String(valor);
  x += "}";
  server.print(x);
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
            mandarDatosEthernet(7, presionMinima);
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
              mandarDatosEthernet(10, presionResinaRecirculacion);
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
              mandarDatosEthernet(12, presionHardenerRecirculacion);
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
              mandarDatosEthernet(45, tiempoRecirculacion);
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
                mandarDatosEthernet(14, presionResinaInyeccion);
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
                mandarDatosEthernet(16, presionHardenerInyeccion);
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
              tP2Aire.setText(dtostrf(presionAireInyeccion/10, 2, 1, result));
              if(ECS == 0){
                mandarDatosEthernet(18, presionAireInyeccion);
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
                  mandarDatosEthernet(20, presionResinaPurgas);
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
                  mandarDatosEthernet(22, presionHardenerPurgas);
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
  mandarDatosEthernet(24, 1);
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
  mandarDatosEthernet(25, 1);
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
  tP2Aire.setText(dtostrf(presionAireInyeccion/10, 2, 1, result));
  rotaryEncoder.setEncoderValue(presionResinaInyeccion);
  tiempoRecirculacion = 5;
  mandarDatosEthernet(26, 1);
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
  mandarDatosEthernet(27, 1);
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
    mandarDatosEthernet(1, 1);    
    }
  else if(estadoRecirculacion == 1){
    terminarTodasOperaciones();
    Serial2.print("t8.pco=0");
    Serial2.print("\xFF\xFF\xFF");
    estadoRecirculacion = 0;
    mandarDatosSerial(1, 0);
    mandarDatosEthernet(1, 0);
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
    mandarDatosEthernet(2, 1);
    }
  else {
    terminarTodasOperaciones();
    estadoInyeccion = 0;
    bajaPresion = 0;
    estadoPausaInyeccion = 0;
    mandarDatosSerial(2, 0);
    mandarDatosSerial(5, 0);
    mandarDatosEthernet(2, 0);
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
      mandarDatosEthernet(3, 1);
      }
    else if(purgaSeleccionada == 2){
      presionHPMap = map(presionHardenerPurgas, 0, 65, 0, 255);
      analogWrite(itvHardener, presionHPMap);
      mandarDatosSerial(4, 1);
      mandarDatosEthernet(4, 1);
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
      mandarDatosEthernet(3, 0);
      }
    else if(purgaSeleccionada == 2){
      mandarDatosEthernet(4, 0);
      }
    }        
}

void purgaResinaSelect(void *ptr){
  purgaSeleccionada = 1;
  mandarDatosEthernet(36, 1);
}

void purgaHardenerSelect(void *ptr){
  purgaSeleccionada = 2;
  mandarDatosEthernet(37, 1);        
}

void pausaInyeccion(void *ptr){
  if(estadoPausaInyeccion == 0){
    estadoPausaInyeccion = 1;
    analogWrite(itvResina, 0);
    analogWrite(itvHardener, 0);
    mandarDatosSerial(2, 0);
    mandarDatosEthernet(5, 0);
    }
  else{
    estadoPausaInyeccion = 0;
    presionRIMap = map(presionResinaInyeccion, 0, 65, 0, 255);
    presionHIMap = map(presionHardenerInyeccion, 0, 65, 0, 255);
    analogWrite(itvResina, presionRIMap);
    analogWrite(itvHardener, presionHIMap);
    mandarDatosSerial(2, 1);
    mandarDatosEthernet(5, 1);     
    }
}

void airePurgas(void *ptr){
  if(estadoAirePurgas == 0){
    estadoAirePurgas = 1;
    analogWrite(itvAire, 200);
    mandarDatosEthernet(6,1);
    }
  else{
    estadoAirePurgas = 0;
    analogWrite(itvAire, 0);
    mandarDatosEthernet(6,0);
    }
}
void principalPresionMinimaSelect(void *ptr){
  presionMinimaSeleccionada = 1;
  rotaryEncoder.setEncoderValue(presionMinima);
  mandarDatosEthernet(9, 0);  
}
void recirculacionResinaSelect(void *ptr){
  variableSeleccionadaRecirculacion = 0;
  rotaryEncoder.setEncoderValue(presionResinaRecirculacion);
  mandarDatosEthernet(28, 1);
}
void recirculacionHardenerSelect(void *ptr){
  variableSeleccionadaRecirculacion = 1;
  rotaryEncoder.setEncoderValue(presionHardenerRecirculacion);
  mandarDatosEthernet(29, 1);
}
void recirculacionTiempoSelect(void *ptr){
  variableSeleccionadaRecirculacion = 2;
  rotaryEncoder.setEncoderValue(tiempoRecirculacion);
  mandarDatosEthernet(30, 1);
}

void inyeccionResinaSelect(void *ptr){
  variableSeleccionadaInyeccion = 0;
  rotaryEncoder.setEncoderValue(presionResinaInyeccion);
  mandarDatosEthernet(31, 1);
}
void inyeccionHardenerSelect(void *ptr){
  variableSeleccionadaInyeccion = 1;
  rotaryEncoder.setEncoderValue(presionHardenerInyeccion);
  mandarDatosEthernet(32, 1);
}
void inyeccionAireSelect(void *ptr){
  variableSeleccionadaInyeccion = 2;
  rotaryEncoder.setEncoderValue(presionAireInyeccion);
  mandarDatosEthernet(33, 1);
}

void purgasResinaSelect(void *ptr){
  variableSeleccionadaPurgas = 0;
  rotaryEncoder.setEncoderValue(presionResinaPurgas);
  mandarDatosEthernet(34, 1);
}
void purgasHardenerSelect(void *ptr){
  variableSeleccionadaPurgas = 1;
  rotaryEncoder.setEncoderValue(presionHardenerPurgas);
  mandarDatosEthernet(35, 1);
}


void setup() {

  Serial.begin(115200);
  Serial2.begin(115200);

  Ethernet.init(5);
  delay(500);

  Ethernet.begin(mac, ip);
  delay(500);

  server.begin();
  
  nexInit();

  //8.4 BARES, 120 PSI, 4.3V
  
  pinMode(pinSensorTemp1, INPUT);  
  pinMode(pinSensorTemp2, INPUT);
  pinMode(pinSensorPresion1, INPUT);
  pinMode(pinSensorPresion2, INPUT);
  pinMode(pinSensorNivel1, INPUT_PULLUP);
  pinMode(pinSensorNivel2, INPUT_PULLUP);
  
  pinMode(itvResina, OUTPUT);
  pinMode(itvHardener, OUTPUT);
  pinMode(itvAire, OUTPUT);

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

  //SE LEEN LAS PRESIONES GUARDADAS EN MEMORIA FLASH//
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

  ECS = 0;
  
  if(Ethernet.hardwareStatus()==EthernetW5500){
    unsigned long timeout = millis()+3000;

    while(Ethernet.linkStatus() == LinkOFF){
      // Break out if timeout reached
      if(millis() > timeout)
        break;
      delayMicroseconds(100);
    }
    if(Ethernet.linkStatus() == LinkOFF){
      Serial.println(F("Ethernet link is down."));
      }
  }
  
  EthernetClient client = server.available();
  
  if(client){
    
    if(client.connected()){
      
      Serial.println("Client is connected");
      
      if(client.available()){
          
        StaticJsonDocument<200> doc;
        DeserializationError err = deserializeJson(doc, client);
        
        if(err == DeserializationError::Ok){
          
          int operacion = doc["operacion"].as<int>();
          int valor = doc["valor"].as<int>();               

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
              tP2Aire.setText(dtostrf(presionAireInyeccion/10, 2, 1, result));
              break;
            case 19:
              ECS = 1;
              presionAireInyeccion = valor;
              rotaryEncoder.setEncoderValue(presionAireInyeccion);
              preferences.putFloat("aireI", presionAireInyeccion);
              tP2Aire.setText(dtostrf(presionAireInyeccion/10, 2, 1, result));
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
              tP2Aire.setText(dtostrf(presionAireInyeccion/10, 2, 1, result));
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
        else{
          Serial.println("Deserialization Error From Ethernet");
          }
        }
    }
  }

//LECTURA Y MUESTRA DE SENSORES//
  if(paginaSeleccionada == 0){      
      if(sensorCount < 1000){
        lecturaTemperatura1 += analogRead(pinSensorTemp1);
        lecturaTemperatura2 += analogRead(pinSensorTemp2);
        sensorCount += 1;
        }
      if(sensorCount == 1000){
        tP0PresionMinima.setText(dtostrf(presionMinima/10, 2, 1, result));
        mandarDatosEthernet(7, presionMinima);
        valorSensorNivel1 = digitalRead(pinSensorNivel1);
        valorSensorNivel2 = digitalRead(pinSensorNivel2);
        if(valorSensorNivel1 == 0){
          tP0NResina.setText("X");
          mandarDatosEthernet(42, 1);
          Serial2.print("t6.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP0NResina.setText("N");
          mandarDatosEthernet(42, 0);
          Serial2.print("t6.pco=1024");
          Serial2.print("\xFF\xFF\xFF");
          }
        if(valorSensorNivel2 == 0){
          tP0NHardener.setText("X");
          mandarDatosEthernet(43, 1);
          Serial2.print("t7.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP0NHardener.setText("N");
          mandarDatosEthernet(43, 0);
          Serial2.print("t7.pco=1024");
          Serial2.print("\xFF\xFF\xFF");
          }  
        promedioSensorTemperatura1 = lecturaTemperatura1/1000;
        promedioSensorTemperatura2 = lecturaTemperatura2/1000;
        valorSensorTemperatura1 = map(promedioSensorTemperatura1, 0, 4095, -40, 140);
        valorSensorTemperatura2 = map(promedioSensorTemperatura2, 0, 4095, -40, 140);
        tP0Temperatura1.setText(dtostrf(valorSensorTemperatura1, 2, 0, result));
        tP0Temperatura2.setText(dtostrf(valorSensorTemperatura2, 2, 0, result));
        mandarDatosEthernet(38, valorSensorTemperatura1);
        mandarDatosEthernet(39, valorSensorTemperatura2);
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
        mandarDatosEthernet(40, valorSensorPresion1);
        mandarDatosEthernet(41, valorSensorPresion2);
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
        if(valorSensorNivel1 == 1){
          tP2NResina.setText("X");
          mandarDatosEthernet(42, 1);
          Serial2.print("t15.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP2NResina.setText("N");
          mandarDatosEthernet(42, 0);
          Serial2.print("t15.pco=1024");
          Serial2.print("\xFF\xFF\xFF");
          }
        if(valorSensorNivel2 == 1){
          tP2NHardener.setText("X");
          mandarDatosEthernet(43, 1);
          Serial2.print("t16.pco=63488");
          Serial2.print("\xFF\xFF\xFF");
          }
        else{
          tP2NHardener.setText("N");
          mandarDatosEthernet(43, 0);
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
      mandarDatosEthernet(38, valorSensorTemperatura1);
      mandarDatosEthernet(39, valorSensorTemperatura2);
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
        if(bajaPresion == 1){
          if(millis()>inicioBajaPresion+20000){
            Serial.println("DIE ENDE");
            bajaPresion = 0;
            inicioBajaPresion = 0;
            mandarDatosEthernet(2, 0);
            mandarDatosEthernet(8, 0);
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
          mandarDatosEthernet(8, 1);
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
        mandarDatosEthernet(45, tiempoRecirculacion);
        segundosRecirculacion = 0;
        } 
      }
    if(tiempoRecirculacion == 0){
      mandarDatosEthernet(1, 0);
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
  
}
