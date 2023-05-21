#define BASE_I2C_ADDR 0X20
#include <PCA95x5.h>
#include <ArduinoJson.h>


//DECLARACION PINES PLC//
#define OUT1RR PCA95x5::Port::P17
#define OUT2RH PCA95x5::Port::P16
#define OUT3SR PCA95x5::Port::P15
#define OUT4SH PCA95x5::Port::P14
#define OUT5IBP PCA95x5::Port::P13

#define HIGH_P PCA95x5::Level::H
#define LOW_P PCA95x5::Level::L
#define INPUT_P PCA95x5::Direction::IN
#define OUTPUT_P PCA95x5::Direction::OUT
#define RELE_SUPPLY 25

PCA9555 ioexp;

void setup(){
  
  Serial.begin(115200);
  
  Wire.begin();
  ioexp.attach(Wire);
  ioexp.polarity(PCA95x5::Polarity::ORIGINAL_ALL); //INVERTED_ALL ORIGINAL_ALL
  ioexp.direction(PCA95x5::Direction::OUT_ALL);
  ioexp.write(PCA95x5::Level::L_ALL);
  
  ioexp.direction(OUT1RR, OUTPUT_P);
  ioexp.direction(OUT2RH, OUTPUT_P);
  ioexp.direction(OUT3SR, OUTPUT_P);
  ioexp.direction(OUT4SH, OUTPUT_P);
  ioexp.direction(OUT5IBP, OUTPUT_P);
  
  ioexp.write(OUT1RR, LOW_P);
  ioexp.write(OUT2RH, LOW_P);
  ioexp.write(OUT3SR, LOW_P);
  ioexp.write(OUT4SH, LOW_P);
  ioexp.write(OUT5IBP, LOW_P);

  delay(1000);
  pinMode(RELE_SUPPLY, OUTPUT);
  digitalWrite(RELE_SUPPLY, LOW);
  delay(100);
  digitalWrite(RELE_SUPPLY, HIGH);

}

//FUNCION PARA LEER LAS OPERACIONES RECIBIDAS DEL ESP32 PRINCIPAL//
void leerDatosSerial(){
  if(Serial.available()){
    
    StaticJsonDocument<300> doc;

    DeserializationError err = deserializeJson(doc, Serial);
      
    if(err == DeserializationError::Ok){
      
      int operacion = doc["operacion"].as<int>();
      int valor = doc["valor"].as<int>();
      
      switch(operacion){
        case 1:
          if(valor == 1){
            recirculacionEncender();
            }
          else{
            recirculacionApagar();
            }
          break;
        case 2:
          if(valor == 1){
            inyeccionEncender();
            }
          else{
            inyeccionApagar();
            }
          break;
        case 3:
          if(valor == 1){
            purgarResina();
            }
          else{
            terminarPurgaResina();
            }
          break;
        case 4:
          if(valor == 1){
            purgarHardener();
            }
          else{
            terminarPurgaHardener();
            }
          break;
        case 5:
          if(valor == 1){
            presionEntradaBaja();
            }
          else{
            presionEntradaNormal();
            }                
        }
     }     
  }
}

//FUNCIONES PARA ENCENDER Y APAGAR LAS VALVULAS//
void inyeccionEncender(){
  ioexp.write(OUT3SR, HIGH_P);
  ioexp.write(OUT4SH, HIGH_P);
}
void inyeccionApagar(){
  ioexp.write(OUT3SR, LOW_P);
  ioexp.write(OUT4SH, LOW_P);
}
void recirculacionEncender(){
  ioexp.write(OUT1RR, HIGH_P);
  ioexp.write(OUT2RH, HIGH_P);
}
void recirculacionApagar(){
  ioexp.write(OUT1RR, LOW_P);
  ioexp.write(OUT2RH, LOW_P);
}
void purgarResina(){
  ioexp.write(OUT3SR, HIGH_P); 
}
void purgarHardener(){
  ioexp.write(OUT4SH, HIGH_P); 
}
void terminarPurgaResina(){
  ioexp.write(OUT3SR, LOW_P);
}
void terminarPurgaHardener(){
  ioexp.write(OUT4SH, LOW_P);
}
void presionEntradaBaja(){
  ioexp.write(OUT5IBP, HIGH_P);    
}
void presionEntradaNormal(){
  ioexp.write(OUT5IBP, LOW_P);    
}

void loop(){

  leerDatosSerial();

}
