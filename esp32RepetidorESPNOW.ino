#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

bool receivedControlador = false;
bool receivedPrincipal = false;

//DIRECCION DE ESP 1 PRINCIPAL
uint8_t broadcastAddressPrincipal[] = {0xC8,0xF0,0x9E,0xA6,0x65,0xBC};

//DIRECCION DE ESP 3 CONTROLADOR
uint8_t broadcastAddressControlador[] = {0x54,0x43,0xB2,0xA9,0x58,0x60};

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int id;
    int operacion;
    float valor;
    int confirmacion;
} struct_message;

struct_message mensajeESPNOWsaliente;
struct_message mensajeESPNOWentrante;

esp_now_peer_info_t peerInfoPrincipal;
esp_now_peer_info_t peerInfoControlador;

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

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&mensajeESPNOWentrante, incomingData, sizeof(mensajeESPNOWentrante));
  Serial.print(mensajeESPNOWentrante.id);
  if(mensajeESPNOWentrante.id == 1){
    receivedControlador = true;
    }
  else{
    receivedPrincipal = true;  
    }
  mensajeESPNOWsaliente.id = mensajeESPNOWentrante.id;
  mensajeESPNOWsaliente.operacion = mensajeESPNOWentrante.operacion;
  mensajeESPNOWsaliente.valor = mensajeESPNOWentrante.valor;
  mensajeESPNOWsaliente.confirmacion = mensajeESPNOWentrante.confirmacion;
}
 
void setup() {

  Serial.begin(115200);
 
  WiFi.mode(WIFI_STA);

  esp_err_t resultLRProtocol = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  //esp_wifi_config_espnow_rate(WIFI_IF_STA, ?)

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfoPrincipal.peer_addr, broadcastAddressPrincipal, 6);
  peerInfoPrincipal.channel = 0;  
  peerInfoPrincipal.encrypt = false;

  memcpy(peerInfoControlador.peer_addr, broadcastAddressControlador, 6);
  peerInfoControlador.channel = 0;  
  peerInfoControlador.encrypt = false;
       
  if (esp_now_add_peer(&peerInfoPrincipal) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  if (esp_now_add_peer(&peerInfoControlador) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

}
 
void loop() {
  
  if(receivedControlador == true){
    esp_err_t result = esp_now_send(broadcastAddressPrincipal, (uint8_t *) &mensajeESPNOWsaliente, sizeof(mensajeESPNOWsaliente));
    }
    
  if(receivedPrincipal == true){
    esp_err_t result = esp_now_send(broadcastAddressControlador, (uint8_t *) &mensajeESPNOWsaliente, sizeof(mensajeESPNOWsaliente));
    }
    
  receivedControlador = false;
  receivedPrincipal = false;
  
}
