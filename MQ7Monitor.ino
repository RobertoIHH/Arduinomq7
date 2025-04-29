// Incluir las bibliotecas necesarias para trabajar con Bluetooth Low Energy (BLE)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// No incluimos driver/adc.h para evitar conflictos con la biblioteca de ESP32

// Configuración de pines
#define MQ7_PIN 3  // Pin analógico para conectar el sensor MQ7 (ajustar según tu conexión)

// UUID del servicio Bluetooth
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"  // UUID del servicio
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // UUID de la característica

// Variables globales para Bluetooth
BLEServer* pServer = NULL;  // Servidor BLE
BLECharacteristic* pCharacteristic = NULL;  // Característica BLE
bool deviceConnected = false;  // Estado de conexión del dispositivo
bool oldDeviceConnected = false;  // Estado de conexión anterior
unsigned long previousMillis = 0;  // Tiempo anterior
const long interval = 1000;  // Intervalo de envío de datos (1 segundo)

// Configuración del sensor MQ7 para monóxido de carbono
const int RL_VALUE = 10;   // Resistencia RL del módulo en Kilo ohms (valor típico)
// R0 se calcula según la fórmula proporcionada: R0=((10*((4.9-0.626)/(0.626)))/(26.75))
// Valor precalculado de R0
const float R0 = 2.61;     // Resistencia R0 calibrada (en KΩ)

// Configuración para lecturas múltiples para aumentar precisión
const int READ_SAMPLE_INTERVAL = 100;  // Tiempo entre muestras (ms)
const int READ_SAMPLE_TIMES = 5;       // Número de muestras
//--------------------------------------------------------------------------------
// Definición de los gases que puede medir el MQ7
enum GasType {
  GAS_CO = 0,        // Monóxido de carbono
  GAS_HYDROGEN = 1,  // Hidrógeno
  GAS_COUNT = 2      // Total de gases soportados
};

// Estructura para almacenar parámetros de calibración
struct GasCalibration {
  const char* name;   // Nombre del gas
  float X0;           // Punto X0 (ppm)
  float Y0;           // Punto Y0 (Rs/R0)
  float X1;           // Punto X1 (ppm)
  float Y1;           // Punto Y1 (Rs/R0)
  float scope;        // Pendiente calculada
  float coord;        // Coordenada calculada
};

// Calibraciones para cada gas
GasCalibration gasCalibrations[GAS_COUNT] = {
  // Monóxido de Carbono (CO)
  {
    "CO", 
    50.0f, 1.66f,       // Punto inicial
    4000.0f, 0.052f,    // Punto final
    0.0f, 0.0f          // Scope y coord se calcularán
  },
  // Hidrógeno (H2)
  {
    "H2",
    50.0f, 1.36f,       // Punto inicial
    4000.0f, 0.09f,     // Punto final
    0.0f, 0.0f          // Scope y coord se calcularán
  }
};

// Variables de control para rotación de gases
unsigned long lastGasChangeTime = 0;
const unsigned long GAS_CHANGE_INTERVAL = 10000; // Cambiar gas cada 10 segundos
int currentGasIndex = GAS_CO;

// Función para inicializar calibraciones
void initGasCalibrations() {
  for (int i = 0; i < GAS_COUNT; i++) {
    // Calcular puntos logarítmicos, pendiente y coordenada para cada gas
    float logX0 = log10(gasCalibrations[i].X0);
    float logY0 = log10(gasCalibrations[i].Y0);
    float logX1 = log10(gasCalibrations[i].X1);
    float logY1 = log10(gasCalibrations[i].Y1);
    
    gasCalibrations[i].scope = (logY1 - logY0) / (logX1 - logX0);
    gasCalibrations[i].coord = logY0 - logX0 * gasCalibrations[i].scope;
    
    Serial.print("Gas: ");
    Serial.print(gasCalibrations[i].name);
    Serial.print(", Scope: ");
    Serial.print(gasCalibrations[i].scope);
    Serial.print(", Coord: ");
    Serial.println(gasCalibrations[i].coord);
  }
}

// Obtener concentración para un gas específico
float getGasConcentration(float rs_ro_ratio, int gasIndex) {
  return pow(10, gasCalibrations[gasIndex].coord + gasCalibrations[gasIndex].scope * log10(rs_ro_ratio));
}
//--------------------------------------------------------------------------------
// Clase para manejar los eventos de conexión Bluetooth
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;  // Establecer estado de conexión
      Serial.println("Dispositivo conectado");  // Mensaje de conexión
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;  // Establecer estado de desconexión
      Serial.println("Dispositivo desconectado");  // Mensaje de desconexión
    }
};

// Obtener la resistencia del sensor a partir de la lectura analógica
float getMQResistance(int raw_adc) {
  // Convertir ADC a voltaje (para ESP32 con ADC de 12 bits)
  // El ESP32 trabaja con 0-1V por defecto, pero podemos ajustar en el cálculo
  float voltage = raw_adc * (3.3 / 4095.0);  // Conversión de ADC a voltaje
  
  // Calculamos Rs en función del voltaje y RL_VALUE
  // Rs = ((Vc - Vout) / Vout) * RL
  return (((5.0 - voltage) / voltage) * RL_VALUE);  // Cálculo de Rs
}

// Obtener la resistencia promedio en N muestras
float readMQ(int mq_pin) {
  float rs = 0;  // Inicializar suma de resistencias
  for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += getMQResistance(analogRead(mq_pin));  // Leer resistencia y sumar
    delay(READ_SAMPLE_INTERVAL);  // Esperar entre lecturas
  }
  return rs / READ_SAMPLE_TIMES;  // Devolver promedio de resistencias
}


void setup() {
  Serial.begin(115200);  // Iniciar comunicación serial
  Serial.println("Iniciando sensor MQ7 con Bluetooth...");  // Mensaje de inicio
   initGasCalibrations();
  
  // Para ESP32, usamos analogReadResolution en lugar de las funciones de driver/adc.h
  analogReadResolution(12);  // Configurar resolución ADC a 12 bits
  
  // Configuración del Bluetooth
  BLEDevice::init("ESP32S3-MQ7-Sensor");  // Inicializar dispositivo BLE
  pServer = BLEDevice::createServer();  // Crear servidor BLE
  pServer->setCallbacks(new MyServerCallbacks());  // Establecer callbacks de conexión
  BLEService *pService = pServer->createService(SERVICE_UUID);  // Crear servicio BLE
  pCharacteristic = pService->createCharacteristic(
                     CHARACTERISTIC_UUID,
                     BLECharacteristic::PROPERTY_READ |
                     BLECharacteristic::PROPERTY_WRITE |
                     BLECharacteristic::PROPERTY_NOTIFY
                   );  // Crear característica BLE
  pCharacteristic->addDescriptor(new BLE2902());  // Agregar descriptor de notificación
  pService->start();  // Iniciar servicio BLE
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();  // Obtener objeto de publicidad
  pAdvertising->addServiceUUID(SERVICE_UUID);  // Agregar UUID de servicio a publicidad
  pAdvertising->setScanResponse(true);  // Establecer respuesta de escaneo
  pAdvertising->setMinPreferred(0x06);  // Establecer intervalo de publicidad mínimo
  pAdvertising->setMinPreferred(0x12);  // Establecer intervalo de publicidad mínimo
  BLEDevice::startAdvertising();  // Iniciar publicidad
  
  Serial.println("ESP32-S3 MQ7 BLE está listo para conectarse!");  // Mensaje de listo
  Serial.println("Calibración para CO:");  // Mensaje de calibración
  Serial.print("R0 (KΩ): ");  // Imprimir R0
  Serial.println(R0);
}

void loop() {
  unsigned long currentMillis = millis();  // Obtener tiempo actual
  // Rotación entre gases
  if (currentMillis - lastGasChangeTime >= GAS_CHANGE_INTERVAL) {
    lastGasChangeTime = currentMillis;
    currentGasIndex = (currentGasIndex + 1) % GAS_COUNT;
    Serial.print("Cambiando a gas: ");
    Serial.println(gasCalibrations[currentGasIndex].name);
  }


  // Enviar datos cada intervalo definido
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Actualizar tiempo anterior
    
    if (deviceConnected) {
      // Leer la resistencia promedio del sensor
      float rs_med = readMQ(MQ7_PIN);  // Leer resistencia promedio
      
      // Calcular relación Rs/R0
      float rs_ro_ratio = rs_med / R0;  // Calcular relación Rs/R0
      
      // Calcular la concentración de CO en PPM
      float ppm = getGasConcentration(rs_ro_ratio, currentGasIndex);  // Calcular concentración para el gas actual
      
      // Valor ADC puro para referencia
      int mq7Value = analogRead(MQ7_PIN);  // Leer valor ADC
      
      // Convertir a voltaje (usando 3.3V como referencia típica en ESP32)
      float voltage = mq7Value * (3.3 / 4095.0);  // Conversión de ADC a voltaje
      
      // Crear string con los datos en formato JSON
      char txString[80];  // Buffer para string
      sprintf(txString, "{\"ADC\":%d,\"V\":%.2f,\"Rs\":%.2f,\"Rs/R0\":%.3f,\"ppm\":%.2f,\"gas\":\"%s\"}", 
              mq7Value, voltage, rs_med, rs_ro_ratio, ppm, gasCalibrations[currentGasIndex].name);  // Formatear string
      
      // Enviar los datos a través de Bluetooth
      pCharacteristic->setValue(txString);  // Establecer valor de característica
      pCharacteristic->notify();  // Notificar a los dispositivos conectados
      
      // Mostrar valores por el puerto serie
      Serial.print("Gas: "); //GAS
      Serial.print(gasCalibrations[currentGasIndex].name);
      Serial.print(", Rs: "); //RESISTENCIA
      Serial.print(rs_med);
      Serial.print(" KΩ, Rs/R0: "); //RELACION RS/R0
      Serial.print(rs_ro_ratio);
      Serial.print(", Concentración: "); //CONCENTRACION
      Serial.print(ppm);
      Serial.println(" PPM"); //PPM
      Serial.print("Enviando: ");
      Serial.println(txString);
    }
  }
  
  // Manejar reconexión
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);  // Esperar un momento antes de reiniciar publicidad
    pServer->startAdvertising();  // Reiniciar publicidad
    Serial.println("Iniciando anuncios");  // Mensaje de reinicio de publicidad
    oldDeviceConnected = deviceConnected;  // Actualizar estado de conexión anterior
  }
  
  // Dispositivo conectado
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;  // Actualizar estado de conexión anterior
  }
}
