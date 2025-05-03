// Incluir las bibliotecas necesarias para trabajar con Bluetooth Low Energy (BLE)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Configuración de pines
#define MQ7_PIN 3  // Pin analógico para conectar el sensor MQ7 (ajustar según tu conexión)

// UUID del servicio Bluetooth
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"  // UUID del servicio
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // UUID de la característica
#define COMMAND_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // UUID para característica de comandos
#define STATUS_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26aa"  // UUID para característica de estado

// Variables globales para Bluetooth
BLEServer* pServer = NULL;  // Servidor BLE
BLECharacteristic* pCharacteristic = NULL;  // Característica BLE para datos
BLECharacteristic* pCommandCharacteristic = NULL;  // Característica BLE para comandos
BLECharacteristic* pStatusCharacteristic = NULL;  // Característica BLE para estado
bool deviceConnected = false;  // Estado de conexión del dispositivo
bool oldDeviceConnected = false;  // Estado de conexión anterior
unsigned long previousMillis = 0;  // Tiempo anterior
const long interval = 1000;  // Intervalo de envío de datos (1 segundo)

// Variables para la gestión de cambio de gas
bool gasChangeRequested = false;  // Flag para indicar si hay un cambio pendiente
unsigned long gasChangeRequestTime = 0;  // Momento en que se solicitó el cambio
const long gasChangeConfirmationInterval = 500;  // Intervalo para enviar confirmación (500ms)
unsigned long lastGasConfirmationTime = 0;  // Último momento de confirmación
String requestedGasType = "";  // Tipo de gas solicitado

// Configuración del sensor MQ7 para monóxido de carbono
const int RL_VALUE = 10;   // Resistencia RL del módulo en Kilo ohms (valor típico)
const float R0 = 2.61;     // Resistencia R0 calibrada (en KΩ)

// Configuración para lecturas múltiples para aumentar precisión
const int READ_SAMPLE_INTERVAL = 100;  // Tiempo entre muestras (ms)
const int READ_SAMPLE_TIMES = 5;       // Número de muestras

// Definición de los gases que puede medir el MQ7
enum GasType {
  GAS_CO = 0,        // Monóxido de carbono
  GAS_HYDROGEN = 1,  // Hidrógeno
  GAS_LPG = 2,       // Gas licuado de petróleo (LPG)
  GAS_METHANE = 3,   // Metano (CH4)
  GAS_ALCOHOL = 4,   // Alcohol
  GAS_COUNT = 5      // Total de gases soportados
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
  },
  // Gas licuado de petróleo (LPG)
  {
    "LPG",
    50.0f, 9.0f,       // Punto inicial
    4000.0f, 5.0f,    // Punto final
    0.0f, 0.0f          // Scope y coord se calcularán
  },
  // Metano (CH4)
  {
    "CH4",
    50.0f, 15.0f,       // Punto inicial
    4000.0f, 9.0f,    // Punto final
    0.0f, 0.0f          // Scope y coord se calcularán
  },
  // Alcohol
  {
    "Alcohol",
    50.0f, 17.0f,       // Punto inicial
    4000.0f, 13.0f,      // Punto final
    0.0f, 0.0f          // Scope y coord se calcularán
  }
};

// Variable para el gas actual seleccionado por la aplicación
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

// Función para buscar el índice de un gas por su nombre
int findGasIndexByName(String gasName) {
  for (int i = 0; i < GAS_COUNT; i++) {
    if (gasName.equalsIgnoreCase(gasCalibrations[i].name)) {
      return i;
    }
  }
  return -1;  // No encontrado
}

// Función para enviar confirmación de cambio de gas
void sendGasChangeConfirmation() {
  if (!deviceConnected) return;
  
  // Solo enviar confirmación si hay una solicitud pendiente
  if (gasChangeRequested) {
    char confirmationJson[150];
    sprintf(confirmationJson, 
            "{\"command\":\"gas_changed\",\"to\":\"%s\",\"success\":true,\"timestamp\":%lu,\"requested\":\"%s\"}", 
            gasCalibrations[currentGasIndex].name, 
            millis(),
            requestedGasType.c_str());
    
    // Enviar por la característica de datos
    pCharacteristic->setValue(confirmationJson);
    pCharacteristic->notify();
    
    // También enviar al estado para que la app pueda consultar
    pStatusCharacteristic->setValue(confirmationJson);
    
    Serial.print("Enviando confirmación: ");
    Serial.println(confirmationJson);
    
    lastGasConfirmationTime = millis();
  }
}

// Función para actualizar el estado del sensor
void updateSensorStatus() {
  if (!deviceConnected) return;
  
  char statusJson[150];
  sprintf(statusJson, 
          "{\"status\":\"ok\",\"current_gas\":\"%s\",\"gas_index\":%d,\"timestamp\":%lu}", 
          gasCalibrations[currentGasIndex].name, 
          currentGasIndex,
          millis());
  
  pStatusCharacteristic->setValue(statusJson);
  
  // No hacemos notificación porque el cliente puede sondear esta característica cuando lo necesite
}

// Clase para manejar los eventos de conexión Bluetooth
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;  // Establecer estado de conexión
      Serial.println("Dispositivo conectado");  // Mensaje de conexión
      
      // Actualizar el estado del sensor cuando un dispositivo se conecta
      updateSensorStatus();
    };
    
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;  // Establecer estado de desconexión
      Serial.println("Dispositivo desconectado");  // Mensaje de desconexión
      
      // Resetear variables de cambio de gas al desconectar
      gasChangeRequested = false;
      requestedGasType = "";
    }
};

// Clase para manejar lecturas de la característica de estado
class StatusCallbacks: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic) {
      // Actualizar el estado antes de que el cliente lo lea
      updateSensorStatus();
      Serial.println("Cliente solicitó lectura de estado");
    }
};

// Clase para manejar los comandos recibidos desde la aplicación
class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      // Obtener la cadena como un array de bytes y su longitud
      String value = pCharacteristic->getValue();
      
      if (value.length() > 0) {
        Serial.print("Comando recibido: ");
        Serial.println(value);
        
        // Verificar si el comando contiene un timestamp/ID (formato: "GAS:TIMESTAMP")
        int separatorIndex = value.indexOf(':');
        String gasName;
        String timestamp = "";
        
        if (separatorIndex != -1) {
          gasName = value.substring(0, separatorIndex);
          timestamp = value.substring(separatorIndex + 1);
          Serial.print("Gas: ");
          Serial.print(gasName);
          Serial.print(", Timestamp: ");
          Serial.println(timestamp);
        } else {
          gasName = value;
        }
        
        // Guardar el gas solicitado originalmente para incluirlo en las confirmaciones
        requestedGasType = gasName;
        
        // Procesar el comando para cambiar el tipo de gas
        int newGasIndex = -1;
        
        if (gasName == "CO") {
            newGasIndex = GAS_CO;
        }
        else if (gasName == "H2") {
            newGasIndex = GAS_HYDROGEN;
        }
        else if (gasName == "LPG") {
            newGasIndex = GAS_LPG;
        }
        else if (gasName == "CH4") {
            newGasIndex = GAS_METHANE;
        }
        else if (gasName == "ALCOHOL") {
            newGasIndex = GAS_ALCOHOL;
        }
        
        // Si el gas es válido, programar el cambio
        if (newGasIndex >= 0) {
          currentGasIndex = newGasIndex;
          Serial.print("Cambiando a gas: ");
          Serial.println(gasCalibrations[currentGasIndex].name);
          
          // Marcar que hay un cambio de gas pendiente de confirmar
          gasChangeRequested = true;
          gasChangeRequestTime = millis();
          
          // Enviar la confirmación inmediatamente
          sendGasChangeConfirmation();
          
          // Actualizar también el estado
          updateSensorStatus();
        } else {
          Serial.println("Gas no reconocido");
        }
      }
    }
};

// Obtener la resistencia del sensor a partir de la lectura analógica
float getMQResistance(int raw_adc) {
  // Convertir ADC a voltaje (para ESP32 con ADC de 12 bits)
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
  Serial.println("Iniciando sensor MQ7 con Bluetooth (Sistema de sondeo mejorado)...");
  initGasCalibrations();
  
  // Para ESP32, usamos analogReadResolution en lugar de las funciones de driver/adc.h
  analogReadResolution(12);  // Configurar resolución ADC a 12 bits
  
  // Configuración del Bluetooth
  BLEDevice::init("ESP32S3-MQ7-Sensor");  // Inicializar dispositivo BLE
  pServer = BLEDevice::createServer();  // Crear servidor BLE
  pServer->setCallbacks(new MyServerCallbacks());  // Establecer callbacks de conexión
  
  // Crear servicio BLE
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Característica para datos del sensor (notificaciones)
  pCharacteristic = pService->createCharacteristic(
                     CHARACTERISTIC_UUID,
                     BLECharacteristic::PROPERTY_READ |
                     BLECharacteristic::PROPERTY_NOTIFY
                   );
  pCharacteristic->addDescriptor(new BLE2902());  // Agregar descriptor de notificación
  
  // Característica para comandos desde la app (escritura)
  pCommandCharacteristic = pService->createCharacteristic(
                     COMMAND_CHAR_UUID,
                     BLECharacteristic::PROPERTY_WRITE
                   );
  pCommandCharacteristic->setCallbacks(new CommandCallbacks());
  
  // Característica para estado del sensor (lectura)
  pStatusCharacteristic = pService->createCharacteristic(
                     STATUS_CHAR_UUID,
                     BLECharacteristic::PROPERTY_READ
                   );
  pStatusCharacteristic->setCallbacks(new StatusCallbacks());
  
  // Iniciar servicio y publicidad
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("ESP32-S3 MQ7 BLE con sondeo está listo para conectarse!");
  Serial.print("R0 (KΩ): ");
  Serial.println(R0);
}

void loop() {
  unsigned long currentMillis = millis();

  // Manejar el envío periódico de datos
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    if (deviceConnected) {
      // Leer y procesar datos del sensor
      float rs_med = readMQ(MQ7_PIN);
      float rs_ro_ratio = rs_med / R0;
      float ppm = getGasConcentration(rs_ro_ratio, currentGasIndex);
      int mq7Value = analogRead(MQ7_PIN);
      float voltage = mq7Value * (3.3 / 4095.0);
      
      // Crear string con los datos en formato JSON
      char txString[150];  // Buffer más grande para datos adicionales
      sprintf(txString, 
              "{\"ADC\":%d,\"V\":%.2f,\"Rs\":%.2f,\"Rs/R0\":%.3f,\"ppm\":%.2f,\"gas\":\"%s\",\"timestamp\":%lu,\"gas_index\":%d}", 
              mq7Value, voltage, rs_med, rs_ro_ratio, ppm, 
              gasCalibrations[currentGasIndex].name, millis(), currentGasIndex);
      
      // Enviar los datos
      pCharacteristic->setValue(txString);
      pCharacteristic->notify();
      
      // Mostrar valores por el puerto serie
      Serial.print("Gas: ");
      Serial.print(gasCalibrations[currentGasIndex].name);
      Serial.print(", Rs: ");
      Serial.print(rs_med);
      Serial.print(" KΩ, Rs/R0: ");
      Serial.print(rs_ro_ratio);
      Serial.print(", Concentración: ");
      Serial.print(ppm);
      Serial.println(" PPM");
    }
  }
  
  // Gestión de confirmaciones de cambio de gas
  if (gasChangeRequested) {
    // Si ha pasado suficiente tiempo desde la última confirmación, enviar otra
    if (currentMillis - lastGasConfirmationTime >= gasChangeConfirmationInterval) {
      sendGasChangeConfirmation();
      
      // Si ya pasaron varios intentos, asumir que la app recibió el cambio
      if (currentMillis - gasChangeRequestTime >= 3000) {  // 3 segundos de intentos
        gasChangeRequested = false;
        Serial.println("Tiempo de confirmación de cambio de gas agotado");
      }
    }
  }
  
  // Manejar reconexión
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Iniciando anuncios");
    oldDeviceConnected = deviceConnected;
    
    // Resetear flags de cambio de gas al desconectar
    gasChangeRequested = false;
  }
  
  // Dispositivo conectado (cambio de estado)
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    
    // Actualizar estado inmediatamente después de conectar
    updateSensorStatus();
  }
}
