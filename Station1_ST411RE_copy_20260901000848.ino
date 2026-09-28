#include <SPI.h>
#include <LoRa.h>

// ---------------- PIN DRAGINO -------------------------
const int LORA_SS    = 10;
const int LORA_RST   = 9;
const int LORA_DIO0  = 2;

// ---------------- PARAMETRI LORA ----------------------
const long LORA_FREQ = 868E6;

// ---------------- BUFFER ------------------------------
char rxBuf[64];   // buffer dove scrivo il messaggio ricevuto

// ---------------- VARIABILI E PARAMETRI ----------------------------
String currentTx;
String nameTx = "Station1";     // nome della stazione remota
unsigned long packetNumber = 0;
unsigned long ACKNumber = 0;
String action;
int caricoH2O = 0; // 0 acqua chiusa  1 acqua aperta
int scaricoH2O = 0; // 0 acqua chiusa  1 acqua aperta
int livello = 0; // livello % dell'acqua nella vasca

unsigned long lastSend = 0;
const unsigned long sendInterval = 10000; // 10 secondi
unsigned long ledTxTime = 0;
unsigned long ledRxTime = 0;
const unsigned long ledDuration = 200;   // durata del LED acceso

// ---------------- PARAMETRI SERBATOIO ----------------------------
#define TRIG_PIN 4
#define ECHO_PIN 5
const float SERBATOIO_ALTEZZA = 150.0;  // cm
#define LEDTX 6   // LED sul pin 6 PING ricevuto
#define LEDRX 8   // LED sul pin 8 PONG ricevuto

// =======================================================
// MisuraLivello
// =======================================================
float misuraLivello() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long durata = pulseIn(ECHO_PIN, HIGH, 30000);

  if (durata == 0) {
    return -1;   // nessuna misura
  }

  // velocità del suono: circa 0,0343 cm/us
  float distanza = durata * 0.0343 / 2.0;

  // livello acqua
  float livello = SERBATOIO_ALTEZZA - distanza;

  // limiti
  if (livello < 0) {
    livello = 0;
  }
  if (livello > SERBATOIO_ALTEZZA) {
    livello = SERBATOIO_ALTEZZA;
  }
  return livello;
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(115200);

  pinMode(LEDTX, OUTPUT);
  pinMode(LEDRX, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println();
  Serial.println("================================");
  Serial.println("AVVIO PROGRAMMA");
  Serial.println("================================");

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  Serial.println("Pin LoRa impostati");
/*
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(12);
*/
  Serial.println("Parametri LoRa impostati");

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("ERRORE LoRa");
    while (1) {
      digitalWrite(LEDTX, HIGH);
      delay(200);
      digitalWrite(LEDTX, LOW);
      delay(200);
    }
  }

  Serial.println("LoRa OK");
  Serial.println("Programma avviato");
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(12);
}
// =======================================================
// LOOP
// =======================================================
void loop() {

  // =====================================================
  // SPEGNIMENTO LED DOPO 200 ms
  // =====================================================

  if (digitalRead(LEDTX) == HIGH && millis() - ledTxTime >= ledDuration) {
    digitalWrite(LEDTX, LOW);
  }

  if (digitalRead(LEDRX) == HIGH && millis() - ledRxTime >= ledDuration) {
    digitalWrite(LEDRX, LOW);
  }

  // =====================================================
  // TRASMISSIONE OGNI 10 SECONDI
  // =====================================================

  if (millis() - lastSend > sendInterval) {

    lastSend = millis();

    currentTx = nameTx + "," +
                String(packetNumber) + "," +
                String(misuraLivello()) + "," +
                String(caricoH2O) + "," +
                String(scaricoH2O);

    Serial.print("[TX] Invio: ");
    Serial.println(currentTx);

    LoRa.beginPacket();
    LoRa.print(currentTx);
    LoRa.endPacket();
 
    // LED TX acceso per 200 ms
    digitalWrite(LEDTX, HIGH);
    ledTxTime = millis();

    packetNumber++;
  }


  // =====================================================
  // RICEZIONE
  // =====================================================

  int packetSize = LoRa.parsePacket();

  if (packetSize) {

    // LED RX acceso per 200 ms
    digitalWrite(LEDRX, HIGH);
    ledRxTime = millis();
    int len = 0;
    while (LoRa.available() && len < sizeof(rxBuf) - 1) {
      rxBuf[len++] = (char)LoRa.read();
    }
    rxBuf[len] = '\0';
    Serial.print("[TX] Ricevuto: ");
    Serial.println(rxBuf);

    // =================================================
    // PARSING DEL MESSAGGIO
    // =================================================

    char *token = strtok(rxBuf, ",");

    if (token != NULL) {
      String nome = String(token);

      // Secondo token
      token = strtok(NULL, ",");

      if (token != NULL) {
        ACKNumber = atoi(token);
      }

      // Terzo token
      token = strtok(NULL, ",");

      if (token != NULL) {
        action = atoi(token);
      }

      // Stampa i risultati
      Serial.print("Source: ");
      Serial.println(nome);

      Serial.print("ACK Number: ");
      Serial.println(ACKNumber);

      Serial.print("Action: ");
      Serial.println(action);
    }
  }
}

