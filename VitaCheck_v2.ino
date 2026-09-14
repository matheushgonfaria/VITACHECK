// ============================================================
// VITACHECK v2 - MALETA DE TRIAGEM
// ESP32 + MAX30102 + MLX90614 + DS18B20 + TCS34725 + BUZZER + BT
// ============================================================
//
// O QUE MUDOU DA v1:
//   1. TEMPOS NOVOS
//        Batimentos + Oxigenacao (SpO2) ..... 30 s   (antes 60 s, sem SpO2)
//        Temperatura da testa ............... 5 s    (antes 60 s)
//        Temperatura axilar ................. ate 90 s, com parada
//                                             automatica no platô
//   2. SpO2 DE VERDADE, usando o algoritmo maxim da biblioteca
//      SparkFun MAX3010x (antes o app so tinha um campo vazio).
//   3. TEMPERATURA DA TESTA em modo "max hold": pega o MAIOR valor
//      valido dos 5 s enquanto voce varre a testa. E assim que
//      termometro de testa de verdade funciona.
//   4. TEMPERATURA AXILAR com deteccao de platô: se a temperatura
//      parar de subir (variacao < 0,02 C em 15 s), ja finaliza.
//      Isso e o que os termometros digitais fazem antes de apitar.
//   5. URINALISE COM COR DE VERDADE:
//        - le uma PASTILHA BRANCA de referencia antes de tudo
//          (balanco de branco = corrige luz, sujeira, envelhecimento)
//        - subtrai o "escuro" (leitura com LED apagado) pra tirar
//          luz ambiente que vaza
//        - manda R, G, B crus E corrigidos pro app
//        - o app e que classifica a cor (fica facil recalibrar sem
//          precisar regravar o ESP32)
//        - avanca quadradinho por quadradinho, disparado pela
//          CHAVE do mecanismo (11 cliques: 10 pads + a pastilha
//          branca, que no cartucho fica depois do decimo pad)
//   6. COMANDO NOVO "LERCOR" pra calibrar o app com a tabela de
//      cores do frasco da fita.
//
// LIGACOES:
//   MAX30102  SDA->21 SCL->22  VIN->3.3V GND->GND
//   MLX90614  SDA->21 SCL->22  VCC->3.3V GND->GND
//   TCS34725  SDA->21 SCL->22  VIN->3.3V GND->GND
//             LED->26  (opcional, pra apagar o LED e medir o escuro)
//   DS18B20   DATA->4  (resistor 4.7k entre DATA e 3.3V)
//   BUZZER    +->25  -->GND
//   CHAVE DO CLIQUE (micro switch do mecanismo) -> 32 e GND
//
// BIBLIOTECAS (Gerenciador de Bibliotecas do Arduino IDE):
//   SparkFun MAX3010x Pulse and Proximity Sensor Library
//   Adafruit MLX90614 Library
//   Adafruit TCS34725
//   OneWire + DallasTemperature
// ============================================================

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <Adafruit_MLX90614.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Adafruit_TCS34725.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth nao habilitado! Va em Tools -> Board e selecione ESP32 Dev Module
#endif

// ============================================================
// PINOS E TEMPOS  (mexa aqui se quiser mudar)
// ============================================================
#define ONE_WIRE_BUS   4
#define SPEAKER_PIN    25
#define TCS_LED_PIN    26    // -1 se nao ligou o pino LED do TCS34725
#define CLIQUE_PIN     32    // chave do mecanismo (para GND quando clica)

const unsigned long T_BPM_SPO2 = 30000UL;   // 30 s
const unsigned long T_TESTA    = 5000UL;    // 5 s
const unsigned long T_AXILAR   = 90000UL;   // 90 s (teto)
const unsigned long T_REACAO   = 60000UL;   // 60 s de reacao da fita

// Platô da axilar: se variar menos que isso durante a janela, encerra.
const float  PLATO_DELTA   = 0.02;    // graus C
const unsigned long PLATO_JANELA = 15000UL;  // 15 s
const unsigned long T_AXILAR_MIN = 30000UL;  // nunca encerra antes de 30 s

const int  N_PADS = 10;

// ============================================================
// SENSORES
// ============================================================
MAX30105 particleSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS,
                                          TCS34725_GAIN_4X);
BluetoothSerial SerialBT;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

bool temTCS = false;
bool temDS  = false;

// ============================================================
// RESULTADOS GUARDADOS
// ============================================================
int   resultadoBPM  = 0;
int   resultadoSpO2 = 0;
float resultadoTT   = 0;    // testa (pele)
float resultadoTA   = 0;    // axilar
float ambienteTesta = 0;    // temperatura ambiente na hora da testa
bool  axilarEstabilizou = false;

int  padR[N_PADS], padG[N_PADS], padB[N_PADS];   // ja corrigidos
int  padRc[N_PADS], padGc[N_PADS], padBc[N_PADS]; // crus
bool temResultadoUrina = false;

// Branco de referencia da corrida atual
uint16_t brancoR = 0, brancoG = 0, brancoB = 0, brancoC = 0;
bool temBranco = false;

const char* nomesUrina[N_PADS] = {
  "glicose","bilirrubina","cetonas","densidade","sangue",
  "ph","proteina","urobilinogenio","nitrito","leucocitos"
};

// ============================================================
// UTILIDADES
// ============================================================
void bipe(int freq, int dur) { tone(SPEAKER_PIN, freq, dur); }
void bipeOk()   { bipe(2000, 120); }
void bipePad()  { bipe(2600, 90); }
void bipeErro() { bipe(500, 250); }
void bipeFim()  { bipe(2500, 250); delay(300); bipe(2500, 250); }

void envia(const String& s) {
  SerialBT.println(s);
  Serial.println(s);
}

bool tempValida(float t) {
  if (isnan(t)) return false;
  if (t < 5.0 || t > 45.0) return false;
  return true;
}

// mediana de um vetor de int (usa copia, ordena por insercao)
int mediana(int* v, int n) {
  if (n <= 0) return 0;
  int tmp[64];
  if (n > 64) n = 64;
  for (int i = 0; i < n; i++) tmp[i] = v[i];
  for (int i = 1; i < n; i++) {
    int k = tmp[i], j = i - 1;
    while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = k;
  }
  return tmp[n / 2];
}

// ============================================================
// ETAPA 1 - BATIMENTOS + OXIGENACAO (30 s)
// ============================================================
// Como funciona o algoritmo maxim:
//   - precisa de 100 amostras pra primeira conta (~4 s a 25 Hz)
//   - depois desliza a janela: joga fora 25 amostras antigas,
//     pega 25 novas (1 s) e recalcula
//   - por isso o primeiro resultado so aparece depois de ~4 s
// ============================================================
uint32_t irBuffer[100];
uint32_t redBuffer[100];

void medirBpmSpo2() {
  envia("{\"etapa\":\"bpm\",\"status\":\"iniciando\",\"total\":30}");
  bipeOk();

  // Configuracao boa pra SpO2 (diferente da configuracao so de BPM)
  byte ledBrightness = 60;   // 0 = desligado ... 255 = 50 mA
  byte sampleAverage = 4;
  byte ledMode       = 2;    // 2 = vermelho + infravermelho (precisa dos 2)
  byte sampleRate    = 100;  // 100 Hz / media de 4 = 25 amostras por segundo
  int  pulseWidth    = 411;
  int  adcRange      = 4096;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode,
                       sampleRate, pulseWidth, adcRange);

  int32_t spo2 = 0, heartRate = 0;
  int8_t  spo2Valido = 0, hrValido = 0;

  int listaSpo2[64]; int nSpo2 = 0;
  int listaBpm[64];  int nBpm  = 0;

  unsigned long inicio = millis();
  bool dedoPresente = false;

  // --- 1) enche as 100 primeiras amostras ---
  for (byte i = 0; i < 100; i++) {
    while (particleSensor.available() == false) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer,
                                         &spo2, &spo2Valido,
                                         &heartRate, &hrValido);

  // --- 2) janela deslizante ate fechar os 30 s ---
  while (millis() - inicio < T_BPM_SPO2) {
    // descarta as 25 mais antigas
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25]  = irBuffer[i];
    }
    // pega 25 novas (1 segundo)
    for (byte i = 75; i < 100; i++) {
      while (particleSensor.available() == false) particleSensor.check();
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i]  = particleSensor.getIR();
      particleSensor.nextSample();
    }

    dedoPresente = (irBuffer[99] > 50000);

    maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer,
                                           &spo2, &spo2Valido,
                                           &heartRate, &hrValido);

    if (dedoPresente) {
      if (spo2Valido && spo2 > 70 && spo2 <= 100 && nSpo2 < 64)
        listaSpo2[nSpo2++] = (int)spo2;
      if (hrValido && heartRate > 30 && heartRate < 220 && nBpm < 64)
        listaBpm[nBpm++] = (int)heartRate;
    }

    int segRestantes = (int)((T_BPM_SPO2 - (millis() - inicio)) / 1000);
    if (segRestantes < 0) segRestantes = 0;

    String msg = "{\"etapa\":\"bpm\",\"status\":\"medindo\"";
    msg += ",\"valor\":"  + String(hrValido   && dedoPresente ? (int)heartRate : 0);
    msg += ",\"spo2\":"   + String(spo2Valido && dedoPresente ? (int)spo2      : 0);
    msg += ",\"resta\":"  + String(segRestantes);
    msg += ",\"dedo\":"   + String(dedoPresente ? "true" : "false");
    msg += ",\"amostras\":" + String(nBpm) + "}";
    envia(msg);
  }

  resultadoBPM  = mediana(listaBpm,  nBpm);
  resultadoSpO2 = mediana(listaSpo2, nSpo2);

  // Qualidade: com poucas amostras validas o valor nao vale muito
  String qualidade = "boa";
  if (nBpm < 5 || nSpo2 < 5) qualidade = "ruim";
  else if (nBpm < 12 || nSpo2 < 12) qualidade = "media";

  if (qualidade == "ruim") bipeErro(); else bipeOk();

  String fim = "{\"etapa\":\"bpm\",\"status\":\"ok\"";
  fim += ",\"valor\":" + String(resultadoBPM);
  fim += ",\"spo2\":"  + String(resultadoSpO2);
  fim += ",\"qualidade\":\"" + qualidade + "\"";
  fim += ",\"amostras\":" + String(nBpm) + "}";
  envia(fim);

  // volta o sensor pro modo economico
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
}

// ============================================================
// ETAPA 2 - TEMPERATURA DA TESTA (5 s, modo MAX HOLD)
// ============================================================
// Em 5 s voce varre da sobrancelha ate a linha do cabelo.
// O firmware guarda o MAIOR valor valido - que e o ponto mais
// perto da arteria temporal. Media nao serve aqui: varreu por
// cima do cabelo, a media desaba.
// ============================================================
void medirTempTesta() {
  envia("{\"etapa\":\"tt\",\"status\":\"iniciando\",\"total\":5}");
  bipeOk();

  unsigned long inicio = millis();
  unsigned long ultimoAviso = 0;
  float maxLido = 0;
  float amb = 0;
  int nLeituras = 0;

  while (millis() - inicio < T_TESTA) {
    float leitura = mlx.readObjectTempC();
    float a = mlx.readAmbientTempC();
    if (tempValida(leitura)) {
      if (leitura > maxLido) maxLido = leitura;
      nLeituras++;
    }
    if (!isnan(a)) amb = a;

    if (millis() - ultimoAviso >= 500) {
      ultimoAviso = millis();
      int segRestantes = (int)((T_TESTA - (millis() - inicio)) / 1000) + 1;
      String msg = "{\"etapa\":\"tt\",\"status\":\"medindo\",\"valor\":";
      msg += String(maxLido, 1);
      msg += ",\"resta\":" + String(segRestantes) + "}";
      envia(msg);
    }
    delay(120);
  }

  resultadoTT   = maxLido;
  ambienteTesta = amb;

  if (nLeituras < 3 || maxLido < 28.0) bipeErro(); else bipeOk();

  String fim = "{\"etapa\":\"tt\",\"status\":\"ok\",\"valor\":";
  fim += String(resultadoTT, 1);
  fim += ",\"ambiente\":" + String(ambienteTesta, 1);
  fim += ",\"leituras\":" + String(nLeituras) + "}";
  envia(fim);
}

// ============================================================
// ETAPA 3 - TEMPERATURA AXILAR (ate 90 s, para no platô)
// ============================================================
void medirTempAxilar() {
  envia("{\"etapa\":\"ta\",\"status\":\"iniciando\",\"total\":90}");
  bipeOk();

  unsigned long inicio = millis();
  unsigned long ultimoAviso = 0;
  unsigned long marcoPlato = millis();

  float ultimaBoa = 0;
  float refPlato  = -100;
  axilarEstabilizou = false;

  sensors.requestTemperatures();

  while (millis() - inicio < T_AXILAR) {
    float leitura = sensors.getTempCByIndex(0);
    if (leitura != DEVICE_DISCONNECTED_C && tempValida(leitura)) {
      ultimaBoa = leitura;
    }
    sensors.requestTemperatures();

    // --- deteccao de platô ---
    if (ultimaBoa > 0) {
      if (refPlato < -50) { refPlato = ultimaBoa; marcoPlato = millis(); }
      if (fabs(ultimaBoa - refPlato) > PLATO_DELTA) {
        refPlato = ultimaBoa;
        marcoPlato = millis();
      } else if (millis() - marcoPlato >= PLATO_JANELA &&
                 millis() - inicio >= T_AXILAR_MIN) {
        axilarEstabilizou = true;
        break;
      }
    }

    if (millis() - ultimoAviso >= 1000) {
      ultimoAviso = millis();
      int segRestantes = (int)((T_AXILAR - (millis() - inicio)) / 1000);
      int segEstavel   = (int)((millis() - marcoPlato) / 1000);
      String msg = "{\"etapa\":\"ta\",\"status\":\"medindo\",\"valor\":";
      msg += String(ultimaBoa, 2);
      msg += ",\"resta\":" + String(segRestantes);
      msg += ",\"estavel\":" + String(segEstavel) + "}";
      envia(msg);
    }
    delay(300);
  }

  resultadoTA = ultimaBoa;
  bipeFim();

  String fim = "{\"etapa\":\"ta\",\"status\":\"ok\",\"valor\":";
  fim += String(resultadoTA, 2);
  fim += ",\"estabilizou\":" + String(axilarEstabilizou ? "true" : "false");
  fim += ",\"tempo\":" + String((int)((millis() - inicio) / 1000)) + "}";
  envia(fim);
}

// ============================================================
// LEITURA DE COR
// ============================================================
// 1. apaga o LED, mede o "escuro" (luz ambiente que vazou)
// 2. acende o LED, mede a cor
// 3. subtrai o escuro
// 4. divide pelo branco de referencia (balanco de branco)
// ============================================================
void ledTcs(bool ligado) {
#if TCS_LED_PIN >= 0
  digitalWrite(TCS_LED_PIN, ligado ? HIGH : LOW);
  delay(ligado ? 60 : 30);
#endif
}

void lerCruAcumulado(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c,
                     int amostras) {
  long sR = 0, sG = 0, sB = 0, sC = 0;
  uint16_t rr, gg, bb, cc;
  for (int i = 0; i < amostras; i++) {
    tcs.getRawData(&rr, &gg, &bb, &cc);
    sR += rr; sG += gg; sB += bb; sC += cc;
  }
  r = sR / amostras; g = sG / amostras; b = sB / amostras; c = sC / amostras;
}

/// Le uma amostra ja com escuro subtraido. Devolve valores crus do ADC.
void lerCorLiquida(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  uint16_t dR = 0, dG = 0, dB = 0, dC = 0;

#if TCS_LED_PIN >= 0
  ledTcs(false);
  lerCruAcumulado(dR, dG, dB, dC, 3);
#endif

  ledTcs(true);
  uint16_t lR, lG, lB, lC;
  lerCruAcumulado(lR, lG, lB, lC, 6);
  ledTcs(false);

  r = (lR > dR) ? (lR - dR) : 0;
  g = (lG > dG) ? (lG - dG) : 0;
  b = (lB > dB) ? (lB - dB) : 0;
  c = (lC > dC) ? (lC - dC) : 0;
}

/// Converte cru -> 0..255 usando o branco de referencia (se existir).
void corParaRgb(uint16_t r, uint16_t g, uint16_t b, uint16_t c,
                int &R, int &G, int &B) {
  if (temBranco && brancoR > 20 && brancoG > 20 && brancoB > 20) {
    // Balanco de branco: o branco vira 245 em cada canal
    long vr = (long)r * 245 / brancoR;
    long vg = (long)g * 245 / brancoG;
    long vb = (long)b * 245 / brancoB;
    R = constrain((int)vr, 0, 255);
    G = constrain((int)vg, 0, 255);
    B = constrain((int)vb, 0, 255);
  } else {
    // Sem branco: normaliza pelo canal "clear" (jeito antigo)
    uint16_t cc = (c == 0) ? 1 : c;
    R = constrain((int)((long)r * 255 / cc), 0, 255);
    G = constrain((int)((long)g * 255 / cc), 0, 255);
    B = constrain((int)((long)b * 255 / cc), 0, 255);
  }
}

/// Comando LERCOR: uma leitura avulsa (usada pra calibrar o app).
void comandoLerCor() {
  if (!temTCS) { envia("{\"etapa\":\"cor\",\"status\":\"erro\"}"); return; }
  uint16_t r, g, b, c;
  lerCorLiquida(r, g, b, c);
  int R, G, B;
  corParaRgb(r, g, b, c, R, G, B);
  String m = "{\"etapa\":\"cor\",\"status\":\"ok\"";
  m += ",\"r\":" + String(R) + ",\"g\":" + String(G) + ",\"b\":" + String(B);
  m += ",\"rc\":" + String(r) + ",\"gc\":" + String(g) + ",\"bc\":" + String(b);
  m += ",\"clear\":" + String(c) + "}";
  envia(m);
  bipePad();
}

/// Comando BRANCO: grava a pastilha branca do cartucho como referencia.
void comandoBranco() {
  if (!temTCS) { envia("{\"etapa\":\"branco\",\"status\":\"erro\"}"); return; }
  uint16_t r, g, b, c;
  lerCorLiquida(r, g, b, c);
  if (c < 80) {
    temBranco = false;
    envia("{\"etapa\":\"branco\",\"status\":\"escuro\"}");
    bipeErro();
    return;
  }
  brancoR = r; brancoG = g; brancoB = b; brancoC = c;
  temBranco = true;
  String m = "{\"etapa\":\"branco\",\"status\":\"ok\",\"clear\":";
  m += String(c) + "}";
  envia(m);
  bipeOk();
}

// ============================================================
// ESPERA UM CLIQUE DO MECANISMO
// A chave do mecanismo fecha pra GND toda vez que a lingueta cai
// dentro de um dente. Debounce simples de 40 ms.
// Devolve false se estourar o tempo.
// ============================================================
bool esperaClique(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  // primeiro garante que a chave esta solta
  while (digitalRead(CLIQUE_PIN) == LOW) {
    if (millis() - t0 > timeoutMs) return false;
    delay(5);
  }
  // agora espera fechar
  while (true) {
    if (digitalRead(CLIQUE_PIN) == LOW) {
      delay(40);
      if (digitalRead(CLIQUE_PIN) == LOW) return true;
    }
    // o app pode mandar LERPAD pra forcar (caso a chave falhe)
    if (SerialBT.available()) {
      String c = SerialBT.readStringUntil('\n');
      c.trim();
      if (c == "LERPAD") return true;
      if (c == "CANCELAR") return false;
    }
    if (millis() - t0 > timeoutMs) return false;
    delay(8);
  }
}

// ============================================================
// ETAPA 4 - URINALISE
// ============================================================
void fazerUrinalise() {
  if (!temTCS) {
    envia("{\"etapa\":\"urina\",\"status\":\"erro\",\"msg\":\"sensor de cor nao encontrado\"}");
    bipeErro();
    return;
  }

  envia("{\"etapa\":\"urina\",\"status\":\"iniciando\",\"pads\":10}");
  bipeOk();

  // --- Reacao da fita ---
  unsigned long t0 = millis();
  while (millis() - t0 < T_REACAO) {
    int resta = (int)((T_REACAO - (millis() - t0)) / 1000);
    envia("{\"etapa\":\"urina\",\"status\":\"reagindo\",\"resta\":" +
          String(resta) + "}");
    delay(2000);
  }
  bipeFim();

  // --- 11 cliques: 10 quadradinhos + a pastilha branca no fim ---
  // A pastilha branca fica DEPOIS do decimo quadradinho no cartucho,
  // entao ela e lida por ultimo. So no fim da pra aplicar o balanco
  // de branco - por isso as cores vao duas vezes: uma provisoria (pro
  // app mostrar progresso) e a definitiva no fim.
  uint16_t cruR[N_PADS + 1], cruG[N_PADS + 1];
  uint16_t cruB[N_PADS + 1], cruC[N_PADS + 1];
  temBranco = false;

  for (int q = 0; q <= N_PADS; q++) {
    bool ehBranco = (q == N_PADS);

    String aviso = "{\"etapa\":\"urina\",\"status\":\"aguardando\",\"quad\":" +
                   String(q + 1);
    aviso += ",\"nome\":\"" + String(ehBranco ? "Pastilha branca"
                                              : nomesUrina[q]) + "\"";
    aviso += ",\"branco\":" + String(ehBranco ? "true" : "false") + "}";
    envia(aviso);

    if (!esperaClique(60000UL)) {
      envia("{\"etapa\":\"urina\",\"status\":\"timeout\",\"quad\":" +
            String(q + 1) + "}");
      bipeErro();
      return;
    }

    delay(250);  // deixa o cartucho assentar no dente
    lerCorLiquida(cruR[q], cruG[q], cruB[q], cruC[q]);

    if (ehBranco) {
      if (cruC[q] < 80) {
        envia("{\"etapa\":\"urina\",\"status\":\"branco_escuro\"}");
        bipeErro();
      } else {
        brancoR = cruR[q]; brancoG = cruG[q];
        brancoB = cruB[q]; brancoC = cruC[q];
        temBranco = true;
        envia("{\"etapa\":\"urina\",\"status\":\"branco\",\"clear\":" +
              String(cruC[q]) + "}");
        bipeOk();
      }
    } else {
      // cor provisoria (normalizada pelo canal clear) so pro app
      // mostrar o quadradinho na hora
      int R, G, B;
      corParaRgb(cruR[q], cruG[q], cruB[q], cruC[q], R, G, B);
      padRc[q] = cruR[q]; padGc[q] = cruG[q]; padBc[q] = cruB[q];

      String msg = "{\"etapa\":\"urina\",\"status\":\"lido\",\"quad\":" +
                   String(q + 1);
      msg += ",\"nome\":\"" + String(nomesUrina[q]) + "\"";
      msg += ",\"r\":" + String(R) + ",\"g\":" + String(G) + ",\"b\":" + String(B);
      msg += ",\"clear\":" + String(cruC[q]) + "}";
      envia(msg);
      bipePad();
    }
  }

  // --- agora sim: aplica o balanco de branco nos 10 quadradinhos ---
  String fim = "{\"etapa\":\"urina\",\"status\":\"corrigido\",\"pads\":[";
  for (int q = 0; q < N_PADS; q++) {
    int R, G, B;
    corParaRgb(cruR[q], cruG[q], cruB[q], cruC[q], R, G, B);
    padR[q] = R; padG[q] = G; padB[q] = B;
    fim += "{\"r\":" + String(R) + ",\"g\":" + String(G) +
           ",\"b\":" + String(B) + "}";
    if (q < N_PADS - 1) fim += ",";
  }
  fim += "],\"branco\":" + String(temBranco ? "true" : "false") + "}";
  envia(fim);

  temResultadoUrina = true;
  bipeFim();
  envia("{\"etapa\":\"urina\",\"status\":\"ok\"}");
}

// ============================================================
// TRIAGEM FINAL
// Manda os numeros crus. Quem decide verde/amarelo/vermelho e o
// app (assim da pra ajustar a regra sem regravar o ESP32).
// ============================================================
void enviarTriagemFinal() {
  String json = "{\"etapa\":\"final\"";
  json += ",\"bpm\":"  + String(resultadoBPM);
  json += ",\"spo2\":" + String(resultadoSpO2);
  json += ",\"tt\":"   + String(resultadoTT, 1);
  json += ",\"ta\":"   + String(resultadoTA, 2);
  json += ",\"ambiente\":" + String(ambienteTesta, 1);
  json += ",\"axilar_estavel\":" + String(axilarEstabilizou ? "true" : "false");

  if (temResultadoUrina) {
    json += ",\"urina\":[";
    for (int i = 0; i < N_PADS; i++) {
      json += "{\"nome\":\"" + String(nomesUrina[i]) + "\"";
      json += ",\"r\":" + String(padR[i]);
      json += ",\"g\":" + String(padG[i]);
      json += ",\"b\":" + String(padB[i]) + "}";
      if (i < N_PADS - 1) json += ",";
    }
    json += "]";
  }
  json += "}";

  envia(json);
  bipeFim();
}

void resetarResultados() {
  resultadoBPM = 0; resultadoSpO2 = 0;
  resultadoTT = 0;  resultadoTA = 0;
  ambienteTesta = 0; axilarEstabilizou = false;
  temResultadoUrina = false;
  temBranco = false;
  for (int i = 0; i < N_PADS; i++) {
    padR[i] = padG[i] = padB[i] = 0;
    padRc[i] = padGc[i] = padBc[i] = 0;
  }
  envia("{\"etapa\":\"reset\",\"status\":\"ok\"}");
  bipeOk();
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== VitaCheck v2 ===");

  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(CLIQUE_PIN, INPUT_PULLUP);
#if TCS_LED_PIN >= 0
  pinMode(TCS_LED_PIN, OUTPUT);
  digitalWrite(TCS_LED_PIN, LOW);
#endif

  Wire.begin();
  Wire.setClock(100000);

  if (!mlx.begin()) { Serial.println("ERRO MLX90614!"); }
  else Serial.println("MLX90614 OK");

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("ERRO MAX30102!");
  } else {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("MAX30102 OK");
  }

  temTCS = tcs.begin();
  Serial.println(temTCS ? "TCS34725 OK" : "AVISO: TCS34725 nao encontrado");

  sensors.begin();
  sensors.setResolution(12);           // 0.0625 C de resolucao
  sensors.setWaitForConversion(false);
  temDS = (sensors.getDeviceCount() > 0);
  Serial.println(temDS ? "DS18B20 OK" : "AVISO: DS18B20 nao encontrado");
  if (temDS) sensors.requestTemperatures();

  bipe(1000, 150); delay(200); bipe(1500, 150);

  SerialBT.begin("VitaCheck");
  Serial.println("Bluetooth 'VitaCheck' no ar.");
  Serial.println("Comandos: MEDIR1 MEDIR2 MEDIR3 MEDIR4 LERCOR BRANCO TRIAGEM RESET");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  String comando = "";

  if (SerialBT.available())      comando = SerialBT.readStringUntil('\n');
  else if (Serial.available())   comando = Serial.readStringUntil('\n');

  comando.trim();
  if (comando.length() == 0) return;

  Serial.print("Comando: "); Serial.println(comando);

  if      (comando == "MEDIR1")  medirBpmSpo2();
  else if (comando == "MEDIR2")  medirTempTesta();
  else if (comando == "MEDIR3")  medirTempAxilar();
  else if (comando == "MEDIR4")  fazerUrinalise();
  else if (comando == "LERCOR")  comandoLerCor();
  else if (comando == "BRANCO")  comandoBranco();
  else if (comando == "TRIAGEM") enviarTriagemFinal();
  else if (comando == "RESET")   resetarResultados();
  else if (comando == "PING")    envia("{\"etapa\":\"ping\",\"status\":\"ok\"}");
  else envia("{\"erro\":\"comando desconhecido\"}");
}
