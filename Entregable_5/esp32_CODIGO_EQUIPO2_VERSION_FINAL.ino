/*
  ============================================================================
  Monitor Cardiovascular — ESP32 + MAX30102 + MPU6050
  ============================================================================
  Diferencia clave entre MAX30100 y MAX30102:
    - MAX30102 tiene LEDs rojo + infrarrojo (igual que el MAX30100).
    - La librería recomendada para MAX30102 es la de SparkFun
      ("SparkFun MAX3010x Pulse and Proximity Sensor Library"),
      NO la "MAX30100lib" que usamos antes (esa era solo para MAX30100).
    - Los pines de conexión son los mismos (I2C: D21=SDA, D22=SCL).
    - El protocolo BLE es IDÉNTICO — la app Flutter no cambia nada para
      lo que ya funcionaba (Heart Rate, SpO2, Status). Solo se AGREGA
      una característica nueva de movimiento; nada existente se quita
      ni se modifica.

  NOVEDAD — MPU6050 (acelerómetro/giroscopio):
    - Comparte el MISMO bus I2C que el MAX30102 (mismos pines SDA/SCL,
      sin GPIO adicionales). Cada chip responde a su propia dirección
      I2C (MAX30102 = 0x57, MPU6050 = 0x68 por defecto), así que no hay
      conflicto al compartir el bus.
    - AHORA tiene DOS propósitos:
        1) Ser la señal de REFERENCIA de un filtro adaptativo NLMS
           (Normalized Least Mean Squares) que limpia la señal IR del
           MAX30102 ANTES de buscar latidos. Esta es la arquitectura
           clásica de MA removal descrita en la literatura de PPG
           (ej. Chan & Zhang 2002; Fallet & Vesin 2015/2017): la señal
           "sucia" (PPG + artefacto de movimiento) y una referencia
           correlacionada con el ruido (aceleración) entran a un
           filtro que APRENDE en tiempo real a predecir y restar la
           parte de la señal explicada por el movimiento. Ver
           nlmsFilterStep() más abajo para el detalle.
        2) Seguir alimentando la bandera de "movimiento detectado"
           (ventana de varias muestras + umbral), que ahora SÍ llega a
           la app vía la característica BLE de movimiento — antes se
           notificaba pero la app no la escuchaba.
    - La decisión de qué hacer con la bandera de movimiento (ignorar la
      lectura, o aceptarla igual si está en Modo Ejercicio) la sigue
      tomando la app Flutter, no el firmware. El filtro NLMS reduce el
      artefacto pero no lo garantiza al 100% en movimientos intensos,
      así que la bandera sigue siendo útil como segunda capa de
      seguridad.
    - NO se usa para detectar o sugerir el Modo Ejercicio de forma
      automática — eso es 100% manual, vía un botón en la app, fuera
      del alcance de este firmware.

  Librerías que debes instalar en Arduino IDE:
    Tools > Manage Libraries > busca "SparkFun MAX3010x" >
    instala "SparkFun MAX3010x Pulse and Proximity Sensor Library"
    de SparkFun Electronics.
    El MPU6050 ya NO necesita ninguna librería adicional — se
    comunica directamente por Wire (I2C nativo del ESP32), que
    confirmamos como funcional con el I2C Scanner (0x68 respondió).

  Conexiones (ver diagrama):
    ESP32 3V3  -> MAX30102 VIN   (cable rojo)
    ESP32 3V3  -> MPU6050  VCC   (mismo punto de 3V3, compartido)
    ESP32 GND  -> MAX30102 GND   (cable negro)
    ESP32 GND  -> MPU6050  GND   (mismo punto de GND, compartido)
    ESP32 D21  -> MAX30102 SDA   (cable azul)
    ESP32 D21  -> MPU6050  SDA   (mismo cable/punto, bus compartido)
    ESP32 D22  -> MAX30102 SCL   (cable amarillo)
    ESP32 D22  -> MPU6050  SCL   (mismo cable/punto, bus compartido)
    INT del MAX30102: dejar sin conectar
    INT del MPU6050: dejar sin conectar (no se usa interrupción, se
      lee por polling igual que el MAX30102, para mantener el mismo
      patrón de diseño ya usado en este firmware)
  ============================================================================
*/

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
// NOTA: ya no se usa #include <MPU6050.h> — se accede directamente
// por Wire, que es más confiable y sin dependencias de versión de
// librería. El I2C Scanner confirmó que el MPU6050 responde en 0x68,
// así que la comunicación directa por Wire funciona perfectamente.

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------------------------------------------------------------------
// Configuración I2C — D21 y D22 son los pines físicos que ves en la placa.
// SIN CAMBIOS respecto al firmware anterior: el MPU6050 se agrega al mismo
// bus, no se tocan estos valores.
// ---------------------------------------------------------------------------
#define I2C_SDA 21
#define I2C_SCL 22

// Ventana de muestras para promediar el BPM. La librería SparkFun entrega
// un BPM calculado latido por latido; promediando los últimos N valores
// se reduce el ruido y los saltos bruscos que se ven cuando hay poca señal.
#define RATE_SIZE 4

// Si no se detecta ningún latido en este tiempo, informamos "sin señal".
#define NO_SIGNAL_TIMEOUT_MS 4000

// Umbral mínimo de brillo infrarrojo para considerar que hay un dedo
// puesto. IMPORTANTE: este valor depende directamente de ledBrightness
// y adcRange configurados en particleSensor.setup() más abajo.
// Calibrado con datos reales: sin nada puesto el IR está en ~1000,
// con el dedo puesto (a brillo medio) sube a varias decenas de miles.
// Se deja con margen amplio entre ambos casos.
#define FINGER_THRESHOLD 20000

// Cuánto tiempo entre cada reporte BLE (en milisegundos).
#define REPORTING_PERIOD_MS 1000

// Período refractario mínimo entre latidos aceptados. Con el sampleRate
// real corregido (~400 muestras/segundo, ver setup() más abajo),
// checkForBeat() a veces dispara dos "latidos" separados por muy pocos
// milisegundos — ruido de la señal cruda interpretado como doble pico,
// nunca un latido real (imposible fisiológicamente por debajo de este
// límite, que corresponde a un generoso máximo de 200 BPM). Estos
// disparos espurios se ignoran por completo antes de tocar lastBeat.
#define MIN_BEAT_INTERVAL_MS 300

// ---------------------------------------------------------------------------
// MPU6050 — umbral de movimiento (NUEVO, no afecta nada del MAX30102)
// ---------------------------------------------------------------------------
// Umbral calibrado con datos reales del sensor (archivo datos_recopilados.txt):
//   - Sin tocar / péndulo suave: desviación promedio 0.021, máximo 0.026
//   - Péndulo rápido: promedio 0.299, sube por encima de 0.05 al inicio
//   - Golpecitos: promedio 0.438, sube por encima de 0.07 al inicio
// Umbral de 0.08 da un margen claro sobre el ruido de reposo (máx 0.026)
// y captura tanto movimiento rápido como golpecitos desde sus primeras
// muestras, sin dispararse con movimiento suave (que da los mismos
// valores que estar completamente quieto).
#define MOTION_THRESHOLD_G 0.08

// Cuántas lecturas de movimiento seguidas se promedian antes de decidir
// "hay movimiento" — evita que un solo pico aislado (ej. un golpecito
// accidental al sensor) dispare una falsa detección de movimiento.
#define MOTION_SAMPLE_WINDOW 5

// ---------------------------------------------------------------------------
// Filtro adaptativo NLMS (ACC como referencia real) — NUEVO
// ---------------------------------------------------------------------------
// Arquitectura tomada de la Fig. 5 de Ismail et al. (2021, EURASIP J. Adv.
// Signal Process.): la señal IR "sucia" (PPG + artefacto de movimiento) y
// una señal de referencia correlacionada con el ruido (aceleración) entran
// a un filtro adaptativo que aprende a predecir la parte de la señal
// explicada por el movimiento y la resta. Usamos NLMS (Normalized LMS) en
// vez de LMS clásico porque normaliza el paso de adaptación según la
// energía de la referencia en cada instante.
#define NLMS_FILTER_ORDER 8

// AJUSTADO tras prueba real (30/07): con MU=0.6 y EPSILON=1e-3 el filtro
// se volvía inestable en reposo. Causa: en reposo la energía de la
// referencia (ACC) es muy baja (~0.0035, calibrada con datos reales:
// desviación promedio 0.021g sobre 8 muestras), y con esos valores el
// paso efectivo mu/(epsilon+energia) llegaba a ~133 — cualquier
// micro-jitter del MPU6050 se amplificaba en vez de ignorarse,
// empeorando la lectura en reposo (BPM promedio subió de ~77 a ~163 en
// prueba real). Corrección: MU mucho más bajo (menos agresivo) y
// EPSILON más alto (domina el denominador en reposo, aplastando el
// paso efectivo cuando no hay nada que corregir).
#define NLMS_MU 0.05f
#define NLMS_EPSILON 0.01f
#define IR_BASELINE_ALPHA 0.98f

// ---------------------------------------------------------------------------
// v6 — CAMBIOS DE ARQUITECTURA (no solo de parámetros)
// ---------------------------------------------------------------------------
// Diagnóstico tras revisar todo el historial v0-v5 (ver RESUMEN_CardioNet...):
//
// 1) PROBLEMA: usar la MAGNITUD del acelerómetro (sqrt(ax²+ay²+az²)-1g)
//    como referencia rompe el supuesto básico de NLMS/LMS: que la
//    referencia se relaciona LINEALMENTE con el artefacto en el IR. La
//    raíz cuadrada es no lineal y además destruye la dirección del
//    movimiento — el filtro nunca puede aprender bien una relación que
//    ya fue distorsionada antes de dársela. SOLUCIÓN v6: usar los 3 ejes
//    (ax, ay, az) por separado, cada uno con su propia línea de retardo
//    y sus propios pesos ("multi-referencia", igual de válido dentro de
//    la misma arquitectura de Ismail et al. 2021 / Chan & Zhang 2002 —
//    ambos permiten múltiples canales de referencia).
//
// 2) PROBLEMA: el filtro se adapta SIEMPRE, incluso en reposo total. En
//    v1 esto amplificó ruido de cuantización del MPU6050 (Avg subió a
//    163 en reposo). v2-v5 lo "arreglaron" subiendo EPSILON de forma
//    global, pero eso también atenúa la respuesta real cuando SÍ hay
//    movimiento — es un parche, no una solución. SOLUCIÓN v6: compuerta
//    de adaptación (gating). Los pesos SOLO se actualizan cuando la
//    energía instantánea de aceleración supera un umbral por encima del
//    ruido de reposo ya calibrado (reposo: prom 0.021g, máx 0.026g →
//    umbral de compuerta 0.035g, con margen). Fuera de eso, se aplica
//    "leaky NLMS" (los pesos decaen suavemente hacia 0 en vez de
//    congelarse en seco), para que una ráfaga de adaptación durante un
//    movimiento no deje una "cicatriz" de pesos que distorsione la señal
//    minutos después en reposo.
//
// 3) NO RESUELTO POR NINGÚN FILTRO BASADO EN ACELERÓMETRO: el hallazgo
//    #4 de la sesión anterior (Avg oscilando 67-148 con IR de rango
//    1513, PERFECTAMENTE estable, sin ningún filtro nuestro de por
//    medio) confirma que buena parte del ruido observado NO es
//    artefacto de movimiento — es inestabilidad propia de la detección
//    de picos de checkForBeat(). Por eso v6 agrega un filtro de MEDIANA
//    (igual que el explorado en esp32_max30102_mpu6050_ble_MEDIANA.ino)
//    aplicado DESPUÉS del NLMS: el NLMS ataca el componente
//    correlacionado con el movimiento real; la mediana limpia los picos
//    aislados que quedan y que el NLMS, al ser lineal, nunca podría
//    explicar ni restar. Son complementarios, no alternativas.
#define NLMS_NUM_AXES 3
// v7: fuga más rápida (antes 0.999, vida media ~1.7s a 400Hz). Se
// detectó en prueba real (muñeca, reposo) un diff IR/IRfilt de 627
// ocurriendo YA con "Movimiento: no" — resto de adaptación de un
// evento de movimiento anterior que todavía no había decaído del
// todo. Con 0.995, vida media ~0.35s: los pesos vuelven a cero mucho
// más rápido una vez que el movimiento real termina.
#define NLMS_LEAK 0.995f

// v10 — RECALIBRADO con datos reales de muñeca (v9, dos pruebas:
// "mediciones_estaticas_quieto_movimiento[_2].txt"). El umbral 0.035
// (calibrado en la sesión original con el sensor sobre una mesa, sin
// pulso ni micro-temblor de sostener el brazo) resultó estar DENTRO del
// rango normal de reposo puesto de verdad: AccDevG en reposo real midió
// 0.034-0.072 (media ~0.04) en ambas pruebas — la compuerta casi nunca
// se cerraba, ni en reposo. Se sube a 0.08 (mismo valor ya validado
// para MOTION_THRESHOLD_G, con buen margen sobre el máximo de reposo
// observado en ambas pruebas: 0.052 y 0.072).
#define NLMS_GATE_THRESHOLD_G 0.08f

// v10 — NUEVA: compuerta ÓPTICA, independiente del acelerómetro.
// Evidencia real (mismas dos pruebas): se confirmaron 3 instantes con
// AccDevG < 0.011 (acelerómetro dice "sin movimiento") pero con
// artefacto óptico real y grande (IRacMax 773-1925, hasta 10x el nivel
// de reposo normal ~150-400). Esto prueba que el acelerómetro NO
// siempre ve el disturbio que sí aparece en el IR — ningún umbral de
// aceleración, por bien calibrado que esté, puede cerrar esa brecha.
// irAcFastPeak (ver más abajo) es un "peak-hold con decaimiento": sube
// de inmediato ante cualquier disturbio óptico y decae solo, sin
// necesidad de acumular una ventana completa antes de reaccionar (a
// diferencia de irAcMaxSinceReport, que solo se conoce al final de
// cada segundo — demasiado tarde para usarlo como compuerta en tiempo
// real). Los umbrales son PROVISIONALES: se calibraron con el único
// dato disponible (IRacMax por segundo, no por muestra), así que se
// sigue imprimiendo irAcFastPeak en el log para refinarlos con datos
// reales del próximo test, igual que ya se hizo con AccDevG.
#define OPTICAL_ADAPT_THRESHOLD 500.0f    // por encima de esto, se asume disturbio óptico real
#define OPTICAL_FREEZE_THRESHOLD 1500.0f  // por encima de esto, no se acepta ningún latido nuevo
#define OPTICAL_PEAK_DECAY 0.995f         // mismo ritmo de decaimiento que NLMS_LEAK, por consistencia

#define ACC_BASELINE_ALPHA 0.98f
#define MEDIAN_WINDOW 5


// v7 — NUEVO: filtro de plausibilidad sobre el BPM.
// v8 — CORREGIDO: la versión v7 comparaba cada latido nuevo contra
// beatAvg (el promedio acumulado de los últimos 4 aceptados). BUG
// CONFIRMADO en prueba real: justo al reconectar el dedo, los primeros
// latidos entran SIN chequeo (buffer vacío, no hay "tendencia" contra
// qué comparar) — y esos primeros latidos son justo los MENOS
// confiables (sensor recién asentándose). Si esos primeros 4 eran
// basura (ej. 180+ BPM), el promedio quedaba anclado ahí, y como el
// filtro solo aceptaba valores parecidos al promedio ACTUAL, nunca más
// dejaba entrar un latido real de 70-80 BPM para corregirlo — quedaba
// atascado hasta la próxima desconexión (visto en log real: 120 → 159
// → 179 → 189 en reconexiones sucesivas, cada vez peor).
// FIX: comparar cada latido nuevo contra el LATIDO CRUDO INMEDIATAMENTE
// ANTERIOR (no contra un promedio acumulado). Dos lecturas
// INDEPENDIENTES que coinciden entre sí son buena evidencia de que
// ambas son reales — y como no depende de ningún estado acumulado, no
// hay ancla mala posible: si dos latidos no coinciden, simplemente se
// recuerda el más reciente como candidato para comparar con el
// siguiente, y en cuanto aparecen dos latidos reales consecutivos
// (aunque sea después de varios espurios), el sistema se recupera solo.
#define BPM_PLAUSIBILITY_MAX_REL_DEVIATION 0.35f
#define BPM_PLAUSIBILITY_MIN_ABS_DEVIATION 20.0f

// ---------------------------------------------------------------------------
// UUIDs BLE — idénticos al firmware anterior, la app no cambia nada
// ---------------------------------------------------------------------------
#define HEART_RATE_SERVICE_UUID     "0000180d-0000-1000-8000-00805f9b34fb"
#define HEART_RATE_MEASUREMENT_UUID "00002a37-0000-1000-8000-00805f9b34fb"
#define CUSTOM_VITALS_SERVICE_UUID  "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CUSTOM_SPO2_CHAR_UUID       "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CUSTOM_STATUS_CHAR_UUID     "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
// NUEVO: característica de movimiento, en el MISMO servicio "vitals" que
// ya existía (6e400001-...) — se agrega una característica más dentro de
// ese servicio, no un servicio nuevo, así la app no necesita descubrir
// servicios distintos para encontrarla, solo suscribirse a una
// característica más dentro de un servicio que ya conocía.
#define CUSTOM_MOTION_CHAR_UUID     "6e400004-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_DEVICE_NAME             "CardioMonitor-ESP32"

MAX30105 particleSensor;
// Dirección I2C del MPU6050 confirmada por I2C Scanner: 0x68 (AD0=GND)
#define MPU6050_ADDR 0x68

// Buffer circular para promediar los últimos RATE_SIZE valores de BPM.
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

// Cuántas muestras válidas van acumuladas en el buffer 'rates'. Antes,
// con menos de RATE_SIZE muestras reales, el promedio ya se reportaba
// como válido (mezclando ceros iniciales del buffer con datos reales)
// — eso da un BPM con mucho ruido justo al empezar a medir, más notorio
// en la muñeca por la señal más débil que en el dedo. Ahora se exige
// el buffer completo antes de notificar nada.
byte validSamples = 0;

// v8 — estado del filtro de plausibilidad por CONSISTENCIA CONSECUTIVA
// (reemplaza la comparación contra beatAvg de v7, que podía quedar
// anclada en un valor malo — ver nota de arquitectura arriba). Guarda
// el último BPM crudo visto (haya sido aceptado o no), para comparar
// contra el próximo.
float lastCandidateBpm = 0.0f;
bool  haveLastCandidate = false;

// v12 — NUEVO: estado del ancla anti-deriva. Evidencia real (v11, log
// latido por latido): la secuencia 143.5 → 177.0 → 140.8 → 185.8 pasó
// el chequeo de v8 en CADA paso (cada uno está a menos del 35% del
// anterior), pero la cadena completa se alejó muy lejos de la
// frecuencia real (161-172 vs. 70-80 real) — una DERIVA GRADUAL, no un
// salto brusco. v8 protege contra saltos de una vez, no contra una
// "caminata" lenta en la misma dirección. Cuando un latido se aleja
// mucho del promedio YA ESTABLECIDO (no del vecino inmediato), se exige
// que varias lecturas seguidas coincidan con el MISMO valor ancla,
// antes de aceptar el nuevo régimen — esto sí detiene la deriva, porque
// cada paso de la caminata (177, 140.8, 185.8...) no coincide entre sí
// dentro de una tolerancia más estricta (20%) contra un ancla fija.
float pendingJumpAnchorBpm = 0.0f;
byte  pendingJumpConfirmCount = 0;

// v12 (corrección tras simular el diseño): comparar el salto contra
// beatAvg NO sirve — beatAvg se deja arrastrar por la misma deriva que
// se quiere detectar (simulación confirmó: para cuando beatAvg se aleja
// lo suficiente para disparar "salto grande", la deriva ya avanzó
// varios pasos). Se necesita un ancla LENTA, separada del promedio
// rápido que se reporta: solo se mueve un poco con cada latido normal
// aceptado, así que un salto real destaca mucho más rápido contra ella.
float trustedBpm = 0.0f;
bool  trustedBpmInitialized = false;
#define TRUST_ANCHOR_ALPHA 0.9f  // cada latido normal aceptado solo mueve el ancla 10%

#define BIG_JUMP_MIN_ABS_DEVIATION 30.0f     // por debajo de esto, no se considera "salto grande"
#define BIG_JUMP_MIN_REL_DEVIATION 0.40f     // 40% respecto al promedio ya establecido
#define JUMP_ANCHOR_MAX_REL_DEVIATION 0.20f  // tolerancia más estricta contra el ancla fija
#define JUMP_ANCHOR_MIN_ABS_DEVIATION 20.0f
// v13 probó bajar esto a 2 — RESULTADO NEGATIVO, confirmado con datos
// reales: dos artefactos que por casualidad se parecen entre sí (ej.
// 188.1 y 191.7, probablemente dobles-detecciones del mismo pico) son
// mucho más comunes que TRES coincidiendo. Con 2 confirmaciones, ese
// par falso quedó "confirmado" y el Avg saltó de 90 a 132, quedándose
// ahí el resto de la prueba (132→144→149→162→149, sin volver a bajar) —
// con AccDevG normal todo el tiempo, o sea sin movimiento real de por
// medio. Se vuelve a 3: más lento para asentarse tras una interrupción,
// pero bajar la exigencia resultó hacer MÁS fácil quedar atrapado en un
// par de artefactos, no más rápido en corregirse.
#define JUMP_CONFIRMATIONS_REQUIRED 3        // cuántas lecturas seguidas deben coincidir con el ancla

// Hueco corto sin detectar un latido (normal, esporádico, no implica
// pérdida de contacto): se ignora, se sigue mostrando el último
// promedio válido sin reiniciar el conteo.
#define STALE_BEAT_TIMEOUT_MS 3000

// Hueco LARGO sin ningún latido: aquí sí asumimos pérdida de contacto
// real (la muñequera se aflojó, el dedo se levantó, etc.) y se reinicia
// el conteo desde cero.
#define LONG_STALE_TIMEOUT_MS 8000

BLEServer          *pServer        = nullptr;
BLECharacteristic  *pHeartRateChar = nullptr;
BLECharacteristic  *pSpo2Char      = nullptr;
BLECharacteristic  *pStatusChar    = nullptr;
BLECharacteristic  *pMotionChar    = nullptr;  // NUEVO

bool deviceConnected    = false;
uint32_t tsLastReport   = 0;

// v9 — NUEVO: diagnóstico real de volatilidad óptica, independiente del
// acelerómetro. En la prueba de "mover la mano" se confirmó con datos:
// el rango del IR crudo se multiplicó por 4.5x (2141 → 9778) durante el
// movimiento, pero la bandera "Movimiento" (basada en el MPU6050) solo
// se activó el 11% del tiempo — el acelerómetro casi no está viendo
// este tipo de disturbio, aunque el sensor óptico claramente sí. Se
// guarda el máximo |irAC| visto DESDE EL ÚLTIMO REPORTE (no se puede
// calibrar un umbral todavía: el log solo tenía 1 muestra/segundo, y
// esta dinámica ocurre a ~400 muestras/segundo — se necesita este dato
// real antes de fijar cualquier número).
float irAcMaxSinceReport = 0.0f;

// v10 — peak-hold con decaimiento del componente AC del IR, actualizado
// CADA loop (no una vez por segundo como irAcMaxSinceReport). Es lo que
// permite usar la volatilidad óptica como compuerta en tiempo real. Ver
// nota de arquitectura junto a OPTICAL_ADAPT_THRESHOLD arriba.
float irAcFastPeak = 0.0f;

// --- NUEVO: estado del MPU6050 ---
bool mpuInitialized = false;
float motionSamples[MOTION_SAMPLE_WINDOW];
byte motionSampleSpot = 0;
byte motionValidSamples = 0;

// --- v6: estado del filtro NLMS multi-eje ---
// Un vector de pesos y una línea de retardo POR EJE (ax, ay, az), en vez
// de uno solo para la magnitud. [eje][tap], más reciente en tap 0.
float nlmsWeights[NLMS_NUM_AXES][NLMS_FILTER_ORDER] = {{0}};
float nlmsRefBuffer[NLMS_NUM_AXES][NLMS_FILTER_ORDER] = {{0}};
// Línea base (componente DC/lenta) de la señal IR, separada del resto
// del firmware para no interferir con el promedio usado en
// FINGER_THRESHOLD.
float irBaseline = 0.0f;
bool  irBaselineInitialized = false;
// v6: línea base por eje del acelerómetro (equivalente a restar la
// gravedad + orientación lenta de la muñeca), para que la referencia
// que entra al filtro sea la variación real de movimiento, no el valor
// absoluto (que siempre incluye ~1g repartido entre los 3 ejes según la
// orientación de la muñeca en ese momento).
float axBaseline = 0.0f, ayBaseline = 0.0f, azBaseline = 0.0f;
bool  accBaselineInitialized = false;

// --- v6: estado del filtro de mediana, aplicado DESPUÉS del NLMS ---
// Ver nota de arquitectura arriba: limpia picos aislados que el NLMS
// (lineal) no puede explicar ni restar — p.ej. la inestabilidad propia
// de checkForBeat() descrita en el hallazgo #4 del resumen de sesión.
long medianBuffer[MEDIAN_WINDOW];
byte medianSpot = 0;
byte medianValidSamples = 0;

// --- NUEVO: contador de velocidad real de loop() ---
// checkForBeat() (librería SparkFun) asume internamente ~400
// muestras/segundo para dimensionar su ventana de detección de picos.
// Si loop() corre más lento (o de forma muy irregular) que eso, el
// algoritmo puede "malinterpretar" la forma de onda incluso con señal
// perfectamente estable. Este contador nos dice, de forma directa, qué
// tan lejos estamos de esa suposición.
uint32_t loopCounter = 0;
uint32_t loopCounterLastReport = 0;

// ---------------------------------------------------------------------------
// Callbacks BLE (idénticos a versiones anteriores)
// ---------------------------------------------------------------------------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    deviceConnected = true;
    Serial.println("[BLE] Teléfono conectado.");
  }
  void onDisconnect(BLEServer *server) override {
    deviceConnected = false;
    Serial.println("[BLE] Desconectado. Reiniciando advertising...");
    delay(300);
    server->getAdvertising()->start();
  }
};

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== Monitor Cardiovascular — ESP32 + MAX30102 ===");

  Wire.begin(I2C_SDA, I2C_SCL);

  // Inicialización del MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30102 no encontrado. Revisa VIN, GND, SDA y SCL.");
    // Continuamos para que el BLE al menos arranque y la app pueda ver
    // el dispositivo, aunque le llegue estado "error de sensor".
  } else {
    Serial.println("MAX30102 inicializado correctamente.");
  }

  // Configuración del sensor:
  //   ledBrightness: con 255 (máximo) el IR se saturaba en 262143
  //     (2^18 - 1, el tope del ADC de 18 bits) apenas se ponía el dedo
  //     — demasiada luz reflejada, perdiendo toda la variación de la
  //     señal de pulso. Bajado a 60, un valor moderado típico para
  //     MAX30102 en dedo. Si con este valor el IR sigue saturándose
  //     (pegado en 262143), bajar más (ej. 30); si en cambio queda muy
  //     bajo con el dedo puesto (menos de ~15000-20000), subir un poco
  //     (ej. 90-120) — siempre revisando el log "IR:" para no tocar a
  //     ciegas.
  //   sampleAverage: promedia 4 muestras internas antes de entregar una.
  //   ledMode: 2 = usa rojo + infrarrojo (necesario para SpO2 y pulso).
  //   sampleRate/sampleAverage: CORREGIDO tras medir loops/s real dos
  //     veces. La tasa REAL de entrega de muestras no es "sampleRate"
  //     solo, es sampleRate/sampleAverage (el sensor promedia
  //     internamente "sampleAverage" lecturas antes de entregar una).
  //     Con (400, sampleAverage=4) medimos 100 loops/s reales — coincide
  //     exacto (400/4=100). Bajar sampleRate a 100 (intento anterior,
  //     erróneo) bajó la tasa real a 25/s (100/4=25), empeorando todo:
  //     el ESP32 NUNCA fue el cuello de botella, getIR() simplemente
  //     espera a que el sensor tenga lista la siguiente muestra
  //     promediada. Corrección real: mantener sampleRate=400 y bajar
  //     sampleAverage a 1 (sin promediado interno), para que la tasa
  //     real de entrega sea 400/1=400 — coincidiendo por fin con lo que
  //     checkForBeat() asume internamente. El posible costo es algo
  //     más de ruido por muestra individual (sin promediado de
  //     hardware), que el filtro NLMS y el baseline EMA ya existentes
  //     deberían poder absorber.
  particleSensor.setup(60, 1, 2, 400, 411, 16384);
  particleSensor.setPulseAmplitudeRed(0x0A);  // mantiene el LED rojo encendido
  particleSensor.setPulseAmplitudeGreen(0);   // LED verde apagado (no lo usamos)

  // --- MPU6050: inicialización directa por Wire (sin librería) ---
  // El I2C Scanner confirmó que el chip responde en 0x68. Se escribe
  // directamente al registro PWR_MGMT_1 (0x6B) con valor 0x00 para
  // sacar el chip del modo sleep (que es el estado por defecto al
  // encender). Esto es todo lo que se necesita para empezar a leer
  // aceleración — no hace falta ninguna librería para este uso básico.
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // registro PWR_MGMT_1
  Wire.write(0x00);  // 0 = salir del modo sleep, usar oscilador interno
  byte mpuError = Wire.endTransmission(true);

  if (mpuError == 0) {
    mpuInitialized = true;
    Serial.println("MPU6050 inicializado correctamente (acceso directo Wire).");
  } else {
    mpuInitialized = false;
    Serial.print("ERROR: MPU6050 no respondio al intentar inicializar. Codigo: ");
    Serial.println(mpuError);
  }

  setupBLE();
  tsLastReport = millis();
}

// ---------------------------------------------------------------------------
// Configuración BLE (igual que siempre)
// ---------------------------------------------------------------------------
void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Servicio estándar Heart Rate
  BLEService *heartService = pServer->createService(HEART_RATE_SERVICE_UUID);
  pHeartRateChar = heartService->createCharacteristic(
      HEART_RATE_MEASUREMENT_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pHeartRateChar->addDescriptor(new BLE2902());
  heartService->start();

  // Servicio custom SpO2 + estado
  BLEService *vitalsService = pServer->createService(CUSTOM_VITALS_SERVICE_UUID);
  pSpo2Char = vitalsService->createCharacteristic(
      CUSTOM_SPO2_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pSpo2Char->addDescriptor(new BLE2902());
  pStatusChar = vitalsService->createCharacteristic(
      CUSTOM_STATUS_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pStatusChar->addDescriptor(new BLE2902());

  // NUEVO: característica de movimiento, mismo servicio "vitals". No se
  // toca pSpo2Char ni pStatusChar de arriba, solo se agrega una más.
  pMotionChar = vitalsService->createCharacteristic(
      CUSTOM_MOTION_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pMotionChar->addDescriptor(new BLE2902());

  vitalsService->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(HEART_RATE_SERVICE_UUID);
  advertising->addServiceUUID(CUSTOM_VITALS_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising iniciado. Esperando conexión del teléfono...");
}

// ---------------------------------------------------------------------------
// Notificaciones BLE
// ---------------------------------------------------------------------------
void notifyHeartRate(uint8_t bpm) {
  uint8_t data[2];
  data[0] = 0x00;   // flags estándar Heart Rate Measurement
  data[1] = bpm;
  pHeartRateChar->setValue(data, 2);
  pHeartRateChar->notify();
}

void notifySpo2(uint8_t spo2) {
  uint8_t data[1] = { spo2 };
  pSpo2Char->setValue(data, 1);
  pSpo2Char->notify();
}

// 0 = OK, 1 = sin dedo / señal débil, 2 = error de sensor
void notifyStatus(uint8_t code) {
  uint8_t data[1] = { code };
  pStatusChar->setValue(data, 1);
  pStatusChar->notify();
}

// NUEVO: 0 = sin movimiento brusco, 1 = movimiento detectado (lectura de
// pulso de ese instante no es confiable salvo que se esté en Modo
// Ejercicio, decisión que toma la app, no este firmware).
void notifyMotion(uint8_t code) {
  uint8_t data[1] = { code };
  pMotionChar->setValue(data, 1);
  pMotionChar->notify();
}

// NUEVO (refactor): antes esta lectura del MPU6050 se hacía por separado
// dentro de readMotionDetected() cada vez que se necesitaba la bandera de
// movimiento. Ahora la necesitamos TAMBIÉN para el filtro NLMS en cada
// ciclo de loop(), así que se separa en una única lectura por ciclo que
// alimenta a ambos consumidores (bandera de movimiento Y filtro NLMS) —
// evita duplicar tráfico I2C y asegura que ambos usen exactamente la
// misma muestra.
//
// Devuelve por referencia axg/ayg/azg (aceleración en "g" por eje) y "ok"
// indica si la lectura fue válida. Si el MPU6050 no se inicializó o la
// lectura I2C falla, ok=false y los valores de salida no deben usarse.
void readAccelRaw(float &axg, float &ayg, float &azg, bool &ok) {
  ok = false;
  axg = ayg = azg = 0.0f;
  if (!mpuInitialized) return;

  // Lectura directa por Wire, sin librería. Los registros de
  // aceleración del MPU6050 empiezan en 0x3B (ACCEL_XOUT_H) y son
  // 6 bytes consecutivos: X alto, X bajo, Y alto, Y bajo, Z alto, Z bajo.
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6, (uint8_t)true);
  if (Wire.available() < 6) return;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  // Escala por defecto del MPU6050: ±2g, 16384 LSB/g.
  axg = ax / 16384.0f;
  ayg = ay / 16384.0f;
  azg = az / 16384.0f;
  ok = true;
}

// NUEVO: mantiene la ventana de varias muestras (MOTION_SAMPLE_WINDOW)
// para decidir la bandera "hay movimiento sí/no", igual que antes, pero
// ahora recibe la desviación ya calculada en loop() (a partir de la
// misma lectura que usa el filtro NLMS) en vez de leer el sensor por su
// cuenta. Si la lectura de este ciclo no fue válida (readOk=false), no
// se agrega al buffer (se mantiene el último estado conocido).
bool updateMotionWindow(float absDeviation, bool readOk) {
  if (!readOk) return false;

  motionSamples[motionSampleSpot++] = absDeviation;
  motionSampleSpot %= MOTION_SAMPLE_WINDOW;
  if (motionValidSamples < MOTION_SAMPLE_WINDOW) motionValidSamples++;

  if (motionValidSamples < MOTION_SAMPLE_WINDOW) {
    // Todavía no hay suficientes muestras para decidir con confianza;
    // por seguridad, no se reporta movimiento (igual de conservador que
    // el patrón ya usado con validSamples del MAX30102 más arriba, pero
    // en la dirección contraria: aquí "no decidir aún" = false, no se
    // bloquea el reporte de BPM por esto).
    return false;
  }

  float avgDeviation = 0;
  for (byte x = 0; x < MOTION_SAMPLE_WINDOW; x++) avgDeviation += motionSamples[x];
  avgDeviation /= MOTION_SAMPLE_WINDOW;

  return avgDeviation > MOTION_THRESHOLD_G;
}

// v6: un paso del filtro adaptativo NLMS MULTI-EJE, con compuerta de
// adaptación y fuga ("leaky NLMS").
//   primaryAC   = componente AC de la señal IR (IR - línea base) — la
//                 señal observada, sucia (PPG + artefacto de
//                 movimiento).
//   axHp/ayHp/azHp = componente de alta frecuencia (ya sin gravedad ni
//                 orientación lenta de la muñeca) de cada eje del
//                 acelerómetro, CON signo.
//   shouldAdapt = compuerta: true solo si hay energía de movimiento real
//                 por encima del ruido de reposo calibrado. Si es false,
//                 los pesos NO se actualizan con la regla NLMS — solo se
//                 aplica la fuga (decaen suavemente hacia 0).
// Devuelve e[n]: la estimación de PPG limpio (componente AC).
float nlmsFilterStepMultiAxis(float primaryAC, float axHp, float ayHp, float azHp, bool shouldAdapt) {
  const float refSample[NLMS_NUM_AXES] = { axHp, ayHp, azHp };

  // 1) desplaza la línea de retardo de cada eje (más reciente en tap 0)
  for (int c = 0; c < NLMS_NUM_AXES; c++) {
    for (int i = NLMS_FILTER_ORDER - 1; i > 0; i--) {
      nlmsRefBuffer[c][i] = nlmsRefBuffer[c][i - 1];
    }
    nlmsRefBuffer[c][0] = refSample[c];
  }

  // 2) salida del filtro: combinación lineal de las últimas
  // NLMS_FILTER_ORDER muestras de CADA eje, sumadas entre ejes — i.e.
  // la parte de "primaryAC" que el filtro cree que se explica por el
  // movimiento en cualquiera de las 3 direcciones.
  float y = 0.0f;
  for (int c = 0; c < NLMS_NUM_AXES; c++) {
    for (int i = 0; i < NLMS_FILTER_ORDER; i++) {
      y += nlmsWeights[c][i] * nlmsRefBuffer[c][i];
    }
  }

  // 3) error = lo que NO se explica por el movimiento = PPG limpio
  float e = primaryAC - y;

  if (shouldAdapt) {
    // 4) energía conjunta de los 3 ejes (para normalizar el paso de
    // adaptación, igual que en NLMS clásico pero sumando todos los
    // canales de referencia).
    float energy = NLMS_EPSILON;
    for (int c = 0; c < NLMS_NUM_AXES; c++) {
      for (int i = 0; i < NLMS_FILTER_ORDER; i++) {
        energy += nlmsRefBuffer[c][i] * nlmsRefBuffer[c][i];
      }
    }

    // 5) actualización de pesos (regla NLMS + fuga). La fuga
    // (NLMS_LEAK < 1) evita que los pesos queden "pegados" en un valor
    // grande de una ráfaga de movimiento pasada — sin ella, un burst de
    // movimiento podía dejar pesos que distorsionan la señal minutos
    // después, en reposo, aunque ya no se estuviera adaptando.
    float step = (NLMS_MU * e) / energy;
    for (int c = 0; c < NLMS_NUM_AXES; c++) {
      for (int i = 0; i < NLMS_FILTER_ORDER; i++) {
        nlmsWeights[c][i] = NLMS_LEAK * nlmsWeights[c][i] + step * nlmsRefBuffer[c][i];
      }
    }
  } else {
    // Sin movimiento real: NO se adapta (evita amplificar ruido de
    // cuantización del MPU6050, la causa raíz del fallo de v1). Solo se
    // aplica la fuga, para que los pesos vayan relajándose hacia 0.
    for (int c = 0; c < NLMS_NUM_AXES; c++) {
      for (int i = 0; i < NLMS_FILTER_ORDER; i++) {
        nlmsWeights[c][i] *= NLMS_LEAK;
      }
    }
  }

  return e;
}

// v6: filtro de mediana simple, aplicado DESPUÉS del NLMS (ver nota de
// arquitectura al inicio del archivo). Sin pesos, sin realimentación —
// matemáticamente no puede oscilar/inestabilizarse. Mientras no se junte
// la ventana completa, devuelve el valor crudo tal cual.
long medianFilter(long newSample) {
  medianBuffer[medianSpot++] = newSample;
  medianSpot %= MEDIAN_WINDOW;
  if (medianValidSamples < MEDIAN_WINDOW) {
    medianValidSamples++;
    return newSample;
  }

  long sorted[MEDIAN_WINDOW];
  for (byte i = 0; i < MEDIAN_WINDOW; i++) sorted[i] = medianBuffer[i];
  for (byte i = 1; i < MEDIAN_WINDOW; i++) {
    long key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }
  return sorted[MEDIAN_WINDOW / 2];
}

// ---------------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------------
void loop() {
  loopCounter++;
  long irValue = particleSensor.getIR();
  bool fingerDetected = (irValue > FINGER_THRESHOLD);

  // --- Una sola lectura del MPU6050 por ciclo, reutilizada por la
  // bandera de movimiento Y por el filtro NLMS (antes se leía el
  // sensor dos veces con la misma información) ---
  float axg, ayg, azg;
  bool mpuReadOk = false;
  readAccelRaw(axg, ayg, azg, mpuReadOk);

  float rawDeviation = 0.0f; // magnitud, CON signo — solo para la bandera de movimiento (BLE)
  float absDeviation  = 0.0f; // valor absoluto — entra a la bandera de movimiento
  if (mpuReadOk) {
    float magnitude = sqrt(axg * axg + ayg * ayg + azg * azg);
    rawDeviation = magnitude - 1.0f; // desviación respecto a 1g (reposo)
    absDeviation = fabs(rawDeviation);
  }

  bool motionDetected = updateMotionWindow(absDeviation, mpuReadOk);

  // --- v6: filtro NLMS multi-eje — limpia la señal IR usando los 3
  // ejes del acelerómetro por separado (no la magnitud, ver nota de
  // arquitectura arriba), ANTES de buscar latidos. ---
  long filteredIrValue = irValue;
  if (fingerDetected && mpuReadOk) {
    if (!irBaselineInitialized) {
      irBaseline = (float)irValue;
      irBaselineInitialized = true;
    } else {
      irBaseline = IR_BASELINE_ALPHA * irBaseline + (1.0f - IR_BASELINE_ALPHA) * (float)irValue;
    }

    // v6: línea base POR EJE del acelerómetro (equivalente a la
    // gravedad + orientación lenta de la muñeca en ese instante). Restar
    // esto, en vez de restar un "1.0f" fijo por eje, deja pasar solo la
    // variación real de movimiento, sin importar cómo esté orientada la
    // muñeca (la gravedad no siempre cae 100% en el eje Z).
    if (!accBaselineInitialized) {
      axBaseline = axg; ayBaseline = ayg; azBaseline = azg;
      accBaselineInitialized = true;
    } else {
      axBaseline = ACC_BASELINE_ALPHA * axBaseline + (1.0f - ACC_BASELINE_ALPHA) * axg;
      ayBaseline = ACC_BASELINE_ALPHA * ayBaseline + (1.0f - ACC_BASELINE_ALPHA) * ayg;
      azBaseline = ACC_BASELINE_ALPHA * azBaseline + (1.0f - ACC_BASELINE_ALPHA) * azg;
    }
    float axHp = axg - axBaseline;
    float ayHp = ayg - ayBaseline;
    float azHp = azg - azBaseline;

    float irAC = (float)irValue - irBaseline; // componente AC (pulso + MA)
    float absIrAC = fabs(irAC);

    // v9: máximo |irAC| visto desde el último reporte (solo para el
    // log, se resetea una vez por segundo — ver bloque de reporte).
    if (absIrAC > irAcMaxSinceReport) irAcMaxSinceReport = absIrAC;

    // v10 — peak-hold con decaimiento, actualizado cada loop: sube de
    // inmediato ante cualquier disturbio óptico y decae solo. Esto SÍ
    // se puede usar en tiempo real como compuerta (a diferencia de
    // irAcMaxSinceReport, que solo se conoce al final del segundo).
    irAcFastPeak = fmax(absIrAC, irAcFastPeak * OPTICAL_PEAK_DECAY);

    // Compuerta de adaptación: DOBLE, acelerómetro U óptica. Evidencia
    // real (ver nota junto a OPTICAL_ADAPT_THRESHOLD arriba): hay
    // momentos con acelerómetro en calma pero artefacto óptico real —
    // el acelerómetro solo no basta.
    bool shouldAdapt = (absDeviation > NLMS_GATE_THRESHOLD_G) ||
                        (irAcFastPeak > OPTICAL_ADAPT_THRESHOLD);

    float cleanedAC = nlmsFilterStepMultiAxis(irAC, axHp, ayHp, azHp, shouldAdapt);
    long nlmsOutput = (long)(irBaseline + cleanedAC);

    // v6: mediana DESPUÉS del NLMS — limpia picos aislados que el NLMS
    // (lineal) no puede explicar (ver hallazgo #4 del resumen: ruido
    // presente incluso sin movimiento ni filtro alguno).
    filteredIrValue = medianFilter(nlmsOutput);
  } else {
    // Sin dedo puesto o sin lectura válida del MPU6050: se reinician
    // ambas líneas base para la próxima vez que haya contacto real.
    irBaselineInitialized = false;
    accBaselineInitialized = false;
    // También se reinicia la ventana de mediana, para no mezclar datos
    // de antes de perder contacto con datos de después.
    medianValidSamples = 0;
    medianSpot = 0;
    // v7 — BUG CORREGIDO: antes NO se reseteaban los pesos ni el buffer
    // de referencia del NLMS al perder contacto. Confirmado en log real
    // (muñeca, con reconexiones): al reconectar aparecían saltos
    // enormes IR vs IRfilt (ej. diff de 5401), porque el filtro seguía
    // aplicando pesos aprendidos con datos de ANTES de la desconexión a
    // una señal completamente nueva que no tiene relación con ellos.
    for (int c = 0; c < NLMS_NUM_AXES; c++) {
      for (int i = 0; i < NLMS_FILTER_ORDER; i++) {
        nlmsWeights[c][i] = 0.0f;
        nlmsRefBuffer[c][i] = 0.0f;
      }
    }
  }

  // IMPORTANTE: checkForBeat() solo se llama cuando hay contacto real
  // (fingerDetected). Antes se llamaba siempre, sin condición — al
  // quitar el dedo, el IR cae a un valor bajo y ruidoso (luz ambiental,
  // ruido eléctrico) que el algoritmo a veces confundía con un latido
  // real, generando BPM delirantes (ej. 250-370) que además quedaban
  // "pegados" varios segundos por la lógica de hueco corto de abajo.
  // AHORA se le pasa filteredIrValue (limpio de artefacto de
  // movimiento) en vez de irValue crudo.
  if (fingerDetected && checkForBeat(filteredIrValue)) {
    long delta = millis() - lastBeat;

    if (delta >= MIN_BEAT_INTERVAL_MS) {
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);

      // Filtra valores claramente fuera del rango fisiológico humano.
      if (beatsPerMinute >= 20 && beatsPerMinute <= 255) {
        // v8 — chequeo RÁPIDO de consistencia consecutiva: se compara
        // este latido crudo contra el latido crudo INMEDIATAMENTE
        // ANTERIOR (haya sido aceptado o no) — no contra un promedio
        // acumulado, para no quedar "anclado" en un valor malo.
        bool matchesLastCandidate = true;
        if (haveLastCandidate) {
          float maxDeviation = lastCandidateBpm * BPM_PLAUSIBILITY_MAX_REL_DEVIATION;
          if (maxDeviation < BPM_PLAUSIBILITY_MIN_ABS_DEVIATION) {
            maxDeviation = BPM_PLAUSIBILITY_MIN_ABS_DEVIATION;
          }
          matchesLastCandidate = (fabs(beatsPerMinute - lastCandidateBpm) <= maxDeviation);
        }
        lastCandidateBpm = beatsPerMinute;
        haveLastCandidate = true;

        // v12 — capa ANTI-DERIVA. El chequeo de arriba (v8) protege
        // contra un salto brusco de una vez, pero NO contra una deriva
        // gradual: una cadena donde cada paso está dentro del margen
        // respecto al anterior, pero la cadena completa se aleja mucho
        // de la frecuencia real (evidencia real: 143.5→177.0→140.8→
        // 185.8, cada paso <35% del anterior, terminó en Avg=161-172
        // vs. 70-80 real). Se compara contra trustedBpm (el ancla
        // LENTA, no beatAvg — ver nota de arquitectura junto a
        // trustedBpm arriba: comparar contra beatAvg no detiene la
        // deriva porque beatAvg se arrastra con ella).
        bool isBigJump = false;
        if (trustedBpmInitialized) {
          float diffFromTrusted = fabs(beatsPerMinute - trustedBpm);
          float bigJumpThreshold = fmax(BIG_JUMP_MIN_ABS_DEVIATION, trustedBpm * BIG_JUMP_MIN_REL_DEVIATION);
          isBigJump = (diffFromTrusted > bigJumpThreshold);
        }

        bool plausible;
        if (!isBigJump) {
          // Lectura cercana al ancla de confianza (o ancla aún sin
          // inicializar): el chequeo rápido de v8 alcanza, sensible a
          // variación fisiológica normal.
          plausible = matchesLastCandidate;
          pendingJumpConfirmCount = 0; // se cancela cualquier racha de salto pendiente
          if (plausible) {
            // El ancla se mueve LENTO con cada latido normal aceptado
            // (10%), o se inicializa directo la primera vez.
            if (!trustedBpmInitialized) {
              trustedBpm = beatsPerMinute;
              trustedBpmInitialized = true;
            } else {
              trustedBpm = TRUST_ANCHOR_ALPHA * trustedBpm + (1.0f - TRUST_ANCHOR_ALPHA) * beatsPerMinute;
            }
          }
        } else {
          // Salto grande respecto al ancla de confianza: se compara
          // contra el ANCLA de la racha de salto (no contra el vecino),
          // con tolerancia más estricta.
          float anchorMaxDeviation = pendingJumpAnchorBpm * JUMP_ANCHOR_MAX_REL_DEVIATION;
          if (anchorMaxDeviation < JUMP_ANCHOR_MIN_ABS_DEVIATION) {
            anchorMaxDeviation = JUMP_ANCHOR_MIN_ABS_DEVIATION;
          }
          bool matchesAnchor = (pendingJumpConfirmCount > 0) &&
              (fabs(beatsPerMinute - pendingJumpAnchorBpm) <= anchorMaxDeviation);

          if (matchesAnchor) {
            pendingJumpConfirmCount++;
          } else {
            // Empieza (o reinicia) una nueva racha de salto, anclada a
            // este valor.
            pendingJumpAnchorBpm = beatsPerMinute;
            pendingJumpConfirmCount = 1;
          }
          plausible = (pendingJumpConfirmCount >= JUMP_CONFIRMATIONS_REQUIRED);
          if (plausible) {
            // Régimen confirmado con evidencia fuerte (3 lecturas
            // seguidas coincidiendo entre sí): el ancla de confianza
            // salta directo al nuevo valor, no gradualmente.
            trustedBpm = pendingJumpAnchorBpm;
            trustedBpmInitialized = true;
          }
        }

        // v10 — congelamiento por volatilidad ÓPTICA (sin cambios):
        // sin importar si pasó los chequeos de arriba, si la óptica
        // está muy inestable AHORA, no se acepta.
        if (irAcFastPeak > OPTICAL_FREEZE_THRESHOLD) {
          plausible = false;
        }

        // v11: log de CADA latido individual (no solo el resumen de
        // una vez por segundo) — necesario para diagnosticar la deriva
        // gradual que llevó a este cambio.
        Serial.print("  latido crudo="); Serial.print(beatsPerMinute, 1);
        Serial.print(" aceptado="); Serial.print(plausible ? "sí" : "no");
        Serial.print(" saltoGrande="); Serial.print(isBigJump ? "sí" : "no");
        Serial.print(" IRacPeak="); Serial.println(irAcFastPeak, 0);

        if (plausible) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;
          if (validSamples < RATE_SIZE) validSamples++;

          // Promedio de los últimos RATE_SIZE latidos.
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
        // else: este latido no coincide con el anterior — todavía no
        // hay evidencia suficiente de cuál de los dos (o ninguno) es
        // real. Se descarta por ahora (lastBeat YA quedó actualizado
        // arriba, se sigue esperando el próximo desde este punto en el
        // tiempo), pero queda guardado como candidato: si el PRÓXIMO
        // latido coincide con este, ambos se aceptan en la siguiente
        // vuelta.
      }
    }
    // else: disparo espurio (demasiado seguido para ser un latido
    // real) — se ignora sin tocar lastBeat ni beatsPerMinute.
  }

  // Si se pierde contacto real (dedo retirado), se limpia todo de
  // inmediato — ya no tiene sentido esperar el timeout de hueco largo,
  // porque sabemos con certeza (vía fingerDetected) que no hay nada
  // que medir, en vez de adivinarlo indirectamente por ausencia de
  // latidos.
  if (!fingerDetected) {
    validSamples = 0;
    rateSpot = 0;
    beatAvg = 0;
    lastBeat = 0;
    // v8: no comparar el primer latido de la próxima sesión de
    // contacto contra el último candidato de la sesión anterior.
    haveLastCandidate = false;
    lastCandidateBpm = 0.0f;
    irAcMaxSinceReport = 0.0f;
    irAcFastPeak = 0.0f;
    // v12: no arrastrar una racha de salto de la sesión anterior.
    pendingJumpConfirmCount = 0;
    pendingJumpAnchorBpm = 0.0f;
    trustedBpm = 0.0f;
    trustedBpmInitialized = false;
  }

  // Antes, si pasaban más de STALE_BEAT_TIMEOUT_MS sin un latido nuevo,
  // se reseteaba validSamples a 0 — obligando a esperar 4 latidos
  // nuevos desde cero. En la práctica, el algoritmo checkForBeat()
  // falla en detectar un latido individual de forma normal y
  // esporádica (no solo por mala señal), y cada fallo de más de 3s
  // reiniciaba TODO el progreso, lo cual se sentía como "tarda 1 minuto
  // en estabilizarse" aunque el dedo nunca se movió. Ahora, un hueco
  // corto sin latido NO borra el historial: seguimos reportando el
  // último promedio válido (es razonable asumir que el BPM no cambió
  // drásticamente en 3 segundos). Solo si el hueco se vuelve largo de
  // verdad (ver LONG_STALE_TIMEOUT_MS más abajo) se considera que se
  // perdió contacto real y se resetea — esto cubre el caso de contacto
  // débil/intermitente, no de remoción franca del dedo (que ya se
  // maneja arriba, de forma inmediata).
  bool beatGapDetected = (lastBeat > 0 && millis() - lastBeat > STALE_BEAT_TIMEOUT_MS);

  if (lastBeat > 0 && millis() - lastBeat > LONG_STALE_TIMEOUT_MS) {
    validSamples = 0;
    rateSpot = 0;
    beatAvg = 0;
  }

  uint32_t now = millis();
  if (now - tsLastReport >= REPORTING_PERIOD_MS) {
    tsLastReport = now;

    Serial.print("IR: ");    Serial.print(irValue);
    Serial.print(" | IRfilt: "); Serial.print(filteredIrValue);
    Serial.print(" | loops/s: "); Serial.print(loopCounter - loopCounterLastReport);
    loopCounterLastReport = loopCounter;
    Serial.print(" | BPM: "); Serial.print(beatsPerMinute, 1);
    Serial.print(" | Avg: "); Serial.print(beatAvg);
    Serial.print(" | Muestras: "); Serial.print(validSamples);
    Serial.print("/"); Serial.print(RATE_SIZE);
    Serial.print(" | Muñeca: "); Serial.print(fingerDetected ? "sí" : "no");
    Serial.print(" | Hueco: "); Serial.print(beatGapDetected ? "sí(>3s sin latido, ignorado)" : "no");
    Serial.print(" | Movimiento: "); Serial.print(motionDetected ? "sí" : "no");
    // v9 — NUEVO: valor numérico real de absDeviation (antes solo se
    // veía el booleano "Movimiento"), y el máximo |irAC| visto en este
    // segundo. Se imprimen juntos para poder comparar directamente si
    // el acelerómetro y la volatilidad óptica se mueven juntos o no
    // durante movimiento real.
    Serial.print(" | AccDevG: "); Serial.print(absDeviation, 4);
    Serial.print(" | IRacMax: "); Serial.print(irAcMaxSinceReport, 0);
    // v10 — NUEVO: irAcFastPeak, el peak-hold en tiempo real usado
    // ahora como compuerta. Se imprime para poder refinar
    // OPTICAL_ADAPT_THRESHOLD / OPTICAL_FREEZE_THRESHOLD (hoy
    // provisionales) con datos reales del próximo test.
    Serial.print(" | IRacPeak: "); Serial.print(irAcFastPeak, 0);
    Serial.print(" | BLE: ");  Serial.println(deviceConnected ? "sí" : "no");
    irAcMaxSinceReport = 0.0f; // se reinicia para el próximo segundo

    if (deviceConnected) {
      if (fingerDetected && validSamples >= RATE_SIZE && beatAvg > 0) {
        notifyHeartRate((uint8_t) beatAvg);
        notifyStatus(0);   // OK
      } else if (!fingerDetected) {
        notifyStatus(1);   // sin dedo / sin contacto con la muñeca
      } else {
        notifyStatus(2);   // dedo/muñeca detectada, aún acumulando muestras
      }
      // NUEVO: se notifica el estado de movimiento en el mismo ciclo de
      // reporte, sin alterar el orden ni el contenido de las
      // notificaciones de heart rate/status de arriba.
      notifyMotion(motionDetected ? 1 : 0);
      // Nota: el MAX30102 puede calcular SpO2 con librerías adicionales
      // (ej. "SparkFun Bio Sensor Hub"). Para esta versión solo enviamos
      // BPM. Agregar SpO2 es una mejora para la siguiente iteración.
    }
  }
}
