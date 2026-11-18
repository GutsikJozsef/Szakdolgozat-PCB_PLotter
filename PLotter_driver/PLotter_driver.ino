#include <avr/io.h>                  // AVR I/O regiszterek definíciói
#include <util/delay.h>              // Késleltetési függvények
#include <stdbool.h>                 // Logikai (bool) típus támogatása
#include <string.h>                  // Karakterlánc-kezelő függvények
#include <stdio.h>                   // Standard I/O függvények
#include <math.h>                    // Matematikai függvények
#include <stdlib.h>                  // Általános segédfüggvények 

#define F_CPU 16000000UL             // Processzor órajel: 16 MHz
#define BAUDRATE 9600                // Soros kommunikációs adatátviteli sebesség
#define UBRR (F_CPU/16/BAUDRATE - 1) // Baud rate regiszter értéke
#define BUFF_SIZE 32                 // Soros üzenet puffer mérete

// Vezérlő pinek

const uint8_t STEP_X = PD2;       // X step -> pin D2
const uint8_t STEP_Y = PD4;       // Y step -> pin D4
const uint8_t DIR_X = PD5;        // X dir -> pin D5
const uint8_t DIR_Y = PD7;        // Y dir -> pin D7
const uint8_t EN = PB0;           // Enable -> pin D8
const uint8_t SERVO_PIN = PB5;    // Servo -> pin D13
const uint8_t DRILL_EN = PD3;     // L298N ENA -> pin D3
const uint8_t DRILL_IN1 = PD6;    // L298N IN1 -> pin D6
const uint8_t DRILL_IN2 = PC3;    // L298N IN2 -> pin A3

// Végálláskapcsolók

const uint8_t LIM_X = PB1;                 // X limit -> pin D9
const uint8_t LIM_Y = PB2;                 // Y limit -> pin D10

// Konstam változók

const uint8_t ramp_points = 100;           // Fix 100 pontból álló gyorsulási profil
const float steps_per_mm_x = 400;          // X tengely felbontása (lépés/mm)
const float steps_per_mm_y = 400;          // Y tengely felbontása (lépés/mm)
const int8_t z_min = -15;                  // Z tengelymenti mozgástartomány
const uint8_t z_origo = 0;                 // Z tengely origója
const uint8_t x_max = 172;                 // Munkatér X tengelymenti határa
const uint8_t y_max = 272;                 // Munkatér Y tengelymenti határa
const uint8_t DRILL_MAX_SPEED = 30;        // PWM maximális kitöltés (0-255)
const uint8_t DRILL_RAMP_MS = 20;          // Felfutási késleltetés (ms / lépés)

// Munkaváltozók

uint32_t period_xy = 60;                   // X, Y sebesség[cite: 7]
uint8_t homing_period_xy = 100;            // X, Y sebesség homing esetén[cite: 7]
uint32_t period_z = 20;                    // Z sebesség[cite: 7]
float current_x = 0.0;                     // Aktuális X pozíció[cite: 7]
float current_y = 0.0;                     // Aktuális Y pozíció[cite: 7]
float current_z = 0.0;                     // Aktuális Z pozíció[cite: 7]
volatile uint16_t current_pulse_ticks = 0; // Szervómotor aktuális adatai[cite: 7]
volatile uint8_t servo_running = 0;        // Szervómotor futási jelző[cite: 7]
volatile uint8_t home_x_done = 0;          // X homing kész jelző[cite: 7]
volatile uint8_t home_y_done = 0;          // Y homing kész jelző[cite: 7]
bool autohoming = 0;                       // Autohoming állapotjelző[cite: 7]
bool error_mode = 0;                       // Hiba állapotjelző[cite: 7]


typedef struct {
    float dx;                                  // X tengelyen mozgandó távolság
    float dy;                                  // Y tengelyen mozgandó távolság
} Vector;

// Gyorsulási beállítások

uint8_t accel_percent = 20;                  // Gyorsulási százalék
uint8_t decel_percent = 20;                  // Lassulási százalék
const uint32_t min_steps_threshold = 40;     // Minimális lépéshatár
const uint32_t min_ramp_steps = 50;          // Minimális gyorsulási lépésszám
const uint32_t max_ramp_steps = 1250;        // Maximális gyorsulási lépésszám
uint16_t accel_ramp_x[ramp_points];          // X tengely gyorsulási profil tömb
uint16_t accel_ramp_y[ramp_points];          // Y tengely gyorsulási profil tömb
int32_t ramp_steps = 2000;                   // Gyorsulási szakasz hossza lépésben 
uint16_t accel_ramp[ramp_points];            // Sebességprofil tárolására szolgáló tömb
uint16_t start_period_xy = 1000;             // Kezdeti sebesség
uint16_t homing_start_period = 1000;         // Kezdeti sebesség homing esetén
uint32_t accel_steps_x = 0;                  // X gyorsulási lépések száma
uint32_t decel_steps_x = 0;                  // X lassulási lépések száma
bool use_ramp_x = false;                     // X rámpa használat jelző
uint32_t accel_steps_y = 0;                  // Y gyorsulási lépések száma
uint32_t decel_steps_y = 0;                  // Y lassulási lépések száma
bool use_ramp_y = false;                     // Y rámpa használat jelző

// ===== Makrók =====

// Végálláskapcsoló időzítőjének indítása/tiltása
#define STOP_LIMIT_TIMER()  (TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20)))  // Timer2 számlálásának leállítása
#define START_LIMIT_TIMER() (TCCR2B |= (1 << CS22) | (1 << CS20))                   // 128-as előosztó beállítása Timer2-n (16MHz / 128 = 125 kHz órajel)

// =====  Soros kommunikáció =====

// Soros port inicializálása (USART)
// UBR0 - USART Baud Rate Register
// UCSR0B - USART Control and status Register B (adatküldést lehet beállítani)
// RXEN0 - Reciever Enable
// TXEN0 - Transmitter Enable
void InitSerial(void) {
    UBRR0H = (unsigned char)(UBRR >> 8);      // Baudrate beállítása
    UBRR0L = (unsigned char)UBRR;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);     // Adás és vétel engedélyezése
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // 8-bit adat, 1 stop bit
}

// Egy karakter küldése
// UDR0 - USART I/O Data Regsiter (ide írjuk a kiküldeni kívánt karaktert illetve inne olvassuk ki a fogadott karaktert)
// UCSR0A - USART Control and Status Register A (itt tárolódnak az állapotot leíró flagek)
// UDRE0 - USART Data Register Empty (1 -> adóbuffer üres)
void SerialTransmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));         // Várakozás, amíg az adóbuffer ürül (UDRE0 bit = 1)
    UDR0 = data;                              // kiküldés
}

// Karakterlánc küldése
void SerialPrint(const char* s) {
    while (*s) {                    // végigiterálunk a karaktereken (string végén null terminátor)
        SerialTransmit(*s++);        // léptetjük a mutatót a köv. memóriacímre
    }
}

// Egy karakter fogadása
// RXC0 - Recieve Complete (1 -> van fogadott adat a fogadó bufferben)
unsigned char SerialRecieve(void) {
    while (!(UCSR0A & (1 << RXC0)));      // Várakozás fogadott adatra
    return UDR0;                          // kiolvasás
}

// Egy üzenet olvasása
void SerialReadLine(char buffer[BUFF_SIZE]){
  uint8_t index = 0;
  char c;

  // 1. Üzenet beolvasása a sorzáró '*' karakterig
  while (1) {
      c = SerialRecieve(); // A te karakter-fogadó függvényed

      if (c == '\n' || c == '\r' || c == '*') {
          buffer[index] = '\0'; // Sztring lezárása
          break;
      }

      // Biztonsági védelem: nehogy túlcsorduljon a buffer
      if (index < BUFF_SIZE - 1) {
          buffer[index++] = c;
      }
  }
}

// Bejövő üzenet ellenőrzése
bool SerialCheckIncoming(){
  return UCSR0A & (1 << RXC0);
}

// Nyugtázó üzenet küldése
void SerialAcknowledge(){
  SerialPrint(("ok\r\n"));   // sikeres végrehajtás esetén nyugtázás
  return;
}

// Hiba esetén visszajelzés
void SerialSendError(int error_type){
    if (error_type == 0){
        SerialPrint(("error0\r\n"));       // Munka közben benyomódott egy végálláskapcsoló
    }
}

// ===== Motorok Vezérlése  =====

// Megteendő vektor kiszámítása X-Y síkon
Vector CalculateVector(char *x_ptr, char *y_ptr) {
    
    Vector result = {0.0, 0.0};             // Alapértelmezett, üres válasz
    
    float target_x = atof(x_ptr + 1);       // Szövegből lebegőpontos szám
    float target_y = atof(y_ptr + 1);       // +1 azért kell, hogy magát betűt átugorjuk kiolvasásnál

    result.dx = target_x - current_x;       // Különbségek kiszámítása
    result.dy = target_y - current_y;
    return result;
}

// Célpozíció kinyerése Z tengelyen
float CalculateZPosition (char * z_ptr){
  return atof(z_ptr + 1);                             
}

// Vezérlő pinek beállítása 
// PORTx - Data Register (a pinek jelszintét határozza meg / pullup ellenállás beállítása)
// DDRx - Data Direction Register (kimeneti-, bemeneti pin beállítása)
// DDRx: 1 -> kimenet, 0 -> bemenet
// ha DDRB[x] = 0, akkor:
// - PORTB[x] = 1 -> Bellső pull-up ellenállás
// - PORTB[x] = 0 -> Külső (nincs) pull-up ellenállás
// ha DDRB[x] = 1, akkor:
// - kimeneti pin jelszintje = PORTB[x]
void InitDrivers(void) {
    DDRD |= (1 << STEP_X) | (1 << STEP_Y);    // Step bitek 
    DDRD |= (1 << DIR_X) | (1 << DIR_Y);      // Dir bitek | - kimenet
    DDRB |= (1 << EN);                        // Enable    |

    PORTB &= ~(1 << EN);                      // Enable = 0 (engedélyezés)
    return;
}

// Servo vezérlő pinek beállítása
// Vezérlőjel periódusidejének beállítása
// TCCR1A, TCCR1B - Timer/Counter 1 Control Register A,B (működési mód-, órajel-előosztó beállítása)
// WGM10-13 - Waveform Generation Mode bitek
// ICR1 - Input Capture Register 1 (periódusidő beállítása)
// CS10-12 - Clock Select bitek
// OCR1 - Output Compare Register 1 (összehasonlíó érték beállítása)
// TIMSK1 - Timer/Counter 1 Interrupt Mask register (hardveres engedélyezések)
// TOIE1 - Túlcsordulásos megszakítás engedélyezése
// OCIE1A - Összehasonlításos megszakítás engedélyezése 
// sei() - Megszakítások globális engedélyezése
void InitServo(void) {
    DDRB |= (1 << SERVO_PIN);                            // kimenet
    PORTB &= ~(1 << SERVO_PIN);                          // servo pin = 0

    TCCR1A = (1 << WGM11);                               // Timer1 beállítása Fast PWM módba
    TCCR1B = (1 << WGM13) | (1 << WGM12); 
    TCCR1B |= (1 << CS11);                               // 16 MHz / 8 = 2 MHz - 1 tick = 0.5 mikroszekundum
    ICR1 = 39999;                                        // 20 ms periódusidő beállítása (4000 tick)
    TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A);              // Túlcsordulás és összehasonlítás megszakítások engedélyezése
    sei();                                               // Globális megszakítások bekapcsolása
    return;
}

// Léptetőmotor forgási irányának beállítása
void SetDirection(uint8_t axis_bit, bool forward) {
    if (forward) {
        PORTD |= (1 << axis_bit);                       // Bit beállítása 1-re 
    } else {
        PORTD &= ~(1 << axis_bit);                      // Bit beállítása 0-ra 
    }
}

// Egy motor léptetése egy lépést
// A megfelelő GPIO regiszter módosításával egy impulzust adunk ki a kimeneten 
// step_bit - adott vezérlő kimenet GPIO bitje
// period_us - Impulzus periódusideje
void StepOne(uint8_t step_mask, uint16_t period_us) {
    PORTD |= step_mask;                                          // step bit = 1
    for (uint16_t i = 0; i < period_us; i++) _delay_us(1); 
    PORTD &= ~step_mask;                                         // step bit = 0
    for (uint16_t i = 0; i < period_us; i++) _delay_us(1);       // kivonjuk belőle a szűrő okozta késleltetést annyiszor, ahány tengelyt léptetünk
    return;
}

// Adott vektor leléptetése
void StepXY(Vector vector) {
    int32_t dx = round(fabs(vector.dx) * steps_per_mm_x);                                 // X lépések száma
    int32_t dy = round(fabs(vector.dy) * steps_per_mm_y);                                 // Y lépések száma
    if (dx == 0 && dy == 0) return;

    GenerateAxisRamp(dx, start_period_xy, period_xy, accel_ramp_x, &accel_steps_x, &decel_steps_x, &use_ramp_x); // X rámpa generálása
    GenerateAxisRamp(dy, start_period_xy, period_xy, accel_ramp_y, &accel_steps_y, &decel_steps_y, &use_ramp_y); // Y rámpa generálása

    if (current_x > 0 && current_y > 0) { START_LIMIT_TIMER(); }

    SetDirection(DIR_X, (vector.dx >= 0) ? 1 : 0);                                        // X irány beállítása
    SetDirection(DIR_Y, (vector.dy >= 0) ? 1 : 0);                                        // Y irány beállítása

    PORTB &= ~(1 << EN);                                                                  // Driverek engedélyezése

    int32_t over = 0;                                                                     // Bresenham hiba
    
    if (dx > dy) {                                                                        // X domináns tengely
        over = dx / 2;
        for (int32_t i = 0; i < dx; i++) {
            
            uint16_t current_period = period_xy;
            if (use_ramp_x) {                                                             // Ha alkalmaz gyorsulást
                if (i < accel_steps_x) {                                                  // Gyorsulási szakasz
                    uint16_t index = (i * ramp_points) / accel_steps_x;
                    if (index >= ramp_points) index = ramp_points - 1;
                    current_period = accel_ramp_x[index];
                } else if (i >= dx - decel_steps_x) {                                     // Lassulási szakasz
                    int32_t steps_from_end = dx - 1 - i;
                    uint16_t index = (steps_from_end * ramp_points) / decel_steps_x;
                    if (index >= ramp_points) index = ramp_points - 1;
                    current_period = accel_ramp_x[index];
                }
            }

            uint8_t step_mask = (1 << STEP_X);                                            // X lépés maszk
            over += dy;
            if (over >= dx) {
                over -= dx;
                step_mask |= (1 << STEP_Y);                                               // Y lépés hozzáadása (átló)
            }
            StepOne(step_mask, current_period);                                           // Lépés kiadása
        }
    } else {                                                                              // Y domináns tengely
        over = dy / 2;
        for (int32_t i = 0; i < dy; i++) {
            
            uint16_t current_period = period_xy;
            if (use_ramp_y) {
                if (i < accel_steps_y) {                                                  // Gyorsulási szakasz
                    uint16_t index = (i * ramp_points) / accel_steps_y;
                    if (index >= ramp_points) index = ramp_points - 1;
                    current_period = accel_ramp_y[index];
                } else if (i >= dy - decel_steps_y) {                                     // Lassulási szakasz
                    int32_t steps_from_end = dy - 1 - i;
                    uint16_t index = (steps_from_end * ramp_points) / decel_steps_y;
                    if (index >= ramp_points) index = ramp_points - 1;
                    current_period = accel_ramp_y[index];
                }
            }

            uint8_t step_mask = (1 << STEP_Y);                                            // Y lépés maszk
            over += dx;
            if (over >= dy) {
                over -= dy;
                step_mask |= (1 << STEP_X);                                               // X lépés hozzáadása (átló)
            }
            StepOne(step_mask, current_period);                                           // Lépés kiadása
        }
    }

    STOP_LIMIT_TIMER();
}

// Gyorsulási profil generálás
void GenerateAxisRamp(uint32_t total_steps, uint16_t start_period, uint16_t min_period, 
                      uint16_t* ramp_array, uint32_t* accel_out, uint32_t* decel_out, bool* use_ramp_out) {
    
    if (total_steps < min_steps_threshold) {                                  // Ha túl keveset lépünk, rámpa nélkül futunk
        *use_ramp_out = false;
        *accel_out = 0;
        *decel_out = 0;
        return;
    }

    *use_ramp_out = true;                                                     // Rámpa engedélyezése
    
    *accel_out = (total_steps * accel_percent) / 100;                         // Gyorsulási lépések számának kiszámítása százalékból
    *decel_out = (total_steps * decel_percent) / 100;                         // Lassulási lépések számának kiszámítása százalékból

    if (*accel_out < min_ramp_steps) *accel_out = min_ramp_steps;             // Alsó korlát a gyorsulási lépésekre
    if (*decel_out < min_ramp_steps) *decel_out = min_ramp_steps;             // Alsó korlát a lassulási lépésekre

    if (*accel_out > max_ramp_steps) *accel_out = max_ramp_steps;             // Felső korlát a gyorsulási lépésekre
    if (*decel_out > max_ramp_steps) *decel_out = max_ramp_steps;             // Felső korlát a lassulási lépésekre

    if ((*accel_out + *decel_out) >= total_steps) {                           // Ha a rámpák összehasonlítva túl hosszúak
        *accel_out = total_steps / 2;                                         // Felezve osztjuk el a lépéseket
        *decel_out = total_steps - *accel_out;
    }

    float start_speed = 1000000.0 / start_period;                             // Kezdeti sebesség lépés/s-ban
    float target_speed = 1000000.0 / min_period;                              // Célsebesség lépés/s-ban
    float speed_change = (target_speed - start_speed) / ramp_points;          // Sebességváltozás mértéke lépésenként

    for (uint16_t i = 0; i < ramp_points; i++) {                              // Rámpapontok feltöltése ciklussal
        float current_speed = start_speed + (i * speed_change);               // Aktuális sebesség kiszámítása
        ramp_array[i] = (uint16_t)(1000000.0 / current_speed);                // Periódusidő mentése a tömbbe
    }
    return;
}

// Servomotor pozíconálása
// 0° -> 544 us = 1088 tick, 180° -> 2400us
// cli() - Megszakítások globális letiltása
void SetServoAngle(uint8_t angle) {
    if (angle > 180) angle = 180;       // Biztonsági korlát[cite: 4]

    angle = 180 - angle;                // Mozgásirány megfordítása

    uint16_t ticks = ((uint32_t)angle * 3712 / 180) + 1088; // Lineáris leképzés[cite: 4]

    cli();                              // Megszakítások szüneteltetése[cite: 4]
    current_pulse_ticks = ticks;        // Változók felülírása[cite: 4]
    servo_running = 1;                  //[cite: 4]
    sei();                              // Megszakítások újra engedélyezése[cite: 4]
    return;                             //[cite: 4]
}

// Szerszámfej mozgatása
void MoveZ(float target_pos){
  float distance_mm = fabs(target_pos - current_z);                     // Tényleges fizikai távolság mm-ben
  if (distance_mm < 0.01) return;                                       // Ha alig van elmozdulás, kilépünk
  float start_angle = 120.0 - (70.0 * (current_z / z_min));             // Szögek kiszámítása
  float end_angle = 120.0 - (70.0 * (target_pos / z_min)); 
  uint32_t total_time_us = (uint32_t)(distance_mm * period_z);          // Teljes mozgási idő kiszámítása mikroszekundumban
  uint32_t steps = total_time_us / 20000;                               // Mozgás felbontása 20 ms-os (20000 us) lépésekre
  if (steps == 0) {                                                     // Ha olyan gyors a mozgás, hogy 20ms alatt lezajlik, 
      SetServoAngle((uint8_t)round(end_angle));                         // azonnal a végpontra ugrunk
      return;
  }
  float angle_step = (end_angle - start_angle) / (float)steps;          // Egy lépésre jutó szögváltozás
  for (uint32_t i = 1; i <= steps; i++) {                               // Fokozatos mozgatás 20 ms-os késleltetésekkel
    float current_step_angle = start_angle + (angle_step * i);
    SetServoAngle((uint8_t)round(current_step_angle));                                
    _delay_ms(20);
  }
  return;
}

// Végálláskapcsolók és fúrómotor PWM közös inicializálása (Timer2)
void InitLimitsAndDrill(void) {
    DDRB &= ~((1 << LIM_X) | (1 << LIM_Y));                 // Végállások beállítása
    PORTB |= (1 << LIM_X) | (1 << LIM_Y);     

    DDRD |= (1 << DRILL_EN) | (1 << DRILL_IN1);             // Fúró lábak beállítása
    DDRC |= (1 << DRILL_IN2);                 
    PORTD &= ~((1 << DRILL_EN) | (1 << DRILL_IN1)); 
    PORTC &= ~(1 << DRILL_IN2);                

    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);   // Közös Timer2 beállítása - Fast PWM mód, TOP = OCR2A (WGM mód 7)
    TCCR2B = (1 << WGM22) | (1 << CS22) | (1 << CS20);      // 128-as előosztó
    
    OCR2A = 124;                                            // Timer csúcsa (TOP) -> 1000 Hz megszakítás frekvencia
    OCR2B = 0;                                              // Kezdeti fúró PWM kitöltés = 0 

    TIMSK2 |= (1 << OCIE2A);                                // Végállás megszakítás engedélyezése
    sei();
}

// Fúró indítása 
void RunDrill(bool spinning_dir, uint8_t target_speed) {
    if (spinning_dir) {                                            // Irány beállítása a port regiszterekkel
        PORTD |= (1 << DRILL_IN1);                                 // IN1 => 1
        PORTC &= ~(1 << DRILL_IN2);                                // IN2 => 0
    } else {
        PORTD &= ~(1 << DRILL_IN1);                                // IN1 => 0
        PORTC |= (1 << DRILL_IN2);                                 // IN2 => 1
    }

    for (uint8_t current_speed = 0; 
                current_speed <= target_speed; current_speed++) {  // Fokozatos felfutás a Timer2 OCR2B regiszterének növelésével
        OCR2B = current_speed; 
        for(uint8_t d = 0; d < DRILL_RAMP_MS; d++) {               // Késleltetés iterációval
            _delay_ms(1);
        }
    }
    return;
}

// Fúró azonnali leállítása
void StopDrill(void) {
    OCR2B = 0;                    // PWM kitöltés nullázása 
    PORTD &= ~(1 << DRILL_IN1);   // Irányjelző pinek = 0
    PORTC &= ~(1 << DRILL_IN2);
    return;
}

// =====  Pozíció frissítése  =====

// X, Y síkon
void UpdateXYPosition(Vector vector){
  current_x += vector.dx;
  current_y += vector.dy;
  return;
}

// Z tengelyen
void UpdateZPosition(float z){
  current_z = z;
  return;
}

// =====  Funkcionális függvények  =====

// Végálláskapcsolók olvasása
uint8_t ReadLimitSwitches(void) {
    uint8_t state = PINB;                               // Beolvassuk a PINB regiszter aktuális állapotát 
    uint8_t val = (((~state >> LIM_X) & 1) << 1) |      // Kiszámoljuk a végálláskapcsolók bitmaszkolt értékét
                   (((~state >> LIM_Y) & 1) << 2);      
   return val;
}

// Autohoming
void AutoHome() {
    autohoming = 1;                         // Globális változók beállítása a számlálók számára
    home_x_done = 0;
    home_y_done = 0;

    MoveZ(z_origo);                         // Előbb a szerszámot mozgatjuk
    SetDirection(DIR_X, 0);                 // Irányok beállítása a végállások felé 
    SetDirection(DIR_Y, 0);

    uint32_t homing_virtual_steps = 5000;   // Virtuális lépésszám gyorsulási rámpához
    GenerateAxisRamp(homing_virtual_steps, homing_start_period, homing_period_xy,// Gyorsulási rámpák generálása 
                     accel_ramp_x, &accel_steps_x, &decel_steps_x, &use_ramp_x);
                     
    GenerateAxisRamp(homing_virtual_steps, homing_start_period, homing_period_xy, 
                     accel_ramp_y, &accel_steps_y, &decel_steps_y, &use_ramp_y);

    START_LIMIT_TIMER();                                                         // Háttérben indul a végállások figyelése
    uint16_t step_count = 0;                                                     // Számláló a felfutáshoz
    while (!home_x_done || !home_y_done) {
        uint16_t current_delay = homing_period_xy;                               // Késleltetés kiszámítása, 
        if (use_ramp_x) {                                                        // az X tengely rámpája alapján, 
            if (step_count < accel_steps_x) {                                    // vagy amíg el nem éri a végét
                uint16_t index = (step_count * ramp_points) / accel_steps_x;
                if (index >= ramp_points) index = ramp_points - 1;
                current_delay = accel_ramp_x[index];
            } else {                                                             // Ha letelt a felfutás, 
                current_delay = homing_period_xy;                                // tartja a sebességet
            }
        }
        if (step_count < 65535) step_count++;                                    // Lépés számláló növelése
        uint8_t step_mask = 0; 
        if (!home_x_done) step_mask |= (1 << STEP_X);                            // Lépés maszk összeállítása
        if (!home_y_done) step_mask |= (1 << STEP_Y);
        StepOne(step_mask, current_delay);                                       // Lépés végrehajtása
    }

    STOP_LIMIT_TIMER();                                                          // Timer leállítása

    SetDirection(DIR_X, 1);                                                      // Irány megfordítása
    SetDirection(DIR_Y, 1); 
    uint32_t max_pull_off = (uint32_t)(2.0 * steps_per_mm_y);                    // 2 mm távolság lépésben
    for (uint32_t i = 0; i < max_pull_off; i++) {                                // Lassú léptetéssel lejövünk a kapcsolóról
        uint8_t step_mask = 0;
        step_mask |= (1 << STEP_X) || (1 << STEP_Y);;                            // Lépés maszk összeállítása (mindkettővel lépünk)
        StepOne(step_mask, homing_period_xy);                                    // Lépés végrehajtása
    }

    autohoming = 0;                                                              // Munkaváltozó, timereknek
    current_x = 0;                                                               // Aktuáli spozíció mentése
    current_y = 0;
    return;
}

// Rajzolási paraméterek beállítása
void SetParameters(char *buffer) {
    char *speed_xy_ptr = strchr(buffer, 'S');                                           // S paraméter -> X, Y sebesség
    char *speed_start_ptr = strchr(buffer, 'P');                                        // P paraméter -> X, Y kezdeti sebesség
    char *speed_z_ptr = strchr(buffer, 'V');                                            // Z paraméter -> Z sebesség
    char *a_ptr = strchr(buffer, 'A');                                                  // A paraméter -> Gyorsulási rámpa (%)

    if (speed_xy_ptr != NULL) {                                                         // X, Y sebesség beállítása mm/s-ből
        float speed_mm_s = atof(speed_xy_ptr + 1);                                      // Kiolvasás
        if (speed_mm_s > 0) {
            float calculated_period = 1000000.0 / (speed_mm_s * steps_per_mm_x);        // Átváltás periódusidőre
            period_xy = (uint32_t)(calculated_period + 0.5);                            // Casting esetén mindig lefelé kerekít -> + 0.5
        }
    }
    if (speed_start_ptr != NULL) {                                                      // Kezdeti sebesség beállítása mm/s-ből
        float start_speed_mm_s = atof(speed_start_ptr + 1);
        if (start_speed_mm_s > 0) {
            float calculated_start_period = 1000000.0 / (start_speed_mm_s * steps_per_mm_x);
            start_period_xy = (uint16_t)(calculated_start_period + 0.5);
        }
    }
    if (speed_z_ptr != NULL) {                                                          // Z tengely sebességének beállítása
        float z_speed_mm_s = atof(speed_z_ptr + 1);
        if (z_speed_mm_s > 0) {
            period_z = (uint32_t)(1000000.0 / z_speed_mm_s); 
        }
    }
    if (a_ptr != NULL) {                                                                // Gyorsulási százalék beállítása 0-50% között
        float accel_percent_val = atof(a_ptr + 1);
        if (accel_percent_val >= 0 && accel_percent_val <= 50) {
            accel_percent = (uint8_t)(accel_percent_val + 0.5);
            decel_percent = (uint8_t)(accel_percent_val + 0.5);
        }
    }
    return;
}

// ===== Üzenet feldolgozása =====
void HandleMessage(char buffer[BUFF_SIZE]){
    char *x_pointer = strchr(buffer, 'X');                                  // Keresőpointerek 
    char *y_pointer = strchr(buffer, 'Y');
    char *z_pointer = strchr(buffer, 'Z');
    char *autohome_pointer = strchr(buffer, 'H');
    char *problem_solved_pointer = strchr(buffer, 'O');
    char *m_pointer = strchr(buffer, 'M');

    while (error_mode){                                                     // Végtelen ciklus hiba esetén: megállítja a gépet, amíg a hiba fennáll
        if (problem_solved_pointer != NULL){                                // Ha érkezett nyugtázó üzenet a számítógéptől l
            error_mode = 0;                                                 // Hibaállapot törlése 
            InitDrivers();                                                  // Léptetőmotor driverek újraengedélyezése
        }
    }
    if (x_pointer != NULL && y_pointer != NULL){                            // Ha az üzenet tartalmaz X és Y koordinátát is 
        Vector current_vector = CalculateVector(x_pointer, y_pointer);      // Vektor kiszámítása a koordinátákból
        StepXY(current_vector);                                             // X-Y motorok léptetése a vektor alapján
        UpdateXYPosition(current_vector);                                   // Aktuális X-Y pozíció frissítése
    } 
    else if (z_pointer != NULL && x_pointer == NULL){                       // Tiszta Z tengelyes mozgás
        float target_z = CalculateZPosition(z_pointer);                     // Cél Z koordináta kiszámítása a kapott értékből
        MoveZ(target_z);                                                    // Z tengely mozgatása a célpozícióra
        UpdateZPosition(target_z);                                          // Aktuális Z pozíció frissítése
    }
    else if (autohome_pointer != NULL) {                                    // Ha érkezett home parancs ('H')
        AutoHome();                                                  
    }
    else if (m_pointer != NULL) {                                           // Ha M-kód érkezett 
        int m_cmd = atoi(m_pointer + 1);                                    // Szám kiolvasása 
        if (m_cmd == 3) {                                                   // M3: Fúró elindítása 
            RunDrill(1, DRILL_MAX_SPEED); 
        } 
        else if (m_cmd == 5) {                                              // M5: Fúró leállítása
            StopDrill(); 
        }
    }
    else if (strchr(buffer, 'S') != NULL || strchr(buffer, 'P') != NULL ||  // Sebesség vagy gyorsulás paraméterek
        strchr(buffer, 'A') != NULL || strchr(buffer, 'V') != NULL){
        SetParameters(buffer);                                              // Paraméterek beállítása 
    }
    
    return;                                                             
}

// ===== Időzítő beállítások =====

// Servo motor időzítője

ISR(TIMER1_OVF_vect) {                      // Timer1 túlcsordulás
    if (servo_running) {
        PORTB |= (1 << SERVO_PIN);          // Vezérlő pin = 1 -> impulzus kezdete (előző alacsony tartomány vége)
        OCR1A = current_pulse_ticks;        // Kívánt szögnek megfelelő érték betöltése (kitöltési tényező beállítása)
    }
}

ISR(TIMER1_COMPA_vect) {                    // Betöltött érték elérése
    if (servo_running) {
        PORTB &= ~(1 << SERVO_PIN);         // Vezérlő pin = 0 -> impulzus vége (alacsony tartomány kezdete)
    }
}

// Végálláskapcsoló olvasás időzítője

ISR(TIMER2_COMPA_vect) {                    // Timer2 túlcsordulás
    if (!(PINB & (1 << LIM_X))) {           // Ha az X végállás lenyomódik
        if (autohoming) {home_x_done = 1;}  // Ha autohoming közben nyomódik le
        else {
            PORTB |= (1 << EN);             // Ha nyomtatás közben nyomódik be, akkor letiltjuk a motor vezérlést
            error_mode = 1;                 // És kiáll a gép hibára
            SerialSendError(0);             // Hibaüzenet küldése
            }          
    }
    if (!(PINB & (1 << LIM_Y))) {           // Ha az Y végállás lenyomódik
        if (autohoming) {home_y_done = 1;}  
        else {
            PORTB |= (1 << EN);
            error_mode = 1;
            SerialSendError(0);
            }          
    }
}


// =====  Főprogram  =====

int main(void){
    InitDrivers();                                      // regiszterek beállítása
    InitSerial();
    InitServo();
    InitLimitsAndDrill();
    
    while (1){
      if (SerialCheckIncoming()) {                      // ha van bejövő adat
        char message[BUFF_SIZE];        
        SerialReadLine(message);                        // üzenet olvasása
        HandleMessage(message);                         // vezérlés
        SerialAcknowledge();                            // nyugtázás
        }
      } 
    return 0;
}

