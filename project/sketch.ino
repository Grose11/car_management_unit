#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // Defina o endereço I2C do módulo e as dimensões do display (20x4)

const int tempSensorPin = A0;          // Pino analógico para o sensor de temperatura da água
const int oilPressureSensorPin = A1;   // Pino analógico para o sensor de pressão de óleo
const int rpmSensorPin = 2;            // Pino digital para o sensor de RPM do motor

volatile unsigned int rpmPulseCount = 0;  // Contador de pulsos de RPM
unsigned long lastUpdateTime = 0;       // Última atualização das RPM
unsigned int rpm = 0;                   // Valor atual de RPM

void rpmSensorISR() {
  rpmPulseCount++;
}

void setup() {
  lcd.init();                      // Inicializa o display
  lcd.backlight();                 // Liga o backlight

  pinMode(rpmSensorPin, INPUT_PULLUP); // Configura o pino do sensor de RPM como entrada com pull-up interno
  attachInterrupt(digitalPinToInterrupt(rpmSensorPin), rpmSensorISR, RISING); // Configura a interrupção para contar pulsos

  Serial.begin(9600);              // Inicializa a comunicação serial
}

void loop() {
  // Leitura da temperatura da água
  int tempSensorValue = analogRead(tempSensorPin);
  float tempVoltage = (tempSensorValue / 1023.0) * 5.0;
  float temperaturaC = (tempVoltage - 0.5) * 100.0;

  // Leitura da pressão de óleo em PSI
  int oilPressureSensorValue = analogRead(oilPressureSensorPin);
  float oilPressureVoltage = (oilPressureSensorValue / 1023.0) * 5.0;
  float pressurePSI = (oilPressureVoltage - 0.5) * 100.0;

  // Convertendo de PSI para bar
  float pressureBar = pressurePSI * 0.0689476;

  // Atualiza as RPM do motor a cada segundo
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= 1000) {
    lastUpdateTime = currentTime;
    noInterrupts();  // Desativa interrupções temporariamente para evitar interferência
    rpm = (rpmPulseCount * 60) / 2; // Calcula as RPM (pulsos por minuto)
    rpmPulseCount = 0; // Zera o contador de pulsos
    interrupts();  // Reativa as interrupções

    // Limpa o display e define o cursor na linha 0, coluna 0
    lcd.clear();
    lcd.setCursor(0, 0);

    // Mostra "ROSA BMW" na primeira linha do display
    lcd.print("ROSA BMW");

    // Define o cursor na linha 1, coluna 0
    lcd.setCursor(0, 1);
    // Mostra as leituras da temperatura, pressão de óleo e RPM simultaneamente
    lcd.print("Temp: ");
    lcd.print(temperaturaC, 1); // Mostra uma casa decimal
    lcd.print("C");

    lcd.setCursor(0, 2);
    lcd.print("Oil: ");
    lcd.print(pressureBar, 2); // Mostra duas casas decimais
    lcd.print(" bar");

    lcd.setCursor(0, 3);
    lcd.print("RPM: ");
    lcd.print(rpm);
    lcd.print(" RPM");
  }
}