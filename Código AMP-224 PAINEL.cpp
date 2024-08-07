//Ampera Racing - AMP-224 PAINEL
//Analista Responsável - Guilherme Lettmann Penha
//Head - Tomas Carrasco Ferrarezi 
//Diretor - Marina Grisotti
//Projetista - Lucas Paiva


#include <Arduino.h>
#include <stdlib.h>
#include <driver/adc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/can.h"
#include "EasyNextionLibrary.h" //Library do Display

//Configuração chave seletora
#define SELECTOR_PIN_1 GPIO_NUM_23
#define SELECTOR_PIN_2 GPIO_NUM_22
#define SELECTOR_PIN_3 GPIO_NUM_19
#define SELECTOR_PIN_4 GPIO_NUM_21
#define SELECTOR_PIN_5 GPIO_NUM_18 // mudei os pinos
portMUX_TYPE selectorMux = portMUX_INITIALIZER_UNLOCKED;

//Configuracao pinos Falha Driverless
#define EBS_PIN GPIO_NUM_5
#define FAULT_DL_PIN GPIO_NUM_4


//Declaração das variáveis
uint16_t motor_current = 0, power = 0, SelectorPosition, bse = 0, lastPosition = 0, accumulatorTemp, apps = 0, RTD, REGEN = 0, GPS, motorTemp, lowVoltage, StateofCharge, fault_bms, fault_inv, fault_ecu, inversorVoltage, RPM;
float speed = 0, highVoltage; // revisao das variaveis a serem mostradas
long double accumulatorCurrent;
int CurrentForm = 0; // variavel para controle da página atual
int period = 100; // tempo entre envio de dados para o display
unsigned long time_now = 0; // variavel para controle de envio de dados para o display
bool display_lock = false; // false: pode mudar a página, true: nao pode mudar a página
bool botao = false; // rtd eh inicialmente low
bool emergency = false; // ebs manda booleana
bool fault_dl = false; // emergencia manda booleana
uint16_t map_speed, map_rpm, inv_temp;

#define REGEN_PIN GPIO_NUM_15
portMUX_TYPE REGENbutton = portMUX_INITIALIZER_UNLOCKED;

#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_16

//Declaracao RTC
uint8_t month = 0;
uint8_t day = 0;
uint8_t hour = 0;
uint8_t minute = 0;
uint8_t sec = 0;

EasyNex myNex(Serial2); //definir o Serial2 como serial do display

//Declaração das tarefas
TaskHandle_t Task1 = NULL;
TaskHandle_t Task2 = NULL;
TaskHandle_t Task3 = NULL;
TaskHandle_t Task4 = NULL;


void setupCan(){
  can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, CAN_MODE_NORMAL);
  can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
  can_filter_config_t f_config;
  f_config.acceptance_code = (0x7FF << 21);
  f_config.acceptance_mask = ~(0x020 << 21);
  f_config.single_filter = true;

  
  if(can_driver_install(&g_config, &t_config, &f_config) == ESP_OK){
    // This installs the CAN driver
    //Serial.print("CAN driver installed\n");
  }
  if (can_start() == ESP_OK) {
    // This starts the CAN driver
    //Serial.print("CAN driver started\n");
  }
}

/*
  functions of each task
  TASK1 -> CITEX selector and regen button handling
  TASK2 -> update display
  TASK3 -> CAN BUS
*/

void Task1code( void * pvParameters ) //task do seletor
{
  while(1)
  {
    vTaskDelay(400 / portTICK_PERIOD_MS);
    //Mudança da chave seletora
    if (display_lock == false){ 
      if (SelectorPosition != CurrentForm && SelectorPosition == 0)
      {
        myNex.writeStr("page page0");
        CurrentForm = 0;
        //Serial.print("page0");
      }
      if (SelectorPosition != CurrentForm && SelectorPosition == 1)
      {
        myNex.writeStr("page page1");
        CurrentForm = 1;
        //Serial.print("page1");
      }
      if (SelectorPosition != CurrentForm && SelectorPosition == 2)
      {
        myNex.writeStr("page page2");
        CurrentForm = 2;
       // Serial.print("page2");
      }
      if (SelectorPosition != CurrentForm && SelectorPosition == 3)
      {
        myNex.writeStr("page page3");
        CurrentForm = 3;
        //Serial.print("page3");
      }
      if (SelectorPosition != CurrentForm && SelectorPosition == 4)
      {
        myNex.writeStr("page page4");
        CurrentForm = 4;
        Serial.print("page4");
      }
    }
  }
  //Acionamento do botão REGEN no volante
  if (digitalRead(REGEN_PIN)==HIGH) {
      REGEN = 1;
    }
  else REGEN = 0; 
  
  vTaskDelay(30 / portTICK_PERIOD_MS);
}

void Task2code( void * pvParameters )
{
  while(1)
  { 
    if (digitalRead(REGEN_PIN)==HIGH) {
      botao = HIGH;
    }
  else { 
    botao = LOW;  
  }
    //Atualização dos objetos do painel
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //Serial.println("------------Entrou 2----------------");
    if (millis() >= time_now + period){ //update display variables every 100 miliseconds 
      time_now += period;
      switch (CurrentForm) // Varia com a página atual do display
      {
         case 0:
          month = myNex.readNumber("n1.val");
          day = myNex.readNumber("n0.val");
          hour = myNex.readNumber("n3.val");
          minute = myNex.readNumber("n4.val");
          sec = myNex.readNumber("n5.val");
          //Serial.println(sec);
        break;
        case 1: //Testes
          //vTaskDelay(20 / portTICK_PERIOD_MS);
          month = myNex.readNumber("n1.val");
          day = myNex.readNumber("n0.val");
          hour = myNex.readNumber("n3.val");
          minute = myNex.readNumber("n4.val");
          sec = myNex.readNumber("n5.val");
          myNex.writeNum("n6.val", highVoltage); //tensao HV
          myNex.writeNum("n7.val", inversorVoltage); //tensao inversor
          myNex.writeNum("n8.val", fault_bms); //Erro BMS
            if (fault_bms != 0) {
            myNex.writeNum("n8.pco", 63488);
            }
            else {
            myNex.writeNum("n8.pco", 24122);
            }
          myNex.writeNum("n9.val", motorTemp); //temperatura Motores
          myNex.writeNum("n10.val", inv_temp); //Temperatura Inversor
            if (inv_temp > 70) {
              myNex.writeNum("n10.pco", 63488);
            }
            else {
              myNex.writeNum("n10.pco", 24122);
            }
          myNex.writeNum("n11.val", fault_inv); //Erro Inversor
            if (fault_inv != 0) {
            myNex.writeNum("n11.pco", 63488);
            }
            else{
              myNex.writeNum("n11.pco", 24122);
            }
          myNex.writeNum("n12.val", accumulatorCurrent); //Corrente Acumulador
          myNex.writeNum("n13.val", StateofCharge); //SOC
            if (digitalRead(REGEN_PIN)==HIGH) {
                myNex.writeNum("n13.pco", 2016);
              }
            else{
              if (StateofCharge < 20) {
              myNex.writeNum("n13.pco", 63488);
            }
              else{
                myNex.writeNum("n13.pco", 24122);
              }
            }
          myNex.writeNum("n14.val", fault_ecu); //Erro ECU
            if (fault_ecu != 0) {
              myNex.writeNum("n14.pco", 63488);
            }
            else{
              myNex.writeNum("n14.pco", 24122);
            }
        break;
        case 2: //Provas Curtas
          vTaskDelay(20 / portTICK_PERIOD_MS);
          month = myNex.readNumber("n1.val");
          day = myNex.readNumber("n0.val");
          hour = myNex.readNumber("n3.val");
          minute = myNex.readNumber("n4.val");
          sec = myNex.readNumber("n5.val");
          myNex.writeNum("n7.val", speed); //Velocidade
          map_speed = map(speed, 0, 100, 0, 180);
          myNex.writeNum("z1.val", map_speed);
          myNex.writeNum("n6.val", RPM);//rpm motor
          map_rpm = map(RPM, 0, 6000, 0, 180);
          myNex.writeNum("z0.val", map_rpm);
          myNex.writeNum("j0.val", bse);
          myNex.writeNum("j1.val", apps);

        break;
        case 3: //Enduro
          vTaskDelay(20 / portTICK_PERIOD_MS);
          month = myNex.readNumber("n1.val");
          day = myNex.readNumber("n0.val");
          hour = myNex.readNumber("n3.val");
          minute = myNex.readNumber("n4.val");
          sec = myNex.readNumber("n5.val");
          myNex.writeNum("n7.val", speed); //Velocidade
          map_speed = map(speed, 0, 100, 0, 180);
          myNex.writeNum("z0.val", map_speed);
          myNex.writeNum("n6.val", accumulatorCurrent); //Coreente
          if (accumulatorTemp > 60 || inv_temp > 70 || motorTemp > 70) {
              myNex.writeNum("c0.val", 1);
            }
            else {
              myNex.writeNum("c0.val", 0);
            }
           if (StateofCharge < 20) {
              myNex.writeNum("c1.val", 1);
            }
            else {
              myNex.writeNum("c1.val", 0);
            }
        break;
        case 4: // DVLESS
          vTaskDelay(20 / portTICK_PERIOD_MS);
          month = myNex.readNumber("n1.val");
          day = myNex.readNumber("n0.val");
          hour = myNex.readNumber("n3.val");
          minute = myNex.readNumber("n4.val");
          sec = myNex.readNumber("n5.val");
        default:
        break;  
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}


void Task3code (void * pvParameters)
{
  while(1) 
  {
  //Recebe a mensagem
  can_message_t message1;
  can_receive(&message1, pdMS_TO_TICKS(10));
    switch (message1.identifier)
    {    
    case 0x0B0:
        fault_inv = message1.data[0];
        fault_ecu = (message1.data[2] << (8) | message1.data[1]);
        //Serial.println("JESUS");
      break;
    case 0x0B1:
        motorTemp = (message1.data[3] << 8 | message1.data[2])/10;
        RPM = ((message1.data[1] << 8 | message1.data[0])-1000);
        motor_current = (message1.data[5] << 8 | message1.data[4])/10;
        speed = (RPM*50*3*60)/500000;
      break;
    case 0x0B2:
        inversorVoltage = (message1.data[1] << 8 | message1.data[0])/10;
        apps = message1.data[4];
        bse = message1.data[5];
        inv_temp = (message1.data[3] << (8) | message1.data[2])/10;
        //Serial.println(apps);
      break;
    case 0x672:
        fault_bms = message1.data[4];
        //Serial.println("BATEEL GARGALO");
      break;
    case 0x673:
        highVoltage = (message1.data[0] << (8) | message1.data[1]);
      break;
    case 0x674:
        accumulatorCurrent = (message1.data[0] << (8) | message1.data[1]);
      break;
    case 0x676:
        StateofCharge = message1.data[0];
      break;
    case 0x677:
        accumulatorTemp = message1.data[0];
      break;
    case 0x0A0:
      emergency = message1.data[0];
      if (fault_dl = true){
          digitalWrite(EBS_PIN, HIGH);
        }
      else {
          digitalWrite(EBS_PIN, LOW);
      }
      break;
    case 0x0A1:
      fault_dl = message1.data[0];
      if (fault_dl = true){
          digitalWrite(FAULT_DL_PIN, HIGH);
        }
      else {
          digitalWrite(FAULT_DL_PIN, LOW);
      }
    default:
      break;}
    
    vTaskDelay(50 / portTICK_PERIOD_MS);
}
}

void Task4code (void * pvParameters)
{
  while(1) 
  {
    //Envia a mensagem
    can_message_t message2;
    //Serial.println("mandandoCAN");
    message2.identifier = 0x777;         // CAN message identifier HYPE
    message2.data_length_code = 6;       // CAN message data length - 6 bytes
    message2.data[0] = (botao | SelectorPosition << 5);
    message2.data[1] = (month);
    message2.data[2] = (day);
    message2.data[3] = (hour);
    message2.data[4] = (minute);
    message2.data[5] = (sec);
    can_transmit(&message2, pdMS_TO_TICKS(10));
    vTaskDelay(50 / portTICK_PERIOD_MS); 
}
}

//Interrupção responsável pela mudança da chave seletora
//Interrupção responsável pela mudança da chave seletora
void IRAM_ATTR selector_change(){
  portENTER_CRITICAL_ISR(&selectorMux);
  //A chave seletora envia nível baixo na posição atual
    if ((digitalRead(SELECTOR_PIN_1))&&(digitalRead(SELECTOR_PIN_2))&&(digitalRead(SELECTOR_PIN_3))&&(digitalRead(SELECTOR_PIN_4))&&(SelectorPosition == 1)){
      SelectorPosition = 0;}
    if (digitalRead(SELECTOR_PIN_1) == 0){
      SelectorPosition = 1;}
    if (digitalRead(SELECTOR_PIN_2) == 0){
      SelectorPosition = 2;}
    if (digitalRead(SELECTOR_PIN_3) == 0){
      SelectorPosition = 3;}
    if (digitalRead(SELECTOR_PIN_4) == 0){
      SelectorPosition = 4;}
  portEXIT_CRITICAL_ISR(&selectorMux);
}
void SetupTasks(){
  xTaskCreatePinnedToCore
  (
                    Task1code,                /* Task function. */
                    "Task1",                  /* name of task. */
                    10000,                    /* Stack size of task */
                    NULL,                     /* parameter of the task */
                    0,                        /* priority of the task */
                    &Task1,                   /* Task handle to keep track of created task */
                    tskNO_AFFINITY);          /* lets the RTOS decide the core*/   

  xTaskCreatePinnedToCore
  (
                    Task2code,                // Task function. 
                    "Task2",                  // name of task. 
                    10000,                    // Stack size of task 
                    NULL,                     // parameter of the task 
                    0,                        // priority of the task 
                    &Task2,                   // Task handle to keep track of created task 
                    tskNO_AFFINITY);          // lets the RTOS decide the core

  xTaskCreatePinnedToCore
  (
                    Task3code,                // Task function. 
                    "Task3",                  // name of task. 
                    10000,                    // Stack size of task 
                    NULL,                     // parameter of the task 
                    0,                        // priority of the task 
                    &Task3,                   // Task handle to keep track of created task 
                    tskNO_AFFINITY);          // lets the RTOS decide the core

   xTaskCreatePinnedToCore
  (
                    Task4code,                // Task function. 
                    "Task4",                  // name of task. 
                    10000,                    // Stack size of task 
                    NULL,                     // parameter of the task 
                    0,                        // priority of the task 
                    &Task4,                   // Task handle to keep track of created task 
                    tskNO_AFFINITY);          // lets the RTOS decide the core

}

void setup(){
  Serial.begin(115200);
  setupCan();

  Serial2.begin(115200, SERIAL_8N1, 25, 33); //Início do serial do display
  myNex.begin(115200);

  delay (500); // give the display time to finish starting up

  //declaração de pinos 
  pinMode (SELECTOR_PIN_1, INPUT_PULLUP); 
  pinMode (SELECTOR_PIN_2, INPUT_PULLUP);
  pinMode (SELECTOR_PIN_3, INPUT_PULLUP); 
  pinMode (SELECTOR_PIN_4, INPUT_PULLUP); 
  pinMode (SELECTOR_PIN_5, INPUT_PULLUP);
  pinMode (REGEN_PIN, INPUT_PULLDOWN);
  pinMode (EBS_PIN, INPUT_PULLDOWN);
  pinMode (FAULT_DL_PIN, INPUT_PULLDOWN);
  
  //Declaração das interrupções
  attachInterrupt(SELECTOR_PIN_1, selector_change, CHANGE);
  attachInterrupt(SELECTOR_PIN_2, selector_change, CHANGE);
  attachInterrupt(SELECTOR_PIN_3, selector_change, CHANGE);
  attachInterrupt(SELECTOR_PIN_4, selector_change, CHANGE);
  attachInterrupt(SELECTOR_PIN_5, selector_change, CHANGE);

  //ver em qual página ta 
  if ((digitalRead(SELECTOR_PIN_1))&&(digitalRead(SELECTOR_PIN_2))&&(digitalRead(SELECTOR_PIN_3))&&(digitalRead(SELECTOR_PIN_4))&&(SelectorPosition == 1)){
      SelectorPosition = 0;
      Serial.println(SelectorPosition);}
  if (digitalRead(SELECTOR_PIN_1) == 0){
      SelectorPosition = 1;
      Serial.println(SelectorPosition);}
  if (digitalRead(SELECTOR_PIN_2) == 0){
      SelectorPosition = 2;
      Serial.println(SelectorPosition);}
  if (digitalRead(SELECTOR_PIN_3) == 0){
      SelectorPosition = 3;
      Serial.println(SelectorPosition);}
  if (digitalRead(SELECTOR_PIN_4) == 0){
      Serial.println(SelectorPosition);
      SelectorPosition = 4;}
 
  SetupTasks();
}

void loop() {
  vTaskDelay(100/ portTICK_PERIOD_MS);  
}
