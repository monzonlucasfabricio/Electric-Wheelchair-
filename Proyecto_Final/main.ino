#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#include "uTimerLib.h"
#include <LiquidCrystal.h> 
#include "Button.h"
#include "test.h"

#define TEST
#define PLATFORMIO

// Only for testing purpose
#ifdef TEST
#define JOYSTICK_READ(x) joistick_read(x)
#define TEMP_READ(x)  temperature_read(x)
#define DIRECTION_READ(x) bw_fw(x)
#else
#define JOISTICK_READ(x) analogRead(x)
#define TEMP_READ(x) analogRead(x)
#define DIRECTION_READ(x) digitalRead(x)
#endif

#define N_AVG 255
#define LEFT -1
#define RIGHT 1
#define V_MIN 300
//#define Button PB0
//Pines

  int JoyX = PA0;
  int JoyY = PA1;
  int pwm = PA2;
  int CalDet = PA3;
  int Bat = PA4;
  int temp_in = PA5;
  int StepDir = PA6 ;
  int StepPulso = PA7;
  const bool pullup = true;
  Button ButtonMenu(PB0, pullup);
  const int buttonPin = PB0;
  int pwmdir = PA8;
  int FRSw = PA9;
  int FAN = PA10;
  int HALL = PB4;
  int Alarm = PB5; 
  const int rs = PB11, en = PB10, d4 = PB9, d5 = PB8, d6 = PB7, d7 = PB6; // Declaramos variables de LCD
  
//Variables.

  double DGrad = 0;
  double Anglast = 90; 
  int Sobj = 0;
  int Sact = 0;
  int steps = 0;
  int caso;
  int Ang = 0;
  int V = 0;
  int StepPos = 0;
  int flag = 0;
  int X,Y,dist;
  int displayflag = 0; 
  int conta = 0; 
  int TempC;
  int buttonPushCounter = 0;   // counter for the number of button presses
  int buttonState = 0;         // current state of the button
  int lastButtonState = 0;     // previous state of the button

//Inicializacion
  LiquidCrystal lcd(rs,en,d4,d5,d6,d7); //Inciamos LCD
  AccelStepper Step(1, StepPulso, StepDir);
  
//Maquina de Estados.  
  typedef enum{
      INI,
      CAL,
      STD,
      RUN,
      //CONFIG,
  }states_e;

  states_e State;
  states_e lastState;

//Funciones.
  double angulo(int a, int b);
  int PyDist(int a, int b);
  int ME_CONTROL(void);
  double RVGrad(void);
  int RVVel(void);
  void goToSteady(void);
  void buttonCheck(void);
  int LCD(void);


void setup() {
  Serial.begin(115200);

  // Creo un timer para llevar el sistema a Steady si pasan 10 segundos sin movimiento
  TimerLib.setInterval_s(goToSteady, 10);

  //Setup de Pantalla LCD
  lcd.begin(16,2); //definimos el tamaño del lcd 16*2
  lcd.setCursor(0,0);
  lcd.print("FREEDOM OF WHEEL");
  lcd.setCursor(3,1);
  lcd.print("Bienvenido");
  
  //Pinout
  pinMode (JoyX, INPUT);
  pinMode (JoyY, INPUT);
  pinMode (StepPulso, OUTPUT);
  pinMode (StepDir, OUTPUT);
  pinMode (pwm, PWM);
  pinMode (CalDet, INPUT);
  pinMode (PC13, OUTPUT);
  pinMode (buttonPin, INPUT);
  pinMode (FRSw, INPUT);
  
  //Stepper
  Step.setMaxSpeed (10000);
  Step.setSpeed (10000);
  Step.setAcceleration(50000);
  digitalWrite(StepPulso, LOW);
  digitalWrite(StepDir, LOW);

  //Etados
  State = INI;
  lastState = INI;
  
  delay(4000);
  lcd.clear();
}


void loop() {
  buttonCheck();  
  int ret = ME_CONTROL();
  if (ret != 0){
    // Control error}
  }
  //sensors.requestTemperatures();
  //TempC = (sensors.getTempCByIndex(0));
  LCD();
  }



//Sentencias creadas
void goToSteady(void){
  if (V == 0){
     State = STD;
  }
}


//Programas de Calculo 
double angulo(int a, int b){
  int deltax = a - 2048;
  int deltay = b - 2048;
  double rad = atan2(deltay,deltax);
  double deg = rad * 57.295779513082320876798154814105;
  return deg;
}


int PyDist(int a, int b){
  int deltax = a - 2048;
  int deltay = b - 2048;
  int dist = sqrt(deltax*deltax + deltay*deltay);
  return dist;
}


//Programas de lectura de variables
double RVGrad(void){
  X = ANALOG_READ(JoyX);
  Y = ANALOG_READ(JoyY);
  double ang = angulo(X,Y);
  return ang;
}

int RVVel(void){
  X = ANALOG_READ(JoyX);
  Y = ANALOG_READ(JoyY);
  int dist = PyDist(X,Y);
  return dist;
}


//Contador de pulsos 
void buttonCheck(void)
{
  if (ButtonMenu.check()==LOW){
    Serial.println("Presionado");
    buttonPushCounter++;
    Serial.println(buttonPushCounter);
  if(buttonPushCounter == 4){
    buttonPushCounter = 0;
    }
  }
}


//Programa del display LCD
int LCD(void){

  switch(buttonPushCounter){
    case 0:
    {
     if (State == INI){
      Serial.println("Caso 0 - INI");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("INICIANDO");
      lcd.setCursor(0,1);
      lcd.print("SISTEMA...");
      //delay(100);
     }
     else if (State == CAL){
      Serial.println("Caso 0 - CAL");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CALIBRANDO");
      lcd.setCursor(0,1);
      lcd.print("...");
      //delay(100);
     }
     else if (State == STD){
      Serial.println("Caso 0 - STD");
      lcd.clear();
      lcd.setCursor(5,0);
      lcd.print("Steady");
      lcd.setCursor(5,1);
      lcd.print("State.");
      //delay(100);

     }
     else if (State == RUN){
      Serial.println("Caso 0 - RUN");
      lcd.clear();
      lcd.setCursor(4,0);
      lcd.print("Running");
      lcd.setCursor(5,1);
      lcd.print("State.");
      //delay(100);

     }
    }
    break;

    case 1:
    {
    //Mostrar la velocidad en el display.
    Serial.println("Caso 1 - Mostrando Velocidad");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Velocidad:");
    lcd.setCursor(0,1);
    lcd.print("km/h");
    //delay(1000);
    }
    break;
    
    case 2:
    {
    //Mostrar la carga de bateria en el display.
    Serial.println("Caso 2 - Mostrando carga de bateria");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Bateria:");
    lcd.setCursor(0,1);
    lcd.print("%");
    //delay(1000);
    }
    break;

    case 3:
    {
    //Mostrar la temperatura del equipo en el display.
    int sumaT[N_AVG];
    int sumaTT = 0;

    for (uint8_t i = 0; i<N_AVG ; i++){
      int temp = TEMP_READ(temp_in);
      temp = temp * 0.08056640625;
      sumaT[i] = temp;
      sumaTT = sumaTT + sumaT[i];
    }

    sumaTT = sumaTT/N_AVG;
    Serial.println("Caso 3 - Mostrando temperatura del equipo");
    Serial.print("La temperatura es:");
    Serial.print(sumaTT);
    Serial.println("°C"); 
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Temperatura:");
    lcd.setCursor(0,1);
    lcd.print(sumaTT);
    lcd.print("°C");
    //delay(1000);
    }
    break;
    
    default:
    {
        // Do nothing
    }
    break;
  }
  
}
  
//Programa de Control 
int ME_CONTROL(void){
  static int count = 0;
  static int diff = 0;
  static int temp = 0;
  static int move_step = 0;
  
  
  switch(State){
      case INI:
      {
          //Ininiciar contador de Velocidad, Bateria y Temperatura
          count++;
          if (count == 1000){
            //Serial.println("Voy a modo run");
            lastState = State;
            State = CAL;
            count = 0;
          }
           
      }
      break;
      case CAL:
      {
          //Buscar 0 haciendo una rotacion total de la rueda
          count++;
          //Entrar en modo calibracion 
          if (count == 1000){
            //Serial.println("Voy a Steady State");
            lastState = State;
            State = STD;
            count = 0;
          }
          /* while(FlagDet == 0){
            bool ret = digitalRead(CalDet);
            if (ret != HIGH){
              step.move(/mov++///);
              /*step.run();
            }
            else{
              FlagDet = 1;
            }
          }*/
      }
      break;
      case STD:
      {
        double grad = RVGrad();
        int vel = RVVel();
        if (vel > 300 && (grad > 100 || grad < 80)){
          lastState = State;
          State = RUN;
        }
        if (vel > 300 && (grad < 100 || grad > 90)){
          lastState = RUN;
          State = RUN;
        }
        if (lastState == CAL && vel > 0){
          lastState = State;
          State = RUN;
        }
        /*double Ang = RVGrad();
        int V = RVVel();  
        DGrad = abs(Anglast - Ang);
        if(DGrad > 10 or V > 200){
          lastState = State;
          State = RUN;   
        
        }
        Anglast = Ang;*/
      }
      break;
      case RUN:
      {
        //Serial.println ("Running");      
        // Variable declaration
        int x,y;
        int sumaX[N_AVG];
        int sumaY[N_AVG];
        float sumaTX = 0;
        float sumaTY = 0;
        int marcha = DIRECTION_READ(FRSw);

        /*analogWrite(pwm,(2*V))*/
        for (uint8_t i = 0; i<N_AVG ; i++){
          #ifdef PLATFORMIO
          analogReadResolution(12);
          #endif
          x = ANALOG_READ(JoyX);
          y = ANALOG_READ(JoyY);
          sumaX[i] = x;
          sumaY[i] = y;
        }
        
        for (uint8_t i = 0; i<N_AVG; i++){
          sumaTX = sumaTX + sumaX[i];
          sumaTY = sumaTY + sumaY[i];
        }
        
        sumaTX = sumaTX/N_AVG;
        sumaTY = sumaTY/N_AVG;
        
        
        if (sumaTX < 10){
          sumaTX = 0;
        }
        if (sumaTY < 10){
          sumaTY = 0;
        }
        
        bool sig = 0;
        temp = angulo(sumaTX,sumaTY);

        if (temp < 0) sig = 1;
        if (temp >= 0) sig = 0;

        Ang = abs(temp);
        V = PyDist(sumaTX,sumaTY);

        if (V < 300){
          V = 0;
          
        }
        if (V == 0 && Ang < 80){
          Ang = 90;
        }


        Serial.print ("El angulo seleccionado es:");
        Serial.println (Ang);
        Serial.print ("La velocidad seleccionada es:");
        Serial.println (V);

        if (Ang >= 0 && Ang < 5.5){
          //Caso = 0; 
          Sobj = 400;
        }
        else if (Ang >= 5.625 && Ang < 16.875){
          //Caso = 1; 
          Sobj = 350;
        }
        else if (Ang >= 16.875 && Ang < 28.125){
          //Caso = 2;
          Sobj = 300;
        }
        else if (Ang >= 28.125 && Ang < 39.375){
          //Caso = 3;
          Sobj = 250;
        }
        else if (Ang >= 39.375 && Ang < 50.625){
          //Caso = 4;
          Sobj = 200;
        }
        else if (Ang >= 50.625 && Ang < 61.875){
          //Caso = 5;
          Sobj = 150;
        }
        else if (Ang >= 61.875 && Ang < 73.125){
          //Caso = 6;
          Sobj = 100;
        }
        else if (Ang >= 73.125 && Ang < 84.375){
          //Caso = 7;
          Sobj = 50;
        }
        else if (Ang >= 84.375 && Ang < 90){
          //Caso = 8;
          Sobj = 0;
        }
        else if (Ang >= 90 && Ang < 95.625){
          Sobj = 0;
        }
        else if (Ang >= 95.625 && Ang <= 106.875){
          //Caso = 9;
          Sobj = -50;
        }
        else if (Ang >= 106.875 && Ang <= 118.125){
          //Caso = 10;
          Sobj = -100;
        }
        else if (Ang >= 118.125 && Ang <= 129.375){
          //Caso = 11;
          Sobj = -150;
        }
        else if (Ang >= 129.375 && Ang <= 140.625){
          //Caso = 12;
          Sobj = -200;
        }
        else if (Ang >= 140.625 && Ang <= 151.875){
          //Caso = 13;
          Sobj = -250;
        }
        else if (Ang >= 151.875 && Ang <= 163.125){
          //Caso = 14;
          Sobj = -300;
        }
        else if (Ang >= 163.125 && Ang <= 174.375){
          //Caso = 15;
          Sobj = -350;
        }
        else if (Ang >= 174.375 && Ang <= 180){
          //Caso = 16;
          Sobj = -400;
        }
        else{
          //Caso = 17;
          Sobj = 0;
        }

        diff = Sobj - steps;
        if (diff != 0){
          //Serial.print ("Nos movemos");
          //Serial.println (steps);
          
          if (Ang >= 90 && diff < 0){
            steps-=10;
            move_step = LEFT;
          }
          else if (Ang >= 90 && diff > 0){
            steps+=10;
            move_step = RIGHT;
          }
          else if (Ang < 90 && diff > 0){
            steps+=10;
            move_step = RIGHT;
          }
          else if (Ang < 90 && diff < 0){
            steps-=10;
            move_step = LEFT;
          }
          // Si no funciona el 1 y -1, cambiar la variable move_step por steps en la siguiente linea
          //Step.moveTo(steps);
          //Step.run();
          Step.runToNewPosition(steps);
        
        }
        uint16_t veljoy;
        switch(marcha){
          case HIGH:
          {
          Serial.println("Forward");
          //Enable en 0
          veljoy = map(V, 0, 2045, 0, 65535);
          Serial.println(veljoy);
          pwmWrite(pwm,veljoy);
          }
          break;
          case LOW:
          {
          Serial.println("Backwards");
          //Enable en 1 
          veljoy = map(V, 0, 2045, 0, 65535);
          Serial.println(veljoy);
          pwmWrite(pwm,veljoy);           
          }
          break;
        }
      }
      //delay(1);
      break;
      default:
      {
        // Do nothing
      }
      break;
  }
  return 0;
}
