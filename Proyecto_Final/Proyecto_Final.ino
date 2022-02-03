#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

#define N_AVG 255
#define LEFT -1
#define RIGHT 1

//Pins y Variables.
  int JoyX = PA0;
  int pwm = PA2;
  int JoyY = PA1;
  int StepPulso = PA7;
  int StepDir = PA6 ;
  int StepPos = 0;
  int flag = 0;
  int X,Y,dist;
  int CalDet = PA3;
  double DGrad = 0;
  double Anglast = 90; 
  int Sobj = 0;
  int Sact = 0;
  int steps = 0;
  int caso;
  int Ang = 0;
  int V = 0;
  
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

void setup() {
  Serial.begin(115200);

  //Pinout
  pinMode (JoyX, INPUT);
  pinMode (JoyY, INPUT);
  pinMode (StepPulso, OUTPUT);
  pinMode (StepDir, OUTPUT);
  pinMode (pwm, OUTPUT);
  pinMode (CalDet, INPUT);

  //Stepper
  AccelStepper Step(1, StepPulso, StepDir);
  Step.setMaxSpeed (10000);
  Step.setSpeed (10000);
  Step.setCurrentPosition(0);
  Step.setAcceleration(50000);
  digitalWrite(StepPulso, LOW);
  digitalWrite(StepDir, LOW);

  //Etados
  State = INI;
  lastState = INI;

}


void loop() {
  int ret = ME_CONTROL();
  if (ret != 0){
    // Control error
  }

}


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


double RVGrad(void){
  X = analogRead(JoyX);
  Y = analogRead(JoyY);
  double ang = angulo(X,Y);
  return ang;
}


int RVVel(void){
  X = analogRead(JoyX);
  Y = analogRead(JoyY);
  int dist = PyDist(X,Y);
  return dist;
}


int ME_CONTROL(void){
  static int count = 0;
  static int diff = 0;
  static int temp = 0;
  static int move = 0;
  
  switch(State){
      case INI:
      {
          Serial.println ("Saludo display");
          //Entrar en modo calibracion 
          //Ininiciar contador de Velocidad, Bateria y Temperatura
          if (count == 2){
            Serial.println("Voy a modo run");
            lastState = State;
            State = RUN;
          }
          count++; 
      }
      break;
      case CAL:
      {
          //Buscar 0 haciendo una rotacion total de la rueda
          Serial.println ("calibrando...");
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
          lastState = State;
          //State = STD;
          State = RUN;
      }
      break;
      case STD:
      {
        //Display print("Listo para moverse")
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
        Serial.println ("Running");

        // Variable declaration
        int x,y;
        int sumaX[N_AVG];
        int sumaY[N_AVG];
        float sumaTX = 0;
        float sumaTY = 0;

        /*analogWrite(pwm,(2*V))*/
        for (uint8_t i = 0; i<N_AVG ; i++){
          analogReadResolution(12);
          x = analogRead(JoyX);
          analogReadResolution(12);
          y = analogRead(JoyY);
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
        else if (Ang >= 84.375 && Ang <= 90){
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
          Serial.print ("Nos movemos");
          Serial.println (steps);
          
          if (Ang > 90 && diff < 0){
            steps--;
            move = LEFT;
          }
          else if (Ang > 90 && diff > 0){
            steps++;
            move = RIGHT;
          }
          else if (Ang < 90 && diff > 0){
            steps++;
            move = RIGHT;
          }
          else if (Ang < 90 && diff < 0){
            steps--;
            move = LEFT;
          }
          // Si no funciona el 1 y -1, cambiar la variable move por steps en la siguiente linea
          Step.moveTo(move);
          Step.run();
        }
      }
      break;
      default:
      {
        // Do nothing
      }
      break;
  }
  return 0;
}