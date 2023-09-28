#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <TimerOne.h>
#include <Wire.h>
#include <SoftwareSerial.h>

/*SUPER GUANTI --- UNA CREACION DE M.A.L. (Magalí, Aylen, Lautaro)*/

#define FALSE 0
#define TRUE 1

#define incremento 13 
#define inicio A0 //El que esta mas cercano al pin 1

#define infra1 12
#define infra2 A3
#define infra3 A2
#define infra4 A1 
#define infra5 11  
// 74hc595
#define pinLatch 9   
#define clockPin 10 
#define dataPin 8  

Servo miservo_1, miservo_2, miservo_3;

LiquidCrystal_I2C lcd(0x3F, 16, 2);

SoftwareSerial Bluetooth(2, 4); //rx, tx

//son variables que funcionan en la funcion que se interrumpe por lo que se declaran como volatile
volatile int tIncremento = 0;
volatile int tInicio = 0;
volatile int tInfras = 0;
volatile int taux = 0;
volatile int tauxmili = 0;
volatile int tlcd = 0;
volatile int tmin = 0;
volatile int tseg = 0;
volatile int thora = 0;

int estadoPrograma = 1;
int estadoRetencionIncremento = 1;
int estadoRetencionInicio = 1;
int estadoLcd = 0;

int viajesSeleccionados = 0;
int viajesRealizados = 0;
int aleatorio = 0;
int numAnterior = 0; 

int grados1 = 0;
int grados2 = 0;
int grados3 = 60;

bool flagPulsoIncremento = FALSE;
bool flagPulsoInicio = FALSE;
bool flagHabilitacionInicio = FALSE;
bool recibodatos = FALSE;

void actualizarLcd();
void juego();
void grua();
void retencionInicio();
void apagarLeds();

void setup(){
  //Inicializacion del Timer2
  cli(); 
  TCCR2A = 0; 
  TCCR2B = 0; 
  TCNT2 = 0;  

  OCR2A = 255; 
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (0b00000111); //1024 (preescala)
  TIMSK2 |= (1 << OCIE2A);
  sei(); 

  Bluetooth.begin(57600); 

  miservo_1.attach(3, 350, 2900);
  miservo_1.write(grados1); 

  miservo_2.attach(5, 1000, 2000); 
  miservo_2.write(grados2); 

  miservo_3.attach(6, 1000, 2000); 
  miservo_3.write(grados3);
  delay(500);

  apagarLeds();

  lcd.init();
  lcd.backlight();
  //Mensaje de bienvenida
  lcd.setCursor(0, 0);
  lcd.print("  Bienvenido a  ");
  lcd.setCursor(0, 1);
  lcd.print("  Super Guanti  ");
  delay(1500);  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Una creacion de:");
  lcd.setCursor(0, 1);
  lcd.print("     M.A.L.     ");
  delay(1000);
  lcd.clear();

  pinMode(incremento, INPUT);
  pinMode(inicio, INPUT);

  pinMode(infra1, INPUT);
  pinMode(infra2, INPUT);
  pinMode(infra3, INPUT);
  pinMode(infra4, INPUT);
  pinMode(infra5, INPUT);

  pinMode(pinLatch, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}

ISR(TIMER2_COMPA_vect){ 
/* Esta funcion se interrumpe cada 32.77us
   La cuenta no es exacta. Salida ejemplo: 4:30(salida arduino) 4:26(cronometro)
*/
  tauxmili++;
  if(tauxmili >= 30){
    tIncremento++; //variables asociadas a la retencion de los pulsadores
    tInicio++;
    taux++;
    
    if(taux >= 60){ //variables asociadas a la cuenta regresiva y la cuenta general del juego
      tlcd--;
      taux = 0;
      if(estadoLcd == 2){
        tseg++;
        if(tseg >= 60){
          tmin++;
          tseg = 0;
          if(tmin >= 60){
            thora++;
            tmin = 0;
          }
        }
      }
    }
  } 
}

void loop(){

  actualizarLcd(); //En esta funcion se engloban todas las salidas en pantalla del programa
  
  switch(estadoPrograma){
    case 1:
    /* En este caso se hace la eleccion de la cantidad de viajes a realizar con el bloque y se da inicio al juego
    * Las MEF son para la retencion de los pulsadores de incremento de viajes y de inicio 
    */
      switch(estadoRetencionIncremento){
        case 1:
          flagPulsoIncremento = FALSE;

          if(digitalRead(incremento) == LOW)
            estadoRetencionIncremento = 1;

          if(digitalRead(incremento) == HIGH){
            tIncremento = 0;
            estadoRetencionIncremento = 2;
          }
        break;
        case 2:
          if(tIncremento < 7)
            estadoRetencionIncremento = 2;
          if(tIncremento >= 7)
            estadoRetencionIncremento = 3;
        break;
        case 3: 
          if(digitalRead(incremento) == HIGH){
            flagPulsoIncremento  = TRUE;
            estadoRetencionIncremento  = 1;
          }
          else{
            flagPulsoIncremento = FALSE;
            estadoRetencionIncremento = 1;
          }
        break;
      } 
      retencionInicio();
      
      /*Si el pulsador verdaderamente esta presionado, se incrementa una vez la variable
      * Se puede dar inicio al juego luego de haber seleccionado minimo un viaje, es entonces que se habilita el boton inicio
      */
      if(flagPulsoIncremento == TRUE){
        viajesSeleccionados++;
        flagHabilitacionInicio = TRUE;
      }
      if(estadoLcd == 2){ //al estado 2 del lcd se accede despues de que termine la cuenta regresiva
        juego(); //se encarga del encendido aleatorio de los leds
        estadoPrograma = 2;
        tmin = 0;
        tseg = 0;
        thora = 0;
      }
    break;
    case 2:
    /* Si se reciben datos por bluetooth se llama a la grua
    *  Al detectar que se pulso un infra se avanza al siguiente estado
    */
      if(Bluetooth.available()){
        grua();
      }

      if(digitalRead(infra1) == HIGH || digitalRead(infra2) == HIGH || digitalRead(infra3) == HIGH || digitalRead(infra4) == HIGH || digitalRead(infra5) == HIGH){
        estadoPrograma = 3;
      }
    break;
    case 3:
    /* Cuando el infra deja de detectar se cuenta como un viaje (es decir que un viaje es valido cuando el bloque se levanta)
    *  Mientras el infra este activado se llama a la grua para poder levantar el bloque
    *  Mientras el lcd diga A JUGAR se llama a la funcion "juego" para prender el siguiente led
    */
      
      if(digitalRead(infra1) == HIGH || digitalRead(infra2) == HIGH || digitalRead(infra3) == HIGH || digitalRead(infra4) == HIGH || digitalRead(infra5) == HIGH){
        estadoPrograma = 3;
        grua();
      }
      if(digitalRead(infra1) == LOW && digitalRead(infra2) == LOW && digitalRead(infra3) == LOW && digitalRead(infra4) == LOW && digitalRead(infra5) == LOW){
        viajesRealizados++;
        if(estadoLcd == 2){
          juego();
        }
        estadoPrograma = 2;
      }
    break;
    case 4:
    /*En este estado se entra desde la condicion anterior y desde las condiciones del lcd al llegar al ultimo caso
    * Se reinician todas las varibles definidas en el inicio para poder volver a jugar sin inconvenientes  
    * La variable flagHabilitacionInicio se desactiva para volver a ingresar la cantidad de viajes que se quieren
    */
      tmin = 0;
      tseg = 0;
      thora = 0;

      viajesSeleccionados = 0;
      viajesRealizados = 0;
      aleatorio = 0;
      numAnterior = 0;

      estadoLcd = 0;
      flagHabilitacionInicio = FALSE;
      estadoPrograma = 1;

      apagarLeds();
    break;
  }
}
void actualizarLcd(){
  /* En esta MEF estan agrupadas todas las salidas en pantalla con sus respectivas condiciones  
   * para cambiar de estado
  */
  switch (estadoLcd)
  {
    case 0:
      lcd.setCursor(0,0);
      lcd.print("Cant de viajes: ");
      lcd.setCursor(0,1);
      lcd.print(viajesSeleccionados);

      if(flagPulsoInicio == TRUE && flagHabilitacionInicio == TRUE){ //se cambia de estado si ya hay viajes seleccionados
        tlcd = 5;
        estadoLcd = 1;
      }
      else{
        estadoLcd = 0;
      }
    break;
    case 1:
      lcd.setCursor(0,0);
      lcd.print("El juego inicia");
      lcd.setCursor(0, 1);
      lcd.print("     en: ");
      lcd.print(tlcd);

      if(tlcd > 0) //es el tiempo que se utiliza en la cuenta regresiva
        estadoLcd = 1;
      else{
        lcd.clear();
        estadoLcd = 2;
      }
    break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("    A JUGAR!    ");
      lcd.setCursor(4, 1);
      lcd.print(thora);
      lcd.print(":");
      lcd.print(tmin);
      lcd.print(":");
      lcd.print(tseg);

      if(viajesRealizados != viajesSeleccionados)
        estadoLcd = 2;
      else{
        tlcd = 7; 
        lcd.clear();
        estadoLcd = 3;
      }
    break;
    case 3:
      apagarLeds();

      lcd.setCursor(0, 0);
      lcd.print("  Felicidades!  ");
      lcd.setCursor(4, 1);
      lcd.print(thora);
      lcd.print(":");
      lcd.print(tmin);
      lcd.print(":");
      lcd.print(tseg);

      if(tlcd <= 0){
        tlcd = 7;
        lcd.clear();
        estadoLcd = 4;
      }
    break;
    case 4:
      lcd.setCursor(0,0);
      lcd.print(" Para reiniciar ");
      lcd.setCursor(0,1);
      lcd.print("presionar inicio");

      retencionInicio();
      if(flagPulsoInicio == TRUE){
        lcd.clear();
        estadoPrograma = 4;
      }
    break;
  }
}
void juego(){
/* En esta funcion se cambia el led que esta encendido, con la condicion de que no se prenda dos veces el mismo
*  Cada caso enciende un led distinto poniendo distintos numeros en la salida del shift Register 74hc595
*  Esta funcion es llamada cuando se detecta como valido un viaje  
*  Luego de encender el led se va al estadoPrograma 2 donde se reciben instrucciones para la grua
*/
  while(aleatorio == numAnterior){
    aleatorio = random(0, 5);
  }
  
  switch(aleatorio)
  {
    case 0:
      digitalWrite(pinLatch, LOW);              
      shiftOut(dataPin, clockPin, MSBFIRST, 1); 
      digitalWrite(pinLatch, HIGH);
      numAnterior = 0;
      estadoPrograma = 2;
    break;
    case 1:
      digitalWrite(pinLatch, LOW);              
      shiftOut(dataPin, clockPin, MSBFIRST, 2); 
      digitalWrite(pinLatch, HIGH);
      numAnterior = 1;
      estadoPrograma = 2;
    break;
    case 2:
      digitalWrite(pinLatch, LOW);              
      shiftOut(dataPin, clockPin, MSBFIRST, 4); 
      digitalWrite(pinLatch, HIGH);
      numAnterior = 2;
      estadoPrograma = 2;
    break;
    case 3:
      digitalWrite(pinLatch, LOW);              
      shiftOut(dataPin, clockPin, MSBFIRST, 8);
      digitalWrite(pinLatch, HIGH);
      numAnterior = 3;
      estadoPrograma = 2;
    break;
    case 4:
      digitalWrite(pinLatch, LOW);               
      shiftOut(dataPin, clockPin, MSBFIRST, 16); 
      digitalWrite(pinLatch, HIGH);
      numAnterior = 4;
      estadoPrograma = 2;
    break;
  }
}
void grua(){
/* Al recibir los pines serial declarados con la libreria SoftwareSerial, 
*  estos se guardan en la variable estadoBluetooth
*  despues dependiendo del caracter recibido se le envian las instrucciones a los servos
*/
  byte estadoBluetooth = Bluetooth.read(); 

  ///SERVO 1 -- DERECHA IZQUIERDA -- 3///
  if(estadoBluetooth == '1'){
    grados1 = grados1 + 3; //Los servos aumentan y disminuyen de a 3 grados para mejorar la velocidad
    if(grados1 >= 180){
      grados1 = 180;
    }
    miservo_1.write(grados1);
  }
  if(estadoBluetooth == '3'){
    grados1 = grados1 - 3;
    if(grados1 <= 0){
      grados1 = 0;
    }
    miservo_1.write(grados1);
  }

  ///SERVO 2 -- ADELANTE ATRAS -- 5///
  if(estadoBluetooth == '5'){
    grados2 = grados2 + 3;
    if(grados2 >= 180){
      grados2 = 180;
    }
    miservo_2.write(grados2);
  }
  if(estadoBluetooth == '7'){
    grados2 = grados2 - 4;
    if(grados2 <= 0){
      grados2 = 0;
    }
    miservo_2.write(grados2);
  }

  ///SERVO 3 -- ABAJO -- 6///
  /*Este servo recibe solo una instruccion
  * Cuando llega al punto minimo vuelve automaticamente a la posicion original
  */
  if(estadoBluetooth == '9'){    
    grados3 = grados3 - 3;        
    if(grados3<=0){
      grados3 = 90;
    }
    miservo_3.write(grados3);
  }  
}
void retencionInicio(){
  switch(estadoRetencionInicio){
    case 1:
      flagPulsoInicio = FALSE;

      if(digitalRead(inicio) == LOW)
        estadoRetencionInicio = 1;

      if(digitalRead(inicio) == HIGH){
        tInicio = 0;
        estadoRetencionInicio = 2;
      }
    break;
    case 2:
      if(tInicio < 7)
        estadoRetencionInicio = 2;
      if(tInicio >= 7)
        estadoRetencionInicio = 3;
    break;
    case 3: 
      if(digitalRead(inicio) == HIGH){
        flagPulsoInicio = TRUE;
        estadoRetencionInicio = 1;
      }
      else{
        flagPulsoInicio = FALSE;
        estadoRetencionInicio = 1;
      }
    break;
  }
}
void apagarLeds(){
  digitalWrite(pinLatch, LOW);              
  shiftOut(dataPin, clockPin, MSBFIRST, 0); 
  digitalWrite(pinLatch, HIGH);
}