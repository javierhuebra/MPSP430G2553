//------------------------------------------------------
// UTN-FRBB - Inform�tica II - Ing. Electr�nica
// Programa sensor de temperatura para MSP430G2553.
// Utiliza un termistor de 1 K y efectUa la mediciOn en
// base el tiempo de descarga de un capacitor.
//------------------------------------------------------
#include <msp430.h>

// MAscaras que representan los pines conectados a Rref, al termistor y al capacitor

#define capPin     (0x01)         // Pin P1.0 del capacitor y CA0
#define RrefPin    (0x02)         // Pin P1.1 : conectado a Resistencia de Referencia
#define TermPin    (0x04)         // Pin P1.2 : conectado al termistor

volatile unsigned long cuenta_ref=0, cuenta=0;
volatile int flag=0, temperatura=0;

//----------------------------------------------------------
// Inicio del programa
//----------------------------------------------------------

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;	  // Detiene el watchdog timer

  // Tabla de valores de resistencias del termistor, para temperaturas desde -10�C a 100 �C
  const unsigned R_termistor[]= {5258,4984,4726,4484,4255,4040,3837,3645,3465,3295,3134,2982,2838,
		                         2703,2574,2453,2338,2230,2127,2029,1937,1849,1766,1688,1613,1542,
								 1475,1411,1350,1292,1237,1185,1135,1088,1043,1000,959,920,883,848,
								 814,782,751,722,694,667,641,617,594,571,550,530,510,491,473,456,
								 440,424,409,395,381,367,355,342,331,319,309,298,288,279,269,261,
								 252,244,236,228,221,214,207,201,195,189,183,177,172,166,161,157,
								 152,147,143,139,135,131,127,124,120,117,113,110,107,104,101,98,96,
								 93,91,88,86,84,81
                                };
  unsigned long rTermistor;
  int i;

  // Configura las frecuencias de reloj.
  // MCLK = 7.8MHz
  // SMCLK = MCLK = 7.8 Mhz
  DCOCTL = DCO1 | DCO0;
  BCSCTL1 = RSEL3 | RSEL2 | RSEL0 ; // RSELx = 13 --> MCLK = 7.8 MHz // Divisor para ACLK: 1
  BCSCTL2 = DIVS_0;                 // Divisor para SMCLK: 1 --> SMCLK = 7.8 MHz
  BCSCTL3 = LFXT1S_2;               // ACLK = VLOCK (12 kHz) --> Obs.: no se usa

  // Configuraci�n de los puertos P1 y P2
  // P1.0 se utiliza como entrada + del comparador (CA0)
  // P1.1 se utiliza como salida, para controlar la carga y descarga del capacitor a trav�s de
  //      la resistencia de referencia Rref
  // P1.2 se utiliza como salida, para controlar la carga y descarga del capacitor a trav�s del
  //      termistor.
  // P1.4 y P1.5 se utilizan como salidas, para seleccionar el d�gito a encender
  // P1.7 salida para el CAOUT ??
  // P2.0 a P2.3 utilizado para conversi�n BCD a 7 segmentos

  P1SEL = 0x00; // Todos los pines como E/S
  P2SEL = 0x00;	// Todos los pines como E/S

  P1OUT = 0x20;	// Selecciona el d�gito a encender inicialmente (en este caso el conectado a P1.4)
  P2OUT = 0x00;

  P1DIR = 0x70;		// P1.4, P1.5 y P1.6 como salidas, el resto como entradas
  P2DIR = 0x0f;		// P2.0 a 2.3 como salidas, el resto como entradas

  // Configuraci�n del comparador anal�gico

  CACTL2 = P2CA0 | CAF;   /* P2CA0 selecciona el pin P1.0 (CA=0) como terminal + del comparador
                             CAF habilita el filtro por eventuales perturbaciones que pueda haber
                             en la entrada anal�gica */

  // Timer A1 --> Genera el temporizado para el refresco del display y
  //              la activaci�n peri�dica del flag con que se ordena iniciar una nueva medici�n
  //              Se mueve con SMCLK/4 (7.8 MHz / 4 = 1.95 MHz)

  TA1CCR0 = 19500;  /* 19500 / 1.95 Mhz = 10 mseg ==> cada d�gito es encendido
                       cada 20 mseg (50 veces por segundo) */

  TA1CTL = TASSEL_2 | ID_2 | MC_1 | TACLR | TAIE; /* TACLR: pone a cero el registro contador del timer
                                                     TASSEL_2: selecciona el reloj SMCLK
                                                     ID_2 : divide SMCLK por 4
                                                     MC_1: modo UP ==> cuenta en forma ascendente hasta
                                                           alcanzar el valor de TA1CCR0
                                                     TAIE: habilita interrupciones para el timer 1 */

  __enable_interrupt();   // Habilitaci�n general de interrupciones


  // Configura el Timer A0, utilizado para medir el tiempo de descarga del capacitor.
  // Se mueve con SMCLK (7.8 MHz). El m�ximo tiempo de descarga corresponde a la temperatura
  // de -10 �C (5258 ohms). Suponiendo un capacitor de 100 nF y descarga hasta 0.25 Vcc, el
  // tiempo de descarga m�ximo es de 138.6 us. A 7.8 Mhz, la cuenta m�xima ser�a de 1081, lo
  // cabe holgadamente en los registro de 16 bits del timer.
  // Para un capacitor de 220 nF la cuenta m�xima ser�a de 2378, que tambi�n cabe sin problemas

  TA0CTL =  TASSEL_2 | ID_0;	/* Habilito el timer desde cero
                                   TASSEL_2 selecciono el SMCLK como reloj para el timer 0
                                   ID_0 divisor del SMCLK por 1 */

  // Configura la modalidad de captura del timer 0

  TA0CCTL1 = CM_2 | CCIS_1 | CCIE | SCS; /* CM_2 Captura con el flanco descendente del comparador (es
                                                 decir, cuando la tensi�n del borne (+) del comparador
                                                 caiga por debajo de la tensi�n en el borne (-).
                                            CCIS_1 CCI1B
                                            CAP modo captura
                                            CIE habilita interrupciones
                                            SCS sincroniza la captura con el clock */

  // Los pines de entrada conectados al capacitor que no se utilizan son deshabilitados,
  // para no generar una descarga adicional al capacitor

  CAPD = capPin | TermPin;	 // Deshabilito el buffer de entrada para P1.0 (CA0) y P1.2

  /* Se debe cargar capacitor como m�nimo 5Tau

     Si R=1K y C=100 nF , Tau=100 us, 5 Tau = 500 us = 0.5 ms
     Como la frecuencia de MCLK es 7.8 MHz, se necesitan 3900 ciclos de reloj para 5 Tau, por
     lo que se utiliza un retardo de 10000 ciclos de reloj, para superar dicho tiempo.
     Si C fuera de 220 nF, se requieren 8580 ciclos para alcanzar los 5 Tau, por lo que 10000
     tambi�n es suficiente */

  //Pone pines en el estado que corresponda
  P1OUT |= RrefPin;	         // Pongo un 1 en el P1.1
  P1DIR |= RrefPin;          // Lo configuro como salida

  __delay_cycles(10000);     //retardo para la carga del Capacitor

  // Luego de la carga efectua la descarga a trav�s de Rref, para tener el valor de referencia.

  CACTL1 = CARSEL | CAREF_1 | CAON ;   /* habilita el comparador
                                          CARSEL : 1 aplica la Vref en terminal - del comparador
                                          CAREF_1 : Vref = 0.25 Vcc
                                          CAON : activa el comparador */

  // Descarga el capacitor a trav�s de Rref. Cuando la tensi�n de entrada + del comparador llegue
  // a ser inferior a Vref se detiene el timer y se produce una interrupci�n, que ser� atendida
  // por la funci�n Timer0_A1()

  CAPD = TermPin;	           // Deshabilita el buffer de entrada para P1.2
  P1OUT &= ~RrefPin;           // Pone a "0" la salida de P1.1, para descargar el capacitor
  TA0CCTL1 |= CAP ;	           // Habilita el modo captura del timer 0
  TA0CTL |= TACLR | MC_2 ;	   /* TACLR: poner el contador del timer en cero
                                  MC_2:  modo continuo */
  while(1)
  {
    if(flag)	// flag se activa cada 500 ms para iniciar una nueva medici�n
    {
      flag=0;   // resetea flag para que no inicie otra medici�n hasta el momento que corresponda

      CAPD = capPin | RrefPin;	    // Deshabilita el buffer de entrada para P1.0 (CA0) y P1.1 (Rref)

      P1OUT |= TermPin;	    // Pone "1" en P1.2, para cargar el capacitor
      P1DIR |= TermPin;     // Configura P1.2 como salida

      // Retardo para cargar el capacitor. Como el valor m�ximo previsto de resistencia para
      // el termistor es de 5258 ohms a -10 �C, el tiempo para 5 x Tau ser�a de
      // 5 * 100 nF * 5258 ohms = 2,6 mseg aprox. La cantidad de ciclos de MCLK es de
      // 0,0026 seg x 7.8 MHz = 20280.
      // Obs.: si el capacitor fuera de 220 nF el valor de cuenta deber�a ser de 44616.
      // Para tener un retardo que sirva para ambas capacidades, se adopta un valor de 50000
      __delay_cycles(50000);

      CACTL1 = CARSEL | CAREF_1 | CAON ;  /* Habilita el comparador
                                             CARSEL = 1 aplica la Vref en terminal (-)
                                             CAREF_1 : Vref = 0.25 Vcc
                                             CAON activa el comparador */

      // Descarga el capacitor a trav�s del termistor

      CAPD = RrefPin;	      // Deshabilita el buffer de entrada para P1.1
      P1OUT &= ~TermPin;      // Pone "0" en la salida de P1.2, para la descarga del capacitor
      TA0CCTL1 |= CAP ;		  // Habilita el modo captura del timer 0
      TA0CTL |= TACLR;	      // Habilita el timer 0 desde cero

      P1OUT ^= 0x40;	      // conmuta el led en P1.6 (verde) al inicio de cada medici�n
    }

    // Actualiza el valor de temperatura a mostrar en el display
    if(cuenta_ref)
    {
      // cuenta: valor proporcional al tiempo de descarga a trav�s del termistor
      // cuenta_ref: valor proporcional al tiempo de descarga a trav�s de Rref.
      // 1000 es el valor de la resistencia de Rref
      // El valor que se almacena en rTermistor es el valor de resistencia del termistor

      rTermistor = (cuenta * 1000) / cuenta_ref ;

      // Luego recorre la tabla de valores de resistencia en funci�n de la temperatura
      // correspondiente al termistor, hasta encontrar el valor m�s cercano.

      for(i=0; i<=110; ++i)
      {
        if( rTermistor >= R_termistor[i] )
          break;
      }
      temperatura=i-10;  // Le resta 10 porque los valores de resistencia est�n tabulados desde -10 �C
    }
  }
}

//----------------------------------------------------------
// Funci�n de atenci�n de interrupci�n del timer 0.
// Se ejecuta cada vez que es detenido el timer A1 por
// acci�n del comparador.
//----------------------------------------------------------

#pragma vector=TIMER0_A1_VECTOR

void __interrupt Timer0_A1(void)
{
  if(TA0IV == 0x02)
  {
    cuenta=TA0CCR1;
    if(!cuenta_ref)
    {
      // Como cuenta_ref es inicialmente cero, se asume que la
      // primera vez corresponde a la descarga del capacitor
      // a trav�s de la resistencia de referencia.
      cuenta_ref=cuenta;
    }
    TA0CCTL1 &= ~CCIFG ;        // Borra el flag de interrupci�n CCR1
    TA0CCTL1 &= ~CAP;           // Deshabilita el modo captura del timer 0
    CACTL1 &= ~CAON;	        // Deshabilita el comparador
  }
}

//----------------------------------------------------
// Rutina de atenci�n de interrupci�n del timer 1.
// Se ejecuta cada 10 mseg.
//----------------------------------------------------
#pragma vector=TIMER1_A1_VECTOR
void __interrupt Timer1_A1(void)
{
  unsigned int i;
  static unsigned int ticks=0;

  // Verifica si es la interrupci�n esperado
  if( TA1IV != 0x0a )
    return;

  // Borra el flag de interrupci�n del timer
  TA1CTL &= ~TAIFG;

  if( temperatura < 0 )
  {
	  // Si la temperatura es negativa s�lo muestra las unidades y apaga el d�gito de las decenas
	  P1OUT &= ~0x10;  // Apaga decena
	  P2OUT = (-temperatura) % 10;
	  P1OUT |= 0x20;   // Enciende unidad
  }else{
	  // Enciende alternadamente un d�gito o el otro.
	  if( !(P1OUT & 0x20) )
	  {
		  // Muestra d�gito izquierdo
		  P1OUT ^= 0x30;
		  i = temperatura % 10;
		  P2OUT = i;
  	  } else {
  		  // Muestra d�gito derecho
  		  P1OUT ^= 0x30;
  		  i = (temperatura / 10) % 10;
  		  P2OUT = i;
  	  }
  }
  // Cada 500 ms activa "flag", que es usado como condici�n para iniciar
  // un nuevo ciclo de medici�n de la temperatura. Cada 10 ms se incrementa
  // "ticks" ==> 50 equivale a 500 ms.
  if( ++ticks >= 50 )
  {
    flag = 1;
    ticks = 0;
  }
}
