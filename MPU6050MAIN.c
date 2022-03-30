#include "Header.h"
#include "stdbool.h"
/**
 * main.c
 */
float fAccel[3], fGyro[3];
signed int acX, acY, acZ,gcX,gcY,gcZ;
float pitch_x=0;
float roll_y=0;
#define CotaS = 3.3;
#define CotaI = 0.0;

//definiciones de frecuencias
#define FREQ_TIMER 1000

#define FPWM (33    );
#define Ft   (0.85);
#define N_PI (3.1415926535);
#define D_T ( 0.001);

uint16_t dato;

float roll_y_1_n=0;
float gradx = 0;
float grady=0;
float roll_Angle_0, roll_error, rollY_errores_antes;
float rad_to_deg = 180/N_PI;
float dt = 0.001;

//---PID-----
float roll_angle_0 = 0;
float eD = 0;
float roll_error = 0;
float roll_error_1=0;
float rollY_1 = 0;
float roll_P = 0;
float roll_I=0;
float roll_D=0;
float ft,ft1;

float roll_kp = 0.58;
float roll_ki = 0.00875/3.0;
float roll_kd = 0.9;
float filtroX, filtroY;
float roll_y_p1 =0;
float roll_y_p2 = 0;
float angx=0;angy=0;


float rollY_prev;
float Grados_out;
uint32_t GRADOSOUT;

//PWM
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t ui32pm;
volatile bool g_bMPU6050Done;
tI2CMInstance g_sI2CInst;
tMPU6050 g_sMPU6050Inst;


/*void Init_GPIOs(void);
void Init_Timer0(void);
void Init_UART(void);
void Init_Interrups(void);
void Init_MPU6050(void);
*/

//void MPU6050Callback( void *pvCallbackData, uint_fast8_t ui8Status );



void Init_Timer0(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {

    }

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    TimerLoadSet(TIMER0_BASE, TIMER_A, 40000);

    TimerEnable(TIMER0_BASE, TIMER_A);
}

void Init_I2C0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0))
    {

    }

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

   GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
   GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
   I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

   HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

   I2CMasterIntEnable(I2C0_BASE);
   IntEnable(INT_I2C0);


}


void Init_GPIOs(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);


}

void Init_Interrups(void)
{
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
}

void Init_UART(void)
{
    // Enable GPIO port A which is used for UART0 pins.
        // TODO: change this to whichever GPIO port you are using.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

        // Configure the pin muxing for UART0 functions on port A0 and A1.
        // This step is not necessary if your part does not support pin muxing.
        // TODO: change this to select the port/pin you are using.
        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);

        // Enable UART0 so that we can configure the clock.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

        // Use the internal 16MHz oscillator as the UART clock source.
        UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

        // Select the alternate (UART) function for these pins.
        // TODO: change this to select the port/pin you are using.
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

        // Initialize the UART for console I/O.
        UARTStdioConfig(0, 115200, 16000000);
}

void Init_MPU6050(void)
{
    g_bMPU6050Done = false;
    UARTprintf("false");
    MPU6050Init(&g_sMPU6050Inst, &g_sI2CInst, 0x68, MPU6050Callback, 0);
    while (!g_bMPU6050Done){

    }
    UARTprintf("false2");

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&g_sMPU6050Inst, MPU6050_O_ACCEL_CONFIG, ~MPU6050_ACCEL_CONFIG_AFS_SEL_M, MPU6050_ACCEL_CONFIG_AFS_SEL_8G, MPU6050Callback, &g_sMPU6050Inst);
    while (!g_bMPU6050Done){

    }
}


int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);
    Init_GPIOs();
    Init_Timer0();
    Init_UART();
    Init_I2C0();
    Init_MyPWM(PWM1_PERIPH, PWM_MOD1_EN, PWM0_MOD1, PUERTOD, PWMPIN0, PWM_GEN_0);



    ui32PWMClock = SysCtlClockGet()/64;
    ui32pm = ui32PWMClock /FPWM;
    ui32Load = ui32pm - 1;
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0,ui32Load);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);

    while (1)
    {
        g_bMPU6050Done = false;

        MPU6050DataRead(&g_sMPU6050Inst, MPU6050Callback, 0);
        while(!g_bMPU6050Done)
        {

        }

        MPU6050DataAccelGetFloat(&g_sMPU6050Inst, &fAccel[0],&fAccel[1],&fAccel[2]);
        MPU6050DataAccelGetFloat(&g_sMPU6050Inst, &fGyro[0],&fGyro[1],&fGyro[2]);



              roll_error = roll_Angle_0 - grady;
              eD = roll_error - rollY_prev;

              roll_P = roll_kp*roll_error;
              roll_I = roll_I + roll_ki*roll_error;
              roll_D = roll_kd*eD;
              Grados_out = roll_P + roll_I + roll_D;
              rollY_prev = roll_error;

              if (Grados_out > 3.3) {
               Grados_out=4095.0;
               }
               if (Grados_out < 0) {
               Grados_out= 0.0;
               }


       GRADOSOUT = (uint32_t)Grad_PWMimpulse(Grados_out);
       PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, GRADOSOUT);
       //UARTprintf("acX %d", (int)acX[n]);
    }
}

void Timer0Handler(void)
{


     TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
     // Trigger the ADC conversio


    pitch_x = atan2(acY,(sqrt(acX*acX + acZ*acZ)))*180.0/N_PI;
    roll_y = atan2(acY,(sqrt(acY*acY + acZ*acZ)))*180.0/N_PI;

    roll_y_p1 = roll_y*Ft
    roll_y_p2 = roll_y_1_n*Ft
    roll_y = roll_y - roll_y_p1 + roll_y_p2;
    roll_y_1_n = roll_y;

    ft= filtroX + gcX*D_T
    ft1 =  filtroY + gcY*D_T

    filtroX = 0.98*ft + 0.02*pitch_x;
    filtroY = 0.98*ft1 + 0.02*roll_y;
}



  /*uint16_t uk_int;
    uk_int = (uint16_t)(Grados_out);
    // Es importante que lo que se haga en el handler no tome más tiempo que el
    // período del temporizador.
     // En este ejemplo siempre se envía el mismo valor por SPI. En otra aplicación,
     // el dato se construye según lo que se necesite enviar al esclavo.
     dato = uk_int;
     // Necesito crear mi dato que envio por SPI, este sera de 32 bits debido a la
    //función de los cuales solo nos importan
     // 16 bits.
     dato = 0b0000111111111111 & dato; // Filtro para creación de dato, borro
    //configuración previa y dejo pasar el valor actual
     dato = 0b0111000000000000 | dato; // Realizo or para dejar pasar valor y hacer
    //set de configuración.
     // 0, para escribir en dac
     // 1, para habilitar el buffer
     // 1, ganancia normal con voltaje de referencia.
     // 1, habilitar salida del DAC
     // Colocar el dato de SPI_ANCHO bits en pui32DataTx
     pui32DataTx[0] = (uint32_t)(dato); // Envio mi información como una
    //variable de 32 bits.
     // Send data
     for(ui32Index = 0; ui32Index < NUM_SPI_DATA; ui32Index++)
     {
     // Send the data using the "blocking" put function. This function
     // will wait until there is room in the send FIFO before returning.
     // This allows you to assure that all the data you send makes it into
     // the send FIFO.
     SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
     }
     // Wait until SSI0 is done transferring all the data in the transmit FIFO.
     while(SSIBusy(SSI0_BASE))
     {
     }*/
