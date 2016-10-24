/*

Nome:
 * 
- Leonardo Kaminski Ferreira
 *
 *                      SLAVE I2C
 *
Engenharia da Computação - 3º Ano - Manhã - Arquitetura e Organização de Computadores


 */
#include <p18cxxx.h>
#include "./USB/usb.h"//chama todas as lib necessarias
#include "/MPLAB.X/HardwareProfile.h" //aqui está configurado o clock e os pinos do microcontrolador
#include "GenericTypeDefs.h"//lib que define os tipos utilizados
#include "./USB/usb_function_cdc.h"//funçoes da serial
#include "Compiler.h"//aqui se define o compilador que está sendo utilizado
#include "usb_config.h"//configura a usb
#include "USB/usb_device.h"
#include "delays.h"
#include "stdlib.h"
#include "stdio.h"
#include "i2c.h"
#include <pwm.h>
#include <timers.h>



//////////////config. dos fuses do microcontrolador///////////////////////////////
#pragma config PLLDIV   = 5         // (usei um cristal de 20 MHz )
#pragma config CPUDIV   = OSC1_PLL2
#pragma config USBDIV   = 2         // o clock do pll será 96MHz PLL/2 = 48Mhz
#pragma config FOSC     = HSPLL_HS
#pragma config FCMEN    = ON // alterado
#pragma config IESO     = ON // alterado
#pragma config PWRT     = OFF
#pragma config BOR      = ON
#pragma config BORV     = 3
#pragma config VREGEN   = ON      //ativa o regulador de voltagem da USB
#pragma config WDT      = OFF
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF
#pragma config CCP2MX   = OFF // sinal de pwm no pino RB3
#pragma config STVREN   = ON
#pragma config LVP      = OFF
//      #pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
#pragma config XINST    = OFF       // Extended Instruction Set
#pragma config CP0      = OFF
#pragma config CP1      = OFF
//      #pragma config CP2      = OFF
//      #pragma config CP3      = OFF
#pragma config CPB      = OFF
//      #pragma config CPD      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
//      #pragma config WRT2     = OFF
//      #pragma config WRT3     = OFF
#pragma config WRTB     = OFF       // proteção de escrita da inicialização (Boot)
#pragma config WRTC     = OFF
//      #pragma config WRTD     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
//      #pragma config EBTR2    = OFF
//      #pragma config EBTR3    = OFF
#pragma config EBTRB    = OFF

/////////////////////variáveis requerida pela USB /////////////////////////////////////////////////////

#pragma udata
char USB_Out_Buffer[CDC_DATA_OUT_EP_SIZE];
char RS232_Out_Data[CDC_DATA_IN_EP_SIZE];

unsigned char NextUSBOut;
unsigned char NextUSBOut;
//char RS232_In_Data;
unsigned char LastRS232Out; // Number of characters in the buffer
unsigned char RS232cp; // current position within the buffer
unsigned char RS232_Out_Data_Rdy = 0;
USB_HANDLE lastTransmission;
BOOL check = 0;
BOOL lixo = 0;


unsigned char string_pwm[] = "xx", string_intensidade[] = "xxx";
unsigned char ok_p1[] = "PWM 1 em uso       ", ok_p2[] = "PWM 2 em uso       ", erro[] = "PWM nao encontrado ";
int verificador = 0;
unsigned char comandoRecebido[] = "Comando recebido via i2c:                 ";



//BOOL stringPrinted;


/**********prototipo das funÃ§Ãµes ***************************************/
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
//void BlinkUSBStatus(void);
void UserInit(void);
void InitializeUSART(void);
unsigned char getcUSART();
void enviarUSB(char *msg);
unsigned int filtro();


/********** remapeando vetores ****************************/
#define REMAPPED_RESET_VECTOR_ADDRESS		0x1000
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
#define APP_VERSION_ADDRESS                     0x1016 //Fixed location, so the App FW image version can be read by the bootloader.
#define APP_SIGNATURE_ADDRESS                   0x1006 //Signature location that must be kept at blaknk value (0xFFFF) in this project (has special purpose for bootloader).

#define APP_FIRMWARE_VERSION_MAJOR  1   //valid values 0-255
#define APP_FIRMWARE_VERSION_MINOR  0   //valid values 0-99

/////////////////defines da porta serial /////////////////////////////

#define mDataRdyUSART() PIR1bits.RCIF
#define mTxRdyUSART()   TXSTAbits.TRMT
//--------------------------------------------------------------------------


// defines dos leds e portas

#define sensor PORTDbits.RD7


//--------------------------------------------------------------------------


#pragma romdata AppVersionAndSignatureSection = APP_VERSION_ADDRESS
ROM unsigned char AppVersion[2] = {APP_FIRMWARE_VERSION_MINOR, APP_FIRMWARE_VERSION_MAJOR};
#pragma romdata AppSignatureSection = APP_SIGNATURE_ADDRESS
ROM unsigned short int SignaturePlaceholder = 0xFFFF;

#pragma code HIGH_INTERRUPT_VECTOR = 0x08

void High_ISR(void) {
    _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code LOW_INTERRUPT_VECTOR = 0x18

void Low_ISR(void) {
    _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
extern void _startup(void); // See c018i.c in your C18 compiler dir
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS

void _reset(void) {
    _asm goto _startup _endasm
}
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS

void Remapped_High_ISR(void) {
    _asm goto YourHighPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS

void Remapped_Low_ISR(void) {
    _asm goto YourLowPriorityISRCode _endasm
}

#pragma code

#pragma interrupt YourHighPriorityISRCode
void YourHighPriorityISRCode() {

    

    #if defined(USB_INTERRUPT)
        USBDeviceTasks();
    #endif




    


} //This return will be a "retfie fast", since this is in a #pragma interrupt section


#pragma interruptlow YourLowPriorityISRCode
void YourLowPriorityISRCode() {



} //This return will be a "retfie", since this is in a #pragma interruptlow section

#pragma code
static void inicializa_pic(void)
{
    ADCON1 |= 0x0F; // coloca todos os pinos como saida
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // veja HardwareProfile.h
    #endif
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // veja HardwareProfile.h
    #endif

    UserInit();
    USBDeviceInit(); //usb_device.c.  inicializa o modulo USB e os SFRs
    //variaveis para saber o status.
}//fim de inicializa_cpu

void UserInit(void) {
    unsigned char i;
    InitializeUSART();
    // 	inicializa vetor
    for (i = 0; i<sizeof (USB_Out_Buffer); i++) {
        USB_Out_Buffer[i] = 0;
    }
    NextUSBOut = 0;
    LastRS232Out = 0;
    lastTransmission = 0;
}//end UserInit

void InitializeUSART(void) {
    unsigned char c;
    UART_TRISRx = 1; // RX
    UART_TRISTx = 0; // TX
    TXSTA = 0x24; // TX enable BRGH=1
    RCSTA = 0x90; // Single Character RX
    SPBRG = 0x71;
    SPBRGH = 0x02; // 0x0271 for 48MHz -> 19200 baud
    BAUDCON = 0x08; // BRG16 = 1
    c = RCREG; // read

}//end InitializeUSART

void putcUSART(char c) {//escreve na serial
    TXREG = c;
}

#if defined(USB_CDC_SET_LINE_CODING_HANDLER)
void mySetLineCodingHandler(void)
{    
    if (cdc_notice.GetLineCoding.dwDTERate.Val <= 115200)//velocidade maxima
     {
        //atualiza o baudrate no drive CDC
        CDCSetBaudRate(cdc_notice.GetLineCoding.dwDTERate.Val);
        //atualiza o baudrate da porta serial
#if defined(__18CXX) || defined(__XC8) // sem este "if" o programa não funciona(bug do compilador)
        {
            DWORD_VAL dwBaud;
            dwBaud.Val = (DWORD) (GetSystemClock() / 4) / line_coding.dwDTERate.Val - 1;
            SPBRG = dwBaud.v[0];
            SPBRGH = dwBaud.v[1];
        }
    }
}
#endif

unsigned char getcUSART() { //pega caracteres vindo da serial
    char c;
    if (RCSTAbits.OERR) // no caso de ocorrer um overrun
    {
        RCSTAbits.CREN = 0; // reseta porta
        c = RCREG;
        RCSTAbits.CREN = 1; // volta a funciona.
    } else {
        c = RCREG;
    }
    return c;
}



///funÃ§oes prontas para os eventos da usb, caso queira usar alguma Ã© sÃ³ inserir os codigo-fonte nas chaves
//////////////////////////
void USBCBSuspend(void) {}
void USBCBWakeFromSuspend(void) {}
void USBCB_SOF_Handler(void) {}
void USBCBErrorHandler(void) {}
void USBCBCheckOtherReq(void)
{ 
    USBCheckCDCRequest();
}
void USBCBStdSetDscHandler(void) {}
void USBCBInitEP(void)
{    //habilita os enpoits do modo CDC
    CDCInitEP();
}

void USBCBSendResume(void) {
    static WORD delay_count;

    if (USBGetRemoteWakeupStatus() == TRUE) {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if (USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE;
            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);

            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}
////////////////////////////////////////////////////////////////////
#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)

void USBCBEP0DataReceived(void) {
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size) {
    switch (event) {
        case EVENT_TRANSFER:
            //funções de retorno da porta usb
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }
    return TRUE;
}

void recebe_usb(void)
{

   
   if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1))
         {
             //se a usb estiver em modo suspenso não faz nada
             return;
         }else{
            if(mUSBUSARTIsTxTrfReady()) // não tem dados pendentes?
            {

                unsigned char recebido[] = "\n";
                  
                if((getsUSBUSART((char *)recebido, sizeof(recebido))) != 0) // se tem dados na USB para que o PIC leia
                  {

                    if(recebido[0] == 'K'){
                        check = 1;
                    }

                  }
                    
            }
         CDCTxService();
         }

}//fim da função de recebimento, verificação e envio para FPGA



void atraso(void) {

    Delay10KTCYx(1); // atraso de 1ms
}

void delaay(void) {

    Delay10KTCYx(80); // atraso de 80ms


}

void atraso60(void) {

    Delay10KTCYx(60); // atraso de 60ms


}

void atraso70(void) {

    Delay10KTCYx(70); // atraso de 70ms


}






void enviarUSB(char *msg){
    // escrita na USB
    if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1))
    {
         //se a usb estiver em modo suspenso não faz nada
         return;
    }else{
        if(mUSBUSARTIsTxTrfReady()) // não tem dados pendentes?
        {

            //putsUSBUSART(msg); // mando os estados das vias via USB para o controlador local
            putUSBUSART((char *) msg, strlen(msg));

        }
     CDCTxService();
     }

}





void main(void)
{


    unsigned int taxa_pwm = 0;
    
    unsigned char toSend[] = "x\n";
    unsigned char teste22[] = "\n";
    unsigned char teste[] = "xxx";
    
    unsigned char sync_mode=0, slew=0, add1,status,temp,w,length=0;

    unsigned char I2C_Send[21] = "MICROCHIP:I2C_SLAVE";
    unsigned char I2C_Recv[21];

    

    inicializa_pic();
  //  #if defined(USB_INTERRUPT) // se definido a usb vai usar as interrupções
    USBDeviceAttach(); // para verifica o status e os eventos gerados
 //   #endif

    // AS DUAS PORTAS PWM ESTÃO LOCALIZADAS EM RC1 E RC2, CANAL 2 E CANAL 1, RESPECTIVAMENTE.

    TRISBbits.RB0 = 1;
    TRISBbits.RB1 = 1;
    TRISBbits.RB3 = 0;
    TRISCbits.RC1 = 0;
    TRISCbits.RC2 = 0;

    Delay10TCYx(5);
    

    OpenTimer2(TIMER_INT_OFF
                &T2_PS_1_16);

    OpenPWM1(199);
    OpenPWM2(199);
    SetDCPWM1(0);
    SetDCPWM2(0);


    
    enviarUSB((char *) toSend);


    while (1)
    {
       #if defined(USB_POLLING) // se definido para verificar de tempo em tempo
        USBDeviceTasks();//executa as tarefas da usb(verifica o status e os eventos gerados)
        #endif


        if (!check) {

                while(1){

                    toSend[0] = 'K';

                    delaay();

                    enviarUSB((char *) toSend);
                    recebe_usb();
                    USBDeviceTasks();

                    if(check){
                        break;
                    }
                }

            }
        
        


        if(check){

                //sprintf(teste, "indice = %i\n", indice);

            while(1){ // ********* começo do WHILE

                USBDeviceTasks();
                
                if((USBGetDeviceState() < CONFIGURED_STATE) || (USBIsDeviceSuspended() == 1)) {
                    continue;
                } else{


                        CloseI2C();	//close i2c if was operating earlier

                //------------------------INITIALISE THE I2C MODULE FOR MASTER MODE WITH 100KHz ---------------------------
                        sync_mode = SLAVE_7;
                        slew = SLEW_OFF;

                        OpenI2C(sync_mode,slew);

                        SSPADD = 0xA2;							//initialze slave address

                //********************* Read the address sent by master from buffer **************




                        for(w=0;w<20;w++){
                            I2C_Recv[w]=0;
                        }



                    while(DataRdyI2C() == 0); // WAIT UNTILL THE DATA IS TRANSMITTED FROM master
                    // retorna 1 se há dados para serem enviados pelo MASTER para o buffer SSP
                    // retorna 0 se não há mais dados para serem enviados pelo MASTER para o buffer SSP

                    temp = ReadI2C();


                //********************* Data reception from master by slave *********************



                                while( getsI2C(I2C_Recv, 6) ); //recieve data string of lenght 6 from master

                                I2C_Recv[6] = '\n';
                                I2C_Recv[7] = '\0';

                                sprintf(comandoRecebido, "Comando recebido via i2c: %s\n", I2C_Recv);

                                if(!lixo){
                                    USBDeviceTasks();
                                    enviarUSB((char *) comandoRecebido);
                                    USBDeviceTasks();

                                    lixo = 1;

                                } else {
                                    USBDeviceTasks();
                                    enviarUSB((char *) teste);
                                    enviarUSB((char *) comandoRecebido);
                                    USBDeviceTasks();
                                }
                                
                                

                                string_pwm[0] = I2C_Recv[0];
                                string_pwm[1] = I2C_Recv[1];
                                string_intensidade[0] = I2C_Recv[3];
                                string_intensidade[1] = I2C_Recv[4];
                                string_intensidade[2] = I2C_Recv[5];

                                taxa_pwm = atoi((char*) string_intensidade);

                                //sprintf(teste, "taxa_pwm = %i\n", taxa_pwm);

                                if(string_pwm[0] == 'P' && string_pwm[1] == '1'){

                                    SetDCPWM1(taxa_pwm * 8);

                                    verificador = 1;

                                    comandoRecebido[15] = '1';
                                    

                                }

                                if(string_pwm[0] == 'P' && string_pwm[1] == '2'){


                                    SetDCPWM2(taxa_pwm * 8);

                                    verificador = 2;

                                    comandoRecebido[15] = '2';
                                    
                                }
                              


                //******************** write sequence from slave *******************************

                                while(SSPSTATbits.S != 1);		//wait untill STOP CONDITION

                //********************* Read the address sent by master from buffer **************

                                while(DataRdyI2C() == 0);			//WAIT UNTILL THE DATA IS TRANSMITTED FROM master

                        temp = ReadI2C();

                //********************* Slave transmission ************************************

                    if(SSPSTAT & 0x04)	{				//check if master is ready for reception

                        if(verificador == 0){
                            while(putsI2C(erro));			// send the data to master
                        }

                        if(verificador == 1){
                            while(putsI2C(ok_p1));			// send the data to master
                        }

                        if(verificador == 2){
                            while(putsI2C(ok_p2));			// send the data to master
                        }

                    }

                //------------- TERMINATE COMMUNICATION FROM SLAVE SIDE ---------------

                        CloseI2C();   

                }

            } // fim do WHILE

                
            

        } // fim do check que verifica se o Java já está pronto para receber informações do PIC

        
    }//fim do while
    
}//fim da main
