#include <pic.h>
#include <xc.h>
#include <pic16f877a.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ 20000000

#define RS RD2
#define EN RD3
#define D4 RD4
#define D5 RD5
#define D6 RD6
#define D7 RD7

#define Trigger RB1 
#define Echo RB2 
#define PhoneN0 "+251965906574"
#define AlerMsg "Ubnormal Condition"
#define GenOnMsg "Gen. on"

#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)



void ADC_init();
int Read_Water_Level(); 
int Read_Temp();  
int Read_Battery();
void UART_Init(unsigned long);
void UART_Write(char);
void GSM_Init();
void SMS_Send(char*, char*);
void send_values();
void fan_On();
void generator_On();
void ADC_Ir();nitialize();
void Lcd_Print_Char(char data);
void Lcd_Start();
void Lcd_Set_Cursor(char a, char b);
void Lcd_Clear();
void __interrupt() timer_isr();


int timer_counter = 0; // Counter to count the number of minutes passed
int time_elapsed = 0; // Flag to indicate if the state has changed

void Lcd_SetBit(char data_bit) //Based on the Hex value Set the Bits of the Data Lines
{
if(data_bit& 1) 
D4 = 1;
else
D4 = 0;
if(data_bit& 2)
D5 = 1;
else
D5 = 0;
if(data_bit& 4)
D6 = 1;
else
D6 = 0;
if(data_bit& 8) 
D7 = 1;
else
D7 = 0;
}

void Lcd_Cmd(char a)
{
RS = 0;           
Lcd_SetBit(a); //Incoming Hex value
EN  = 1;         
__delay_ms(4);
EN  = 0;         
}

void Lcd_Clear()
{
Lcd_Cmd(0); //Clear the LCD
Lcd_Cmd(1); //Move the curser to first position
}
void Lcd_Set_Cursor(char a, char b)
{
char temp,z,y;
if(a== 1)
{
 temp = 0x80 + b - 1; //80H is used to move the curser
z = temp>>4; //Lower 8-bits
y = temp & 0x0F; //Upper 8-bits
Lcd_Cmd(z); //Set Row
Lcd_Cmd(y); //Set Column
}
else if(a== 2)
{
temp = 0xC0 + b - 1;
z = temp>>4; //Lower 8-bits
y = temp & 0x0F; //Upper 8-bits
Lcd_Cmd(z); //Set Row
Lcd_Cmd(y); //Set Column
}
}
void Lcd_Start()
{
  Lcd_SetBit(0x00);
  for(int i=1065244; i<=0; i--)  NOP();  
  Lcd_Cmd(0x03);
  __delay_ms(5);
  Lcd_Cmd(0x03);
  __delay_ms(11);
  Lcd_Cmd(0x03); 
  Lcd_Cmd(0x02); //02H is used for Return home -> Clears the RAM and initializes the LCD
  Lcd_Cmd(0x02); //02H is used for Return home -> Clears the RAM and initializes the LCD
  Lcd_Cmd(0x08); //Select Row 1
  Lcd_Cmd(0x00); //Clear Row 1 Display
  Lcd_Cmd(0x0C); //Select Row 2
  Lcd_Cmd(0x00); //Clear Row 2 Display
  Lcd_Cmd(0x06);
}

void Lcd_Print_Char(char data)  //Send 8-bits through 4-bit mode
{
   char Lower_Nibble,Upper_Nibble;
   Lower_Nibble = data&0x0F;
   Upper_Nibble = data&0xF0;
   RS = 1;             // => RS = 1
   Lcd_SetBit(Upper_Nibble>>4);             //Send upper half by shifting by 4
   EN = 1;
   for(int i=2130483; i<=0; i--)  NOP(); 
   EN = 0;
   Lcd_SetBit(Lower_Nibble); //Send Lower half
   EN = 1;
   for(int i=2130483; i<=0; i--)  NOP();
   EN = 0;
}
void Lcd_Print_String(char *a)
{
int i;
for(i=0;a[i]!='\0';i++)
  Lcd_Print_Char(a[i]);  
}


//**************************ADC and Ultra Sonic************************//


void ADC_Initialize()

{
  ADCON0 = 0b01000001; //ADC ON and Fosc/16 is selected
  ADCON1 = 0b11000000; // Internal reference voltage is selected
}

unsigned int ADC_Read(unsigned char channel){

  ADCON0 &= 0x11000101; 
  ADCON0 |= channel<<3; 
  __delay_ms(2); 
  GO_nDONE = 1; 
  while(GO_nDONE); 
  return ((ADRESH<<8)+ADRESL); 
}

//**********************UART and GSM*******************************//

void UART_Init(unsigned long baud_rate) {  
    TRISC6 = 0; // TX Pin set as output
    TRISC7 = 1; // RX Pin set as input
    //________I/O pins set __________//
    /**Initialize SPBRG register for required 
    baud rate and set BRGH for fast baud_rate**/

    SPBRG = ((_XTAL_FREQ/16)/baud_rate) - 1;
    BRGH  = 1;  // for high baud_rate
    //_________End of baud_rate setting_________//
    //****Enable Asynchronous serial port*******//

    SYNC  = 0;    // Asynchronous
    SPEN  = 1;    // Enable serial port pins
    //_____Asynchronous serial port enabled_______//
    //**Lets prepare for transmission & reception**//
    TXEN  = 1;    // enable transmission
    CREN  = 1;    // enable reception
    //__UART module up and ready for transmission and reception__//
    //**Select 8-bit mode**//  
    TX9   = 0;    // 8-bit reception selected
    RX9   = 0;    // 8-bit reception mode selected
    //__8-bit mode selected__//     
}

void UART_Write(char data) {
    while (!TXSTAbits.TRMT);
    TXREG = data;
}

void UART_Write_Text(char *text) {
    for (int i = 0; text[i] != '\0'; i++) {
        UART_Write(text[i]);
    }
}

void GSM_Init() {
    __delay_ms(100);
    UART_Write_Text("AT\r\n"); // Test AT command
    __delay_ms(1000);
    UART_Write_Text("ATE0\r\n"); // Disable echo
    __delay_ms(1000);
    UART_Write_Text("AT+CMGF=1\r\n"); // Set SMS to text mode
    __delay_ms(1000);
}

void SMS_Send(char *phone_number, char *message) {
    UART_Write_Text("AT+CMGS=\"");
    UART_Write_Text(phone_number);
    UART_Write_Text("\"\r\n");
    __delay_ms(1000);
    UART_Write_Text(message);
    __delay_ms(100);
    UART_Write(0x1A); 
    __delay_ms(1000);
}

//------------------------------------------------Real world Values Calculation---------------------------------------//
int time_taken;
int distance;

int cal_VolFuel(){
        TRISB1 = 0; //Trigger pin of US sensor is sent as output pin
        TRISB2 = 1; //Echo pin of US sensor is set as input pin  
        T1CON=0x20;

        TMR1H =0; TMR1L =0; //clear the timer bits
        Trigger = 1; 
        __delay_us(10);           
        Trigger = 0;  
	
        while (Echo==0);
            TMR1ON = 1;
        while (Echo==1);
            TMR1ON = 0;
	    
        time_taken = (TMR1L | (TMR1H<<8)); 
        distance= ((0.0272*time_taken)/2)+1;
	    //int height_Fuel=5-distance; 
        return distance;	
}

int Read_Temp() {
    int adc_value = ADC_Read(1);
    int temperature_value = (int)(adc_value*0.14663); // Convert ADC value to temperature (assuming LM35 output is 10mV per degree Celsius)
    __delay_ms(10);
    return temperature_value;
}

int cal_VolC() {
    int adc_value = ADC_Read(0);
    int volume_coolant = (int)((adc_value*0.00391)*7854); //assuming hight=4cm for pi=3.143... and r=50 cm
    return volume_coolant;
}

int cal_BatP() {
    int adc_value = ADC_Read(2);
    int percentage = (int)(adc_value * 0.097752); // Convert ADC value to percentage (assuming 1023=100%)
    return percentage;
}



//------------------------------------------------Alert System---------------------------------------//
// Declare global variables B=battery G=good C=coolant F=fuel T=temperature
// Check battery level
int check_bat_level() {
    return cal_BatP() > 25;
}

// Check coolant level 
int check_cool_level() {
    return cal_VolC() > 5000;
}

// Check fuel level
int check_fuel_level() {
    return cal_VolFuel()<=4;
}

// Check temperature level
int check_temp_level() {
    return Read_Temp() < 25;
}


//***************************************display alert on LCD*****************************//
// Display battery alert
void battery_alert() {
    if (!check_bat_level()) {
        Lcd_Clear();
        Lcd_Set_Cursor(1, 4);
        Lcd_Print_String("Low Battery");
        __delay_ms(1000);
    }
}

// Display coolant alert
void coolant_alert() {
    if(!check_cool_level()){
        Lcd_Clear();
        Lcd_Set_Cursor(1, 4);
        Lcd_Print_String("Low Coolant");
        Lcd_Set_Cursor(2, 6);
        Lcd_Print_String("Level");
        __delay_ms(1000);
    }
}

// Display fuel alert

void fuel_alert() {
    if(!check_fuel_level()){
        Lcd_Clear();
        Lcd_Set_Cursor(1, 4);
        Lcd_Print_String("Low Fuel");
        __delay_ms(1000);
    }
}

// Display temperature alert
void temp_alert() {
    if(!check_temp_level()){
        Lcd_Clear();
        Lcd_Set_Cursor(1, 2);
        Lcd_Print_String("High Temperature");
        __delay_ms(1000);}
    
}
void power_alert(){
        Lcd_Clear();
        Lcd_Set_Cursor(1, 2);
        Lcd_Print_String("Power Outage");
        __delay_ms(1000);
}

//-----------------------------------------------Generator and Fan on/off system--------------------------------//
int check_one=1;
int check_two=1;
void generator_On(){
   if(!check_bat_level()){
      battery_alert();
      RC3=1;
      if(check_one==1){
      SMS_Send(PhoneN0, "Gen ON due to Low battry"); // Send the SMS
      check_one=0;
      }
      if(check_bat_level()){
      RC3=0;
      }
   }
   else if(RC4==0){
      power_alert();
      RC3=1;
      if(check_two==1){
      SMS_Send(PhoneN0, "Gen ON due to power outage"); // Send the SMS
      check_two=0;
      }
   }
   else{
      RC3=0;
   }
}

void fan_On(){
   if(!check_temp_level()){
      temp_alert();
      RC2=1; 
   }
   else{
      RC2=0;
   }
}



void init_timer(){

    OPTION_REG = 0b00000111;  
    TMR0=60;       
    TMR0IE=1;       
    GIE=1;        
    PEIE=1;         
}

void display_values(){
	Lcd_Clear();
        Lcd_Set_Cursor(1,1);
        Lcd_Print_String("CL-");
        Lcd_Set_Cursor(2,1);
        Lcd_Print_String("Fh-");
        Lcd_Set_Cursor(1,10);
        Lcd_Print_String("B%-");
        Lcd_Set_Cursor(2,10);
        Lcd_Print_String("Tc-");
    //*******************************//
        int co_v = cal_VolC();
        Lcd_Set_Cursor(1,4);
        Lcd_Print_Char((co_v/10000)%10 + '0');
        Lcd_Print_Char((co_v/1000)%10 + '0');
        Lcd_Print_Char((co_v/100)%10 + '0');
        Lcd_Print_Char((co_v/10)%10 + '0'); 
        Lcd_Print_Char((co_v/1)%10 + '0'); 

        int temp_val = Read_Temp();
        Lcd_Set_Cursor(2,13);
        Lcd_Print_Char((temp_val/100)%10 + '0');
        Lcd_Print_Char((temp_val/10)%10 + '0');
        Lcd_Print_Char((temp_val/1)%10 + '0');

        int bat_per = cal_BatP();
        Lcd_Set_Cursor(1,13);
        Lcd_Print_Char((bat_per/100)%10 + '0');
        Lcd_Print_Char((bat_per/10)%10 + '0');
        Lcd_Print_Char((bat_per/1)%10 + '0');

        int fuei_lev = cal_VolFuel();
        Lcd_Set_Cursor(2,4);
        Lcd_Print_Char((fuei_lev/10000)%10 + '0');
        Lcd_Print_Char((fuei_lev/1000)%10 + '0');
        Lcd_Print_Char((fuei_lev/100)%10 + '0');
        Lcd_Print_Char((fuei_lev/10)%10 + '0');
        Lcd_Print_Char((fuei_lev/1)%10 + '0');
       __delay_ms(2000);
}

//***********************************timer Counter*********************************//

void __interrupt() timer_isr(){ 
    
    if(TMR0IF) {
        TMR0 = 60; // Load 6 to TMR0 to generate 10 millisecond delay
        TMR0IF = 0;
        timer_counter++; // Increment the timer counter
	}
}
void main(void){
    TRISC2 = 0; // set as output fan on/off
    TRISC3 = 0; // for generator on
    TRISC4 = 1; // set as input line power detection
    
    RC2=0;
    RC3=0;
    
    TRISD = 0x00; //PORTD declared as output for interfacing LCD
    
    TRISB0 = 1;        //DEfine the RB0 pin as input to use as interrupt pin
    TRISB1 = 0; //Trigger pin of US sensor is sent as output pin
    TRISB2 = 1; //Echo pin of US sensor is set as input pin 
    
    ADC_Initialize();
    UART_Init(9600); // Initialize UART with a baud rate of 9600
    GSM_Init(); // Initialize the GSM module     
    T1CON=0x20;
    Lcd_Start();
    

    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Print_String("Generator Monto.");
    Lcd_Set_Cursor(2,5);
    Lcd_Print_String("System");
    __delay_ms(2000);
    init_timer();
    
    
    while(1){
       display_values();
       fuel_alert();
       coolant_alert();
       fan_On();
       generator_On(); 
   if(!check_temp_level() || !check_fuel_level() || !check_cool_level() || !check_bat_level()){
            if (timer_counter >= 1800){ // Check if 10 seconds have passed
                timer_counter = 0; // Reset the timer counter
                time_elapsed = 1; // Set the flag to indicate that the state has changed
                if (time_elapsed){
                SMS_Send(PhoneN0, AlerMsg); // Send the SMS 
                __delay_ms(100);  
                Lcd_Clear();
                Lcd_Set_Cursor(1,1);
                Lcd_Print_String("SMS alert Sent");
                __delay_ms(1000);
                time_elapsed = 0; // Reset the flag
                
                }
            } 
        }
   else{
            if (timer_counter >= 3600){ // Check if 1 minute has passed
                timer_counter = 0; // Reset the timer counter
                time_elapsed = 1; // Set the flag to indicate that the state has changed
                if (time_elapsed){
                    char message[80];
                    sprintf(message, "Coolant Level: %d\nOperating Temperature: %d\nBattery Percent: %d\n Fuel Level: %d", cal_VolC(), Read_Temp(), cal_BatP(), cal_VolFuel());
                    char phone_number[] = "+251965906574"; // Replace with the recipient's phone number  
                    SMS_Send(phone_number, message); // Send the SMS  // Send the SMS 
                    __delay_ms(100);
                    time_elapsed = 0; // Reset the flag
                    
                    Lcd_Clear();
                    Lcd_Set_Cursor(1,5);
                    Lcd_Print_String("SMS Sent!");
                    __delay_ms(1000);
                    
                }
            } 
        }   
    }
}
