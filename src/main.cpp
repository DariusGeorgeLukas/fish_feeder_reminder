#include <micro_cac_msp430fr2433.h>


#include <array>


/************************************************************
* STATUS REGISTER BITS
************************************************************/

#define C                      (0x0001)
#define Z                      (0x0002)
#define N                      (0x0004)
#define V                      (0x0100)
#define GIE                    (0x0008)
#define CPUOFF                 (0x0010)
#define OSCOFF                 (0x0020)
#define SCG0                   (0x0040)
#define SCG1                   (0x0080)

#define LPM3_bits              (SCG1+SCG0+CPUOFF)

using namespace msp430fr2433;



    // template< typename T > 
    // class  Interrupt_Counter : public Interrupt_Handler{
    //     public :
    //         void overflow();
    //         T read(){return m_count;};
    //         void reset(){m_count = 0;}
    //     private:
    //         volatile T m_count =0;
    // };

    // using Interrupt_Counter_uint32_t = Interrupt_Counter<u_int32_t>;

    // template< typename T > 
    // void Interrupt_Counter<T>::overflow(){
    //     ++m_count;
    // }


    /// Define all the Hardware Resource objects

    using rtc_one_seconds_rollover_t = RTC<Clock_Source::_XT1CLK,Clock_Divider::_DIV_1, 0x7FFF > ;
   // using rtc_one_seconds_rollover_t = RTC<Clock_Source::_XT1CLK,Clock_Divider::_DIV_1, 0x7FFF/100 > ;
    using rtc_one_millisecond_rollover_t = RTC<Clock_Source::_VLOCLK,Clock_Divider::_DIV_1,7> ;

    // pick Real Time Clock RCT counter step size
  //  rtc_one_millisecond_rollover_t rtc; // with rollover counter
    rtc_one_seconds_rollover_t rtc;
    
   // Describe the UART you want 
    UART<eUSCI_A::_1
        ,Buad_Rate::_115200
        ,CLK_Rate::_16MHz
        , 5
        , 1
        // Optional         
         ,Clock_Source::_SMCLK
         ,USC_Mode::_Uart
         ,Uart_Parity::_disable
         ,Bit_Order::_LSB
        > uart;


    // Describe the Pins
    Pin<Port::_2, PinN::_3, Pin_Function_Output> p2_3_func_timing;
    // Pin<Port::_1, PinN::_7, Pin_Function_Output> p1_7_direction;
   //Pin<Port::_1, PinN::_4, Pin_Function_Output> uart_write_pin;
    Pin<Port::_2, PinN::_7, Pin_Function_Input_PullUp> button_pin;
    

// PWM for P1_2  GREEN LED
Timer_A_PWM< 
     PWM_ouput::_P1_2
    ,Timer_Compare_Output_Mode::_Toggle_Reset
    ,Timer_Mode::_UpDown
    ,Timer_Clock_Source::_SMCLK
    ,Timer_Clock_Divider::_DIV_1
    > pwm_pin_1_2_LED_Green;


// PWM for P1_1  BLUE LED
Timer_A_PWM< 
     PWM_ouput::_P1_1
    ,Timer_Compare_Output_Mode::_Toggle_Reset
    ,Timer_Mode::_UpDown
    ,Timer_Clock_Source::_SMCLK
    ,Timer_Clock_Divider::_DIV_1
    > pwm_pin_1_1_LED_Blue;


// PWM for P1_1  BLUE LED
Timer_A_PWM< 
     PWM_ouput::_P1_4  // Same as TX
    ,Timer_Compare_Output_Mode::_Toggle_Reset
    ,Timer_Mode::_UpDown
    ,Timer_Clock_Source::_SMCLK
    ,Timer_Clock_Divider::_DIV_1
    > pwm_pin_1_4_LED_Red;

Timer_A_Callback<
    Timer_Instance::_2
    ,Timer_Compare_Output_Mode::_Toggle_Reset
    ,Timer_Mode::_Up
    ,Timer_Clock_Source::_ACLK
    ,Timer_Clock_Divider::_DIV_1
    ,Timer_Expansion_Clock_Divider::_DIV_1
    > timerA_callback_2;



Pin<Port::_1, PinN::_0, Pin_Function_Output> p1_0_Onboard_Red;
//Pin<Port::_1, PinN::_4, Pin_Function_Output> p1_4_LED_Red;     // Same as TX
//Pin<Port::_1, PinN::_2, Pin_Function_Output> p1_2_LED_Green;
//Pin<Port::_1, PinN::_1, Pin_Function_Output> p1_1_LED_Blue;



// template< typename RTC , typename T = uint32_t> 
// class Duration{
//     public:
//     Duration():m_last_rtc_value( RTC::read()){}

//     T time_since_last_read(){
//         T ct =  RTC::read();
//         T delta = 0;
//         if( ct > m_last_rtc_value){
//             delta = ct - m_last_rtc_value;
//         }else {
//             delta =  std::numeric_limits<u_int16_t>::max() - m_last_rtc_value + ct ;
//         }
//         m_last_rtc_value = ct;
//         return (delta * 30517)/1000 ;
//     }
//     private:
//         T m_last_rtc_value; 
// };


volatile uint16_t red_brightness = 0;
volatile uint16_t green_brightness = 0;
volatile uint16_t blue_brightness = 0;

volatile uint8_t blink_counter = 0;

volatile uint32_t time_sice_last_pressed =0;

static const uint32_t init_alert_time = 36000; // 10 hours

uint16_t green_blink_brightness = 0;
uint16_t red_blink_brightness = 0;


/*
 *  
 */
int main(int __attribute__((unused))  argc, __attribute__((unused))  char** argv) {

    //cout << " running :  " << argv[0] <<  " argc: " << argc  << endl;
    LOG_DEBUG( "running " <<  argv[0] <<  " argc: " << argc );


    // This will first call config on each resource in order then
    // it will call start on each resource in order
    init_resource(  
        //  p1_7_direction
      //  uart_write_pin
        p2_3_func_timing
        ,button_pin
        ,uart
        ,rtc
        ,p1_0_Onboard_Red
        //,p1_4_LED_Red
        //,p1_2_LED_Green
        ,pwm_pin_1_4_LED_Red
        ,pwm_pin_1_2_LED_Green
        ,pwm_pin_1_1_LED_Blue
        ,timerA_callback_2
    );

    

    //pwm_pin_1_1.setReloadValue(7975/200);   // 100Khz 
    pwm_pin_1_2_LED_Green.setReloadValue(10000);   
    pwm_pin_1_2_LED_Green.setCompareValue( green_brightness );
    pwm_pin_1_1_LED_Blue.setCompareValue(blue_brightness);
    pwm_pin_1_2_LED_Green.start();
    
    pwm_pin_1_4_LED_Red.setReloadValue(10000);   
    pwm_pin_1_4_LED_Red.setCompareValue(red_brightness);
    pwm_pin_1_4_LED_Red.start();


    timerA_callback_2.setReloadValue( 25000); //
    timerA_callback_2.setCompareValue( 18000);
    timerA_callback_2.stop();

    //pwm_pin_1_2_LED_Green.setReloadValue(2000);
   

  //  reload_value = 10000;
    // pwm_pin_1_1.setReloadValue(reload_value);   //
    // pwm_pin_1_1.setCompareValue(20);   

#ifdef __BUILD_FOR_LOCAL__
    generate_optimized_config_code(std::cout);
#endif
    

    

    // uart_write_pin.digitalWrite(LOW);
    



  //  pwm_pin_1_1.start();

    uint32_t last_time = 0;

   // auto& rtc_counter = rtc.get_interrupt_handler();

    msp430fr2433::Duration<rtc_one_seconds_rollover_t>  duration;

  


    #ifndef __BUILD_FOR_LOCAL__
        __bis_SR_register(GIE); //Global interrupt enable
    #endif

    p1_0_Onboard_Red.digitalWrite(HIGH);

    button_pin.interrupt_on_falling_edge();
    button_pin.interrupt_enable();    
    
   // int velocity_change = -50;

   
//   p1_4_LED_Red.digitalWrite(HIGH);
  // p1_2_LED_Green.digitalWrite(LOW);
   //p1_1_LED_Blue.digitalWrite(LOW);

   uart.blocking_write('H'); 
   uart.blocking_write('\n');  
  // p1_4_LED_Red.digitalWrite(LOW);
   
    p1_0_Onboard_Red.digitalWrite(LOW);




    while(1){
        uint32_t current_time = rtc.rtc_counter();
        
        if(  (current_time - last_time ) >= 5){
            
            last_time = current_time;

           // reload_value = pwm_pin_1_1.readReloadValue();
            // if( reload_value > 10000) {
            //     velocity_change = -50;
            //     p1_7_direction.digitalWrite( !p1_7_direction.digitalRead());
            // }
            // else if ( reload_value < 5000 ){
            //     velocity_change = 50;
            // }

            // reload_value = reload_value + velocity_change;
           

            // if(current_time < 100 ){
            //     red_brightness = current_time * 100;
            // }
            
        //    p1_0_Onboard_Red.digitalWrite(LOW);
           // uart_write_pin.digitalWrite(HIGH);

            duration.time_since_last_read() ;
            uart.printNumber (time_sice_last_pressed  );
          //  uart.blocking_write('-'); 
       //     uart.printNumber (duration.time_since_last_read() );
            uart.blocking_write('\n'); 
            // uart.printNumber (stepper_position  );
            // uart.blocking_write(' '); 
            // uart.printNumber (reload_value );
            
         //   uart.printNumber( pwm_pin_1_1.read_compare_timer_control_config() );
         //   uart.blocking_write('\n');  

           
          
        }
        //Enter LPM3 mode with interrupts enabled
        #ifndef __BUILD_FOR_LOCAL__
          LPM3;
        #endif
    }

    return 0;
}






#ifndef __BUILD_FOR_LOCAL__




extern "C" void __attribute__((interrupt(RTC_VECTOR))) RTC_ISR(void)
{
     __bis_SR_register_on_exit(GIE + LPM3_bits);

    switch (__even_in_range(rtc.read_interuppt_flag(),2)){
        case 0: break;  //No interrupts
        case 2:{         //RTC overflow
            msp430fr2433::g_rtc_counter++;
            ++time_sice_last_pressed;

            if( time_sice_last_pressed > init_alert_time ){
                uint16_t delta_t = (time_sice_last_pressed - init_alert_time);
                if( delta_t > 0 and   delta_t % 20 == 0 ){

                    if(  delta_t < (60 * 60) ){
                        green_blink_brightness = delta_t;
                        red_blink_brightness = delta_t;
                    }else{
                        green_blink_brightness = 0;
                        red_blink_brightness = delta_t;
                    }

                    blink_counter = (delta_t / (30 * 60)) + 1; // blink once for every 30 minutes past init_alert_time
                    timerA_callback_2.start();

                }
            }
            #ifndef __BUILD_FOR_LOCAL__
            __bic_SR_register_on_exit(LPM3_bits);
            #endif
    
            
        }
        break;
        default: break;
    }
}



// TO DO Button need to be on PORT  1 or 2 for Interrupts

extern "C" void __attribute__((interrupt(PORT1_VECTOR))) PORT_1_ISR(void)
{
     __bis_SR_register_on_exit(GIE + LPM3_bits );

    switch (__even_in_range( Port::_1::interrupt_vector() ,P1IV_P1IFG7)){
        case P1IV_NONE: break;  //No interrupts
        case P1IV_P1IFG0: break;  
        case P1IV_P1IFG1: break;  
        case P1IV_P1IFG2: 
            
    //         if( stepper_zeroed == -1 ){

        break;  
        case P1IV_P1IFG3: break;  
        case P1IV_P1IFG4: break;  
        case P1IV_P1IFG5: break;  
        case P1IV_P1IFG6: break;  
        case P1IV_P1IFG7: break;  
        default: break;
    }
}


bool button_down = false;

extern "C" void __attribute__((interrupt(PORT2_VECTOR))) PORT_2_ISR(void)
{
     __bis_SR_register_on_exit(GIE );

    switch (__even_in_range( Port::_2::interrupt_vector() ,P2IV_P2IFG7)){
        case P1IV_NONE: break;  //No interrupts
        case P1IV_P1IFG0: break;  
        case P1IV_P1IFG1: break;  
        case P1IV_P1IFG2: 
          
        break;  
        case P1IV_P1IFG3:
            
    
        break;
        case P1IV_P1IFG4: break;  
        case P1IV_P1IFG5: break;  
        case P1IV_P1IFG6: break;  
        case P1IV_P1IFG7:  // 10h = Interrupt Source: Port 2.7 interrupt; Interrupt Flag: P2IFG.7
            {
                 pwm_pin_1_2_LED_Green.start();
                //auto& led = p1_1_LED_Blue;
                if( !blue_brightness and !green_brightness){
                    button_down = true;
                    if( time_sice_last_pressed <  init_alert_time ){
                        blue_brightness = 0;
                        green_brightness = 5000;
                    }else{
                        blue_brightness = 5000;
                        green_brightness = 0;
                    }
                    red_brightness = 0;
                    //led.digitalWrite( HIGH );
                    button_pin.interrupt_on_rising_edge();
                    button_pin.interrupt_enable(); 
                }else{
                    //led.digitalWrite( LOW );
                    blue_brightness = 0;
                    green_brightness = 0;
 
                    if( button_down ){
                        button_down = false;
                        if( time_sice_last_pressed >  init_alert_time ){                        
                            time_sice_last_pressed = 0;
                        }
                    }
                    button_pin.interrupt_on_falling_edge();
                    button_pin.interrupt_enable();
                }     
            }
   
        break;  
        default: break;
    }
}




extern "C" void __attribute__((interrupt(TIMER0_A1_VECTOR))) TIMER0_A1_ISR(void)
{
      __bis_SR_register_on_exit(GIE + LPM3_bits);
    /* Any access, read or write, of the TBxIV register automatically resets the
       highest "pending" interrupt flag. */
    switch( __even_in_range(pwm_pin_1_2_LED_Green.interrupt_vector(),14) )
    {
        case 0:    break;      // No interrupt
        case 0x02:        // Interrupt Source: Capture/compare 1; Interrupt Flag: TAxCCR1 CCIFG
      
        break;
        case 0x04:  break;      // 
        case 0x06:  break;      // 
        case 0x08:  break;      // 
        case 0x0A:  break;      // 
        case 0x0C:  break;      // 
        case 0x0E:   //   
            
             pwm_pin_1_2_LED_Green.stop();
             pwm_pin_1_2_LED_Green.setCompareValue(green_brightness); 
             pwm_pin_1_1_LED_Blue.setCompareValue(blue_brightness);
             if( green_brightness | blue_brightness ){
                pwm_pin_1_2_LED_Green.start();
             }           
        break;      // Overflow
                                 
        default: break;
    }
}



extern "C" void __attribute__((interrupt(TIMER1_A1_VECTOR))) TIMER1_A1_ISR(void)
{
      __bis_SR_register_on_exit(GIE + LPM3_bits);
    /* Any access, read or write, of the TBxIV register automatically resets the
       highest "pending" interrupt flag. */
    switch( __even_in_range(pwm_pin_1_4_LED_Red.interrupt_vector(),14) )
    {
        case 0:    break;      // No interrupt
        case 0x02:        // Interrupt Source: Capture/compare 1; Interrupt Flag: TAxCCR1 CCIFG
      
        break;
        case 0x04:  break;      // 
        case 0x06:  break;      // 
        case 0x08:  break;      // 
        case 0x0A:  break;      // 
        case 0x0C:  break;      // 
        case 0x0E:   //   
            
             pwm_pin_1_4_LED_Red.stop();
             pwm_pin_1_4_LED_Red.setCompareValue(red_brightness); 
             if( red_brightness ){
                pwm_pin_1_4_LED_Red.start();
             }           
        break;      // Overflow
                                 
        default: break;
    }
}






extern "C" void __attribute__((interrupt(TIMER2_A1_VECTOR))) TIMER2_A1_ISR(void)
{
      __bis_SR_register_on_exit(GIE + LPM3_bits);
    /* Any access, read or write, of the TBxIV register automatically resets the
       highest "pending" interrupt flag. */
    switch( __even_in_range(timerA_callback_2.interrupt_vector(),14) )
    {
        case 0:    break;      // No interrupt
        case 0x02:        // Interrupt Source: Capture/compare 1; Interrupt Flag: TAxCCR1 CCIFG

            red_brightness = red_blink_brightness;
            green_brightness = green_blink_brightness;
            pwm_pin_1_4_LED_Red.start();                    

        break;
        case 0x04:      //  Interrupt Source: Capture/compare 1; Interrupt Flag: TAxCCR2 CCIFG
           
            break;      
        case 0x06:  break;      // 
        case 0x08:  break;      // 
        case 0x0A:  break;      // 
        case 0x0C:  break;      // 
        case 0x0E:   //   Overflow  TAxCCR0
            red_brightness = 0;
            green_brightness = 0;
            if( blink_counter > 0 ){
                --blink_counter;
            
                if( blink_counter == 0 ){
                    timerA_callback_2.stop();
                }
            }
 
        break;  
                                 
        default: break;
    }
}







#endif