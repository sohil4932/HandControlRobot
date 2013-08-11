#include <hidef.h>          
#include "derivative.h"     
#include <math.h>
#include "mma845x.h"
#include "iic.h"

static void initialise_hardware (void);
static void initialise_accelerometer (void);
static void read_accelerometer_info (void); 
static void update_green_leds (Bool active);
static void configure_LEDs_for_PWM (void);
static void millisecond_delay (int delay);
static void leds_show_tilt (byte *values);
static void wait_for_movement (void);
void send_code(unsigned long data);
void mark(int time);
void space(int time);
static void microsecond_delay (int delay);

/* Accelerometer I2C slave address */                                   
#define SlaveAddressIIC       (0x1d << 1)      /* Address = 1D in top 7 bits of byte */
#define value 4000/40
#define TOPBIT 0x80000000

/* Demo modes */
enum {
    DEMO_ORIENTATION,
    DEMO_SHAKE,
    DEMO_TAP,
    DEMO_FREEFALL,
    DEMO_TRANSIENT,
    NUM_DEMOS
};

/* Accelerometer can return 8-bit or 14-bit samples */
enum {
    READ_MODE_8BITS,
    READ_MODE_14BITS
};

/* Maximum value to use for PWM in PWM-controlled LED modes */
#define MAX_PWM_VALUE   500

/* 100ms debounce delay used for SW1 - SW3 push buttons */
#define DEBOUNCE_DELAY  100                      

/* Global variables */
static byte CurrentDemo;                       /* Current demo, e.g. DEMO_FREEFALL */
static byte NewDemo;                           /* New demo requested */

static byte CurrentRange;                      /* Current range setting, e.g. FULL_SCALE_2G */
static byte NewRange;                          /* New range setting requested */

static byte CurrentReadMode;                   /* Current read size, e.g. READ_MODE_14BITS */
static byte NewReadMode;                       /* New read size requested */

static byte last_int_source_reg;               /* Last value of INT_SOURCE_REG as read by 
                                                  read_accelerometer_info() 
                                               */ 
static byte last_sysmod_reg;                   /* Last value of SYSMOD_REG */
static byte last_f_status_reg;                 /* Last value of F_STATUS_REG */
static byte last_transient_src_reg;            /* Last value of TRANSIENT_SRC_REG */
static byte last_pl_status_reg;                /* Last value of PL_STATUS_REG */
static byte last_pulse_src_reg;                /* Last value of PULSE_SRC_REG */
static byte last_ff_mt_src_1_reg;              /* Last value of FF_MT_SRC_1_REG */
static byte last_xyz_values [6];               /* Last values of X, Y and Z */ 

/************************************************************************
*       main - Program main entry point                                 *
*************************************************************************/
void main(void) 
{
     /* Initialise system clocks, GPIO, etc */           
    initialise_hardware ();
  
    millisecond_delay (2000);
    /* Force main loop to initially select DEMO_ORIENTATION, 14-bit accuracy, 2g sensitivity */
    NewDemo = DEMO_ORIENTATION;
    NewReadMode = READ_MODE_14BITS;
    NewRange = FULL_SCALE_2G;
    
    CurrentDemo = ~NewDemo;
    
    /* Loop forever to run demos */ 
    for (;;) {

        if (NewRange != CurrentRange || NewReadMode != CurrentReadMode || NewDemo != CurrentDemo) {
            
             /* Yes. Orange LEDs all off */

            /* Short delay to debounce key press */
            millisecond_delay (DEBOUNCE_DELAY);    
           
            /* Reconfigure accelerometer */ 
            CurrentRange = NewRange;
            CurrentReadMode = NewReadMode; 
            CurrentDemo = NewDemo;
            initialise_accelerometer ();
            
            /* Update the green indicator LEDs to show current demo mode, sensitivity, etc */
            update_green_leds (TRUE);
        }
        
        /* ---- Check for accelerometer event(s) ---- */
        read_accelerometer_info (); 
        
        /* ---- Handle any transition from accelerometer Wake -> Sleep mode ---- */
        if (last_int_source_reg & SRC_ASLP_MASK) {
            if ((last_sysmod_reg & SYSMOD_MASK) == 0x02) {
                /* Accelerometer has gone into sleep mode because no activity has been detected
                   in the last 20 seconds. We put the main CPU to sleep too, to save maximum power.
                */
                wait_for_movement ();               /*  Routine doesn't return until 
                                                        accelerometer is awake again */
                                                        
                /* Force reconfiguration now we're awake again */ 
                CurrentDemo = ~NewDemo;
                continue;
            }
        }
       
        /* ---- Update LEDs if accelerometer event occurred ---- */
        switch (CurrentDemo) {
        case DEMO_ORIENTATION:
            /* New XYZ tilt data ready? */
            if (last_int_source_reg & SRC_DRDY_MASK) {            

                /* Temporarily disable interrupts so 'last_xyz_values' data is not overwritten */
                DisableInterrupts;
                
                /* Make a copy of the data, unpacked into 14-bit format if necessary */
                if (CurrentReadMode == READ_MODE_14BITS) {
                    /* Already read 14-bit data into last_xyz_values[] */
                }
                else {
                    /* Unpack 8-bit data into 14 bit format */
                    last_xyz_values [5] = 0;
                    last_xyz_values [4] = last_xyz_values [2];
                    last_xyz_values [3] = 0;
                    last_xyz_values [2] = last_xyz_values [1];
                    last_xyz_values [1] = 0;
                } 
                EnableInterrupts;   
                leds_show_tilt (&last_xyz_values[0]);
            }
            break;
            
            
        default:
            break;
        }

        /* Wait for interrupt signalling accelerometer event or button press,
           unless another event has already occurred
        */
        DisableInterrupts;
        last_int_source_reg = IIC_RegRead(SlaveAddressIIC, INT_SOURCE_REG);   
        if (last_int_source_reg == 0 && NewRange == CurrentRange && NewReadMode == CurrentReadMode && NewDemo == CurrentDemo)  
            asm "wait";
        EnableInterrupts;

    } /* Loop forever */


    /* Program should never reach here ! */
}

void send_code(unsigned long data) 
{  int i;
   configure_LEDs_for_PWM();
   
   data = data << (32 - 20);
   mark(2400);
   space(600);

   for(i = 0 ; i< 20 ; i++) 
   {
    if(data & TOPBIT)
    {
    mark(1200);
    space(600);
    } 
    else
    {
    mark(600);
    space(600);
    }
    data <<=1;
   }
}

void mark(int time) 
{
      TPM2C0V = (int) (value/3);      
      microsecond_delay(time);
}

void space(int time) 
{
    TPM2C0V = 0;      
    microsecond_delay(time);
}



/************************************************************************
*       initialise_hardware - Initialise clock, GPIO, etc               *
*************************************************************************
; This routine initialises the main microprocessor - everything we need
; except the accelerometer which is done separately
*/
static void initialise_hardware (void) 
{
    /* Configure write-once System Options Register (SOPT)
            STOPE=1   : Stop Mode enabled 
            COPE=0    : Disable watchdog     
            BKGDPE=1  : Background Debug Mode Pin Enable
    */
    SOPT1 = 0x23;                   
    
    /* Configure SIM Clock Set and Select Register (SIMCO)
            CS=0      : Don't drive CLKOUT pin 
    */
    SIMCO &= ~0x07;                 
  
    /* Configure System Power Management Status and Control 1 Register (SPMSC1)  
            LVDIE=0   : No interrupt on low voltage detected
            LVDRE=1   : Generate reset on low voltage detected instead [write-once bit]
            LVDSE=1   : Low-voltage detect is enabled in STOP modes
            LVDE=1    : Low-voltage detect is enabled [write-once bit]
    */
    SPMSC1 = 0x1c;                                       

    /* Configure System Power Management Status and Control 2 Register (SPMSC2)  
            PPDE=1    : Partial power down enabled [write-once bit]
            PPDC=0    : STOP3 low power mode enabled
    */
    SPMSC2 = 0x02;                                      

    /* Configure System Power Management Status and Control 3 Register (SPMSC3)  
            LVDV=0    : Low trip point selected for low-voltage detect 
            LVWV=0    : Low trip point selected for low-voltage warning 
            LVWIE=0   : Interrupt on low voltage warning is disabled
    */
    SPMSC3 = 0x00;    
                 
    /* Currently using internal clock. Any trim value stored in flash ? */   
    if (*(unsigned char *) 0xffaf != 0xff) {
        /* Yes. Initialise trim */  
        MCGTRM = * (unsigned char *) 0xffaf;    /* Factory MCGTRM value stored in flash at address 0xFFAF */
        MCGSC = * (unsigned char *) 0xffae;     /* Factory FTRIM value stored in flash at address 0xFFAE */
    }

    /* System clock initialization... 
       We configure the system to use the clock derived from the 16MHz external crystal 
       connected to XOSC2.
       
       System is placed into FEE mode, so
       
            f_MCGOUT = (f_EXT * F) / (R * B)
                     = (16MHz * 512) / (512 * 2) = 8MHz processor clock
                     
            f_BUS = f_MCGOUT / 2 = 4MHz bus clock
    */

    /* Multipurpose Clock Generator Control Register 2 (MCGC2)
            BDIV=01   : Bus clock = selected clock / 2
            RANGE=1
            HGO=1
            LP=0
            EREFS=1   : External clock source requested
            ERCLKEN=0 : External reference clock to ADC inactive
            EREFSTEN=0: External clock disabled in Stop mode
            
        NB: Write to MCGC2 before MCGC1. This sets the multiplier before 
            enabling the FLL
    */
    MCGC2 = 0x74;                          
  
    /* Multipurpose Clock Generator Control Register 3 (MCGC3)
            LOLIE=0
            PLLS=0
            CME=0
            DIV32=1
            VDIV=0001 
    */
    MCGC3 = 0x11;                           
  
    /* Multipurpose Clock Generator Control Register 1 (MCGC1)
            CLKS=00   : Output of FLL is selected as system clock source
            RDIV=100  : Divide-by-512 since RANGE=1 and DIV32=1
            IREFS=0   : External reference clock selected
            IRCLKEN=0 : MCGIRCLK inactive
            IREFSTEN=0   
    */
    MCGC1 = 0x20;                           
  
    /* Wait until external reference is selected */
    while (MCGSC_IREFST)
        ;
                        
    /* Multipurpose Clock Generator Control Register 4 (MCGC4)
        DMX32=0      
        DRST_DRS=0  (Together these give F = 512)
    */
    MCGC4 = 0x00;                           
  
    /* Wait until FLL is locked */
    while(!MCGSC_LOCK)                      
        ;
  
    /* Wait until FLL clock is selected as a bus clock reference */
    while (MCGSC & 0x0c)
        ;
        
    /* Enable clocks to all peripherals...
       Note: To conserve power, clocks to unused peripherals should be disabled.
             However, since this software is likely to be used by programmers
             new to the S08, we enable all the clocks here to make it easier
             for the programmer to get things working
    */
    
    /* SCGC1: CMT=1,TPM2=1,TPM1=1,ADC=1,DAC=1,IIC=1,SCI2=1,SCI1=1 */
    SCGC1 = 0xff;                                      
    
    /* SCGC2: USB=1,PDB=1,IRQ=1,KBI=1,PRACMP=1,TOD=1,SPI2=1,SPI1=1 */
    SCGC2 = 0xff;                                      
    
    /* SCGC3: VREF=1,CRC=1,FLS1=1,TRIAMP2=1,TRIAMP1=1,GPOA2=1,GPOA1=1 */
    SCGC3 = 0xff;                                      
  
    /* Disable TPMs */
    TPM1SC = 0;
    TPM2SC = 0;                    
  
    /* Port A GPIO 

        A7  : Output connected to green 'Orientation' LED, initially 1 [off]
        A6  : Output connected to green 'Transient' LED, initially 1 [off]
        A5  : Output connected to green 'Freefall' LED, initially 1 [off]
        A4  : Output connected to green '14-bit data' LED , initially 1 [off]
        A3  : Input connected to 'Channel Select' button (KBI1P2)
        A2  : Input connected to 'Data 8/14bit' button   (KBI1P1)
        A1  : Input connected to 'g-Range Select' button (KBI1P0)
        A0  : Output connected to green '8-bit data' LED, initially 1 [off]
    */
    PTAPE = ~0xf1;                  /* Pull-ups on inputs */                                               
    PTAIFE = ~0xf1;                 /* Low-pass filter on inputs */                           
    PTASE = 0xf1;                                      
    PTADS = 0xf1;                   
    PTAD = 0xf1;                   
    PTADD = 0xf1;                  

    /* Port B GPIO 
    
        B7  : Not used
        B6  : Not used
        B5  : XTAL2
        B4  : EXTAL2
        B3  : XTAL1
        B2  : EXTAL1
        B1  : Output connected to green 'Tap' LED, initially 1 [off]
        B0  : Output connected to green 'Shake' LED, initially 1 [off]
    */
    PTBPE = ~0x03;                  /* Pull-ups on inputs and unused pins */
    PTBIFE = ~0x03;                 /* Low-pass filter on inputs */
    PTBSE = 0x03;                                      
    PTBDS = 0x03;                                             
    PTBD = 0x03;            
    PTBDD = 0x03;           
                          
    /* Port C GPIO  
    
        C7  : Not used
        C6  : Not used
        C5  : Output connected to orange LED ring, initially 0 [off]
        C4  : Output connected to orange LED ring, initially 0 [off]
        C3  : Output connected to orange LED ring, initially 0 [off]
        C2  : Output connected to orange LED ring, initially 0 [off]
        C1  : Output connected to orange LED ring, initially 0 [off]
        C0  : Output connected to orange LED ring, initially 0 [off]
    */
    PTCPE = ~0x3f;                  /* Pull-ups on inputs and unused pins */ 
    PTCIFE = ~0x3f;                 /* Low-pass filter on inputs  */
    PTCSE = 0x3f;                                      
    PTCDS = 0x3f;                                                  
    PTCD = 0x00;                 
    PTCDD = 0x3f;                
  
    /* Port D GPIO  
    
        D7  : RX1
        D6  : TX1
        D5  : Output connected to orange LED spoke, initially 1 [off]
        D4  : Output connected to orange LED spoke, initially 1 [off]
        D3  : Output connected to orange LED spoke, initially 1 [off]
        D2  : Output connected to orange LED spoke, initially 1 [off]
        D1  : MCU_RESET
        D0  : MCU_BKGD
    */
    PTDIFE = 0x82;                  /* Low pass filter on RX1 and MCU_RESET */
    PTDSE = 0x3c;                                      
    PTDDS = 0x3c;                                            
    PTDD = 0x3c;               
    PTDDD = 0x3c;           
                            
    /* Port E GPIO  

        E7  : Output connected to orange LED spoke, initially 1 [off]
        E6  : MCU_RXD
        E5  : MCU_TXD
        E4  : Not used
        E3  : Not used
        E2  : Not used
        E1  : MMA_INT1 (KBI2P4 input)
        E0  : MMA_INT2 (KBI2P3 input)
    */
    PTEPE = ~0x80;                  /* Pull-ups on inputs and unused pins */ 
    PTEIFE = ~0x80;                 /* Low pass filter on inputs */
    PTESE = 0x80;                                      
    PTEDS = 0x80;           
    PTED = 0x80;               
    PTEDD = 0x80;           

    /* Port F GPIO 
     
        F7  : Output connected to green '4g' LED, initially 1 [off]
        F6  : Output connected to green '2g' LED, initially 1 [off]
        F5  : Not used
        F4  : MMA_SDA
        F3  : MMA_SCL
        F2  : Output connected to orange LED spoke, initially 1 [off]
        F1  : Output connected to orange LED spoke, initially 1 [off]
        F0  : Output connected to orange LED spoke, initially 1 [off]
    */
    PTFPE = ~0xc7;                  /* Pull-ups on inputs and unused pins */      
    PTFIFE = ~0xc7;                 /* Low pass filter on inputs */     
    PTFSE = 0xc7;                                      
    PTFDS = 0xc7;                
    PTFD = 0xc7;                 
    PTFDD = 0xc7;                

    /* Port G GPIO 
    
        G7-1: Not bonded out
        G0  : Output connected to green '8g' LED, initially 1 [off]
    
    
    */  
    PTGPE = ~0x01;                  /* Pull-ups on unused pins */                      
    PTGIFE = ~0x01;                 /* Low pass filter on inputs */                     
    PTGSE = 0x01;                                      
    PTGDS = 0x01;             
    PTGD = 0x01;              
    PTGDD = 0x01;             


    /* Initialize Inter-Integrated Circuit (IIC) used to talk to accelerometer */    
    SOPT3_IICPS = 1;                /* PTF3/PTF4 used for IIC comms */
    
    IICF = 0x02;                    /* IIC Frequency Divider Register
                                            MULT = 0b00 : mul = 1
                                            ICR  = 0b000010 : scl = 24
                                      
                                       IIC Frequency = Bus clock x mul / scl
                                                     = 4 MHz x 1 / 24
                                                     = 167kHz
                                    */
    
    IICC1 = 0x80;                   /* IIC Control Register 1 : Enable IIC module
                                         RSTA     =0  : Not generating repeat start
                                    */

    /* Enable keyboard interrupt from push buttons
       labelled 'g-Range Select', 'Data 8/14bit' and 'Channel Select'
    */
    KBI1SC = 0;                     /* Mask keyboard interrupts */
    KBI1ES = 0;                     /* Detect falling edge */
    PTAPE_PTAPE1 = 1;               /* Enable pull up on KBI1P0 (Port A pin 1) */
    PTAPE_PTAPE2 = 1;               /* Enable pull up on KBI1P1 (Port A pin 2) */
    PTAPE_PTAPE3 = 1;               /* Enable pull up on KBI1P2 (Port A pin 3) */

    KBI1PE = 0x07;                  /* Enable the KBI pins */
    KBI1SC_KB1ACK = 1;              /* Clear any false interrupt */
    KBI1SC_KB1IE = 1;               /* Enable keyboard interrupts */


   /* Configure interrupts from accelerometer:
          MMA_INT1 (KBI2P4 input, Port E pin 1)
          MMA_INT2 (KBI2P3 input, Port E pin 0)
    */
    KBI2SC = 0;                     /* Mask keyboard interrupts */
    KBI2ES = 0;                     /* Detect falling edge */
    PTEPE_PTEPE1 = 1;               /* Enable pull up on KBI2P4 (Port E pin 1) */
    PTEPE_PTEPE0 = 1;               /* Enable pull up on KBI2P3 (Port E pin 0) */

    KBI2PE = 0x18;                  /* Enable the KBI pins */
    KBI2SC_KB2ACK = 1;              /* Clear any false interrupt */
    KBI2SC_KB2IE = 0;               /* Disable MMA_INTx interrupts initially */

}

/************************************************************************
*       initialise_accelerometer - Perform demo-specific initialisation *
*************************************************************************
; This routine configures the accelerometer, the TPMs and the LEDs in
; preparation for the selected demo mode
*/
static void initialise_accelerometer (void)  
{ 
    byte    n, int_mask;
   
    /* Disable MMA_INTx interrupts so there's no danger of a spurious interrupt
       during reconfiguration 
    */               
    KBI2SC_KB2IE = 0;               

    /* Put MMA845xQ into Standby Mode */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, (IIC_RegRead(SlaveAddressIIC, CTRL_REG1) & ~ACTIVE_MASK));
    
    /* Configure sensor for:
         Sleep Mode Poll Rate of 1.56Hz (640ms) for maximum power saving 
         System Output Data Rate (ODR) of 200Hz (5ms)
         User-specified read mode (8-bit or 14-bit data)
    */
    n = ASLP_RATE_640MS + DATA_RATE_5MS;          
    if (CurrentReadMode == READ_MODE_8BITS)
        n |= FREAD_MASK;
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, n);  
  
    /* Select user-specified sensitivity, 2g/4g/8g */
    IIC_RegWrite(SlaveAddressIIC, XYZ_DATA_CFG_REG, CurrentRange);
    
    /* Configure Sleep/Wake modes 
        SMODS1:0   : 1 1    Low power mode when asleep
        SLPE       : 1      Sleep enabled
        MODS1:0    : 0 0    Normal mode when awake
    */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG2, 0x1c);          
    IIC_RegWrite(SlaveAddressIIC, ASLP_COUNT_REG, 62);      /* Sleep after 20 seconds of inactivity */         

    /* Disable the FIFO */
    IIC_RegWrite(SlaveAddressIIC, F_SETUP_REG, 0x00);

    switch (CurrentDemo) {
    case DEMO_TRANSIENT:
    case DEMO_SHAKE:
    case DEMO_ORIENTATION:
        /* In all three of these demo modes we configure the accelerometer to detect
           movement:
           
                DEMO_TRANSIENT   - We configure the accelerometer to detect small movements
                                   of > 0.063g                   
                                   
                DEMO_SHAKE       - We configure the accelerometer to detect movements
                                   of > 0.5g
                                                      
                DEMO_ORIENTATION - We don't care about the movement data itself, but
                                   we use transient detection so that the accelerometer
                                   can tell us when the board isn't being used. When
                                   it transitions to Sleep mode, we can put the main
                                   processor to sleep too.
                                   
          By using the high-pass filter we can remove the constant effect of gravity,
           and only detect changes in acceleration. See Application Note AN4071.
        */

        /* ELE = 1     : Event latch enabled 
           ZTEFE = 1   : Raise event on any of Z, Y, X axes
           YTEFE = 1
           XTEFE = 1
           HBF_BYP = 0 : High-pass filter enabled
        */
        IIC_RegWrite(SlaveAddressIIC, TRANSIENT_CFG_REG, 0x1e);
        
        /* Set High-pass filter cut-off frequency for best sensitivity */
        IIC_RegWrite(SlaveAddressIIC, HP_FILTER_CUTOFF_REG, 0x03);   

        if (CurrentDemo == DEMO_SHAKE) {
            /* Transient is indicated when acceleration on one of the axes
               is above threshold 8 x 0.063g = 0.5g 
             */   
             IIC_RegWrite(SlaveAddressIIC, TRANSIENT_THS_REG, 8);
        } 
        else {
            /* Transient is indicated when acceleration on one of the axes
               is above threshold 1 x 0.063g - i.e. a small movement 
            */   
            IIC_RegWrite(SlaveAddressIIC, TRANSIENT_THS_REG, 1);
        }

        /* Internal debounce counter. For an ODR of 200Hz in Normal mode,
           motion is indicated after 5 x 1 = 5ms. 
        */
        IIC_RegWrite(SlaveAddressIIC, TRANSIENT_COUNT_REG, 1);

        /* Interrupt signalled on INT1 when transient detected */
        int_mask = INT_EN_TRANS_MASK;

        if (CurrentDemo == DEMO_ORIENTATION) {
            /* Interrupt also signalled when new data is ready */
             int_mask |= INT_EN_DRDY_MASK;
         
            /* Set up TPMs to produce edge-aligned PWM signals */
            configure_LEDs_for_PWM ();    
        }
        break;
        
    }
   
    /* Configure interrupts */
    int_mask |= INT_EN_ASLP_MASK;                           /* Also generate interrupt on 
                                                               Sleep <--> Wake transition 
                                                            */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG4, int_mask);
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG5, 0xfd);         /* All interrupts mapped to MMA_INT1 */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG3, 0x78);         /* Active-low interrupts, all functions 
                                                               wake system 
                                                            */  

    /* Throw away any stale interrupt info */
    last_int_source_reg = 0;
    
    /* Put MMA845xQ into Active Mode */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, (IIC_RegRead(SlaveAddressIIC, CTRL_REG1) | ACTIVE_MASK));

    /* Enable MMA_INTx interrupts */
    KBI2SC_KB2IE = 1;               
}

/************************************************************************
*       read_accelerometer_info - Read any state change info            *
*************************************************************************
; This routine reads the accelerometer's INT_SOURCE_REG to check whether
; any events have occured. It then reads any event-specific data in order
; to clear the interrupts.
;
; Values are returned in the following global variables:
;
;       last_int_source_reg       - INT_SOURCE_REG
;       last_sysmod_reg           - SYSMOD_REG                     
;       last_f_status_reg         - F_STATUS_REG                    
;       last_transient_src_reg    - TRANSIENT_SRC_REG               
;       last_pl_status_reg        - PL_STATUS_REG                   
;       last_pulse_src_reg        - PULSE_SRC_REG                   
;       last_ff_mt_src_1_reg      - FF_MT_SRC_1_REG                 
;       last_xyz_values           - OUT_X_MSB_REG, etc          
;
; See the comments in the routine 'accelerometer_isr' for more information 
*/
static void read_accelerometer_info (void)
{
    /* Which source caused an interrupt, if any ? */
    last_int_source_reg = IIC_RegRead(SlaveAddressIIC, INT_SOURCE_REG);

    /* Sleep <--> Wake transition detected ? */
    if (last_int_source_reg & SRC_ASLP_MASK) {            
        /* Yes - Clear the event */
        last_sysmod_reg = IIC_RegRead(SlaveAddressIIC, SYSMOD_REG);
    } 
    
    /* FIFO event detected ? */
    if (last_int_source_reg & SRC_FIFO_MASK) {            
        /* Yes - Clear the event 
           Note that our demos don't use this event, so no further processing is required
        */
        last_f_status_reg = IIC_RegRead(SlaveAddressIIC, F_STATUS_REG);
    } 
    
    /* Transient detected ? */
    if (last_int_source_reg & SRC_TRANS_MASK) {            
        /* Yes - Clear the transient event */
        last_transient_src_reg = IIC_RegRead(SlaveAddressIIC, TRANSIENT_SRC_REG);
    } 
    
    /* Landscape/portrait orientation change detected ? */
    if (last_int_source_reg & SRC_LNDPRT_MASK) {            
        /* Yes - Clear the event 
           Note that our demos don't use this event, so no further processing is required
        */
        last_pl_status_reg = IIC_RegRead(SlaveAddressIIC, PL_STATUS_REG);
    } 
    
    /* Tap/pulse event detected ? */
    if (last_int_source_reg & SRC_PULSE_MASK) {
        /* Yes - Clear the pulse event */
        last_pulse_src_reg = IIC_RegRead(SlaveAddressIIC, PULSE_SRC_REG);
    }
    
    /* Freefall detected ? */
    if (last_int_source_reg & SRC_FF_MT_1_MASK) {            
        /* Yes - Clear the freefall event */
        last_ff_mt_src_1_reg = IIC_RegRead(SlaveAddressIIC, FF_MT_SRC_1_REG);
    }             

    /* Freefall detected ? */
    if (last_int_source_reg & SRC_DRDY_MASK) {            
        /* Yes - read the XYZ data to clear the event */
         if (CurrentReadMode == READ_MODE_14BITS) {
             /* Read 14-bit XYZ results using a 6 byte IIC access */
             IIC_RegReadN(SlaveAddressIIC, OUT_X_MSB_REG, 6, (byte *) &last_xyz_values[0]);
         }
         else {
             /* Read 8-bit XYZ results using a 3 byte IIC access */
             IIC_RegReadN(SlaveAddressIIC, OUT_X_MSB_REG, 3, (byte *) &last_xyz_values[0]);
         }
    }      
}

/************************************************************************
*       update_green_leds - Update status of green indicator LEDs       *
*************************************************************************
; The green LEDs are used to indicate what mode the board's in:
;
;       LEDs D1-D2 indicate the accelerometer read mode 
;       LEDs D3-D5 indicate the accelerometer sensitivity 
;       LEDs D6-D10 indicate the current demo mode
;
; Arguments:
;   Bool       active        - Specifies whether the board is active 
;                              or asleep (turn all green LEDs off) 
;                           
*/
static void update_green_leds (Bool active)
{
    /* Green LEDs all off initially */
    PTAD_PTAD0 = 1;                             /* 8-bit/14-bit data LEDs */
    PTAD_PTAD4 = 1;             

    PTFD_PTFD6 = 1;                             /* 2g/4g/8g LEDs */
    PTFD_PTFD7 = 1;
    PTGD_PTGD0 = 1;
    
    PTAD_PTAD5 = 1;                             /* Channel select LEDs */
    PTAD_PTAD6 = 1;
    PTAD_PTAD7 = 1;
    PTBD_PTBD0 = 1;
    PTBD_PTBD1 = 1;

    if (active) {
        /* Use green LEDs D1-D2 to indicate current accelerometer read mode. 
           NB: Internally the accelerometer always uses 14 bits, but it can be configured to
           only return 8 bits to achieve a faster IIC transfer speed)
        */
        if (CurrentReadMode == READ_MODE_8BITS) 
            PTAD_PTAD0 = 0;             /* Turn on '8-bit data' LED */
        else
            PTAD_PTAD4 = 0;             /* Turn on '14-bit data' LED */
            
        /* Use green LEDs D3-D5 to indicate current accelerometer sensitivity - 2g, 4g or 8g */
        switch (CurrentRange) {
        case FULL_SCALE_2G:
            /* Turn on '2g' green LED indicator */
            PTFD_PTFD6 = 0;
            break;
             
        case FULL_SCALE_4G:     
            /* Turn on '4g' green LED indicator */
            PTFD_PTFD7 = 0;
            break;
             
        case FULL_SCALE_8G:
            /* Turn on '8g' green LED indicator */
            PTGD_PTGD0 = 0;
            break;
        }
             
        /* Use green LEDs D6-D10 to indicate current demo mode */   
        switch (CurrentDemo) {
        case DEMO_ORIENTATION:
            /* Turn 'Orientation' LED on */
            PTAD_PTAD7 = 0;
            break;
                
        case DEMO_SHAKE:
            /* Turn 'Shake' LED on */
            PTBD_PTBD0 = 0;
            break;
                
        case DEMO_TAP:
            /* Turn 'Tap' LED on */
            PTBD_PTBD1 = 0;
            break;
                
        case DEMO_FREEFALL:
            /* Turn 'Freefall' LED on */
            PTAD_PTAD5 = 0;
            break;
                
        case DEMO_TRANSIENT:
            /* Turn 'Transient' LED on */
            PTAD_PTAD6 = 0;
            break;
                
        }
    }
}

/************************************************************************
*       millisecond_delay - Pause execution for specified time          *
*************************************************************************
; NB: Millisecond timings are approximate only. 
;
; Arguments:
;   int        delay        - Approximate number of milliseconds to delay
*/
static void millisecond_delay (int delay)
{
    unsigned short count;
    
    /* Configure TPM1 as a free-running counter with approximately 32 microsecond tick... */  
    
    while (delay > 0) {
        if (delay > (0xffffUL/32)) {
            /* Need to loop multiple times to achieve full delay because a value 
               of 2048 or more would overflow 16 bits in "count = (count << 5) - count" below 
            */
            count = (0xffffUL/32);
            delay -= (int) (0xffffUL/32);
        }
        else {
            /* Do the whole thing in one hit */  
            count = (unsigned short) delay;
            delay = 0;
        }
        
        /* Multiply by 31, since 31 ticks = 31 x 32 microseconds = 0.992 ms,
           or approximately 1 ms. We don't care about the slight inaccuracy 
        */   
        count = (count << 5) - count;  
            
        TPM1SC = 0;                     /* Stop the timer */  
        TPM1SC_TOF = 0;                 /* Clear the TOF flag */  
             
        TPM1C0SC = 0;                   /* Configure the timer */  
        TPM1MOD = count;           
        TPM1CNT = 0;
        TPM1SC = 0x0f;                  /* Restart the timer: CLKS = 01 (Use BUSCLK), PS=111 (Divide-by-128)
                                           Frequency = fbus / 128, so period = 128 / 4 MHz = 32 microseconds 
                                         */
        
        /* Wait for timer to expire */  
        while (!_TPM1SC.Bits.TOF)  
            ;
    }
}
static void microsecond_delay (int delay)
{
   
            
        TPM1SC = 0;                     /* Stop the timer */  
        TPM1SC_TOF = 0;                 /* Clear the TOF flag */  
             
        TPM1C0SC = 0;                   /* Configure the timer */  
        TPM1MOD = delay;           
        TPM1CNT = 0;
        TPM1SC = 0x0a;                  /* Restart the timer: CLKS = 01 (Use BUSCLK), PS=111 (Divide-by-128)
                                           Frequency = fbus / 128, so period = 128 / 4 MHz = 32 microseconds 
                                         */
        
        /* Wait for timer to expire */  
        while (!_TPM1SC.Bits.TOF)  
            ;
    
}

static void leds_show_tilt (byte *values)
{
    int     x, y, g, r; 
    int diff =0;
    byte pwm_r=0,pwm_l=0;
    unsigned long data=0;
    double  theta, x2, y2, radius;
    
    /* Get X and Y values */
    x = (values [0] << 8) | values [1];
    y = (values [2] << 8) | values [3];
    
    /* What reading would correspond to normal gravity (at FULL_SCALE_2G) ? */
    if (CurrentReadMode == READ_MODE_8BITS)
        g = 0x3f00;                         
    else
        g = 0x3ffc;
        
    /* Indicate how much the board is tipped 
    */
    x2 = (double) x / g; 
    y2 = (double) y / g;
    radius = sqrt (x2 * x2 + y2 * y2); 
   
   if(radius < 1 / 6.0) {
    data = 0;
   } 
   else{
    
        for (r = 1; r < 6; r++) {
        if (radius > r / 6.0)
            diff += 20;}  
   
   
    if (x == 0 && y == 0)
        theta = 0.0;
    else {    
        theta = atan2(-x, -y);
        if (theta < 0.0)
            theta +=  2.0 * _M_PI;
         }
   
   if(theta <= ( 5.0 * _M_PI / 4.0) && theta >= ( 3.0 * _M_PI / 4.0)) {
      pwm_r = 150 + diff;
     pwm_l = 150 + diff;
     data = (pwm_l << 12) | pwm_r;
     
   }
    else if(theta > (_M_PI / 4.0) && theta < ( 3.0 * _M_PI / 4.0)) {
      pwm_r = 150 + diff;
     pwm_l = 150 + diff;
     data = (pwm_l << 12) | pwm_r;
     data = data | (1 << 9);
    }
    else if(theta > (5.0*_M_PI / 4.0) && theta < ( 7.0 * _M_PI / 4.0)){
      pwm_r = 150 + diff;
     pwm_l = 150 + diff;
     data = (pwm_l << 12) | pwm_r;
     data = data | (1 << 8);
    } 
    else {
     pwm_r = 150 + diff;
     pwm_l = 150 + diff;
     data = (pwm_l << 12) | pwm_r;
     data = data | (3 << 8);}
   }
    send_code(data); 
    millisecond_delay(45);
    }

static void configure_LEDs_for_PWM (void)
{
    TPM2SC = 0;      
    TPM2MOD = value ;
     
    TPM2C0SC = 0x28;       
    
    TPM2SC = 0x08;     
}

/************************************************************************
*       button_press_isr - Interrupt handler for push buttons           *
*************************************************************************
; Interrupt handler for push buttons SW1-SW3 as follows:
;
;       g-Range Select (SW1) : Select 2g, 4g or 8g mode
;       Data 8/14bit   (SW2) : Configure accelerometer to return 8- or 14-bit data 
;       Channel Select (SW3) : Select a new demo
;
; In each case we just update a global variable, leaving it upto non-ISR
; code to change the configuration at a convenient time. Note that we can't
; directly change the accelerometer here, because a transition from 
; STANDBY -> ACTIVE mode will cause certain registers to be clobbered
*/
interrupt void button_press_isr (void)
{
    byte    val;
    
    /* Acknowledge the interrupt */  
    val = PTAD;
    KBI1SC_KB1ACK = 1;
    
    /* Which button was pressed ?  */
    val = ~val;                         /* Active low */

    if (val & (1 << 1)) {
        /* User pressed 'g-Range Select' button (SW1) 
           Request switch to next range setting
         */
         if (NewRange == CurrentRange) {
            switch (CurrentRange) {
            case FULL_SCALE_2G:
                NewRange = FULL_SCALE_4G;
                break;
            case FULL_SCALE_4G:
                NewRange = FULL_SCALE_8G;
                break;
            case FULL_SCALE_8G:
            default:
                NewRange = FULL_SCALE_2G;
                break;
            }
        }      
        else {
            /* Ignore key bounce. Main loop hasn't yet seen and responded
               to previous key press
            */
        }
    }

    if (val & (1 << 2)) {
        /* User pressed 'Data 8/14bit' button (SW2) 
           Request toggle current mode
        */
        if (NewReadMode == CurrentReadMode) {
            if (CurrentReadMode == READ_MODE_14BITS)  
                NewReadMode = READ_MODE_8BITS;    
            else    
                NewReadMode = READ_MODE_14BITS;            
        }
        else {
            /* Ignore key bounce. Main loop hasn't yet seen and responded
               to previous key press
            */
        }
    }
    
    if (val & (1 << 3)) {
        /* User pressed 'Channel Select' button (SW3)
           Request new demo
        */
        if (NewDemo == CurrentDemo) {
            /* Bump and wrap */
            NewDemo = CurrentDemo + 1;
            if (NewDemo == NUM_DEMOS)
                NewDemo = 0;
        }
        else {
            /* Ignore key bounce. Main loop hasn't yet seen and responded
               to previous key press
            */
        }
    } 
}

/************************************************************************
*       accelerometer_isr - Interrupt handler for accelerometer         *
*************************************************************************
; The accelerometer signals an interrupt by driving MMA_INT1 low
; low, which causes an edge-triggered interrupt via KBI2. The interrupt will
; wake the main processor up if it's sleeping.
;
; To tell the accelerometer that we've seen the interrupt, so that it stops
; driving MMA_INT1 low, it's necessary to read an event-specific register.  
; For example, to clear a 'Transient' interrupt you would read the TRANS_SRC 
; register.
; 
; However, this requires a bit of thought to achieve the best design:
;
; (a) It's good practice to keep ISRs as short as possible. We don't want to
;     perform IIC reads here if we can avoid it
;
; (b) Reading a register like TRANS_SRC also has another effect besides 
;     clearing MMA_INT1. Normally the bits in TRANS_SRC are latched to 
;     indicate which transient occurred. Reading TRANS_SRC un-freezes the bits 
;     and starts the next accelerometer transient detection, so that re-reading
;     it wouldn't return the same result.
; 
; The solution relies on recognising that although the accelerometer will 
; continue to drive MMA_INT1 low until we've cleared the interrupt, this won't
; cause a new falling edge on KBI2.
;
; Thus, provided we acknowledge the KBI2 interrupt here, the processor
; isn't immediately interrupted again. We can leave it up to non-interrupt 
; code in the routine 'read_accelerometer_info' to read TRANS_SRC, etc
*/
interrupt void accelerometer_isr (void)
{
    /* Acknowledge KBI2 interrupt */
    KBI2SC_KB2ACK = 1;
}

/************************************************************************
*       wait_for_movement - Wait until accelerometer indicates movement *
*************************************************************************
; This routine is called when the accelerometer indicates that no event of 
; the current type has occurred for 20 seconds. (For example if we're
; detecting 'taps', the routine is called if the user hasn't tapped the
; board for 20 seconds).
;
; We reconfigure the accelerometer to detect any movement, then put the
; CPU into a low power 'STOP 3' mode
;
; The subroutine doesn't return until the accelerometer indicates
; movement - e.g. the user has picked the board up.
*/
static void wait_for_movement (void)
{
    /* Configure accelerometer for motion detection in low power mode... */
      
    /* Disable MMA_INTx interrupts so there's no danger of a spurious interrupt
       during reconfiguration 
    */               
    DisableInterrupts;
    KBI2SC_KB2IE = 0;                  

    /* Put MMA845xQ into Standby Mode */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, (IIC_RegRead(SlaveAddressIIC, CTRL_REG1) & ~ACTIVE_MASK));
    
    /* ELE = 1     : Event latch enabled 
       ZTEFE = 1   : Raise event on any of Z, Y, X axes
       YTEFE = 1
       XTEFE = 1
       HBF_BYP = 0 : High-pass filter enabled
    */
    IIC_RegWrite(SlaveAddressIIC, TRANSIENT_CFG_REG, 0x1e);
    IIC_RegWrite(SlaveAddressIIC, HP_FILTER_CUTOFF_REG, 0x00);   
    
    /* Transient is indicated when acceleration on one of the axes
       is above threshold 1 x 0.063g  
    */   
    IIC_RegWrite(SlaveAddressIIC, TRANSIENT_THS_REG, 1);
    IIC_RegWrite(SlaveAddressIIC, TRANSIENT_COUNT_REG, 1);

    /* Interrupt signalled on INT1 when transient detected, or on Sleep <--> Wake transition */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG4, (INT_EN_TRANS_MASK | INT_EN_ASLP_MASK));
    
    /* Since reconfiguring the accelerometer here will wake it up again,
       tell it to go back to sleep as soon as possible
    */ 
    IIC_RegWrite(SlaveAddressIIC, ASLP_COUNT_REG, 1);              

    /* Put MMA845xQ back into Active Mode */
    IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, (IIC_RegRead(SlaveAddressIIC, CTRL_REG1) | ACTIVE_MASK));

    /* Enable MMA_INTx interrupts */
    last_int_source_reg = 0;
    KBI2SC_KB2IE = 1;               

    
    /* Turn green LEDs off too */
    update_green_leds (FALSE);

    /* Sleep until accelerometer wakes up */
    for (;;) {
    
        /* Wait until we have event from accelerometer or push button */
        DisableInterrupts;
        last_int_source_reg = IIC_RegRead(SlaveAddressIIC, INT_SOURCE_REG);   
        if (last_int_source_reg == 0)  
            asm "stop";
        read_accelerometer_info ();
        EnableInterrupts;

        /* Accelerometer wake-up event? */
        if (last_int_source_reg & SRC_ASLP_MASK) {
            if ((last_sysmod_reg & SYSMOD_MASK) != 0x02) {
                /* Yes */  
                break;
            }
        }
       
        /* Did user press one of the buttons SW1 - SW3 ?
           If so, return to active mode
        */
        if (NewRange != CurrentRange || NewReadMode != CurrentReadMode || NewDemo != CurrentDemo) 
            break;
   }

   
}             
