// RTOS Framework - Fall 2016
// J Losh

// Student Name: Venkata Dinesh Jakkampudi
// TO DO: Add your name on this line.  Do not include your ID number.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// xx_rtos_coop.c   Cooperative version of your project
// xx_rtos_prempt.c Premptive version of your project
// (xx is a unique number that will be issued in class)

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 4 Pushbuttons and 4 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for green and yellow LEDs (temporary to guarantee compilation)
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define YELLOW_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))
#define ORANGE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON_1 (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))
#define PUSH_BUTTON_2  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))
#define PUSH_BUTTON_3  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON_4  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))


//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_QUEUE_SIZE 10
struct semaphore
{
    unsigned int count;
    unsigned int queueSize;
    unsigned int processQueue[MAX_QUEUE_SIZE]; // store task index here
} *s, keyPressed, keyReleased, flashReq, pi;

// task
#define STATE_INVALID    0 // no task
#define STATE_READY      1 // ready to run
#define STATE_BLOCKED    2 // has run, but now blocked by semaphore
#define STATE_DELAYED    3 // has run, but now awaiting timer

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks


struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *pi;
    void *sp;                      // location of stack pointer for thread
    uint8_t priority;              // 0=highest, 7=lowest
    uint8_t currentPriority;
    uint8_t basePriority;// used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    uint64_t timer;
    float cpuPercent;
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];
//-----------------------------------------------------------------------------
// RTOS Kernel
//-----------------------------------------------------------------------------

void rtosInit()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
    
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC;
    NVIC_ST_RELOAD_R = 0X9C68;
    
    // REQUIRED: systick for 1ms system timer
}

int rtosScheduler()
{
    // REQUIRED: Implement prioritization to 16 levels
    bool ok,found;
    static uint8_t task = 0xFF;
    ok = false;
    found = false;
    while(!found)
    {
        while (!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY);
        }
        if(tcb[task].currentPriority!=0)
        {
            tcb[task].currentPriority--;
            ok=false;
        }
        else
        {
            tcb[task].currentPriority = tcb[task].basePriority;
            found=true;
        }
    }
    return task;
}


bool createThread(_fn fn, char name[], int priority)
{
    bool ok = false;
    uint8_t i = 0,j;
    bool found = false;
    // REQUIRED: store the thread name
    
    // REQUIRED: take steps to ensure a task switch cannot occur
    // save starting address if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_READY;
            tcb[i].pid = fn;
            tcb[i].pi = fn;
            strcpy(tcb[i].name,name);
            stack[i][255]=tcb[i].pid;
            for(j=254;j>246;j--)
                stack[i][j]=1;
            // REQUIRED: preload stack to look like the task had run before
            tcb[i].sp = &stack[i][247]; // REQUIRED: + offset as needed for the pre-loaded stack
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            tcb[i].basePriority = priority;
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to destroy a process
void destroyProcess(_fn fn)
{
    int i=0,j=0;
    bool found = false;
    while (!found && (i < taskCount))
    {
        found = (tcb[i++].pid ==  fn);
    }
    if(found)
    {
        tcb[i-1].state=STATE_INVALID;
        tcb[i-1].pid=0;
        taskCount--;
        found = false;
    }
    remque(&keyPressed,i-1);
    remque(&keyReleased,i-1);
    remque(&flashReq,i-1);
    remque(&pi,i-1);
    
}

void remque(void* pSemaphore, int x)
{
    s=pSemaphore;
    int i=0,j;
    bool found=false;
    while(!found && (i<s->queueSize))
    {
        found=(s->processQueue[i++]==x);
    }
    if(found)
    {
        for(j=i-1;j<s->queueSize;j++)
            s->processQueue[j]=s->processQueue[j+1];
        s->queueSize--;
    }
}
putsp(uint32_t x)
{
    return x;
}
getsp()
{
    __asm(" MOV R0, R13");
}
void rtosStart()
{
    // REQUIRED: add code to call the first task to be run, restoring the preloaded context
    _fn fn;
    taskCurrent = rtosScheduler();
    putsp(tcb[taskCurrent].sp);
    __asm("  MOV R13, R0");
    __asm("  POP {R14,R11,R10,R9,R8,R7,R6,R5,R4}");
    WTIMER5_TAV_R = 0; // reset counter for next period
    WTIMER5_TBV_R = 0;
    __asm(" BX LR");
    // Add code to initialize the SP with tcb[task_current].sp;
    // Restore the stack to run the first process
}

void init(void* p, int count)
{
    s = p;
    s->count = count;
    s->queueSize = 0;
}

// REQUIRED: modify this function to yield execution back to scheduler
void yield()
{
    
    __asm(" POP {R1,R2}");
    __asm(" PUSH {R14,R11,R10,R9,R8,R7,R6,R5,R4}");
    tcb[taskCurrent].sp=getsp();
    tcb[taskCurrent].timer =  tcb[taskCurrent].timer+(WTIMER5_TAV_R | (WTIMER5_TBV_R << 32));
    
    taskCurrent = rtosScheduler();
    putsp(tcb[taskCurrent].sp);
    __asm("  MOV R13, R0");
    __asm("  POP {R14,R11,R10,R9,R8,R7,R6,R5,R4}");
    WTIMER5_TAV_R = 0; // reset counter for next period
    WTIMER5_TBV_R = 0;
    __asm(" BX LR");
    // push registers, call scheduler, pop registers, return to new function
    
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses
void sleep(uint32_t tick)
{
    tcb[taskCurrent].ticks=tick;
    __asm(" POP {R1,R2}");
    __asm(" PUSH {R14,R11,R10,R9,R8,R7,R6,R5,R4}");
    tcb[taskCurrent].sp=getsp();
    tcb[taskCurrent].state = STATE_DELAYED;
    tcb[taskCurrent].timer =  tcb[taskCurrent].timer+(WTIMER5_TAV_R | (WTIMER5_TBV_R << 32));
    taskCurrent = rtosScheduler();
    putsp(tcb[taskCurrent].sp);
    __asm("  MOV R13, R0");
    __asm("  POP {R14,R11,R10,R9,R8,R7,R6,R5,R4}");
    WTIMER5_TAV_R=0;WTIMER5_TBV_R=0;
    __asm(" BX LR");
    // push registers, set state to delayed, store timeout, call scheduler, pop registers,
    // return to new function (separate unrun or ready processing)
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler
void wait(void* pSemaphore)
{
    s=pSemaphore;
    int i=0;bool found;found=false;
    __asm(" POP {R1,R2}");
    __asm(" PUSH {R14,R11,R10,R9,R8,R7,R6,R5,R4}");
    tcb[taskCurrent].sp=getsp();
    tcb[taskCurrent].timer =  tcb[taskCurrent].timer+(WTIMER5_TAV_R | (WTIMER5_TBV_R << 32));
    if( s->count > 0)
        s->count--;
    
    else
    {
        for(i=0;i<taskCount;i++)
        {
            if(tcb[i].semaphore==s)
            {found=true;
                break;}
        }
        if(found)
        {
            if(tcb[i].priority> tcb[taskCurrent].priority)
                tcb[i].basePriority=tcb[taskCurrent].priority;
        }
        s->processQueue[s->queueSize]=taskCurrent;
        s->queueSize++;;
        tcb[taskCurrent].state=STATE_BLOCKED;
        taskCurrent = rtosScheduler();
    }
    putsp(tcb[taskCurrent].sp);
    __asm("  MOV R13, R0");
    __asm("  POP {R14,R11,R10,R9,R8,R7,R6,R5,R4}");
    WTIMER5_TAV_R=0;WTIMER5_TBV_R=0;
    __asm(" BX LR");
}

// REQUIRED: modify this function to signal a semaphore is available
void post(void* pSemaphore)
{
    int i,j;
    s=pSemaphore;
    
    s->count++;
    if(s->count > 0)
    {
        tcb[taskCurrent].basePriority=tcb[taskCurrent].priority;
        tcb[taskCurrent].semaphore=0;
        if(s->queueSize > 0)
        {
            j=s->processQueue[0];
            tcb[j].state=STATE_READY;
            tcb[j].semaphore=s;
            for(i=0;i<s->queueSize;i++)
                s->processQueue[i]=s->processQueue[i+1];
            s->queueSize--;
            s->count--;
        }
        
    }
    
}

// REQUIRED: modify this function to add support for the system timer
void systickIsr()
{
    int j;
    tcb[taskCurrent].timer =  tcb[taskCurrent].timer+(WTIMER5_TAV_R | (WTIMER5_TBV_R << 32));
    
    for(j=0;j<10;j++)
    {
        if(tcb[j].state==STATE_DELAYED && tcb[j].ticks!=0)
            tcb[j].ticks--;
        if (tcb[j].state==STATE_DELAYED && tcb[j].ticks==0)
            tcb[j].state=STATE_READY;
    }
    WTIMER5_TAV_R=0;
    WTIMER5_TBV_R=0;
}
// REQUIRED: modify this function to add support for the service call
void svcCallIsr()
{
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // REQUIRED: Add initialization for orange, red, green, and yellow LEDs
    //           4 pushbuttons, and uart
    
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    SYSCTL_GPIOHBCTL_R = 0;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOA |SYSCTL_RCGC2_GPIOF;
    GPIO_PORTE_DIR_R = 0x01E;
    GPIO_PORTE_DR2R_R = 0x1E;
    GPIO_PORTE_DEN_R = 0x1E;
    GPIO_PORTD_DEN_R = 0x0F;
    GPIO_PORTD_PUR_R = 0x0F;
    GPIO_PORTF_DEN_R = 0x10;
    GPIO_PORTF_PUR_R = 0x10;
    
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    GPIO_PORTA_DEN_R |= 0x00000003;
    GPIO_PORTA_AFSEL_R |= 3;
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
    
    UART0_CTL_R = 0;
    UART0_CC_R = UART_CC_CS_SYSCLK;
    UART0_IBRD_R = 21;
    UART0_FBRD_R = 45;
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
    
    // Configure FREQ_IN for frequency counter
    GPIO_PORTD_AFSEL_R |= 0x40;                      // select alternative functions for FREQ_IN pin
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD6_M;           // map alt fns to FREQ_IN
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD6_WT5CCP0;
    GPIO_PORTD_DEN_R |= 0x40;                        // enable bit 6 for digital input
    
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 0;                               // configure as 64-bit counter (A+B)
    WTIMER5_TAMR_R =  TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;                               //
    WTIMER5_IMR_R = 0;                               // turn-off interrupts
    WTIMER5_TAV_R = 0;
    WTIMER5_TBV_R = 0; // zero counter for first period
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R &= ~(1 << (INT_WTIMER5A-16-96));      // turn-off interrupt 120 (WTIMER5A)
    
    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
    GPIO_PORTD_AFSEL_R |= 0x0F;                      // select alternative functions for AN0 (PE3) AN1 (PE2) AN2 (PE1)
    GPIO_PORTD_DEN_R &= ~0x0F;                       // turn off digital operation on pin PE3 PE2 PE1
    GPIO_PORTD_AMSEL_R |= 0x0F;                      // turn on analog operation on pin PE3 PE2 PE1
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN2 ;
    // ADC0_PSSI_R = ADC_PSSI_SS2;
    // disable sample sequencer 2 (SS2) for programming
    ADC0_EMUX_R = ADC_EMUX_EM2_PROCESSOR;            // select SS2 bit in ADCPSSI as trigger
    ADC0_SSMUX2_R = 0x00007654;                               // set first sample to AN0
    ADC0_SSCTL2_R = ADC_SSCTL2_END3;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN2;                 // enable SS2 for operation
    ADC0_SAC_R=ADC_SAC_AVG_32X;
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
    // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-15 indicating which of 4 PBs are pressed
uint8_t readPbs()
{
    int d1=0,d2=0,d3=0,d4=0;
    if((!PUSH_BUTTON_1) || (!PUSH_BUTTON_2) || (!PUSH_BUTTON_3) || (!PUSH_BUTTON_4))
    {
        if(!PUSH_BUTTON_1)
            d1=0x1;
        if(!PUSH_BUTTON_2)
            d2=0x2;
        if(!PUSH_BUTTON_3)
            d3=0x4;
        if(!PUSH_BUTTON_4)
            d4=0x8;
        return(d1|d2|d3|d4);
    }
    else
        return 0;
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        //wait(&pi);
        GREEN_LED ^= 1;
        sleep(125);
        //post(&pi);
    }
}

void oneshot()
{
    while(true)
    {
        wait(&flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(1000);
    // give another process a chance
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        //wait(&pi);
        for (i = 0; i < 4000; i++)
        {
            
            partOfLengthyFn();
            
        }
        RED_LED ^= 1;
        //post(&pi);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(&keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            
            buttons = readPbs();
            yield();
        }
        post(&keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(&flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            createThread(flash4Hz, "Flash_4hz", 0);
        }
        if ((buttons & 8) != 0)
        {
            destroyProcess(flash4Hz);
        }
        
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(&keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(&keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

#define MAX_CHARS 80
char string[MAX_CHARS+1];
int parseCount;
char type[10];
int position[10];
uint8_t d;


void shell()
{
    while (true)
    {
        bool found=false;int i=0,j;char printstring[10];char stc[4];double k=0;
        // REQUIRED: add processing for the shell commands here
        putsUart0("\r\n");
        putsUart0("Enter Command:\r\n");
        getString();
        call(string);
        if(isCommand(string)==true)
        {
            if(d==1)
            {
                for(j=0;j<taskCount;j++)
                    k=k+tcb[j].timer;
                for(i=0;i<taskCount;i++){
                    tcb[i].cpuPercent=((tcb[i].timer)/k)*100;tcb[i].timer=0;}
                putsUart0("------------------------------------------------------------\r\n");
                putsUart0("PID id\t\tProcessName\t\t% of CPU\r\n");
                putsUart0("------------------------------------------------------------\r\n");
                for(i=0;i<taskCount;i++)
                {
                    sprintf(printstring,"%d		%s		%f",tcb[i].pid,tcb[i].name,tcb[i].cpuPercent);
                    putsUart0(printstring);
                    putsUart0("\r\n");
                }
            }
            
            if(d==2)
            {
                putsUart0("------------------------------------------------------------\r\n");
                putsUart0("Semaphore\t\tTask Waiting On Semaphore\r\n");
                putsUart0("------------------------------------------------------------\r\n");
                i=ipcsPrint(&keyPressed);
                if(i!=0){
                    sprintf(printstring,"keyPressed		%s",tcb[i].name);
                    putsUart0(printstring);}
                putsUart0("\r\n");
                i=ipcsPrint(&keyReleased);
                if(i!=0){
                    sprintf(printstring,"keyReleased		%s",tcb[i].name);
                    putsUart0(printstring);
                    putsUart0("\r\n");}
                i=ipcsPrint(&flashReq);
                if(i!=0){
                    sprintf(printstring,"flahReq     		%s",tcb[i].name);
                    putsUart0(printstring);
                    putsUart0("\r\n");}
                i=ipcsPrint(&pi);
                if(i!=0){
                    sprintf(printstring,"pi     			%s",tcb[i].name);
                    putsUart0(printstring);
                    putsUart0("\r\n");}
            }
            
            if(d==3)
            {
                strcpy(stc,&string[position[1]]);
                i=atoi(stc);
                if(i!=*idle)
                    destroyProcess(i);
            }
            
            if(d==4)
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            
            if(d==5)
            {
                while(!found && (i<taskCount))
                {
                    found=strcmp1(&string[position[1]],tcb[i++].name);
                }
                if(found)
                {
                    sprintf(stc,"%d",tcb[i-1].pid);
                    putsUart0("\r\n");
                    putsUart0(stc);
                    putsUart0("\r\n");
                }
                
            }
            
            if(d==6)
            {
                while(!found && (i<taskCount))
                {
                    found=strcmp1(&string[position[0]],tcb[i++].name);
                }
                if(found)
                    createThread(tcb[i-1].pi, tcb[i-1].name, 0);
                
            }
            
            if(d == 7)
                magnet();
            
        }
        else
        {
            putsUart0("Invalid Command\r\n");
        }
    }
}

int ipcsPrint(void* pSemaphore)
{
    s=pSemaphore;
    int i=0;
    if(s->queueSize >0)
        i=s->processQueue[0];
    return i;
}


void magnet()
{
    int raw[4],k,raw1[4],raw2[4]=0,raw3[4]=0;uint16_t i1=0,i2=0,i3=0,i4=0,d;
    char str[10],str1[10];
    
    while(1)
    {
        ADC0_PSSI_R |= ADC_PSSI_SS2;                     // set start bit
        
        
        raw[0]=ADC0_SSFIFO2_R;
        raw[1]=ADC0_SSFIFO2_R;
        raw[2]=ADC0_SSFIFO2_R;
        raw[3]=ADC0_SSFIFO2_R;
        if(i1==0)
        {
            raw1[0]=raw[0];
            raw1[1]=raw[1];
            raw1[2]=raw[2];
            raw1[3]=raw[3];i1=1;i2=1;i3=1;i4=1;}
        
        if(raw[0]<raw1[0])
        {
            i1++;
            if(i1>2)
            {
                raw1[0]=raw[0];i1=1;}
        }
        
        if(raw[1]<raw1[1])
        {
            i2++;
            if(i2>2){
                raw1[1]=raw[1];i2=1;}
        }
        
        if(raw[2]<raw1[2])
        {
            i3++;
            if(i3>2)
            {
                raw1[2]=raw[2];i3=1;}
        }
        
        if(raw[3]<raw1[3])
        {
            i4++;
            if(i4>2)
            {
                raw1[3]=raw[3];i4=1;}
        }
        for(k=0;k<4;k++)
        {
            if(raw[k]>raw1[k])
            {
                if((raw2[k]<(raw[k]-raw1[k])) && raw2[k]<100)
                    raw2[k]=(raw[k]-raw1[k]);
            }
        }
        
        for(k=0;k<4;k++)
        {
            if((raw[k]-raw1[k])>raw2[k])
                raw3[k]=1;
            else raw3[k]=0;
        }
        
        if(raw3[3]==1)
        {
            if(raw3[3]==1 && raw3[2]==1)
            { d=(raw[3]-raw1[3]-raw2[3])/60;
                d=d+15;
                sprintf(str,"%dmm to left",d);
                putsUart0(str);
                putsUart0("\r\n");waitMicrosecond(200000);
            }
            else
                putsUart0("22mm to left\r\n");waitMicrosecond(200000);
        }
        
        if(raw3[0]==1)
        {
            if(raw3[0]==1 && raw3[1]==1)
            { d=(raw[0]-raw1[0]-raw2[0])/60;
                d=d+15;
                sprintf(str,"%dmm to right",d);
                putsUart0(str);
                putsUart0("\r\n");waitMicrosecond(200000);
            }
            else
                putsUart0("22mm to right\r\n");waitMicrosecond(200000);
        }
        
        if(raw3[3]!=1 && raw3[2]==1 && raw3[1]!=1)
        { d=(raw[2]-raw1[2]-raw2[2])/50;
            d=22-d;
            sprintf(str,"%dmm to left",d);
            putsUart0(str);
            putsUart0("\r\n");waitMicrosecond(200000);
        }
        
        if(raw3[0]!=1 && raw3[1]==1 && raw3[2]!=1)
        { d=(raw[1]-raw1[1]-raw2[1])/50;
            d=22-d;
            sprintf(str,"%dmm to right",d);
            putsUart0(str);
            putsUart0("\r\n");waitMicrosecond(200000);
        }
        if(raw3[2]==1 && raw3[1]==1)
        {
            putsUart0("Bot is on lane\r\n");
            //waitMicrosecond(200000);
        }
        yield();
    }
}


uint32_t strlen1(char* string)
{
    int i=0;
    bool found=false;
    while(!found)
    {
        found=(string[i++]=='\0');
    }
    return i-1;
}

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

void putsUart0(char* str)
{
    int i=0;
    while(str[i]!=0)
    {
        putcUart0(str[i]);
        i++;
    }
}

char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
    {yield();}
    return UART0_DR_R & 0xFF;
}


char *getString()
{
    char c;
    int i,count=0,k;
    for(k=0;k<80;k++)
        string[k]=0;
    while(1)
    {
        c=getcUart0();
        if (c == 8)
        {
            if (count > 0)
            {
                count--;
                continue;
            }
            else
                continue;
        }
        else if (c == 13)
        {
            if(count==0)
                continue;
            else
            {
                string[count]=0;
                break;
            }
        }
        else if (c >= ' ')
        {
            string[count++]=c;
            if (count == MAX_CHARS)
            {
                string[count]=0;
                break;
                
            }
            else
                continue;
        }
        else
            continue;
    }
    
    return string;
}

void call(char string[80])
{
    
    
    int k,i,j=0;
    k=strlen(string);
    for (i=0;i<=k;i++)
    {
        if(i==0)
        {
            if((string[0]<=57 && string[0]>=48) || (string[0]<=90 && string[0]>=65) || (string[0]<=122 && string[0]>=97) || string[0]==95)
            {
                position[j]=0;
                parseCount=j+1;
                if(isalpha(string[0]))
                    type[j]='a';
                else
                    type[j]='n';
                j++;
                
            }
            else
            {
                if((string[0]==60) || (string[0]==62))
                    string[0]=NULL ;
            }
            
        }
        else
        {
            if((string[i]<=57 && string[i]>=48) || (string[i]<=90 && string[i]>=65) || (string[i]<=122 && string[i]>=97) || string[i]==95 || string[i]==38)
            {
                if((string[i-1]<=57 && string[i-1]>=48) || (string[i-1]<=90 && string[i-1]>=65) || (string[i-1]<=122 && string[i-1]>=97)  || string[i-1]==95 || string[i-1]==38)
                {
                    
                    continue;
                }
                else
                {
                    
                    position[j]=i;
                    parseCount=j+1;
                    if(isalpha(string[i]))
                        type[j]='a';
                    else
                        type[j]='n';
                    j++;
                }
            }
            else
            {
                if((string[i]==60) || (string[i]==62))
                    string[i]=NULL ;
            }
        }
        
    }
    
}

int strcmp1 ( const char * str1, const char * str2 )
{
    int i,j;
    for(i=0;i<strlen1(str1);i++)
    {
        if(str1[i]==str2[i])
            j=1;
        else j=0;
    }
    return j;
}

bool isCommand(char string[80])
{
    int z=false,i;
    if(strcmp1(&string[position[0]],"ps")  && parseCount == 1)
    {z=true;d=1;}
    if(strcmp1(&string[position[0]],"ipcs") && parseCount == 1)
    {z=true;d=2;}
    if(strcmp1(&string[position[0]],"kill") && type[1]!='a' && parseCount == 2)
    {z=true;d=3;}
    if(strcmp1(&string[position[0]],"reboot") && parseCount == 1)
    {z=true;d=4;}
    if(strcmp1(&string[position[0]],"magnet") && parseCount == 1)
    {z=true;d=7;}
    if(strcmp1(&string[position[0]],"pidof") && type[1]!='n' && parseCount == 2)
    {z=true;d=5;}
    for(i=0;i<taskCount;i++)
    {
        if(strcmp1(&string[position[0]],tcb[i].name) && strcmp1(&string[position[1]],"&") && parseCount == 2)
        {z=true;d=6;}
    }
    return z;
}



//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)

{
    bool ok;
    
    // Initialize hardware
    initHw();
    rtosInit();
    // Power-up flash
    RED_LED = 1;
    waitMicrosecond(250000);
    RED_LED = 0;
    waitMicrosecond(250000);
    
    // Initialize semaphores
    init(&keyPressed, 1);
    init(&keyReleased, 0);
    init(&flashReq, 5);
    init(&pi, 1);
    
    // Add required idle process
    ok =  createThread(idle, "Idle", 15);
    // Add other processes
    ok &= createThread(flash4Hz, "Flash_4hz", 0);
    ok &= createThread(lengthyFn, "Lengthy_fn", 10);
    ok &= createThread(oneshot, "One_shot", 4);
    ok &= createThread(readKeys, "Read_keys", 2);
    ok &= createThread(debounce, "Debounce", 6);
    ok &= createThread(uncooperative, "Uncoop", 7);
    ok &= createThread(shell, "Shell", 12);
    ok &= createThread(magnet, "Magnet", 8);
    
    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;
    return 0;
    // don't delete this unreachable code
    // if a function is only called once in your code, it will be
    // accessed with two goto instructions instead of call-return,
    // so any stack-based code will not function correctly
    yield(); sleep(0); wait(0); post(0);
}
