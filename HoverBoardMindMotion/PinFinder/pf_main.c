//PinFinder start: config, power latch, uart (searched by loop-back if not configured), then the front end:
//Enter starts the TUI, a line starting with '@' agent mode (sticky; "tui" goes back).
#include "pf.h"
#include "main.h"
#include "initialize.h"
#include "sim_eeprom.h"
#include "uart.h"
#include "hal_rcc.h"

//state the shared modules use (main.h, pinout.h)
volatile uint32_t millis;
uint8_t uarten = 1;
uint16_t pinstorage[128];    //CfgDefaults() first (an initializer here would be a second 256 byte copy)

static char line[64];
static uint8_t len;

static void agentByte(int c){
	if(c=='@'){ len = 0; return; }    //a command starts here: line noise before it (a reconnect, a break) is dropped
	if(c=='\r'||c=='\n'){
		line[len] = 0;
		char* s = line;
		while(*s=='@'||*s==' ') s++;
		len = 0;
		if(*s) Command(s);
	}else if(c=='\b'||c==127){
		if(len) len--;
	}else if(c>=' '&&len<sizeof(line)-1){
		line[len++] = (char)c;
	}
}

int main(void){
	SysTick_Config(SystemCoreClock/1000);
	NVIC_SetPriority(SysTick_IRQn, 0);
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA|RCC_AHBENR_GPIOB|RCC_AHBENR_GPIOC|RCC_AHBENR_GPIOD, ENABLE);
	CfgDefaults();    //the extension words stay default if the board has no extension record
	if(ConfigLoad()) ConfigMigrate();
	CfgMarkSaved();
#ifdef PF_FIXED_CAL
	//This target is for the measured Ghost board only. Keep the values in RAM; the Flash config
	//is intentionally not changed by calibration firmware.
	if(ITOTALPIN==PA7&&IPHASEAPIN==PA2&&IPHASEBPIN==PA6){
		if(IPHASE_CFG==0xFFFF) IPHASE_CFG = 1;       //PA2=B, PA6=C, both falling with positive current
		if(IPHASE_GAIN==0xFFFF) IPHASE_GAIN = 354;   //R004 fit, median of three measurements
	}
#endif
	LatchHold();    //first: the user may be holding the power button
	MotorInit();
	AdcInit();
#if defined(PF_HALL_ONLY) || defined(PF_FIXED_CAL)
	if(TXPIN>=PINCOUNT||RXPIN>=PINCOUNT) for(;;);    //fixed config required; no UART pin search
#else
	if(TXPIN>=PINCOUNT||RXPIN>=PINCOUNT){
		while(!UartFind());    //the user connects the board's TX and RX together
	}
#endif
	uartUsed = UART_GPIO_Init();
	for(uint8_t i=0;i<UARTCOUNT;i++){    //push-pull: one terminal on the line, not the RemoteUartBus
		if(uarts[i].io==TXPIN&&uarts[i].tx) pinMode(TXPIN, OUTPUT_AF);
	}
	uarten = uartUsed;
	UARTX_Init(PF_BAUD, uartUsed);
	SerialInit();
	IoInit(uartUsed);

#if defined(PF_HALL_ONLY) || defined(PF_FIXED_CAL)
	agent = 1;
	#ifdef PF_FIXED_CAL
	out("Fixed motor calibration: @detect current [200..500 phase, 600 or 1400 gain], @calibrate hall 1500|2000|3000|4000|5000, @calibrate rl 1500");
	#else
	out("Fixed Hall mapping: @calibrate hallmap [mV], @hall, @get <name>");
	#endif
	nl();
	while(1){
		int c = inbyte();
		if(c>=0) agentByte(c);
	}
#else
	uint8_t started = 0;
	uint32_t banner = millis-2000;
	while(1){
		int c = inbyte();
		if(!started){
			if(millis-banner>=2000){
				banner = millis;
				out("PinFinder " PF_VERSION ": Enter = menu, @<command> = agent mode (@help)");
				nl();
			}
			if(c=='@'){ started = 1; agent = 1; len = 0; }
			else if(c=='\r'||c=='\n'){ started = 1; agent = 0; TuiStart(); }
			continue;
		}
		if(c>=0){
			if(agent) agentByte(c);
			else if(c=='@'&&!tuiLine){ agent = 1; len = 0; nl(); }
			else TuiKey(c);
		}
		if(!agent) TuiPoll();
	}
#endif
}
