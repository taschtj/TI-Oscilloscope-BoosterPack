/*
 * MainTestV2.c
 *
 *  Created on: Dec 14, 2014
 *      Author: Ruoyu Zhuang and Thomas Tasch
 *
 * This is the main C file for the oscilloscope BoosterPack program to be loaded
 * onto a TM4C1294XL LaunchPad. This file both creates the touch screen user
 * interface to be used with a BOOSTXL-K350QVG-S1 QVGA Display BoosterPack and
 * the data acquisiton programming needed to capture the ditial signals from
 * an ADS4222 Analog to Digital Converter.
 *
 * Special Thanks is given to the TI employees who wrote "Creating IoT Solutions
 * with the Tiva® C Series Connected LaunchPad Workshop" and the "TivaWare™
 * Peripheral Driver Library User's Guide". The examples and documentation within
 * these books greatly helped with our design process.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"
#include "grlib/pushbutton.h"
#include "grlib/container.h"
#include "grlib/radiobutton.h"
#include "grlib/slider.h"
#include "Kentec320x240x16_ssd2119_8bit.h"
#include "touch.h"
#include <math.h>
#include "driverlib/fpu.h"
#include "driverlib/debug.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "inc/hw_nvic.h"
#include "driverlib/flash.h"
#include "driverlib/udma.h"
#include "utils/ustdlib.h"
#include "inc/hw_epi.h"
#include "driverlib/epi.h"
#include "driverlib/timer.h"

// define epi port pins to be used
#define EPI_PORTA_PINS (GPIO_PIN_6 | GPIO_PIN_7)
#define EPI_PORTB_PINS (GPIO_PIN_2 | GPIO_PIN_3)
#define EPI_PORTC_PINS (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define EPI_PORTG_PINS (GPIO_PIN_0)
#define EPI_PORTK_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7)
#define EPI_PORTL_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4)
#define EPI_PORTM_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)


// define ADC control pins
#define PADC_Output_Mode (GPIO_PIN_2)
#define PADC_CTRL_1 (GPIO_PIN_5)
#define FADC_CTRL_2 (GPIO_PIN_2)
#define FADC_CTRL_3 (GPIO_PIN_3)
#define FADC_CLK_OUT (GPIO_PIN_1)
#define LADC_CLK_IN (GPIO_PIN_5)

// define gpio pins for DVGA
#define ECh1_DVGA_Mode 	(GPIO_PIN_4)
#define ECh2_DVGA_Mode 	(GPIO_PIN_5)
#define ECh1_DVGA_D0	(GPIO_PIN_1)
#define DCh1_DVGA_D1	(GPIO_PIN_3)
#define ECh1_DVGA_D2	(GPIO_PIN_3)
#define MCh1_DVGA_D3	(GPIO_PIN_4)
#define ECh2_DVGA_D0	(GPIO_PIN_2)
#define ECh2_DVGA_D1	(GPIO_PIN_0)
#define DCh2_DVGA_D2	(GPIO_PIN_7)
#define MCh2_DVGA_D3	(GPIO_PIN_5)

// define gpio pins for multiplexer
#define BCh1_Mult_A0	(GPIO_PIN_4)
#define BCh1_Mult_A1	(GPIO_PIN_5)
#define ACh2_Mult_A0	(GPIO_PIN_4)
#define ACh2_Mult_A1	(GPIO_PIN_5)


// The size of the memory buffer used for the DMA and the Maximum Size
// of the circular buffer used to hold of the data
#define MEM_BUFFER_SIZE         1024
#define MaxSize					1024*20 // Must be multiple of MEM_BUFFER_SIZE

// Number of samples to determine the average frequency measured and
// number of time instances to measure over for the frequency
#define MeasureAvg				10
#define TimeAvg					320


// The destination buffers used for memory transfers.
static uint32_t *g_ui32DstBuf[MEM_BUFFER_SIZE];
static uint32_t *g_ui32DstBuf2[MEM_BUFFER_SIZE];

// The 12-bit values from the ADC channels
uint16_t values[MaxSize], values2[MaxSize];

// The raw inputs from the EPI
uint32_t inputs[MEM_BUFFER_SIZE], inputs2[MEM_BUFFER_SIZE];


// uDMA controller control table
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif
////////////////////////////////////////////////////////

// Clock frequency
uint32_t ui32SysClkFreq;

// pixel length of screen
#define SERIES_LENGTH 319

//
//Intro pictures
extern const uint8_t g_pui8Image[];
extern const uint8_t g_pui9Image[];

// global variables
uint32_t totalsA[SERIES_LENGTH], totalsB[SERIES_LENGTH]; // sum of values when using acquring mode
uint16_t totalA, totalB; // calculated value of 12-bit inputs from ADC
uint16_t l1 = 0, l2 = 0; //variables to keep track of which delta time is measured
uint16_t measnum = 0; //variable to keep track of how many frequency measurements have been made
uint32_t i = 0, j = 0, f = 0, k = 0, m = 0; // various variables to keep track of which index an array is at
uint32_t EPIDivide = 5; // The clock frequency divider used to determine how fast the EPI should clock at
uint16_t freqref1 = 0, freqref2 = 0; // signal value that frequency calculation is based on
uint16_t t1[TimeAvg], t2[TimeAvg]; // array of delta times to calculate frequency
uint8_t freqstart2 = 0, freqstop2 = 0, freqstart1 = 0, freqstop1 = 0; // variables to help determine when to start and stopcounting delta time instances
float t2Avg, t1Avg; // average number of delta time instances
uint32_t totalt1 = 0, totalt2 = 0; // sum of number of delta time instances
uint16_t NumAvgt1 = 0, NumAvgt2 = 0; // number of measured delta time instances greater than 0
uint32_t Frequency1 = 0, Frequency2; // calculated average frequency
uint64_t Frequency1Total = 0, Frequency2Total = 0; // sum of all calculated frequencies
uint8_t NumFreqs1 = 0, NumFreqs2 = 0; // number of measured frequencies greater than 0
uint16_t Amp1[4], Amp2[4]; // Amplitude information for signals (0-Min,1-Max,2-Amplitude in pixels,3-Amplitude in mV)
uint16_t NumAvg = 10; // Number of sets to average when using averaging acquire mode
uint16_t *PTriggerLevel, TriggerLevel = 1000; // Trigger level of signal in pixels and pointer for it
uint32_t Freq1[MeasureAvg], Freq2[MeasureAvg]; // calcuated frequencies
uint32_t receive[24]; // array of all bits received from ADC
uint32_t CountSize = 1024; // length of count size for non blocking EPI read assignment
uint8_t pri, alt; // variables to set when primary or alternate DMA transfers are complete
uint8_t TriggerStart = 0, Trigger = 0; // to set when to start triggering and to set if a trigger was found within an iteration
uint8_t GoThrough = 0; // varialbe to set if no trigger is found when going through iteration
uint8_t CaptureMode = 0, TriggerMode = 0; // variables that determine the acquire (0-normal,1-average) and trigger (0-positive edge,1-negative-edge) modes for the scope
uint8_t begin = 0; // variable to help determine when the values before the trigger position have already been found
uint8_t TriggerSource = 1; // determines which signal to trigger off of (1-source 1,2-source 2)
uint16_t NumSkip = 2; // sets the number of values to skip over in order to achieve correct time scale
uint16_t TriggerPosition = 0; // pixel number to start drawing after trigger is found
uint16_t old1[SERIES_LENGTH], old2[SERIES_LENGTH], pixels[SERIES_LENGTH], pixels2[SERIES_LENGTH]; // pixel heights of both current and last signal
uint16_t midlevel1, midlevel2; // 0V level for both channels 1 and 2 in pixels calibrated to pixel_divider
uint16_t *plevel1, *plevel2, level1 = 80, level2 = 160; // 0V level for both channels 1 and 2 in pixels
uint16_t desiredlevel1 = 80, desiredlevel2 = 160; // Desired 0V level for both channels 1 and 2 in pixels
uint32_t EPIMode; // mode for the EPI interrupt
int *EPISource; // pointer to starting address of EPI input values
uint8_t stop = 0; // variable to determine whether the signal should keep updating or not
uint8_t Ch1on = 1, Ch2on = 1, Ch1off = 0, Ch2off = 0; // variables to keep track of which signals should be displayed
uint8_t Mag1 = 0, Mag2 = 0; // variables that keep track of which vertical scale division is being used
uint8_t Time = 6; // variable that keeps track of which horiztonal scale divison is being used
uint8_t minusbelow1 = 0, minusbelow2 = 0, minusabove1 = 0, minusabove2 = 0; // variables that keep track of whether the previous pixel was out of bounds (in pushbutton region)
uint8_t outbelow1 = 0, outbelow2 = 0, outabove1 = 0, outabove2 = 0; // variables that keep track of whether the current pixel is out of bounds (in pushbutton region)
uint8_t transfer_done[2] = {0,0}; // keeps track of when the primary or alternate DMA transfer has completed
uint8_t stopped = 0; // keeps track of whether the signal was stopped from updating
float pixel_divider1 = 5.461, pixel_divider2 = 5.461; // values describing how much the 12-bit ADC input should be divided by to obtain correct vertical scale divisions
float mvpixel[14], secpixel[29]; // array of mV/pixel and seconds/pixel for every scale division
char MagDisplay1[7], MagDisplay2[7]; // string of ASCII characters that display the peak-to-peak voltage of both signals
char FreqDisplay1[9], FreqDisplay2[9]; // string of ASCII characters that display the frequencies of both signals

//Define Widgets
tContext sContext;
tRectangle sRect;
extern tCanvasWidget g_sBackground;
extern tCanvasWidget g_sWaveform;
extern tCanvasWidget g_sTop;
extern tCanvasWidget g_sBottom;
extern tCanvasWidget g_sAddMinusC1;
extern tCanvasWidget g_sAddMinusC2;
extern tCanvasWidget g_sAddMinusTime;
extern tPushButtonWidget g_sPushBtnAddC1;
extern tPushButtonWidget g_sPushBtnMinusC1;
extern tPushButtonWidget g_sPushBtnAddC2;
extern tPushButtonWidget g_sPushBtnMinusC2;
extern tPushButtonWidget g_sPushBtnAddTime;
extern tPushButtonWidget g_sPushBtnMinusTime;
extern tContainerWidget g_sContainerAcquire;
extern tContainerWidget g_sContainerChannels;
extern tContainerWidget g_sContainerTriggers;
extern tContainerWidget g_sContainerTriggerSource;
extern tContainerWidget g_sContainerTriggerMode;
extern tContainerWidget g_sContainerFreMagnitudeC1;
extern tContainerWidget g_sContainerFreMagnitudeC2;
extern tContainerWidget g_sContainerVolMagnitudeC1;
extern tContainerWidget g_sContainerVolMagnitudeC2;
extern tContainerWidget  g_sContainerMath;
extern tSliderWidget g_sTriggerSliderVertical;
extern tSliderWidget g_sTriggerSliderHorizontal;
extern tSliderWidget g_sC1Slider;
extern tSliderWidget g_sC2Slider;

//Used for grid
int x;
int y;
///////////////

//Used for scale display//////
char *tempMagVolDivC2;
char magVolDivC2[] = { 32, 50, 48, 109, 86, 47, 100, 105, 118, 0 };
char *tempMagVolDivC1;
char magVolDivC1[] = { 32, 50, 48, 109, 86, 47, 100, 105, 118, 0 };
char *tempTimVolDivC1;
char timVolDivC1[] = { 32, 32, 50, 117, 115, 47, 100, 105, 118, 0 };
//////////////////////////////
void ClrScreen(void);
void DRadioAcquire(tWidget *pWidgetR);
void DRadioChannels(tWidget *pWidgetR);
void TriggerSelectRadioBtns(tWidget *psWidget, uint32_t bSelected);
void ChannelSelectRadioBtns(tWidget *psWidget, uint32_t bSelected);
void DRadioFreMagnitudeC1(tWidget *pWidgetR);
void DRadioFreMagnitudeC2(tWidget *pWidgetR);
void DRadioVolMagnitudeC1(tWidget *pWidgetR);
void DRadioVolMagnitudeC2(tWidget *pWidgetR);
void DWaveForm(tWidget *pWidgetR, tContext *psContext);
void AddMinusFunctionC1(tWidget *pWidget);
void AddMinusFunctionC2(tWidget *pWidget);
void AddMinusFunctionTime(tWidget *pWidget);
void AddMagDivC1(tWidget *psWidget);
void MinusMagDivC1(tWidget *psWidget);
void AddMagDivC2(tWidget *psWidget);
void MinusMagDivC2(tWidget *psWidget);
void AddTimeDiv(tWidget *psWidget);
void MinusTimeDiv(tWidget *psWidget);
void ClrMyWidget();
void TriggerFunction(tWidget *pWidget);
void OnSliderChangeVertical(tWidget *psWidget, int32_t i32Value);
void OnSliderChangeHorizontal(tWidget *psWidget, int32_t i32Value);
void OnSliderChangeC1(tWidget *psWidget, int32_t i32Value);
void OnSliderChangeC2(tWidget *psWidget, int32_t i32Value);
void RunStop(tWidget *psWidget);
void UpdateMeasurements(void);
void CalibrateOffset(void);
void OffSet(tWidget *psWidget);
void AcquireSelectRadioBtns(tWidget *psWidget, uint32_t bSelected);
void MathSelectRadioBtns(tWidget *psWidget, uint32_t bSelected);
void TriggerModeSelect(tWidget *psWidget, uint32_t bSelected);
void TriggerSourceSelect(tWidget *psWidget, uint32_t bSelected);
tPushButtonWidget g_psTopButtons[];
tPushButtonWidget g_psBotButtons[];
void setup(void);
void SetupVoltageDivision(uint8_t Scale, uint8_t Channel);
void SetupTimeDivision(uint8_t Scale);
void SetupTrigger(uint16_t Level, uint8_t Start_Position, uint8_t Mode, uint8_t Source);
void PixelsCalculation(uint32_t input[MEM_BUFFER_SIZE]);



Canvas(g_sBackground, WIDGET_ROOT, 0, &g_sTop, &g_sKentec320x240x16_SSD2119, 0,
		0, 320, 240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);

Canvas(g_sWaveform, WIDGET_ROOT, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 29, 320,
		182, CANVAS_STYLE_APP_DRAWN, ClrBlack, 0, 0, 0, 0, 0, DWaveForm);

Canvas(g_sTop, &g_sBackground, &g_sBottom, g_psTopButtons,
		&g_sKentec320x240x16_SSD2119, 0, 0, 320, 28, CANVAS_STYLE_FILL,
		ClrBlack, 0, 0, 0, 0, 0, 0);
Canvas(g_sBottom, &g_sBackground, 0, g_psBotButtons,
		&g_sKentec320x240x16_SSD2119, 0, 212, 320, 28, CANVAS_STYLE_FILL,
		ClrBlack, 0, 0, 0, 0, 0, 0);

Canvas(g_sAddMinusC1, 0, 0, &g_sPushBtnAddC1, &g_sKentec320x240x16_SSD2119, 0,
		29, 52, 52,
		CANVAS_STYLE_FILL|CANVAS_STYLE_OUTLINE|CANVAS_STYLE_TEXT_VCENTER|CANVAS_STYLE_TEXT,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, magVolDivC1, 0, 0);

Canvas(g_sAddMinusC2, 0, 0, &g_sPushBtnAddC2, &g_sKentec320x240x16_SSD2119, 53,
		29, 52, 52,
		CANVAS_STYLE_FILL|CANVAS_STYLE_OUTLINE|CANVAS_STYLE_TEXT_VCENTER|CANVAS_STYLE_TEXT,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, magVolDivC2, 0, 0);
Canvas(g_sAddMinusTime, 0, 0, &g_sPushBtnAddTime, &g_sKentec320x240x16_SSD2119, 106,
		29, 52, 52,
		CANVAS_STYLE_FILL|CANVAS_STYLE_OUTLINE|CANVAS_STYLE_TEXT_VCENTER|CANVAS_STYLE_TEXT,
		ClrBlack, ClrWhite, ClrWhite, g_psFontCmss12, timVolDivC1, 0, 0);

///Top Buttons///////////////////////////////////////////////////////
tPushButtonWidget g_psTopButtons[] =
		{
								RectangularButtonStruct(&g_sTop,
										g_psTopButtons + 1, 0,
										&g_sKentec320x240x16_SSD2119, 0, 0, 52,
										28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrRed,
										g_psFontCmss12, "20mV/div", 0, 0, 0, 0,
										AddMinusFunctionC1),

								RectangularButtonStruct(&g_sTop,
										g_psTopButtons + 2, 0,
										&g_sKentec320x240x16_SSD2119, 53, 0, 52,
										28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrYellow,
										g_psFontCmss12, "20mV/div", 0, 0, 0, 0,
										AddMinusFunctionC2),

								RectangularButtonStruct(&g_sTop,
										g_psTopButtons + 3, 0,
										&g_sKentec320x240x16_SSD2119, 106, 0,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCmss12, "2us/div", 0, 0, 0, 0, AddMinusFunctionTime),

								RectangularButtonStruct(&g_sTop,
										g_psTopButtons + 4, 0,
										&g_sKentec320x240x16_SSD2119, 159, 0,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCm16, "Trigger", 0, 0, 0, 0, TriggerFunction),

								RectangularButtonStruct(&g_sTop,
										g_psTopButtons + 5, 0,
										&g_sKentec320x240x16_SSD2119, 212, 0,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCm12, "Channels", 0, 0, 0, 0, DRadioChannels),

								RectangularButtonStruct(&g_sTop, 0, 0,
										&g_sKentec320x240x16_SSD2119, 265, 0,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCm16, "Acquire", 0, 0, 0, 0,
										DRadioAcquire) };
////////////////////////////////////////////////////////////////////////////////////////////
////Magnitude Division Buttons//////////////////////////////////////////////////////////////
RectangularButton(g_sPushBtnAddC1, &g_sAddMinusC1, &g_sPushBtnMinusC1, 0,
		&g_sKentec320x240x16_SSD2119, 1, 30, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrRed, g_psFontCm16, "+", 0, 0, 0, 0,
		AddMagDivC1);

RectangularButton(g_sPushBtnMinusC1, &g_sAddMinusC1, 0, 0,
		&g_sKentec320x240x16_SSD2119, 1, 61, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrRed, g_psFontCm16, "-", 0, 0, 0, 0,
		MinusMagDivC1);

RectangularButton(g_sPushBtnAddC2, &g_sAddMinusC2, &g_sPushBtnMinusC2, 0,
		&g_sKentec320x240x16_SSD2119, 54, 30, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrYellow, g_psFontCm16, "+", 0, 0, 0, 0,
		AddMagDivC2);

RectangularButton(g_sPushBtnMinusC2, &g_sAddMinusC2, 0, 0,
		&g_sKentec320x240x16_SSD2119, 54, 61, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrYellow, g_psFontCm16, "-", 0, 0, 0, 0,
		MinusMagDivC2);
/////////////////////////////////////////////////////////////////////////////////////////
///Time Division Buttons ////////////////////////////////////////////////////////////////
RectangularButton(g_sPushBtnAddTime, &g_sAddMinusTime, &g_sPushBtnMinusTime, 0,
		&g_sKentec320x240x16_SSD2119, 106, 30, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrWhite, g_psFontCm16, "+", 0, 0, 0, 0,
		AddTimeDiv);

RectangularButton(g_sPushBtnMinusTime, &g_sAddMinusTime, 0, 0,
		&g_sKentec320x240x16_SSD2119, 106, 61, 50, 18,
		(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
		ClrGray, ClrWhite, ClrWhite, ClrWhite, g_psFontCm16, "-", 0, 0, 0, 0,
		MinusTimeDiv);


////Sliders for trigger/////////////////////////////////////////////////////////////////
Slider(g_sTriggerSliderVertical,0, 0, 0, &g_sKentec320x240x16_SSD2119, 310, 29, 10, 183, 29, 211, 29,
                ( SL_STYLE_BACKG_FILL|SL_STYLE_FILL|SL_STYLE_OUTLINE | SL_STYLE_VERTICAL),
                ClrWhite, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                &g_sFontCm20, 0, 0, 0, OnSliderChangeVertical);
Slider(g_sTriggerSliderHorizontal,0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 202, 310, 10, 0, 300, 120,
                ( SL_STYLE_BACKG_FILL|SL_STYLE_FILL|SL_STYLE_OUTLINE),
                ClrWhite, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                &g_sFontCm20, 0, 0, 0, OnSliderChangeHorizontal);

////Sliders for channel 1 and 2 ////////////////////////////////////////////////////////
Slider(g_sC1Slider,0, 0, 0, &g_sKentec320x240x16_SSD2119, 300, 29, 20, 183, 29, 211, 29,
                ( SL_STYLE_BACKG_FILL|SL_STYLE_FILL|SL_STYLE_OUTLINE | SL_STYLE_VERTICAL),
                ClrWhite, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                &g_sFontCm20, 0, 0, 0, OnSliderChangeC1);
Slider(g_sC2Slider,0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 29, 20, 183, 29, 211, 29,
                ( SL_STYLE_BACKG_FILL|SL_STYLE_FILL|SL_STYLE_OUTLINE|SL_STYLE_VERTICAL),
                ClrWhite, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                &g_sFontCm20, 0, 0, 0, OnSliderChangeC2);


///////////////////////////////////////////////////////////////////////////
///Bottom Buttons//////////////////////////////////////////////////////////
tPushButtonWidget g_psBotButtons[] =
		{
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 1, 0,
										&g_sKentec320x240x16_SSD2119, 0, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrRed,
										g_psFontCm12, ("Hz"), 0, 0, 0, 0,
										DRadioFreMagnitudeC1),
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 2, 0,
										&g_sKentec320x240x16_SSD2119, 53, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrYellow,
										g_psFontCm12, "Hz", 0, 0, 0, 0, DRadioFreMagnitudeC2),
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 3, 0,
										&g_sKentec320x240x16_SSD2119, 106, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrRed,
										g_psFontCm14, "V", 0, 0, 0, 0, DRadioVolMagnitudeC1),
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 4, 0,
										&g_sKentec320x240x16_SSD2119, 159, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrYellow,
										g_psFontCm14, "V", 0, 0, 0, 0, DRadioVolMagnitudeC2),
								RectangularButtonStruct(&g_sBottom,
										g_psBotButtons + 5, 0,
										&g_sKentec320x240x16_SSD2119, 212, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCm16, "Offset", 0, 0, 0, 0, OffSet),
								RectangularButtonStruct(&g_sBottom, 0, 0,
										&g_sKentec320x240x16_SSD2119, 265, 212,
										52, 28,
										(PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT | PB_STYLE_FILL),
										ClrGray, ClrWhite, ClrWhite, ClrWhite,
										g_psFontCmss14, "Run/Stop", 0, 0, 0, 0,
										RunStop) };
//Radio Buttons for the trigger button///////////////////////////////////////

tRadioButtonWidget g_psRadioBtnTriggers[] = {
RadioButtonStruct(&g_sContainerTriggers, g_psRadioBtnTriggers + 1, 0,
		&g_sKentec320x240x16_SSD2119, 159, 30, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrWhite, g_psFontCmss14, "Source", 0,
		TriggerSelectRadioBtns),
RadioButtonStruct(&g_sContainerTriggers, g_psRadioBtnTriggers + 2, 0,
		&g_sKentec320x240x16_SSD2119, 159, 51, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrWhite, g_psFontCmss14, "Level", 0,
		TriggerSelectRadioBtns),
RadioButtonStruct(&g_sContainerTriggers, g_psRadioBtnTriggers + 3, 0,
		&g_sKentec320x240x16_SSD2119, 159, 72, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrWhite, g_psFontCmss14, "Position", 0, TriggerSelectRadioBtns),
RadioButtonStruct(&g_sContainerTriggers, 0, 0,
				&g_sKentec320x240x16_SSD2119, 159, 93, 48, 20, RB_STYLE_TEXT,
				10, ClrBlack, ClrWhite, ClrWhite, g_psFontCmss14, "Mode", 0, TriggerSelectRadioBtns)};
#define NUM_RADIO_BUTTONS_Triggers      (sizeof(g_psRadioBtnTriggers) /   \
                                 sizeof(g_psRadioBtnTriggers[0]))

tRadioButtonWidget g_psRadioBtnSource[] = {
RadioButtonStruct(&g_sContainerTriggerSource, g_psRadioBtnSource + 1, 0,
		&g_sKentec320x240x16_SSD2119, 212, 30, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrRed, g_psFontCmss14, "  1", 0,
		TriggerSourceSelect),
RadioButtonStruct(&g_sContainerTriggerSource, 0, 0,
		&g_sKentec320x240x16_SSD2119, 212, 51, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrYellow, g_psFontCmss14, "  2", 0,
		TriggerSourceSelect)};
#define NUM_RADIO_BUTTONS_Sources      (sizeof(g_psRadioBtnSource) /   \
                                 sizeof(g_psRadioBtnSource[0]))
Container(g_sContainerTriggerSource, 0, 0, g_psRadioBtnSource,
		&g_sKentec320x240x16_SSD2119, 212, 28, 52, 45,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);

tRadioButtonWidget g_psRadioBtnTriggerMode[] = {
RadioButtonStruct(&g_sContainerTriggerMode, g_psRadioBtnTriggerMode + 1, 0,
		&g_sKentec320x240x16_SSD2119, 212, 81, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrRed, g_psFontCmss14, "Pedge", 0,
		TriggerModeSelect),
RadioButtonStruct(&g_sContainerTriggerMode, 0, 0,
		&g_sKentec320x240x16_SSD2119, 212, 102, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrYellow, g_psFontCmss14, "Nedge", 0,
		TriggerModeSelect)};
#define NUM_RADIO_BUTTONS_TriggerMode     (sizeof(g_psRadioBtnTriggerMode) /   \
                                 sizeof(g_psRadioBtnTriggerMode[0]))

Container(g_sContainerTriggerMode, 0, 0, g_psRadioBtnTriggerMode,
		&g_sKentec320x240x16_SSD2119, 212, 80, 52, 45,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);
///////////////////////////////////////////////////////////////

tRadioButtonWidget g_psRadioBtnChannels[] = {
RadioButtonStruct(&g_sContainerChannels, g_psRadioBtnChannels + 1, 0,
		&g_sKentec320x240x16_SSD2119, 212, 30, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrRed, g_psFontCmss14, "  1", 0,
		ChannelSelectRadioBtns),
RadioButtonStruct(&g_sContainerChannels, g_psRadioBtnChannels + 2, 0,
		&g_sKentec320x240x16_SSD2119, 212, 51, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrYellow, g_psFontCmss14, "  2", 0,
		ChannelSelectRadioBtns),
RadioButtonStruct(&g_sContainerChannels, 0, 0,
		&g_sKentec320x240x16_SSD2119, 212, 71, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrRed, g_psFontCmss14, "1 & 2", 0, ChannelSelectRadioBtns)};
#define NUM_RADIO_BUTTONS_Channels      (sizeof(g_psRadioBtnChannels) /   \
                                 sizeof(g_psRadioBtnChannels[0]))

tRadioButtonWidget g_psRadioBtnAcquire[] = {
RadioButtonStruct(&g_sContainerAcquire, g_psRadioBtnAcquire + 1, 0,
		&g_sKentec320x240x16_SSD2119, 266, 40, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Normal", 0,
		AcquireSelectRadioBtns),
RadioButtonStruct(&g_sContainerAcquire,  0, 0,
		&g_sKentec320x240x16_SSD2119, 266, 61, 48, 20, RB_STYLE_TEXT,
		10, ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Average", 0,
		AcquireSelectRadioBtns)};
#define NUM_RADIO_BUTTONS_Acquire      (sizeof(g_psRadioBtnAcquire) /   \
                                 sizeof(g_psRadioBtnAcquire[0]))

////Radio Buttons for Hz and V Pushbuttons////////////////////////////////////
tRadioButtonWidget g_psRadioBtnFreqMagC1[] = {
RadioButtonStruct(&g_sContainerFreMagnitudeC1, g_psRadioBtnFreqMagC1 + 1, 0,
		&g_sKentec320x240x16_SSD2119, 1, 142, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Maximum", 0, 0),
RadioButtonStruct(&g_sContainerFreMagnitudeC1, g_psRadioBtnFreqMagC1+ 2, 0,
		&g_sKentec320x240x16_SSD2119, 1, 163, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Minimum", 0, 0),
RadioButtonStruct(&g_sContainerFreMagnitudeC1, 0, 0,
		&g_sKentec320x240x16_SSD2119, 1, 184, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Average", 0, 0)

};
tRadioButtonWidget g_psRadioBtnFreqMagC2[] = {
RadioButtonStruct(&g_sContainerFreMagnitudeC2, g_psRadioBtnFreqMagC2 + 1, 0,
		&g_sKentec320x240x16_SSD2119, 54, 142, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Maximum", 0, 0),
RadioButtonStruct(&g_sContainerFreMagnitudeC2, g_psRadioBtnFreqMagC2+ 2, 0,
		&g_sKentec320x240x16_SSD2119, 54, 163, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Minimum", 0, 0),
RadioButtonStruct(&g_sContainerFreMagnitudeC2, 0, 0,
		&g_sKentec320x240x16_SSD2119, 54, 184, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Average", 0, 0)

};
tRadioButtonWidget g_psRadioBtnVolMagC1[] = {
RadioButtonStruct(&g_sContainerVolMagnitudeC1, g_psRadioBtnVolMagC1 + 1, 0,
		&g_sKentec320x240x16_SSD2119, 107, 142, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Maximum", 0, 0),
RadioButtonStruct(&g_sContainerVolMagnitudeC1, g_psRadioBtnVolMagC1+ 2, 0,
		&g_sKentec320x240x16_SSD2119, 107, 163, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Minimum", 0, 0),
RadioButtonStruct(&g_sContainerVolMagnitudeC1, 0, 0,
		&g_sKentec320x240x16_SSD2119, 107, 184, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrRed, g_psFontCmss12, "Average", 0, 0)

};
tRadioButtonWidget g_psRadioBtnVolMagC2[] = {
RadioButtonStruct(&g_sContainerVolMagnitudeC2, g_psRadioBtnVolMagC2 + 1, 0,
		&g_sKentec320x240x16_SSD2119, 160, 142, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Maximum", 0, 0),
RadioButtonStruct(&g_sContainerVolMagnitudeC2, g_psRadioBtnVolMagC2+ 2, 0,
		&g_sKentec320x240x16_SSD2119, 160, 163, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Minimum", 0, 0),
RadioButtonStruct(&g_sContainerVolMagnitudeC2, 0, 0,
		&g_sKentec320x240x16_SSD2119, 160, 184, 48, 20, RB_STYLE_TEXT, 10,
		ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "Average", 0, 0)

};
tRadioButtonWidget g_psRadioBtnMath[] = {
		RadioButtonStruct(&g_sContainerMath, g_psRadioBtnMath+ 1, 0,
				&g_sKentec320x240x16_SSD2119, 212, 29, 48, 20, RB_STYLE_TEXT, 10,
				ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "+", 0, MathSelectRadioBtns),
		RadioButtonStruct(&g_sContainerMath, 0, 0,
				&g_sKentec320x240x16_SSD2119, 212, 49, 48, 20, RB_STYLE_TEXT, 10,
				ClrBlack, ClrWhite, ClrYellow, g_psFontCmss12, "-", 0, MathSelectRadioBtns)
};
#define NUM_RADIO_BUTTONS_Math      (sizeof(g_psRadioBtnMath) /   \
                                 sizeof(g_psRadioBtnMath[0]))
//////////////////////////////////////////////////////////////////////////////////
Container(g_sContainerMath, 0, 0, g_psRadioBtnMath,
		&g_sKentec320x240x16_SSD2119, 212, 29, 52, 40,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrYellow,
		g_psFontCm14, 0);

Container(g_sContainerTriggers, 0, 0, g_psRadioBtnTriggers,
		&g_sKentec320x240x16_SSD2119, 159, 28, 52, 85,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);


Container(g_sContainerChannels, 0, 0, g_psRadioBtnChannels,
		&g_sKentec320x240x16_SSD2119, 212, 28, 52, 75,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);

Container(g_sContainerAcquire, 0, 0, g_psRadioBtnAcquire,
		&g_sKentec320x240x16_SSD2119, 265, 28, 52, 60,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);
Container(g_sContainerFreMagnitudeC1, 0, 0, g_psRadioBtnFreqMagC1,
		&g_sKentec320x240x16_SSD2119, 0, 141, 52, 70,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);
Container(g_sContainerFreMagnitudeC2, 0, 0, g_psRadioBtnFreqMagC2,
		&g_sKentec320x240x16_SSD2119, 52, 141, 52, 70,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrYellow,
		g_psFontCm14, 0);
Container(g_sContainerVolMagnitudeC1, 0, 0, g_psRadioBtnVolMagC1,
		&g_sKentec320x240x16_SSD2119, 106, 141, 52, 70,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrRed,
		g_psFontCm14, 0);
Container(g_sContainerVolMagnitudeC2, 0, 0, g_psRadioBtnVolMagC2,
		&g_sKentec320x240x16_SSD2119, 159, 141, 52, 70,
		(CTR_STYLE_OUTLINE |CTR_STYLE_FILL ), ClrBlack, ClrWhite, ClrYellow,
		g_psFontCm14, 0);

#define NUM_RADIO1_BUTTONS      (sizeof(g_psRadioBtnVolDiv0) /   \
                                 sizeof(g_psRadioBtnVolDiv0[0]))
////Add function for magnitude division for channel 1/////////////
void AddMagDivC1(tWidget *psWidget) {

	// make sure the variable keeping track of voltage division doesn't go out of bounds
	if(Mag1 == 10)
		Mag1 = 10;
	else
		Mag1++;

	// setup the new voltage divison
	SetupVoltageDivision(Mag1, 1);

	// determine new midlevel with new pixel_divider
	midlevel1 = 2048/pixel_divider1 + level1;
	midlevel2 = 2048/pixel_divider2 + level2;

	//make it 25V
	if (magVolDivC1[1] == 50 && magVolDivC1[2] == 48 && magVolDivC1[3] == 32)
		magVolDivC1[2] = 53;
	//stay at 25V
	else if (magVolDivC1[1] == 50 && magVolDivC1[2] == 53
			&& magVolDivC1[3] == 32)
		magVolDivC1[2] = 53;
	// mV to V at 1 V
	else if (magVolDivC1[0] == 53) {
		tempMagVolDivC1 = magVolDivC1 + 2;
		*tempMagVolDivC1 = 49;
		magVolDivC1[0] = 32;
		magVolDivC1[1] = 32;
		magVolDivC1[3] = 32;
	}
	//every time from 5 to 10
	else if (*tempMagVolDivC1 == 53) {
		*tempMagVolDivC1 = 48;
		tempMagVolDivC1--;
		*tempMagVolDivC1 = 49;
	}
	//every time from 1 to 2
	else if (*tempMagVolDivC1 == 49)
		*tempMagVolDivC1 = 50;
	//every time from 2 to 5
	else if (*tempMagVolDivC1 == 50)
		*tempMagVolDivC1 = 53;


	CanvasTextSet(&g_sAddMinusC1, magVolDivC1);
	PushButtonTextSet(&g_psTopButtons[0], magVolDivC1);
	WidgetPaint((tWidget * ) &g_psTopButtons[0]);
	WidgetPaint((tWidget * ) &g_sAddMinusC1);
}

////Minus function for magnitude division for channel 1/////////////
void MinusMagDivC1(tWidget *psWidget) {

	// make sure the voltage divison variable doesn't go out of bounds
	if(Mag1 == 0)
		Mag1 = 0;
	else
		Mag1--;

	// update voltage divison
	SetupVoltageDivision(Mag1, 1);

	// update midlevel based on new pixel_divider
	midlevel1 = 2048/pixel_divider1 + level1;
	midlevel2 = 2048/pixel_divider2 + level2;

	//25V to 20V
	if (magVolDivC1[1] == 50 && magVolDivC1[2] == 53 && magVolDivC1[3] == 32) {
		magVolDivC1[2] = 48;
		tempMagVolDivC1 = magVolDivC1 + 1;
	}
	//1V to 500mV
	else if (magVolDivC1[1] == 32 && magVolDivC1[2] == 49
			&& magVolDivC1[3] == 32) {
		tempMagVolDivC1 = magVolDivC1;
		*tempMagVolDivC1 = 53;
		magVolDivC1[1] = 48;
		magVolDivC1[2] = 48;
		magVolDivC1[3] = 109;
	}
	//stay at 20mv/div
	else if (magVolDivC1[1] == 50 && magVolDivC1[3] == 109)
		magVolDivC1[1] = 50;
	else if (*tempMagVolDivC1 == 50)
		*tempMagVolDivC1 = 49;
	else if (*tempMagVolDivC1 == 49) {
		*tempMagVolDivC1 = 32;
		tempMagVolDivC1++;
		*tempMagVolDivC1 = 53;
	} else if (*tempMagVolDivC1 == 53)
		*tempMagVolDivC1 = 50;

	CanvasTextSet(&g_sAddMinusC1, magVolDivC1);
	PushButtonTextSet(&g_psTopButtons[0], magVolDivC1);
	WidgetPaint((tWidget * ) &g_psTopButtons[0]);
	WidgetPaint((tWidget * ) &g_sAddMinusC1);
}

////Add function for magnitude division for channel 2/////////////
void AddMagDivC2(tWidget *psWidget) {

	// make sure voltage divison variable doesn't go out of bounds
	if(Mag2 == 10)
		Mag2 = 10;
	else
		Mag2++;

	// update voltage divison parameters
	SetupVoltageDivision(Mag2, 2);

	// update midlevel based on new pixel_divider
	midlevel1 = 2048/pixel_divider1 + level1;
	midlevel2 = 2048/pixel_divider2 + level2;

	//make it 25V
	if (magVolDivC2[1] == 50 && magVolDivC2[2] == 48 && magVolDivC2[3] == 32)
		magVolDivC2[2] = 53;
	//stay at 25V
	else if (magVolDivC2[1] == 50 && magVolDivC2[2] == 53
			&& magVolDivC2[3] == 32)
		magVolDivC2[2] = 53;
	// mV to V at 1 V
	else if (magVolDivC2[0] == 53) {
		tempMagVolDivC2 = magVolDivC2 + 2;
		*tempMagVolDivC2 = 49;
		magVolDivC2[0] = 32;
		magVolDivC2[1] = 32;
		magVolDivC2[3] = 32;
	}
	//every time from 5 to 10
	else if (*tempMagVolDivC2 == 53) {
		*tempMagVolDivC2 = 48;
		tempMagVolDivC2--;
		*tempMagVolDivC2 = 49;
	}
	//every time from 1 to 2
	else if (*tempMagVolDivC2 == 49)
		*tempMagVolDivC2 = 50;
	//every time from 2 to 5
	else if (*tempMagVolDivC2 == 50)
		*tempMagVolDivC2 = 53;
	CanvasTextSet(&g_sAddMinusC2, magVolDivC2);
	PushButtonTextSet(&g_psTopButtons[1], magVolDivC2);
	WidgetPaint((tWidget * ) &g_psTopButtons[1]);
	WidgetPaint((tWidget * ) &g_sAddMinusC2);
}

////Minus function for magnitude division for channel 2/////////////
void MinusMagDivC2(tWidget *psWidget) {

	// make sure voltage division variable doesn't go out of bounds
	if(Mag2 == 0)
		Mag2 = 0;
	else
		Mag2--;

	// update voltage divison parameters
	SetupVoltageDivision(Mag2, 2);

	// update midlevel based on new pixel_divider
	midlevel1 = 2048/pixel_divider1 + level1;
	midlevel2 = 2048/pixel_divider2 + level2;

	//25V to 20V
	if (magVolDivC2[1] == 50 && magVolDivC2[2] == 53 && magVolDivC2[3] == 32) {
		magVolDivC2[2] = 48;
		tempMagVolDivC2 = magVolDivC2 + 1;
	}
	//1V to 500mV
	else if (magVolDivC2[1] == 32 && magVolDivC2[2] == 49
			&& magVolDivC2[3] == 32) {
		tempMagVolDivC2 = magVolDivC2;
		*tempMagVolDivC2 = 53;
		magVolDivC2[1] = 48;
		magVolDivC2[2] = 48;
		magVolDivC2[3] = 109;
	}
	//stay at 20mv/div
	else if (magVolDivC2[1] == 50 && magVolDivC2[3] == 109)
		magVolDivC2[1] = 50;
	else if (*tempMagVolDivC2 == 50)
		*tempMagVolDivC2 = 49;
	else if (*tempMagVolDivC2 == 49) {
		*tempMagVolDivC2 = 32;
		tempMagVolDivC2++;
		*tempMagVolDivC2 = 53;
	} else if (*tempMagVolDivC2 == 53)
		*tempMagVolDivC2 = 50;

	CanvasTextSet(&g_sAddMinusC2, magVolDivC2);
	PushButtonTextSet(&g_psTopButtons[1], magVolDivC2);
	WidgetPaint((tWidget * ) &g_psTopButtons[1]);
	WidgetPaint((tWidget * ) &g_sAddMinusC2);
}

////Add function for time division for channel 1 & 2 /////////////
void AddTimeDiv(tWidget *psWidget) {

	// make sure time division variable doesn't go out of bounds
	if(Time == 28)
		Time = 28;
	else
		Time++;

	// update time divison parameters
	SetupTimeDivision(Time);

	// update EPI clock rate with new EPIDivide
	EPIDividerSet(EPI0_BASE, EPIDivide);

	//make it 50 s
	if (timVolDivC1[1] == 53 && timVolDivC1[2] == 48 && timVolDivC1[3] == 32)
		timVolDivC1[2] = 48;
	//stay at 50 s
	else if (timVolDivC1[1] == 53 && timVolDivC1[2] == 48
			&& timVolDivC1[3] == 32)
		timVolDivC1[1] =53 ;

	// from 500 ns to 1 us
	else if(timVolDivC1[0]==53 && timVolDivC1[3]==110){
		tempTimVolDivC1=timVolDivC1+2;
		*tempTimVolDivC1=49;
		timVolDivC1[0]=32;
		timVolDivC1[1]=32;
		timVolDivC1[3]=117;
	}
	// from 500 us to 1 ms
	else if(timVolDivC1[0]==53 && timVolDivC1[3]==117){
		tempTimVolDivC1=timVolDivC1+2;
		*tempTimVolDivC1=49;
		timVolDivC1[0]=32;
		timVolDivC1[1]=32;
		timVolDivC1[3]=109;
	}
	// from 500 ms to 1 s
	else if(timVolDivC1[0]==53 && timVolDivC1[3]==109){
		tempTimVolDivC1=timVolDivC1+2;
		*tempTimVolDivC1=49;
		timVolDivC1[0]=32;
		timVolDivC1[1]=32;
		timVolDivC1[3]=32;
	}
	//every time from 5 to 10
	else if (*tempTimVolDivC1 == 53) {
		*tempTimVolDivC1 = 48;
		tempTimVolDivC1--;
		*tempTimVolDivC1 = 49;
	}
	//every time from 1 to 2
	else if (*tempTimVolDivC1 == 49)
		*tempTimVolDivC1 = 50;
	//every time from 2 to 5
	else if (*tempTimVolDivC1 == 50)
		*tempTimVolDivC1 = 53;
	CanvasTextSet(&g_sAddMinusTime, timVolDivC1);
	PushButtonTextSet(&g_psTopButtons[2], timVolDivC1);
	WidgetPaint((tWidget * ) &g_psTopButtons[2]);
	WidgetPaint((tWidget * ) &g_sAddMinusTime);
}

////Minus function for time division for channel 1 & 2 /////////////
void MinusTimeDiv(tWidget *psWidget) {

	// make sure time division variable doesn't go out of bounds
	if(Time == 6)
		Time = 6;
	else
		Time--;

	// update time division parameters
	SetupTimeDivision(Time);

	// update EPI clock rate based on new EPIDivide
	EPIDividerSet(EPI0_BASE, EPIDivide);

	if (timVolDivC1[0] == 32 && timVolDivC1[1] == 32
			&& timVolDivC1[2] == 49&& timVolDivC1[3] == 32) {
		tempTimVolDivC1 = timVolDivC1;
		*tempTimVolDivC1 = 53;
		timVolDivC1[1] = 48;
		timVolDivC1[2] = 48;
		timVolDivC1[3] = 109;
	}
	//1ms to 500us
	else if  (timVolDivC1[0] == 32 && timVolDivC1[1] == 32
			&& timVolDivC1[2] == 49 && timVolDivC1[3] == 109)  {
		tempTimVolDivC1 = timVolDivC1;
		*tempTimVolDivC1 = 53;
		timVolDivC1[1] = 48;
		timVolDivC1[2] = 48;
		timVolDivC1[3] = 117;
	}
	//1us to 500ns
	else if  (timVolDivC1[0] == 32 && timVolDivC1[1] == 32
			&& timVolDivC1[2] == 49 && timVolDivC1[3] == 117)  {
		tempTimVolDivC1 = timVolDivC1;
		*tempTimVolDivC1 = 53;
		timVolDivC1[1] = 48;
		timVolDivC1[2] = 48;
		timVolDivC1[3] = 110;
	}
	//stay at 2us/div
	else if (timVolDivC1[2] == 50 && timVolDivC1[3] == 117)
		timVolDivC1[2] = 50;

	//from 5 to 2
	else if (*tempTimVolDivC1 == 53)
		*tempTimVolDivC1 = 50;
	//from 2 to 1
	else if (*tempTimVolDivC1 == 50)
		*tempTimVolDivC1 = 49;
   //from 1 to 0
	else if (*tempTimVolDivC1 == 49) {
		*tempTimVolDivC1 = 32;
		tempTimVolDivC1++;
		*tempTimVolDivC1 = 53;
	}

	CanvasTextSet(&g_sAddMinusTime, timVolDivC1);
	PushButtonTextSet(&g_psTopButtons[2], timVolDivC1);
	WidgetPaint((tWidget * ) &g_psTopButtons[2]);
	WidgetPaint((tWidget * ) &g_sAddMinusTime);
}


bool ButtonTF = false;
void AddMinusFunctionC1(tWidget *pWidget) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

		if(stop == 1){
			stopped = 1;
		}
		else{
			stop = 1;
			stopped = 0;
		}
		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sAddMinusC1);
		SysCtlDelay(3);
		WidgetPaint((tWidget * ) &g_sAddMinusC1);
	} else {
		ClrMyWidget();
		if(stopped == 1){
			stopped = 0;
		}
		else{
			stop = 0;
			stopped = 0;
		}
	}

}
void AddMinusFunctionC2(tWidget *pWidget) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

		if(stop == 1){
			stopped = 1;
		}
		else{
			stop = 1;
			stopped = 0;
		}
		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sAddMinusC2);
		WidgetPaint((tWidget * ) &g_sAddMinusC2);
	} else {
		ClrMyWidget();
		if(stopped == 1){
			stopped = 0;
		}
		else{
			stop = 0;
			stopped = 0;
		}
	}

}
void AddMinusFunctionTime(tWidget *pWidget) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

		if(stop == 1){
			stopped = 1;
		}
		else{
			stop = 1;
			stopped = 0;
		}
		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sAddMinusTime);
		WidgetPaint((tWidget * ) &g_sAddMinusTime);
	} else {
		ClrMyWidget();
		if(stopped == 1){
			stopped = 0;
		}
		else{
			stop = 0;
			stopped = 0;
		}
	}

}
///Not Using////////////////////////////////////////////
void DRadioFreMagnitudeC1(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

	} else {
		ClrMyWidget();
	}

}
void DRadioFreMagnitudeC2(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

	} else {
		ClrMyWidget();
	}

}
void DRadioVolMagnitudeC1(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

	} else {
		ClrMyWidget();
	}

}
void DRadioVolMagnitudeC2(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

	} else {
		ClrMyWidget();
	}

}
///////////////////////////////////////////////////////////////
///Display channels select options/////////////////////////////
void DRadioChannels(tWidget *pWidgetR){
	ButtonTF = !ButtonTF;
	if (ButtonTF) {
		if(stop == 1){
			stopped = 1;
		}
		else{
			stop = 1;
			stopped = 0;
		}
		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerChannels);
		WidgetPaint((tWidget * )&g_sContainerChannels);
	}
	else {
		if(stopped == 1){
			stopped = 0;
		}
		else{
			stop = 0;
			stopped = 0;
		}
		ClrMyWidget();

	}
}

void MathSelectRadioBtns(tWidget *psWidget, uint32_t bSelected){

	  uint32_t ui32Idx;
	  for(ui32Idx = 0; ui32Idx < NUM_RADIO_BUTTONS_Math; ui32Idx++)
	  {
	      if(psWidget == (tWidget *)(g_psRadioBtnTriggers + ui32Idx))
	      {
	          break;
	      }
	  }

	  //Math add function
	  if(ui32Idx==0){


	  }
	  //Math minus function
	  else if(ui32Idx==1){

	  }
}

///Trigger function//////////////////////////////////////////
void TriggerSelectRadioBtns(tWidget *psWidget, uint32_t bSelected){

	  uint32_t ui32Idx;
	  for(ui32Idx = 0; ui32Idx < NUM_RADIO_BUTTONS_Triggers; ui32Idx++)
	  {
	      if(psWidget == (tWidget *)(g_psRadioBtnTriggers + ui32Idx))
	      {
	          break;
	      }
	  }

	  if(ui32Idx==0){
		  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerTriggerSource);
		  WidgetPaint((tWidget * )&g_sContainerTriggerSource);
		  WidgetRemove((tWidget *) &g_sContainerTriggerMode);
		  WidgetRemove((tWidget *) &g_sTriggerSliderVertical);
		  WidgetRemove((tWidget *) &g_sTriggerSliderHorizontal);

	  }
	  else if(ui32Idx==1){
		  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sTriggerSliderVertical);
		  WidgetPaint((tWidget * )&g_sTriggerSliderVertical);
		  WidgetRemove((tWidget *) &g_sContainerTriggerSource);
		  WidgetRemove((tWidget *) &g_sContainerTriggerMode);
		  WidgetRemove((tWidget *) &g_sTriggerSliderHorizontal);
	  }
	  else if(ui32Idx==2){
		  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sTriggerSliderHorizontal);
		  WidgetPaint((tWidget * )&g_sTriggerSliderHorizontal);
		  WidgetRemove((tWidget *) &g_sContainerTriggerSource);
		  WidgetRemove((tWidget *) &g_sContainerTriggerMode);
		  WidgetRemove((tWidget *) &g_sTriggerSliderVertical);

	  	  }
	  else{
		  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerTriggerMode);
		  WidgetPaint((tWidget * )&g_sContainerTriggerMode);
		  WidgetRemove((tWidget *) &g_sContainerTriggerSource);
		  WidgetRemove((tWidget *) &g_sTriggerSliderVertical);
		  WidgetRemove((tWidget *) &g_sTriggerSliderHorizontal);

	  }
}

void TriggerModeSelect(tWidget *psWidget, uint32_t bSelected){
	uint32_t ui32Idx;
		  for(ui32Idx = 0; ui32Idx < NUM_RADIO_BUTTONS_TriggerMode; ui32Idx++)
		  {
		      if(psWidget == (tWidget *)(g_psRadioBtnTriggerMode + ui32Idx))
		      {
		          break;
		      }
		  }

/////Postive edge////////////
		  if(ui32Idx==0){
			  TriggerMode  = 0;

		  }
/////Negative edge///////////
		  else  if(ui32Idx==1) {
			  TriggerMode  = 1;
		  }


}

void TriggerSourceSelect(tWidget *psWidget, uint32_t bSelected){
	uint32_t ui32Idx;
		  for(ui32Idx = 0; ui32Idx < NUM_RADIO_BUTTONS_Sources; ui32Idx++)
		  {
		      if(psWidget == (tWidget *)(g_psRadioBtnSource + ui32Idx))
		      {
		          break;
		      }
		  }

/////Source 1/////////////////////////
		  if(ui32Idx==0){
			  TriggerSource = 1;

		  }
/////Source 2/////////////////////////
		  else  if(ui32Idx==1) {
			  TriggerSource = 2;
		  }
}

////Channel select function///////////////////////////////
void ChannelSelectRadioBtns(tWidget *psWidget, uint32_t bSelected){
	  uint32_t ui32Idx;
	  for(ui32Idx = 0; ui32Idx < NUM_RADIO_BUTTONS_Channels; ui32Idx++)
	  {
	      if(psWidget == (tWidget *)(g_psRadioBtnChannels + ui32Idx))
	      {
	          break;
	      }
	  }
	  if(ui32Idx==0){
		  Ch1on = 1;
		  Ch2on = 0;
		  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sC1Slider);
		  WidgetPaint((tWidget * )&g_sC1Slider);
		  WidgetRemove((tWidget *) &g_sC2Slider);

	  }
	  else if(ui32Idx==1){
		  Ch1on = 0;
		  Ch2on = 1;
		  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sC2Slider);
		  WidgetPaint((tWidget * )&g_sC2Slider);
		  WidgetRemove((tWidget *) &g_sC1Slider);
	  }
	  else{
		  Ch1on = 1;
		  Ch2on = 1;
		  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sC1Slider);
		  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sC2Slider);
		  WidgetPaint((tWidget * )&g_sC1Slider);
		  WidgetPaint((tWidget * )&g_sC2Slider);
	  }
}


void DRadioAcquire(tWidget *pWidgetR) {
	ButtonTF = !ButtonTF;
	if (ButtonTF) {

		if(stop == 1){
			stopped = 1;
		}
		else{
			stop = 1;
			stopped = 0;
		}
		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerAcquire);
		WidgetPaint((tWidget * )&g_sContainerAcquire);
	} else {
		ClrMyWidget();
		if(stopped == 1){
			stopped = 0;
		}
		else{
			stop = 0;
			stopped = 0;
		}
	}

}
void AcquireSelectRadioBtns(tWidget *psWidget, uint32_t bSelected){
	  uint32_t ui32Idx;
		  for(ui32Idx = 0; ui32Idx < NUM_RADIO_BUTTONS_Acquire; ui32Idx++)
		  {
		      if(psWidget == (tWidget *)(g_psRadioBtnAcquire + ui32Idx))
		      {
		          break;
		      }
		  }

		  if(ui32Idx==0){
			  CaptureMode = 0;

		  }
		  else if(ui32Idx==1){
			  CaptureMode = 1;
		  }


}

void DWaveForm(tWidget *pWidgetR, tContext *psContext) {
///////////////////////////////////////////////////////////////////////


	tRectangle Rect1, Rect2;
	if(Ch1on == 1){
		GrContextForegroundSet(&sContext, ClrRed);
		Rect1.i16XMin = 0;
		Rect1.i16YMin = desiredlevel1-2;
		Rect1.i16XMax = 4;
		Rect1.i16YMax = desiredlevel1+1;
		GrRectFill(&sContext, &Rect1);
	}
	else if(Ch1on == 0 && Ch1off == 0){
		GrContextForegroundSet(&sContext, ClrBlack);
		Rect1.i16XMin = 0;
		Rect1.i16YMin = desiredlevel1-2;
		Rect1.i16XMax = 4;
		Rect1.i16YMax = desiredlevel1+1;
		GrRectFill(&sContext, &Rect1);
	}

	if(Ch2on == 1){
		GrContextForegroundSet(&sContext, ClrYellow);
		Rect2.i16XMin = 0;
		Rect2.i16YMin = desiredlevel2-2;
		Rect2.i16XMax = 4;
		Rect2.i16YMax = desiredlevel2+1;
		GrRectFill(&sContext, &Rect2);
	}
	else if(Ch2on == 0 && Ch2off == 0){
		GrContextForegroundSet(&sContext, ClrBlack);
		Rect2.i16XMin = 0;
		Rect2.i16YMin = desiredlevel2-2;
		Rect2.i16XMax = 4;
		Rect2.i16YMax = desiredlevel2+1;
		GrRectFill(&sContext, &Rect2);
	}


	// redraw over previous signal in black and remove signal if one of the channels have been disabled
	for (x = 7; x < SERIES_LENGTH; x++) {
		GrContextForegroundSet(&sContext, ClrBlack);
		if(Ch1on == 1){
			Ch1off = 0;
			GrLineDraw(&sContext,x-1,old1[x-1],x,old1[x]);
		}
		else if(Ch1on == 0 && Ch1off == 0){
			  ClrMyWidget();
			  Ch1off = 1;
		}
		if(Ch2on == 1){
			Ch2off = 0;
			GrLineDraw(&sContext,x-1,old2[x-1],x,old2[x]);
		}
		else if(Ch2on == 0 && Ch2off == 0){
			  ClrMyWidget();
			  Ch2off = 1;
		}

		// if channel 2 should be on, draw lines to create signal
		if(Ch2on == 1){
			GrContextForegroundSet(&sContext, ClrYellow);
			// if stopped, don't draw an updated signal
			if(stop == 1){
				GrLineDraw(&sContext,x-1,old2[x-1],x,old2[x]);
			}
			else{
				// determine which pixel values will be out of bounds
				if((midlevel2 - pixels2[x-1] / pixel_divider2) < 29){
					minusbelow2 = 1;
				}
				else{
					minusbelow2 = 0;
				}
				if ((midlevel2 - pixels2[x-1] / pixel_divider2) > 210){
					minusabove2 = 1;
				}
				else{
					minusabove2 = 0;
				}
				if((midlevel2 - pixels2[x] / pixel_divider2) < 29){
					outbelow2 = 1;
				}
				else{
					outbelow2 = 0;
				}
				if ((midlevel2 - pixels2[x] / pixel_divider2) > 210){
					outabove2 = 1;
				}
				else{
					outabove2 = 0;
				}

				// draw the lines from  pixel to pixel based on acquired values unless they would go out of bounds
				if(minusbelow2 == 0 && minusabove2 == 0 && outbelow2 == 0 && outabove2 == 0){
					GrLineDraw(&sContext,x-1,old2[x-1]=midlevel2 - pixels2[x-1] / pixel_divider2,x,midlevel2 - pixels2[x] / pixel_divider2);
				}
				else if(minusbelow2 == 1 && outbelow2 == 0 && outabove2 == 0){
					GrLineDraw(&sContext,x-1,old2[x-1]=29,x,midlevel2 - pixels2[x] / pixel_divider2);
				}
				else if(minusbelow2 == 1 && outbelow2 == 1){
					GrLineDraw(&sContext,x-1,old2[x-1]=29,x,29);
				}
				else if(minusbelow2 == 1 && outabove2 == 1){
					GrLineDraw(&sContext,x-1,old2[x-1]=29,x,210);
				}
				else if(minusabove2 == 1 && outbelow2 == 0 && outabove2 == 0){
					GrLineDraw(&sContext,x-1,old2[x-1]=210,x,midlevel2 - pixels2[x] / pixel_divider2);
				}
				else if(minusabove2 == 1 && outbelow2 == 1){
					GrLineDraw(&sContext,x-1,old2[x-1]=210,x,29);
				}
				else if(minusabove2 == 1 && outabove2 == 1){
					GrLineDraw(&sContext,x-1,old2[x-1]=210,x,210);
				}
				else if(outbelow2 == 1 && minusabove2 == 0 && minusbelow2 == 0){
					GrLineDraw(&sContext,x-1,old2[x-1]=midlevel2 - pixels2[x-1] / pixel_divider2,x,29);
				}
				else if(outabove2 == 1 && minusabove2 == 0 && minusbelow2 == 0){
					GrLineDraw(&sContext,x-1,old2[x-1]=midlevel2 - pixels2[x-1] / pixel_divider2,x,210);
				}
				else{
					GrLineDraw(&sContext,x-1,old2[x-1]=210,x,210);
				}
			}
		}

		// draw channel 1 signal if desired
		if(Ch1on == 1){
			GrContextForegroundSet(&sContext, ClrRed);
			// if stopped, don't update signal
			if(stop == 1){
				GrLineDraw(&sContext,x-1,old1[x-1],x,old1[x]);
			}
			else{
				// determine whici pixel values go out of bounds
				if((midlevel1 - pixels[x-1] / pixel_divider1) < 29){
					minusbelow1 = 1;
				}
				else{
					minusbelow1 = 0;
				}
				if ((midlevel1 - pixels[x-1] / pixel_divider1) > 210){
					minusabove1 = 1;
				}
				else{
					minusabove1 = 0;
				}
				if((midlevel1 - pixels[x] / pixel_divider1) < 29){
					outbelow1 = 1;
				}
				else{
					outbelow1 = 0;
				}
				if ((midlevel1 - pixels[x] / pixel_divider1) > 210){
					outabove1 = 1;
				}
				else{
					outabove1 = 0;
				}

				// draw a line from pixel to pixel based on acquired values unless they go out of bounds
				if(minusbelow1 == 0 && minusabove1 == 0 && outbelow1 == 0 && outabove1 == 0){
					GrLineDraw(&sContext,x-1,old1[x-1]=midlevel1 - pixels[x-1] / pixel_divider1,x,midlevel1 - pixels[x] / pixel_divider1);
				}
				else if(minusbelow1 == 1 && outbelow1 == 0 && outabove1 == 0){
					GrLineDraw(&sContext,x-1,old1[x-1]=29,x,midlevel1 - pixels[x] / pixel_divider1);
				}
				else if(minusbelow1 == 1 && outbelow1 == 1){
					GrLineDraw(&sContext,x-1,old1[x-1]=29,x,29);
				}
				else if(minusbelow1 == 1 && outabove1 == 1){
					GrLineDraw(&sContext,x-1,old1[x-1]=29,x,210);
				}
				else if(minusabove1 == 1 && outbelow1 == 0 && outabove1 == 0){
					GrLineDraw(&sContext,x-1,old1[x-1]=210,x,midlevel1 - pixels[x] / pixel_divider1);
				}
				else if(minusabove1 == 1 && outbelow1 == 1){
					GrLineDraw(&sContext,x-1,old1[x-1]=210,x,29);
				}
				else if(minusabove1 == 1 && outabove1 == 1){
					GrLineDraw(&sContext,x-1,old1[x-1]=210,x,210);
				}
				else if(outbelow1 == 1 && minusabove1 == 0 && minusbelow1 == 0){
					GrLineDraw(&sContext,x-1,old1[x-1]=midlevel1 - pixels[x-1] / pixel_divider1,x,29);
				}
				else if(outabove1 == 1 && minusabove1 == 0 && minusbelow1 == 0){
					GrLineDraw(&sContext,x-1,old1[x-1]=midlevel1 - pixels[x-1] / pixel_divider1,x,210);
				}
				else{
					GrLineDraw(&sContext,x-1,old1[x-1]=210,x,210);
				}
			}
		}

	}
	GrContextForegroundSet(&sContext, ClrBlack);
	GrLineDraw(&sContext,319,29,319,210);
	GrLineDraw(&sContext,318,29,318,210);
	GrLineDraw(&sContext,317,29,317,210);
	GrContextForegroundSet(&sContext, ClrWhite);
	GrCircleFill(&sContext, 160, 119, 3);
//////////////////////////////////////////////////////////////////////
///Grid for vertical division
	for (x = 0; x <= 320; x += 4) {
		for (y = 29; y <= 210; y += 15)
			GrPixelDraw(&sContext, x, y);
	}
///Grid for horizontal division
	for (x = 0; x < 321; x += 40) {
		for (y = 29; y <= 210; y += 4)
			GrPixelDraw(&sContext, x, y);
	}

}

void TriggerFunction(tWidget *pWidget){
	ButtonTF = !ButtonTF;
	if (ButtonTF) {
		if(stop == 1){
			stopped = 1;
		}
		else{
			stop = 1;
			stopped = 0;
		}
		WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sContainerTriggers);
		WidgetPaint((tWidget * )&g_sContainerTriggers);
	}
	else {
		if(stopped == 1){
			stopped = 0;
		}
		else{
			stop = 0;
			stopped = 0;
		}
		ClrMyWidget();
	}
}
void
OnSliderChangeVertical(tWidget *psWidget, int32_t i32Value){

	TriggerLevel = (midlevel1 - (240 - (uint16_t) i32Value));

}
void OnSliderChangeHorizontal(tWidget *psWidget, int32_t i32Value){

TriggerPosition = i32Value;

}


////C1 level Change
void OnSliderChangeC1(tWidget *psWidget, int32_t i32Value){

desiredlevel1 = 240 - i32Value;
midlevel1 = 2048/pixel_divider1 + desiredlevel1;

}


////C2 level Change
void OnSliderChangeC2(tWidget *psWidget, int32_t i32Value){

desiredlevel2 = 240 - i32Value;
midlevel2 = 2048/pixel_divider2 + desiredlevel2;

}

// determine whether to keep signal updating
void RunStop(tWidget *psWidget){
	if(stop == 0)
		stop = 1;
	else
		stop = 0;
}

// remove DC offset for testing purposes
void OffSet(tWidget *psWidget){
	CalibrateOffset();
}

//
int main(void) {
	// Set clock frequency to 120 MHz
	ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
	SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
	SYSCTL_CFG_VCO_480), 120000000);
	tempMagVolDivC1 = magVolDivC1 + 1;
	tempMagVolDivC2 = magVolDivC2 + 1;
	tempTimVolDivC1 = timVolDivC1 + 2;



	Kentec320x240x16_SSD2119Init(ui32SysClkFreq);
	GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);
	TouchScreenInit(ui32SysClkFreq);
	TouchScreenCallbackSet(WidgetPointerMessage);

////Intro pictures//////////////////////////
	GrImageDraw(&sContext, g_pui8Image, 0, 0);
	GrFlush(&sContext);
	SysCtlDelay(ui32SysClkFreq);

	GrImageDraw(&sContext, g_pui9Image, 0, 0);
	GrFlush(&sContext);
	SysCtlDelay(ui32SysClkFreq);
	ClrScreen();
////////////////////////////////////////////
	WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sBackground);
	WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sWaveform);
	WidgetPaint(WIDGET_ROOT);

	// setup all peripherals
	setup();

	WidgetPaint(WIDGET_ROOT);
	while (1) {

		// update all values when the primary transfer is complete
		if (transfer_done[0] == 1) {
				transfer_done[0] = 0;
				PixelsCalculation(inputs);
		}

		// update all values when the alternate transfer is complete
		if (transfer_done[1] == 1) {
				transfer_done[1] = 0;
				PixelsCalculation(inputs2);
		}

				// Issue paint request to the widgets.

				if(stop == 0){
					WidgetPaint((tWidget *) &g_sWaveform);
					WidgetPaint((tWidget * ) &g_psBotButtons[0]);
					WidgetPaint((tWidget * ) &g_psBotButtons[1]);
					WidgetPaint((tWidget * ) &g_psBotButtons[2]);
					WidgetPaint((tWidget * ) &g_psBotButtons[3]);
				}

				//
				// Process any messages in the widget message queue.
				//
				WidgetMessageQueueProcess();

	}

}

void PixelsCalculation(uint32_t input[MEM_BUFFER_SIZE]) {
	// determine which bits are high and low
	for (f = 0; f < MEM_BUFFER_SIZE; f++) {

		receive[0] = (input[f] & 0b00000000000000000000000000000001)/0b00000000000000000000000000000001;
		receive[1] = (input[f] & 0b00000000000000000000000000000010)/0b00000000000000000000000000000010;
		receive[2] = (input[f] & 0b00000000000000000000000000000100)/0b00000000000000000000000000000100;
		receive[3] = (input[f] & 0b00000000000000000000000000001000)/0b00000000000000000000000000001000;
		receive[4] = (input[f] & 0b00000000000000000000000000010000)/0b00000000000000000000000000010000;
		receive[5] = (input[f] & 0b00000000000000000000000000100000)/0b00000000000000000000000000100000;
		receive[6] = (input[f] & 0b00000000000000000000000001000000)/0b00000000000000000000000001000000;
		receive[7] = (input[f] & 0b00000000000000000000000010000000)/0b00000000000000000000000010000000;
		receive[8] = (input[f] & 0b00000000000000000000000100000000)/0b00000000000000000000000100000000;
		receive[9] = (input[f] & 0b00000000000000000000001000000000)/0b00000000000000000000001000000000;
		receive[10] = (input[f] & 0b00000000000000000000100000000000)/0b00000000000000000000100000000000;
		receive[11] = (input[f] & 0b00000000000000000001000000000000)/0b00000000000000000001000000000000;
		receive[12] = (input[f] & 0b00000000000000000010000000000000)/0b00000000000000000010000000000000;
		receive[13] = (input[f] & 0b00000000000000000100000000000000)/0b00000000000000000100000000000000;
		receive[14] = (input[f] & 0b00000000000000001000000000000000)/0b00000000000000001000000000000000;
		receive[15] = (input[f] & 0b00000000000000010000000000000000)/0b00000000000000010000000000000000;
		receive[16] = (input[f] & 0b00000000000000100000000000000000)/0b00000000000000100000000000000000;
		receive[17] = (input[f] & 0b00000000000001000000000000000000)/0b00000000000001000000000000000000;
		receive[18] = (input[f] & 0b00000000000010000000000000000000)/0b00000000000010000000000000000000;
		receive[19] = (input[f] & 0b00000001000000000000000000000000)/0b00000001000000000000000000000000;
		receive[20] = (input[f] & 0b00000010000000000000000000000000)/0b00000010000000000000000000000000;
		receive[21] = (input[f] & 0b00000100000000000000000000000000)/0b00000100000000000000000000000000;
		receive[22] = (input[f] & 0b00001000000000000000000000000000)/0b00001000000000000000000000000000;
		receive[23] = (input[f] & 0b00010000000000000000000000000000)/0b00010000000000000000000000000000;

		// calculate decimal value if number is positive
		if(receive[11] == 0){

			totalA = 2048 + (receive[0] + 2*receive[1] + 4*receive[2] + 8*receive[3]
			         + 16*receive[4] + 32*receive[5] + 64*receive[6] + 128*receive[7]
			         + 1*(256*receive[8] + 512*receive[9] + 1024*receive[10]));

			values[f + k*MEM_BUFFER_SIZE] = totalA & 0b00000000000000000000111111111111;
		}
		// calculate decimal value if number is negative
		else{

			// first negate all bits
			if(receive[0])
				receive[0] = 0;
			else
				receive[0] = 1;

			if(receive[1])
				receive[1] = 0;
			else
				receive[1] = 1;

			if(receive[2])
				receive[2] = 0;
			else
				receive[2] = 1;

			if(receive[3])
				receive[3] = 0;
			else
				receive[3] = 1;

			if(receive[4])
				receive[4] = 0;
			else
				receive[4] = 1;

			if(receive[5])
				receive[5] = 0;
			else
				receive[5] = 1;

			if(receive[6])
				receive[6] = 0;
			else
				receive[6] = 1;

			if(receive[7])
				receive[7] = 0;
			else
				receive[7] = 1;

			if(receive[8])
				receive[8] = 0;
			else
				receive[8] = 1;

			if(receive[9])
				receive[9] = 0;
			else
				receive[9] = 1;

			if(receive[10])
				receive[10] = 0;
			else
				receive[10] = 1;

			totalA = 2048 - (1 + receive[0] + 2*receive[1] + 4*receive[2] + 8*receive[3]
			         + 16*receive[4] + 32*receive[5] + 64*receive[6] + 128*receive[7]
			         + 1*(256*receive[8] + 512*receive[9] + 1024*receive[10]));

			values[f + k*MEM_BUFFER_SIZE] = totalA & 0b00000000000000000000111111111111;
		}

		// calculate decimal value if number is positive
		if(receive[23] == 0){

			totalB = 2048 + (receive[12] + 2*receive[13] + 4*receive[14] + 8*receive[15]
			         + 16*receive[16] + 32*receive[17] + 64*receive[18] + 128*receive[19]
			         + 1*(256*receive[20] + 512*receive[21] + 1024*receive[22]));

			values2[f + k*MEM_BUFFER_SIZE] = totalB & 0b00000000000000000000111111111111;
		}
		// calculate decimal value if number is negative
		else{

			// first negate all bits
			if(receive[12])
				receive[12] = 0;
			else
				receive[12] = 1;

			if(receive[13])
				receive[13] = 0;
			else
				receive[13] = 1;

			if(receive[14])
				receive[14] = 0;
			else
				receive[14] = 1;

			if(receive[15])
				receive[15] = 0;
			else
				receive[15] = 1;

			if(receive[16])
				receive[16] = 0;
			else
				receive[16] = 1;

			if(receive[17])
				receive[17] = 0;
			else
				receive[17] = 1;

			if(receive[18])
				receive[18] = 0;
			else
				receive[18] = 1;

			if(receive[19])
				receive[19] = 0;
			else
				receive[19] = 1;

			if(receive[20])
				receive[20] = 0;
			else
				receive[20] = 1;

			if(receive[21])
				receive[21] = 0;
			else
				receive[21] = 1;

			if(receive[22])
				receive[22] = 0;
			else
				receive[22] = 1;

			totalB = 2048 - (1 + receive[12] + 2*receive[13] + 4*receive[14] + 8*receive[15]
			         + 16*receive[16] + 32*receive[17] + 64*receive[18] + 128*receive[19]
			         + 1*(256*receive[20] + 512*receive[21] + 1024*receive[22]));

			values2[f + k*MEM_BUFFER_SIZE] = totalB & 0b00000000000000000000111111111111;
		}


		// determine which trigger mode and source is being used and then start the trigger if the values increment (positive edge)
		// or decrement (negative edge) over the desired trigger level
		if(TriggerMode == 0){
			if(TriggerSource == 1){
				if(values[f + k*MEM_BUFFER_SIZE-1] <= *PTriggerLevel && values[f + k*MEM_BUFFER_SIZE] >= *PTriggerLevel){
					TriggerStart = 1;
					Trigger = 1;
				}
			}
			else
				if(values2[f + k*MEM_BUFFER_SIZE-1] <= *PTriggerLevel && values2[f + k*MEM_BUFFER_SIZE] >= *PTriggerLevel){
					TriggerStart = 1;
					Trigger = 1;
				}
		}
		else if(TriggerMode == 1){
			if(TriggerSource == 1){
				if(values[f + k*MEM_BUFFER_SIZE-1] >= *PTriggerLevel && values[f + k*MEM_BUFFER_SIZE] <= *PTriggerLevel){
					TriggerStart = 1;
					Trigger = 1;
				}
			}
			else
				if(values2[f + k*MEM_BUFFER_SIZE] >= *PTriggerLevel && values2[f + k*MEM_BUFFER_SIZE] <= *PTriggerLevel){
					TriggerStart = 1;
					Trigger = 1;
				}
		}

		// Start finding pixel values when trigger found
		if(TriggerStart == 1){
			// for normal acquire mode
			if(CaptureMode == 0){
				if(begin == 0){
					begin=1;
					// find all pixel values before trigger position
					for(i=0;i<TriggerPosition;i++){
						if((int32_t) (f + k*MEM_BUFFER_SIZE - TriggerPosition + i*(NumSkip+1)) < 0){
							pixels[i] = values[MaxSize - (f + k*MEM_BUFFER_SIZE - TriggerPosition + i*(NumSkip+1))];
							pixels2[i] = values2[MaxSize - (f + k*MEM_BUFFER_SIZE - TriggerPosition + i*(NumSkip+1))];
						}
						else{
							pixels[i] = values[f + k*MEM_BUFFER_SIZE - TriggerPosition + i*(NumSkip+1)];
							pixels2[i] = values2[f + k*MEM_BUFFER_SIZE - TriggerPosition + i*(NumSkip+1)];
						}
					}
				}
				// wait until the desired number of values have been skipped
				if(j<NumSkip){
					j++;
				}
				else{
					j=0;
					// set all pixel values to the values obtained from the ADC
					if(m < SERIES_LENGTH){
						pixels[m] = values[f + k*MEM_BUFFER_SIZE];
						pixels2[m] = values2[f + k*MEM_BUFFER_SIZE];
						m++;
					}
					// update measurements and restart trigger if all pixel values have been found
					else{
						UpdateMeasurements();
						m = 0;
						begin = 0;
						TriggerStart = 0;
					}
				}
			}
			// for averaging mode
			else if(CaptureMode == 1){
				if(begin == 0){
					begin=1;
					if(j<NumAvg){
						// find all pixel values before trigger position
						for(i=0;i<TriggerPosition;i++){
							if((int32_t) (f + k*MEM_BUFFER_SIZE - TriggerPosition) < 0){
								totalsA[i] = totalsA[i] + values[MaxSize - (f + k*MEM_BUFFER_SIZE - TriggerPosition + i)];
								totalsB[i] = totalsB[i] + values2[MaxSize - (f + k*MEM_BUFFER_SIZE - TriggerPosition + i)];
							}
							else{
								totalsA[i] = totalsA[i] + values[f + k*MEM_BUFFER_SIZE - TriggerPosition + i];
								totalsB[i] = totalsB[i] + values2[f + k*MEM_BUFFER_SIZE - TriggerPosition + i];
							}
						}
					}
				}
				// keep summing up each pixel value until the number of sets have been gone through
				if(j<NumAvg){
					if(m < SERIES_LENGTH){
						totalsA[m] = totalsA[m]+ values[f + k*MEM_BUFFER_SIZE];
						totalsB[m] = totalsB[m]+ values2[f + k*MEM_BUFFER_SIZE];
						m++;
					}
					else{
						j++;
						m=0;
						begin = 0;
						TriggerStart = 0;
					}
				}
				// determine averaged pixel values and then reset trigger and sums
				else{
					j = 0;
					for(i=0;i<TriggerPosition;i++){
						totalsA[i] = totalsA[i]/2;
						totalsB[i] = totalsB[i]/2;
					}
					for(i=0;i<SERIES_LENGTH;i++){
						pixels[i] = totalsA[i]/NumAvg;
						pixels2[i] = totalsB[i]/NumAvg;
					}
					for(i=0;i<SERIES_LENGTH;i++){
						totalsA[i] = 0;
						totalsB[i] = 0;
					}
					UpdateMeasurements();
					m = 0;
					begin = 0;
					TriggerStart = 0;
				}
			}
		}
	}
	if(Trigger == 1){
		GoThrough = 0;
		Trigger = 0;
	}
	// increment Gothrough if a trigger wasn't found in this iteration
	else
		GoThrough++;

	if(GoThrough > 200){
		GoThrough = 20;
	}

	// when enough iterations have passed, provide a trigger so the signal can be seen
	if(GoThrough >= 10){
		TriggerStart = 1;
		UpdateMeasurements();
	}


// keep going further in the values array as long as the Maximum Size won't be reached in the next iteration
if(k*MEM_BUFFER_SIZE + MEM_BUFFER_SIZE < MaxSize)
	k++;

else
	k = 0;
}

void ClrScreen() {
	sRect.i16XMin = 0;
	sRect.i16YMin = 0;
	sRect.i16XMax = 319;
	sRect.i16YMax = 239;
	GrContextForegroundSet(&sContext, ClrBlack);
	GrRectFill(&sContext, &sRect);
	GrFlush(&sContext);
}

//Clean all the running widget and repaint the BackGround and the Waveform
void ClrMyWidget(){
	WidgetRemove((tWidget *) &g_sContainerChannels);
//	WidgetRemove((tWidget *) &g_sC1Slider);
//	WidgetRemove((tWidget *) &g_sContainerMath);
	WidgetRemove((tWidget *) &g_sContainerTriggerSource);
	WidgetRemove((tWidget *) &g_sContainerTriggerMode);
	WidgetRemove((tWidget *) &g_sTriggerSliderVertical);
	WidgetRemove((tWidget *) &g_sTriggerSliderHorizontal);
	WidgetRemove((tWidget *) &g_sContainerTriggers);
	WidgetRemove((tWidget *) &g_sContainerAcquire);
	WidgetRemove((tWidget *) &g_sContainerFreMagnitudeC1);
	WidgetRemove((tWidget *) &g_sContainerFreMagnitudeC2);
	WidgetRemove((tWidget *) &g_sContainerVolMagnitudeC1);
	WidgetRemove((tWidget *) &g_sContainerVolMagnitudeC2);
	WidgetRemove((tWidget *) &g_sAddMinusC1);
	WidgetRemove((tWidget *) &g_sAddMinusC2);
	WidgetRemove((tWidget *) &g_sAddMinusTime);
	//WidgetRemove((tWidget *) &g_sTriggerSlider);

	WidgetPaint((tWidget * )&g_sBackground);
	WidgetPaint((tWidget * )&g_sWaveform);
}

////////////////////////////////////////////////////////
// EPI interrupt fuction
//////////////////////////////////////////////////
void EPIIntHandler(void) {

	// Check EPI interrupt status and clear interrupt
	EPIMode = EPIIntStatus(EPI0_BASE, true);
	uDMAIntClear(UDMA_CHANNEL_SW);
	EPIIntErrorClear(EPI0_BASE, EPI_INT_ERR_DMARDIC);
	if (EPIMode == EPI_INT_DMA_RX_DONE) {

		// determine if primary or alternate transfer is complete
		pri = pui8ControlTable[488] & 0b11;
		alt = pui8ControlTable[1000] & 0b11;

		if (pri == 0) {
			// reset primary transfer
			transfer_done[0] = 1;
			uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
			UDMA_MODE_PINGPONG, EPISource, g_ui32DstBuf[0],
			MEM_BUFFER_SIZE);

			uDMAChannelEnable(UDMA_CHANNEL_SW);

		 	EPINonBlockingReadConfigure(EPI0_BASE, 0, EPI_NBCONFIG_SIZE_32, 0);
			EPINonBlockingReadStart(EPI0_BASE, 0, CountSize);
		}

		if (alt == 0) {
			// reset alternate transfer
			transfer_done[1] = 1;
			uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_ALT_SELECT,
			UDMA_MODE_PINGPONG, EPISource, g_ui32DstBuf2[0],
			MEM_BUFFER_SIZE);

			uDMAChannelEnable(UDMA_CHANNEL_SW);

			EPINonBlockingReadConfigure(EPI0_BASE, 1, EPI_NBCONFIG_SIZE_32, 0);
			EPINonBlockingReadStart(EPI0_BASE, 1, CountSize);
		}

	}

}

// Sets up DMA to work with EPI
void SetupDMA(void) {

	//
	// Enable interrupts from the uDMA channel.
	//
	IntEnable(INT_UDMA);

	//
	// Setup attributes for UDMA channel 30
	//
	uDMAChannelAttributeDisable(UDMA_CHANNEL_SW,
	UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | (UDMA_ATTR_HIGH_PRIORITY |
	UDMA_ATTR_REQMASK));

	uDMAChannelAttributeEnable(UDMA_CHANNEL_SW, UDMA_ATTR_HIGH_PRIORITY);

	//
	// Configure the control parameters for channel 30
	//
	uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
	UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
	UDMA_ARB_8);

	uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_ALT_SELECT,
	UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
	UDMA_ARB_8);

	//
	// Set up the transfer parameters for the channel 30
	//
	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_ALT_SELECT,
	UDMA_MODE_PINGPONG, EPISource, g_ui32DstBuf2[0],
	MEM_BUFFER_SIZE);

	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
	UDMA_MODE_PINGPONG, EPISource, g_ui32DstBuf[0],
	MEM_BUFFER_SIZE);

	//
	// Enable UDMA channel 30
	//
	uDMAChannelEnable(UDMA_CHANNEL_SW);
}

// Main program//////////////////////////////////////////////////////////////////////////////
void setup(void) {

	// point the destination buffer to the correct input arrays
	for (f = 0; f < MEM_BUFFER_SIZE; f++) {
		g_ui32DstBuf[f] = &inputs[f];
	}

	for (f = 0; f < MEM_BUFFER_SIZE; f++) {
		g_ui32DstBuf2[f] = &inputs2[f];
	}

	// intialzie a value of midlevels
	midlevel1 = 2048/pixel_divider1 + level1;
	midlevel2 = 2048/pixel_divider2 + level2;

	// Give initial conditions for totals for averaging
	for(i=0;i<SERIES_LENGTH;i++){
		totalsA[i] = 0;
	}
	for(i=0;i<SERIES_LENGTH;i++){
		totalsB[i] = 0;
	}
	for(i=0;i<SERIES_LENGTH;i++){
		old1[i] = 120;
	}
	for(i=0;i<SERIES_LENGTH;i++){
		old2[i] = 120;
	}

	// Setup the mV per pixel for each vertical scale division
	mvpixel[0] = 20.0/15;
	mvpixel[1] = 50.0/15;
	mvpixel[2] = 100.0/15;
	mvpixel[3] = 200.0/15;
	mvpixel[4] = 500.0/15;
	mvpixel[5] = 1000.0/15;
	mvpixel[6] = 2000.0/15;
	mvpixel[7] = 5000.0/15;
	mvpixel[8] = 10000.0/15;
	mvpixel[9] = 20000.0/15;
	mvpixel[10] = 50000.0/15;

	// Setup the seconds per pixel for horixtonal scale division
	secpixel[0] = (8*.00000002)/240;
	secpixel[1] = (8*.00000005)/240;
	secpixel[2] = (8*.0000001)/240;
	secpixel[3] = (8*.0000002)/240;
	secpixel[4] = (8*.0000005)/240;
	secpixel[5] = (8*.000001)/240;
	secpixel[6] = (8*.000002)/240;
	secpixel[7] = (8*.000005)/240;
	secpixel[8] = (8*.00001)/240;
	secpixel[9] = (8*.00002)/240;
	secpixel[10] = (8*.00005)/240;
	secpixel[11] = (8*.0001)/240;
	secpixel[12] = (8*.0005)/240;
	secpixel[13] = (8*.0002)/240;
	secpixel[14] = (8*.001)/240;
	secpixel[15] = (8*.002)/240;
	secpixel[16] = (8*.005)/240;
	secpixel[17] = (8*.01)/240;
	secpixel[18] = (8*.02)/240;
	secpixel[19] = (8*.05)/240;
	secpixel[20] = (8*.1)/240;
	secpixel[21] = (8*.2)/240;
	secpixel[22] = (8*.5)/240;
	secpixel[23] = (8*1.0)/240;
	secpixel[24] = (8*2.0)/240;
	secpixel[25] = (8*5.0)/240;
	secpixel[26] = (8*10.0)/240;
	secpixel[27] = (8*20.0)/240;
	secpixel[28] = (8*50.0)/240;


	// Point the trigger level pointer to the address containing the trigger level
	PTriggerLevel = &TriggerLevel;

	// Point the level offsets for the channels
	plevel1 = &level1;
	plevel2 = &level2;

	// Intialize Frequency Values
	for(i=0;i<MeasureAvg;i++){
		Freq1[i] = 0;
		Freq2[i] = 0;
	}

	// Enable FPU
	FPULazyStackingEnable();
	FPUEnable();

	// Enable the uDMA and interrupts
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
	IntEnable(INT_UDMAERR);
	uDMAEnable();

	// Point at the control table for uDMA
	uDMAControlBaseSet(pui8ControlTable);

	// Map Channel perhiperhal for 30
	uDMAChannelAssign(UDMA_CH30_EPI0RX);

	// Point EPISource to FIFOs of EPI
	EPISource = EPI0_BASE + EPI_O_READFIFO0;

	SetupDMA();

	// Enable GPIO and Timer peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	// Enable EPI Peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EPI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

	// Enable Control Periphs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// Enable PWM Periphs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	// Configure GPIO pins for EPI mode. Address obtained from pin_map.h
	GPIOPinConfigure(GPIO_PA6_EPI0S8);
	GPIOPinConfigure(GPIO_PC4_EPI0S7);
	GPIOPinConfigure(GPIO_PC5_EPI0S6);
	GPIOPinConfigure(GPIO_PC6_EPI0S5);
	GPIOPinConfigure(GPIO_PC7_EPI0S4);
	GPIOPinConfigure(GPIO_PG0_EPI0S11);
	GPIOPinConfigure(GPIO_PL0_EPI0S16);
	GPIOPinConfigure(GPIO_PL1_EPI0S17);
	GPIOPinConfigure(GPIO_PL2_EPI0S18);
	GPIOPinConfigure(GPIO_PL3_EPI0S19);
	GPIOPinConfigure(GPIO_PL4_EPI0S26);
	GPIOPinConfigure(GPIO_PM3_EPI0S12);
	GPIOPinConfigure(GPIO_PK0_EPI0S0);
	GPIOPinConfigure(GPIO_PK1_EPI0S1);
	GPIOPinConfigure(GPIO_PK2_EPI0S2);
	GPIOPinConfigure(GPIO_PK3_EPI0S3);
	GPIOPinConfigure(GPIO_PA7_EPI0S9);
	GPIOPinConfigure(GPIO_PM2_EPI0S13);
	GPIOPinConfigure(GPIO_PM1_EPI0S14);
	GPIOPinConfigure(GPIO_PM0_EPI0S15);
	GPIOPinConfigure(GPIO_PK7_EPI0S24);
	GPIOPinConfigure(GPIO_PK6_EPI0S25);
	GPIOPinConfigure(GPIO_PB2_EPI0S27);
	GPIOPinConfigure(GPIO_PB3_EPI0S28);

	GPIOPinTypeEPI(GPIO_PORTA_BASE, EPI_PORTA_PINS);
	GPIOPinTypeEPI(GPIO_PORTB_BASE, EPI_PORTB_PINS);
	GPIOPinTypeEPI(GPIO_PORTC_BASE, EPI_PORTC_PINS);
	GPIOPinTypeEPI(GPIO_PORTG_BASE, EPI_PORTG_PINS);
	GPIOPinTypeEPI(GPIO_PORTK_BASE, EPI_PORTK_PINS);
	GPIOPinTypeEPI(GPIO_PORTL_BASE, EPI_PORTL_PINS);
	GPIOPinTypeEPI(GPIO_PORTM_BASE, EPI_PORTM_PINS);

	// Set GPIO output and input pins for ADC control
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, FADC_CTRL_2 | FADC_CTRL_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, PADC_Output_Mode | PADC_CTRL_1);
	GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, LADC_CLK_IN);

	// Set inital controls for ADC
	GPIOPinWrite(GPIO_PORTP_BASE, PADC_Output_Mode| PADC_CTRL_1,0);
	GPIOPinWrite(GPIO_PORTF_BASE, FADC_CTRL_2 | FADC_CTRL_3, 0);

	// Set GPIO output pins for multiplexer
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, BCh1_Mult_A0);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, BCh1_Mult_A1);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, ACh2_Mult_A0);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, ACh2_Mult_A1);

	GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,0);
	GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,0);
	GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,0);
	GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,0);

	// Set GPIO output pins for DVGA
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh1_DVGA_Mode);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh2_DVGA_Mode);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh1_DVGA_D0);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, DCh1_DVGA_D1);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh1_DVGA_D2);
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, MCh1_DVGA_D3);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh2_DVGA_D0);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ECh2_DVGA_D1);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, DCh2_DVGA_D2);
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, MCh2_DVGA_D3);

	GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_Mode,0);
	GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_Mode,0);
	GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
	GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
	GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
	GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
	GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
	GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
	GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
	GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);

	// Select general purpose mode for EPI
	EPIModeSet(EPI0_BASE, EPI_MODE_GENERAL);

	// Configure GPIO pins for PWM
	GPIOPinConfigure(GPIO_PF1_M0PWM1);
	GPIOPinTypePWM(GPIO_PORTF_BASE, FADC_CLK_OUT);

	// Enable master interrupt
	IntMasterEnable();

    // Set the PWM clock to the system clock.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	// Configure PWM for count down mode and immediate updates
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

	// Set PWM period
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 4);

	// Set pulse width of PWM1 for 50% duty cycle
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 2);

	// Start timers in generator 0
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);

	// Enable Outputs
	PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

	// Configure EPI settings
	EPIDividerSet(EPI0_BASE, EPIDivide);
	EPIAddressMapSet(EPI0_BASE, EPI_ADDR_PER_SIZE_256B | EPI_ADDR_PER_BASE_A);
	EPIConfigGPModeSet(EPI0_BASE,
			EPI_GPMODE_DSIZE_32 | EPI_GPMODE_ASIZE_NONE | EPI_GPMODE_CLKPIN, 0,
			0);
	EPINonBlockingReadConfigure(EPI0_BASE, 0, EPI_NBCONFIG_SIZE_32, 0);
	EPINonBlockingReadStart(EPI0_BASE, 0, CountSize);

	EPIFIFOConfig(EPI0_BASE, EPI_FIFO_CONFIG_RX_1_2);
	EPIIntEnable(EPI0_BASE, EPI_INT_DMA_RX_DONE); //| EPI_INT_RXREQ );
	IntEnable(INT_EPI0);

}

void SetupVoltageDivision(uint8_t Scale, uint8_t Channel){


	if(Channel == 1){
		switch (Scale) {
		case 0: // 20mV/div
			pixel_divider1 = 5.461;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			break;
		case 1: // 50mV/div
			pixel_divider1 = 13.653;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			break;
		case 2: // 100mV/div
			pixel_divider1 = 21.639;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			break;
		case 3: // 200mV/div
			pixel_divider1 = 21.69;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			break;
		case 4: // 500mV/div
			pixel_divider1 = 21.586;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			break;
		case 5: // 1V/div
			pixel_divider1 = 21.586;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,ACh2_Mult_A0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			break;
		case 6: // 2V/div
			pixel_divider1 = 12.335;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,ACh2_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			break;
		case 7: // 5V/div
			//pixel_divider1 = 18.728;
			pixel_divider1 = 43.073;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,ACh2_Mult_A0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,ACh2_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			break;
		case 8: // 10V/div
			//pixel_divider1 = 27.772;
			pixel_divider1 = 43.176;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,ACh2_Mult_A0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,ACh2_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			break;
		case 9: // 20V/div
			//pixel_divider1 = 23.6887;
			pixel_divider1 = 54.484;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,ACh2_Mult_A0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,ACh2_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			break;
		case 10: // 25V/div
			//pixel_divider1 = 59.222;
			pixel_divider1 = 68.105;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,ACh2_Mult_A0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,ACh2_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			break;
		case 11: // 10V/div
			// Multiplexer
			pixel_divider1 = 118.443;
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,ACh2_Mult_A0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,ACh2_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			break;
		case 12: // 20V/div
			// Multiplexer
			pixel_divider1 = 236.887;
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,ACh2_Mult_A0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,ACh2_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			break;
		case 13: // 25V/div
			// Multiplexer
			pixel_divider1 = 296.108;
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,ACh2_Mult_A0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,ACh2_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			break;
		default:
			// Multiplexer
			pixel_divider1 = 296.108;
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,ACh2_Mult_A0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,ACh2_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			break;
		}
	}
	else if(Channel == 2){
		switch (Scale) {
		case 0: // 2mV/div
			pixel_divider2 = 5.461;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 1: // 5mV/div
			pixel_divider2 = 13.653;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 2: // 10mV/div
			pixel_divider2 = 21.639;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 3: // 20mV/div
			pixel_divider2 = 21.69;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 4: // 50mV/div
			pixel_divider2 = 21.586;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		case 5: // 100mV/div
			pixel_divider2 = 21.586;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		case 6: // 200mV/div
			pixel_divider2 = 21.586;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,BCh1_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		case 7: // 500mV/div
			//pixel_divider2 = 18.728;
			pixel_divider2 = 43.073;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,BCh1_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 8: // 1V/div
			//pixel_divider2 = 27.772;
			pixel_divider2 = 43.176;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,BCh1_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		case 9: // 2V/div
			//pixel_divider2 = 23.6887;
			pixel_divider2 = 54.484;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,BCh1_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,ECh2_DVGA_D1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		case 10: // 5V/div 2.5V/div
			//pixel_divider2 = 59.222;
			pixel_divider2 = 68.105;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,BCh1_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,ECh2_DVGA_D1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		case 11: // 10V/div
			pixel_divider2 = 118.443;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,BCh1_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,ECh2_DVGA_D1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		case 12: // 20V/div
			pixel_divider2 = 236.887;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,BCh1_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,ECh2_DVGA_D1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		case 13: // 25V/div
			pixel_divider2 = 296.108;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,BCh1_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,ECh2_DVGA_D1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		default:
			pixel_divider2 = 296.108;
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,BCh1_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,ECh2_DVGA_D1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		}

		// Testing cases
		/*switch (Scale) {
		case 0:
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0,BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1,BCh1_Mult_A1);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,ACh2_Mult_A0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1,ACh2_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,ECh2_DVGA_D1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		case 1:
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1, BCh1_Mult_A1);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0, 0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1, ACh2_Mult_A1);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		case 2:
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0, BCh1_Mult_A0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1, 0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0,ACh2_Mult_A0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1, 0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,MCh1_DVGA_D3);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,MCh2_DVGA_D3);
			break;
		case 3:
			// Multiplexer
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A0, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, BCh1_Mult_A1, 0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A0, 0);
			GPIOPinWrite(GPIO_PORTA_BASE, ACh2_Mult_A1, 0);
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,ECh2_DVGA_D1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 4:
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,ECh2_DVGA_D1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 5:
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 6:
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,ECh1_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,DCh2_DVGA_D2);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 7:
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,ECh2_DVGA_D1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 8:
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,DCh1_DVGA_D1);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,ECh2_DVGA_D1);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 9:
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,ECh1_DVGA_D0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,ECh2_DVGA_D0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		case 10:
			// DVGA
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh1_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh1_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh1_DVGA_D3,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D0,0);
			GPIOPinWrite(GPIO_PORTE_BASE, ECh2_DVGA_D1,0);
			GPIOPinWrite(GPIO_PORTD_BASE, DCh2_DVGA_D2,0);
			GPIOPinWrite(GPIO_PORTM_BASE, MCh2_DVGA_D3,0);
			break;
		default:
			break;
		}*/
	}
}

void SetupTimeDivision(uint8_t Scale){

	switch(Scale){
	case 0: // 20ns/div
		EPIDivide = 5;
		NumSkip = 0;
		break;
	case 1: // 50ns/div
		EPIDivide = 10;
		NumSkip = 0;
		break;
	case 2: // 100ns/div
		EPIDivide = 35;
		NumSkip = 0;
		break;
	case 3: // 200ns/div
		EPIDivide = 75;
		NumSkip = 0;
		break;
	case 4: // 500ns/div
		EPIDivide = 0;
		NumSkip = 0;
		break;
	case 5: // 1us/div
		EPIDivide = 1;
		NumSkip = 0;
		break;
	case 6: // 2us/div
		EPIDivide = 2;
		NumSkip = 0;
		break;
	case 7: // 5us/div
		EPIDivide = 6;
		NumSkip = 0;
		break;
	case 8: // 10us/div
		EPIDivide = 14;
		NumSkip = 0;
		break;
	case 9: // 20us/div
		EPIDivide = 8;
		NumSkip = 2;
		break;
	case 10: // 50us/div
		EPIDivide = 74;
		NumSkip = 0;
		break;
	case 11: // 100us/div
		EPIDivide = 148;
		NumSkip = 0;
		break;
	case 12: // 200us/div
		EPIDivide = 298;
		NumSkip = 0;
		break;
	case 13: // 500us/div
		EPIDivide = 748;
		NumSkip = 0;
		break;
	case 14: // 1ms/div
		EPIDivide = 1498;
		NumSkip = 0;
		break;
	case 15: // 2ms/div
		EPIDivide = 2998;
		NumSkip = 0;
		break;
	case 16: // 5ms/div
		EPIDivide = 7498;
		NumSkip = 0;
		break;
	case 17: // 10ms/div
		EPIDivide = 14998;
		NumSkip = 0;
		break;
	case 18: // 20ms/div
		EPIDivide = 29998;
		NumSkip = 0;
		break;
	case 19: // 50ms/div
		EPIDivide = 59998;
		NumSkip =0;
		break;
	case 20: // 100ms/div
		EPIDivide = 49998;
		NumSkip = 2;
		break;
	case 21: // 200ms/div
		EPIDivide = 59998;
		NumSkip = 4;
		break;
	case 22: // 500ms/div
		EPIDivide = 59998;
		NumSkip = 9;
		break;
	case 23: // 1s/div
		EPIDivide = 59998;
		NumSkip = 24;
		break;
	case 24: // 2s/div
		EPIDivide = 59998;
		NumSkip = 49;
		break;
	case 25: // 5s/div
		EPIDivide = 59998;
		NumSkip = 149;
		break;
	case 26: // 10s/div
		EPIDivide = 59998;
		NumSkip = 249;
		break;
	case 27: // 20s/div
		EPIDivide = 59998;
		NumSkip = 499;
		break;
	case 28: // 50s/div
		EPIDivide = 59998;
		NumSkip = 2499;
		break;
	default:
		EPIDivide = 2998;
		NumSkip = 0;
		break;
	}
}


// function that helps setup the trigger variables
void SetupTrigger(uint16_t Level, uint8_t Start_Position, uint8_t Mode, uint8_t Source){

	TriggerLevel = Level;
	TriggerMode = Mode; // 0 - positive edge, 1 - negative edge
	TriggerSource = Source;
	TriggerPosition = Start_Position;
	Trigger = 0;
	TriggerStart = 0;
	GoThrough = 0;
}


// function to update the voltage and frequency measurements
void UpdateMeasurements(void){
	// give inital values to all variables
	Amp1[1] = 0;
	Amp1[0] = 4097;
	Amp2[1] = 0;
	Amp2[0] = 4097;
	freqref1 = pixels[3];
	freqstart1 = 0;
	freqref2 = pixels2[3];
	freqstart2 = 0;
	for(i=0;i<TimeAvg;i++){
		t1[i] = 0;
		t2[i] = 0;
	}
	l1 = 0;
	l2 = 0;
	for(i=0;i<SERIES_LENGTH;i++){
		// determine if a min or max has been found
		if(pixels[i] < Amp1[0]){
			Amp1[0] = pixels[i];
		}
		if(pixels[i] > Amp1[1]){
			Amp1[1] = pixels[i];
		}
		if(pixels2[i] < Amp2[0]){
			Amp2[0] = pixels2[i];
		}
		if(pixels2[i] > Amp2[1]){
			Amp2[1] = pixels2[i];
		}
		// determine amount of delta time instances occur between positive triggered events
		if(TriggerMode == 0 && i > 0){
			if(pixels[i-1] <= freqref1 && pixels[i] >= freqref1 && freqstart1 == 0){
				freqstart1 = 1;
			}
			else if(pixels[i-1] <= freqref1 && pixels[i] >= freqref1 && freqstart1 == 1){
				freqstart1 = 0;
				if(l1 < TimeAvg){
					l1++;
				}
			}
			if(freqstart1 == 1){
				t1[l1] = t1[l1] + 1;
			}

			if(pixels2[i-1] <= freqref2 && pixels2[i] >= freqref2 && freqstart2 == 0 && freqstop2 == 0){
				freqstart2 = 1;
			}
			else if(pixels2[i-1] <= freqref2 && pixels2[i] >= freqref2){
				freqstart2 = 0;
				if(l2 < TimeAvg){
					l2++;
				}
			}
			if(freqstart2 == 1){
				t2[l2] = t2[l2] + 1;
			}

		}
		// determine amount of delta time instances occur between negative triggered events
		else if(TriggerMode == 1 && i > 0){
			if(pixels[i-1] >= freqref1 && pixels[i] <= freqref1 && freqstart1 == 0){
				freqstart1 = 1;
			}
			else if(pixels[i-1] >= freqref1 && pixels[i] <= freqref1 && freqstart1 == 1){
				freqstart1 = 0;
				if(l1 < TimeAvg){
					l1++;
				}
			}
			if(freqstart1 == 1){
				t1[l1] = t1[l1] + 1;
			}

			if(pixels2[i-1] >= freqref2 && pixels2[i] <= freqref2 && freqstart2 == 0){
				freqstart2 = 1;
			}
			else if(pixels2[i-1] >= freqref2 && pixels2[i] <= freqref2 && freqstart2 == 1){
				freqstart2 = 0;
				if(l2 < TimeAvg){
					l2++;
				}
			}
			if(freqstart2 == 1){
				t2[l2] = t2[l2] + 1;
			}
		}
	}
	// initialze totals and number of nonzero delta times found
	totalt1 = 0;
	totalt2 = 0;
	NumAvgt1 = 0;
	NumAvgt2 = 0;
	// determine the aomount of nonzero delta times found and add to total
	for(i=0;i<TimeAvg;i++){
		if(t1[i] != 0){
			totalt1 = totalt1 + t1[i];
			NumAvgt1++;
		}
		if(t2[i] != 0){
			totalt2 = totalt2 + t1[i];
			NumAvgt2++;
		}
	}
	// determine the average number of delta times
	t1Avg = totalt1/NumAvgt1;
	t2Avg = totalt2/NumAvgt2;
	// reinitilze totals and number of nonzero delta times found
	totalt1 = 0;
	totalt2 = 0;
	NumAvgt1 = 0;
	NumAvgt2 = 0;
	// determine the amount of delta times that were within +- 50% of the previous average and add to total
	for(i=0;i<TimeAvg;i++){
		if(t1[i] != 0 && t1[i] > t1Avg*0.5 && t1[i] < t1Avg*1.5){
			totalt1 = totalt1 + t1[i];
			NumAvgt1++;
		}
		if(t2[i] != 0&& t2[i] > t2Avg*0.5 && t2[i] < t2Avg*1.5){
			totalt2 = totalt2 + t1[i];
			NumAvgt2++;
		}
	}
	// determine the new average number of delta times
	t1Avg = totalt1/NumAvgt1;
	t2Avg = totalt2/NumAvgt2;
	// update the measured frequency if the average number of delta times is nonzero
	if(t1Avg != 0){
		Freq1[measnum] = 1000/(t1Avg*secpixel[Time]);
	}
	if(t2Avg != 0){
		Freq2[measnum] = 1000/(t2Avg*secpixel[Time]);
	}

	// Calcualte peak to peak amplitude in pixels
	Amp1[2] = Amp1[1] - Amp1[0];
	Amp2[2] = Amp2[1] - Amp2[0];

	// Calculate peak to peak amplutide in mV
	Amp1[3] = (Amp1[2]/pixel_divider1)*mvpixel[Mag1];
	Amp2[3] = (Amp2[2]/pixel_divider2)*mvpixel[Mag2];

	// increment number of measurements found
	measnum++;
	// calculate frequency to be displayed when the desired number of measurements has been found
	if(measnum == MeasureAvg){
		measnum = 0;
		// average all the frequency values found
		for(i=0;i<MeasureAvg;i++){
			if(Freq1[i] != 0){
				Frequency1Total = Frequency1Total + Freq1[i];
				NumFreqs1++;
			}
			if(Freq2[i] != 0){
				Frequency2Total = Frequency2Total + Freq2[i];
				NumFreqs2++;
			}
		}
		Frequency1 = Frequency1Total/NumFreqs1;
		Frequency2 = Frequency2Total/NumFreqs2;
		Frequency1Total = 0;
		Frequency2Total = 0;
		NumFreqs1 = 0;
		NumFreqs2 = 0;
		// Intialize Frequency Values
		for(i=0;i<MeasureAvg;i++){
			Freq1[i] = 0;
			Freq2[i] = 0;
		}

		// display frequency value using ASCII characters if channel 1 is on
		if(Ch1on == 1){
			if(Frequency1 > 1000000000){
				FreqDisplay1[0] = (Frequency1/1000000000)%10 + 48;
				FreqDisplay1[1] = 46; // '.'
				FreqDisplay1[2] = (Frequency1/1000000000)%10 + 48;
				FreqDisplay1[3] = (Frequency1/100000000)%10 + 48;
				FreqDisplay1[4] = 32; // ' '
				FreqDisplay1[5] = 77; // 'M'
				FreqDisplay1[6] = 72; // 'H'
				FreqDisplay1[7] = 122; // 'z'
				FreqDisplay1[8] = 0;
			}
			else if(Frequency1 > 100000000){
				FreqDisplay1[0] = (Frequency1/100000000)%10 + 48;
				FreqDisplay1[1] = (Frequency1/10000000)%10 + 48;
				FreqDisplay1[2] = (Frequency1/1000000)%10 + 48;
				FreqDisplay1[3] = 32; // ' '
				FreqDisplay1[4] = 107; // 'k'
				FreqDisplay1[5] = 72; // 'H'
				FreqDisplay1[6] = 122; // 'z'
				FreqDisplay1[7] = 0;
				FreqDisplay1[8] = 0;
			}
			else if(Frequency1 > 10000000){
				FreqDisplay1[0] = (Frequency1/10000000)%10 + 48;
				FreqDisplay1[1] = (Frequency1/1000000)%10 + 48;
				FreqDisplay1[2] = 46; // '.'
				FreqDisplay1[3] = (Frequency1/100000)%10 + 48;
				FreqDisplay1[4] = 32; // ' ';
				FreqDisplay1[5] = 107; // 'k'
				FreqDisplay1[6] = 72; // 'H'
				FreqDisplay1[7] = 122; // 'z'
				FreqDisplay1[8] = 0;
			}
			else if(Frequency1 > 1000000){
				FreqDisplay1[0] = (Frequency1/1000000)%10 + 48;
				FreqDisplay1[1] = 46; // '.'
				FreqDisplay1[2] = (Frequency1/100000)%10 + 48;
				FreqDisplay1[3] = (Frequency1/10000)%10 + 48;
				FreqDisplay1[4] = 32; // ' ';
				FreqDisplay1[5] = 107; // 'k'
				FreqDisplay1[6] = 72; // 'H'
				FreqDisplay1[7] = 122; // 'z'
				FreqDisplay1[8] = 0;
			}
			else if(Frequency1 > 100000){
				FreqDisplay1[0] = (Frequency1/100000)%10 + 48;
				FreqDisplay1[1] = (Frequency1/10000)%10 + 48;
				FreqDisplay1[2] = (Frequency1/1000)%10 + 48;
				FreqDisplay1[3] = 32; // ' ';
				FreqDisplay1[4] = 72; // 'H'
				FreqDisplay1[5] = 122; // 'z'
				FreqDisplay1[6] = 0;
				FreqDisplay1[7] = 0;
				FreqDisplay1[8] = 0;
			}
			else if(Frequency1 > 10000){
				FreqDisplay1[0] = (Frequency1/10000)%10 + 48;
				FreqDisplay1[1] = (Frequency1/1000)%10 + 48;
				FreqDisplay1[2] = 46; // '.'
				FreqDisplay1[3] = (Frequency1/100)%10 + 48;
				FreqDisplay1[4] = 32; // ' ';
				FreqDisplay1[5] = 72; // 'H'
				FreqDisplay1[6] = 122; // 'z'
				FreqDisplay1[7] = 0;
				FreqDisplay1[8] = 0;
			}
			else if(Frequency1 > 1000){
				FreqDisplay1[0] = (Frequency1/1000)%10 + 48;
				FreqDisplay1[1] = 46; // '.'
				FreqDisplay1[2] = (Frequency1/100)%10 + 48;
				FreqDisplay1[3] = (Frequency1/10)%10 + 48;
				FreqDisplay1[4] = 32; // ' ';
				FreqDisplay1[5] = 72; // 'H'
				FreqDisplay1[6] = 122; // 'z'
				FreqDisplay1[7] = 0;
				FreqDisplay1[8] = 0;
			}
			else{
				FreqDisplay1[0] = (Frequency1/100)%10 + 48;
				FreqDisplay1[1] = (Frequency1/10)%10 + 48;
				FreqDisplay1[2] = Frequency1%10 + 48;
				FreqDisplay1[3] = 32; // ' ';
				FreqDisplay1[4] = 109; // 'm'
				FreqDisplay1[5] = 72; // 'H'
				FreqDisplay1[6] = 122; // 'z'
				FreqDisplay1[7] = 0;
				FreqDisplay1[8] = 0;
				// create space if leading digit is 0
				if(FreqDisplay1[0] == 48){
					FreqDisplay1[0] = 32; // ' ';
					if(FreqDisplay1[1] == 48){
						FreqDisplay1[1] = 32; // ' ';
					}
				}
			}
		}
		// display 'Hz' if channel 1 not on
		else{
			FreqDisplay1[0] = 72; // 'H'
			FreqDisplay1[1] = 122; // 'z'
			FreqDisplay1[2] = 0;
		}

		// display frequency value using ASCII characters if channel 2 is on
		if(Ch2on == 1){
			if(Frequency2 > 1000000000){
				FreqDisplay2[0] = (Frequency2/1000000000)%10 + 48;
				FreqDisplay2[1] = 46; // '.'
				FreqDisplay2[2] = (Frequency2/1000000000)%10 + 48;
				FreqDisplay2[3] = (Frequency2/100000000)%10 + 48;
				FreqDisplay2[4] = 32; // ' ';
				FreqDisplay2[5] = 77; // 'M';
				FreqDisplay2[6] = 72; // 'H'
				FreqDisplay2[7] = 122; // 'z'
				FreqDisplay2[8] = 0;
			}
			else if(Frequency2 > 100000000){
				FreqDisplay2[0] = (Frequency2/100000000)%10 + 48;
				FreqDisplay2[1] = (Frequency2/10000000)%10 + 48;
				FreqDisplay2[2] = (Frequency2/1000000)%10 + 48;
				FreqDisplay2[3] = 32; // ' ';
				FreqDisplay2[4] = 107; // 'k'
				FreqDisplay2[5] = 72; // 'H'
				FreqDisplay2[6] = 122; // 'z'
				FreqDisplay2[7] = 0;
				FreqDisplay2[8] = 0;
			}
			else if(Frequency2 > 10000000){
				FreqDisplay2[0] = (Frequency2/10000000)%10 + 48;
				FreqDisplay2[1] = (Frequency2/1000000)%10 + 48;
				FreqDisplay2[2] = 46; // '.'
				FreqDisplay2[3] = (Frequency2/100000)%10 + 48;
				FreqDisplay2[4] = 32; // ' ';
				FreqDisplay2[5] = 107; // 'k'
				FreqDisplay2[6] = 72; // 'H'
				FreqDisplay2[7] = 122; // 'z'
				FreqDisplay2[8] = 0;
			}
			else if(Frequency2 > 1000000){
				FreqDisplay2[0] = (Frequency2/1000000)%10 + 48;
				FreqDisplay2[1] = 46; // '.'
				FreqDisplay2[2] = (Frequency2/100000)%10 + 48;
				FreqDisplay2[3] = (Frequency2/10000)%10 + 48;
				FreqDisplay2[4] = 32; // ' ';
				FreqDisplay2[5] = 107; // 'k'
				FreqDisplay2[6] = 72; // 'H'
				FreqDisplay2[7] = 122; // 'z'
				FreqDisplay2[8] = 0;
			}
			else if(Frequency2 > 100000){
				FreqDisplay2[0] = (Frequency2/100000)%10 + 48;
				FreqDisplay2[1] = (Frequency2/10000)%10 + 48;
				FreqDisplay2[2] = (Frequency2/1000)%10 + 48;
				FreqDisplay2[3] = 32; // ' ';
				FreqDisplay2[4] = 72; // 'H'
				FreqDisplay2[5] = 122; // 'z'
				FreqDisplay2[6] = 0;
				FreqDisplay2[7] = 0;
				FreqDisplay2[8] = 0;
			}
			else if(Frequency2 > 10000){
				FreqDisplay2[0] = (Frequency2/10000)%10 + 48;
				FreqDisplay2[1] = (Frequency2/1000)%10 + 48;
				FreqDisplay2[2] = 46; // '.'
				FreqDisplay2[3] = (Frequency2/100)%10 + 48;
				FreqDisplay2[4] = 32; // ' ';
				FreqDisplay2[5] = 72; // 'H'
				FreqDisplay2[6] = 122; // 'z'
				FreqDisplay2[7] = 0;
				FreqDisplay2[8] = 0;
			}
			else if(Frequency2 > 1000){
				FreqDisplay2[0] = (Frequency2/1000)%10 + 48;
				FreqDisplay2[1] = 46; // '.'
				FreqDisplay2[2] = (Frequency2/100)%10 + 48;
				FreqDisplay2[3] = (Frequency2/10)%10 + 48;
				FreqDisplay2[4] = 32; // ' ';
				FreqDisplay2[5] = 72; // 'H'
				FreqDisplay2[6] = 122; // 'z'
				FreqDisplay2[7] = 0;
				FreqDisplay2[8] = 0;
			}
			else{
				FreqDisplay2[0] = (Frequency2/100)%10 + 48;
				FreqDisplay2[1] = (Frequency2/10)%10 + 48;
				FreqDisplay2[2] = Frequency2%10 + 48;
				FreqDisplay2[3] = 32; // ' ';
				FreqDisplay2[4] = 109; // 'm'
				FreqDisplay2[5] = 72; // 'H'
				FreqDisplay2[6] = 122; // 'z'
				FreqDisplay2[7] = 0;
				FreqDisplay2[8] = 0;
				// create space if leading with 0
				if(FreqDisplay2[0] == 48){
					FreqDisplay2[0] = 32; // ' ';
					if(FreqDisplay2[1] == 48){
						FreqDisplay2[1] = 32; // ' ';
					}
				}
			}
		}
		// display 'Hz' if channel 2 not on
		else{
			FreqDisplay2[0] = 72; // 'H'
			FreqDisplay2[1] = 122; // 'z'
			FreqDisplay2[2] = 0;
		}

		PushButtonTextSet(&g_psBotButtons[0], FreqDisplay1);
		PushButtonTextSet(&g_psBotButtons[1], FreqDisplay2);
	}

	// display voltage measurement for channel 1 with ASCII characters
	if(Ch1on == 1){
		if(Amp1[3] < 1000){
			MagDisplay1[0] = (Amp1[3]/100)%10 + 48;
			MagDisplay1[1] = (Amp1[3]/10)%10 + 48;
			MagDisplay1[2] = Amp1[3]%10 + 48;
			MagDisplay1[3] = 32; // ' ';
			MagDisplay1[4] = 109; // 'm'
			MagDisplay1[5] = 86; // 'V'
			MagDisplay1[6] = 0;
			// create spaces for leading 0
			if(MagDisplay1[0] == 48){
				MagDisplay1[0] = 32; // ' ';
				if(MagDisplay1[1] == 48){
					MagDisplay1[1] = 32; // ' ';
				}
			}
		}
		else{
			MagDisplay1[0] = (Amp1[3]/10000)%10 + 48;
			MagDisplay1[1] = (Amp1[3]/1000)%10 + 48;
			MagDisplay1[2] = 46; // '.'
			MagDisplay1[3] = (Amp1[3]/100)%10 + 48;
			MagDisplay1[4] = (Amp1[3]/10)%10 + 48;
			MagDisplay1[5] = 32; // ' ';
			MagDisplay1[6] = 86; // 'V'
			// create spaces for leading 0
			if(MagDisplay1[0] == 48){
				MagDisplay1[0] = 32; // ' ';
				if(MagDisplay1[1] == 48){
					MagDisplay1[1] = 32; // ' ';
				}
			}
		}
	}
	// display 'V' if channel 1 not on
	else{
		MagDisplay1[0] = 86; // 'V'
		MagDisplay1[1] = 0;
	}
	// display voltage measurement for channel 2 with ASCII characters
	if(Ch2on == 1){
		if(Amp2[3] < 1000){
			MagDisplay2[0] = (Amp2[3]/100)%10 + 48;
			MagDisplay2[1] = (Amp2[3]/10)%10 + 48;
			MagDisplay2[2] = Amp2[3]%10 + 48;
			MagDisplay2[3] = 32; // ' ';
			MagDisplay2[4] = 109; // 'm'
			MagDisplay2[5] = 86; // 'V'
			MagDisplay2[6] = 0;
			// create spaces of leading 0
			if(MagDisplay2[0] == 48){
				MagDisplay2[0] = 32; // ' ';
				if(MagDisplay2[1] == 48){
					MagDisplay2[1] = 32; // ' ';
				}
			}
		}
		else{
			MagDisplay2[0] = (Amp2[3]/10000)%10 + 48;
			MagDisplay2[1] = (Amp2[3]/1000)%10 + 48;
			MagDisplay2[2] = 46; // '.'
			MagDisplay2[3] = (Amp2[3]/100)%10 + 48;
			MagDisplay2[4] = (Amp2[3]/10)%10 + 48;
			MagDisplay2[5] = 32; // ' ';
			MagDisplay2[6] = 86; // 'V'
			// create spaces for leading 0
			if(MagDisplay2[0] == 48){
				MagDisplay2[0] = 32; // ' ';
				if(MagDisplay2[1] == 48){
					MagDisplay2[1] = 32; // ' ';
				}
			}
		}
	}
	// display 'V' if channel 2 not on
	else{
		MagDisplay2[0] = 86; // 'V'
		MagDisplay2[1] = 0;
	}
	PushButtonTextSet(&g_psBotButtons[2], MagDisplay1);
	PushButtonTextSet(&g_psBotButtons[3], MagDisplay2);
}

void CalibrateOffset(){
	uint32_t CalibrateTotal1 = 0, CalibrateTotal2 = 0;
	uint16_t CalibrateAvg1 = 0, CalibrateAvg2 = 0;

	// determine average value of pixels
	for(i=0;i<SERIES_LENGTH;i++){
		CalibrateTotal1 = CalibrateTotal1 + pixels[i];
		CalibrateTotal2 = CalibrateTotal2 + pixels2[i];
	}

	CalibrateAvg1 = CalibrateTotal1/SERIES_LENGTH;
	CalibrateAvg2 = CalibrateTotal2/SERIES_LENGTH;

	// compensate midlevel for channels 1 and 2 based on average value of pixels so that the
	// average and midlevel are equal
	level1 = CalibrateAvg1/pixel_divider1 - 2048/pixel_divider1 + desiredlevel1;
	level2 = CalibrateAvg2/pixel_divider2 - 2048/pixel_divider2 + desiredlevel2;
	midlevel1 = 2048/pixel_divider1 + level1;
	midlevel2 = 2048/pixel_divider2 + level2;
}
