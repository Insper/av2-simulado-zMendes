#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_uart_serial.h"
#include "maxTouch/maxTouch.h"

#include "fonts/tfont.h"
#include "fonts/sourcecodepro_28.h"
#include "fonts/calibri_36.h"
#include "fonts/arial_72.h"

#include "image.h"
#include "arm_math.h"

/************************************************************************/
/* RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LCD_STACK_SIZE            (6*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_FIR_STACK_SIZE            (6*1024/sizeof(portSTACK_TYPE))
#define TASK_FIR_STACK_PRIORITY        (tskIDLE_PRIORITY)

typedef struct {
  uint x;
  uint y;
} touchData;

typedef struct {
  uint value;
} adcData;

typedef struct {
  uint raw;
  uint filtrado;
} t_plot;

QueueHandle_t xQueueTouch;
QueueHandle_t xQueueADC, xQueuePlot;
SemaphoreHandle_t xSemaphore;

/************************************************************************/
/* LCD + TOUCH                                                          */
/************************************************************************/
#define MAX_ENTRIES        10
#define BUT_SIZE 100
#define BUT_SPACE 15

struct ili9488_opt_t g_ili9488_display_opt;


typedef struct {
  uint8_t status;
  uint32_t width;
  uint32_t height;
  uint32_t border;
  uint32_t colorOn;
  uint32_t colorOff;
  uint32_t x;
  uint32_t y;
} t_but;

t_but but0 = {.width = BUT_SIZE, .height = BUT_SIZE, .border = 0,
  .colorOn = COLOR_TOMATO, .colorOff = COLOR_GRAY,
  .x = BUT_SIZE/2 + BUT_SPACE, .y = ILI9488_LCD_HEIGHT - BUT_SIZE,
.status = 1};

t_but but1 = {.width = BUT_SIZE, .height = BUT_SIZE, .border = 0,
  .colorOn = COLOR_VIOLET, .colorOff = COLOR_BLACK,
  .x = BUT_SIZE/2 + BUT_SIZE+ 2*BUT_SPACE, .y = ILI9488_LCD_HEIGHT - BUT_SIZE,
.status = 1};

t_but but2 = {.width = BUT_SIZE, .height = BUT_SIZE, .border = 0,
  .colorOn = COLOR_MAGENTA, .colorOff = COLOR_DARKVIOLET,
  .x = BUT_SIZE/2 + 2*BUT_SIZE+ 3*BUT_SPACE, .y = ILI9488_LCD_HEIGHT - BUT_SIZE,
.status = 1};

t_but but3 = {.width = BUT_SIZE, .height = BUT_SIZE, .border = 0,
  .colorOn = COLOR_GREEN, .colorOff = COLOR_DARKGREEN,
  .x = BUT_SIZE/2 + 3*BUT_SIZE+ 4*BUT_SPACE, .y = ILI9488_LCD_HEIGHT - BUT_SIZE,
.status = 1};

/************************************************************************/
/* afec e fir                                                           */
/************************************************************************/
#define AFEC_POT AFEC1
#define AFEC_POT_ID ID_AFEC1
#define AFEC_POT_CHANNEL 6 // Canal do pino PC31

#define NUM_TAPS   8       // ordem do filtro (quantos coefientes)
#define BLOCK_SIZE 1       // se será processado por blocos, no caso não.

const float32_t firCoeffs32[NUM_TAPS] ={0.12269166637219883,
  0.12466396327768503,
  0.1259892807712678,
  0.12665508957884833,
  0.12665508957884833,
  0.1259892807712678,
  0.12466396327768503,
0.12269166637219883};

/************************************************************************/
/* RTOS hooks                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName)
{
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  /* If the parameters have been corrupted then inspect pxCurrentTCB to
  * identify which task has overflowed its stack.
  */
  for (;;) {
  }
}

extern void vApplicationIdleHook(void)
{
}

extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_lcd(void){
  /* Initialize display parameter */
  g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
  g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
  g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
  g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

  /* Initialize LCD */
  ili9488_init(&g_ili9488_display_opt);
  ili9488_set_display_direction(PORTRAIT);
}

void draw_screen(void) {
  ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
  ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT);
}

void clear_plot(void) {
  ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
  ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1,  ILI9488_LCD_HEIGHT - BUT_SIZE);
}

void erase_plot(void) {
  ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
  ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-BUT_SIZE-BUT_SIZE/2);
}

uint32_t convert_axis_system_x(uint32_t touch_y) {
  // entrada: 4096 - 0 (sistema de coordenadas atual)
  // saida: 0 - 320
  return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
  // entrada: 0 - 4096 (sistema de coordenadas atual)
  // saida: 0 - 320
  return ILI9488_LCD_HEIGHT*touch_x/4096;
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
  char *p = text;
  while(*p != NULL) {
    char letter = *p;
    int letter_offset = letter - font->start_char;
    if(letter <= font->end_char) {
      tChar *current_char = font->chars + letter_offset;
      ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
      x += current_char->image->width + spacing;
    }
    p++;
  }
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
  uint32_t ul_div;
  uint32_t ul_tcclks;
  uint32_t ul_sysclk = sysclk_get_cpu_hz();

  pmc_enable_periph_clk(ID_TC);

  tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
  tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
  tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

  NVIC_SetPriority((IRQn_Type) ID_TC, 4);
  NVIC_EnableIRQ((IRQn_Type) ID_TC);
  tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

  tc_start(TC, TC_CHANNEL);
}

static void AFEC_pot_Callback(void){
  adcData adc;
  adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback){
  /*************************************
  * Ativa e configura AFEC
  *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel,	callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}

void draw_button(t_but but){
  uint32_t color;
  if (but.status > 0)
  color = but.colorOn;
  else
  color = but.colorOff;

  ili9488_set_foreground_color(COLOR_CONVERT(color));
  ili9488_draw_filled_rectangle(but.x-but.width/2, but.y-but.height/2,   but.x+but.width/2, but.y+but.height/2);
  ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
  //ili9488_draw_string(but.x-but.width/4, but.y-10, but.name);
}

int process_touch(t_but botoes[], touchData touch, uint32_t n){
  for(int i = 0; i < n; i++) {
    if(touch.x >= botoes[i].x -botoes[i].width/2  && touch.x <= botoes[i].x + botoes[i].width /2){
      if(touch.y >= botoes[i].y - botoes[i].height/2 && touch.y <= botoes[i].y + botoes[i].height/2  ){
        return i;
      }
    }
  }
  return -1;
}

/************************************************************************/
/* IRQ  handlers                                                        */
/************************************************************************/

void TC1_Handler(void){
  volatile uint32_t ul_dummy;

  ul_dummy = tc_get_status(TC0, 1);

  /* Avoid compiler warning */
  UNUSED(ul_dummy);

  /* Selecina canal e inicializa conversão */
  afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
  afec_start_software_conversion(AFEC_POT);
}

void mxt_handler(struct mxt_device *device, uint *x, uint *y)
{
  /* USART tx buffer initialized to 0 */
  uint8_t i = 0; /* Iterator */

  /* Temporary touch event data struct */
  struct mxt_touch_event touch_event;

  /* first touch only */
  uint first = 0;

  /* Collect touch events and put the data in a string,
  * maximum 2 events at the time */
  do {

    /* Read next next touch event in the queue, discard if read fails */
    if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
      continue;
    }

    /************************************************************************/
    /* Envia dados via fila RTOS                                            */
    /************************************************************************/
    if(first == 0 ){
      *x = ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_event.x/4096;
      *y = ILI9488_LCD_HEIGHT - ILI9488_LCD_HEIGHT*touch_event.y/4096;
      first = 1;
    }

    i++;

    /* Check if there is still messages in the queue and
    * if we have reached the maximum numbers of events */
  } while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));
}

/************************************************************************/
/* tasks                                                                */
/************************************************************************/

void task_mxt(void){

  struct mxt_device device; /* Device data container */
  mxt_init(&device);       	/* Initialize the mXT touch device */
  touchData touch;          /* touch queue data type*/

  while (true) {
    /* Check for any pending messages and run message handler if any
    * message is found in the queue */
    if (mxt_is_message_pending(&device)) {
      mxt_handler(&device, &touch.x, &touch.y);
      xQueueSend( xQueueTouch, &touch, 0);           /* send mesage to queue */
      vTaskDelay(200);

      // limpa touch
      while (mxt_is_message_pending(&device)){
        mxt_handler(&device, NULL, NULL);
        vTaskDelay(50);
      }
    }

    vTaskDelay(300);
  }
}

void task_lcd(void){
  xQueueTouch = xQueueCreate( 10, sizeof( touchData ) );
  xQueuePlot = xQueueCreate( 10, sizeof( t_plot ) );

  // inicializa LCD e pinta de branco
  configure_lcd();
  draw_screen();

  // strut local para armazenar msg enviada pela task do mxt
  touchData touch;
  t_plot plot;
  int x = 0;
  
  uint scale = 16;
  uint f_record = 0;
  
  t_but botoes[] = {but0, but1, but2, but3};

  botoes[1].status = 0;
  botoes[2].status = 0;

  for(uint i = 0; i < sizeof(botoes)/sizeof(t_but); i++){
    draw_button(botoes[i]);
  }
  
  while (true) {
    if (xQueueReceive( xQueueTouch, &(touch), ( TickType_t )  0 / portTICK_PERIOD_MS)) {

      int b = process_touch(botoes, touch, sizeof(botoes)/sizeof(t_but));
      if(b >= 0){
        botoes[b].status = ! botoes[b].status;
        draw_button(botoes[b]);
      }
      
      // apaga record
      if (botoes[0].status == 0){
        ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
        ili9488_draw_filled_rectangle(0,0,Image.width, Image.height);
        f_record = 0;
      }
      
      if (botoes[1].status == 1){
        scale--;
        botoes[1].status = 0;
      }
      
      if (botoes[2].status == 1){
        scale++;
        botoes[2].status = 0;
      }

      printf("Scale: %x\n", scale);
    }

    if (xQueueReceive( xQueuePlot, &(plot), ( TickType_t )  100 / portTICK_PERIOD_MS)) {

      if(botoes[0].status){
        if(x > ILI9488_LCD_WIDTH){
          erase_plot();  x = 0;
          } else {
          x = x + 6;
        }
        
        // desenha record
        if(f_record == 0){
          ili9488_draw_pixmap(0, 0, Image.width, Image.height, Image.data);
          f_record = 1;
        }
        
        // o gráfico passa por cima dos botoes!
        // teria que limitar a area que ele pode plotar.
        if(botoes[3].status){
          ili9488_set_foreground_color(COLOR_CONVERT(COLOR_RED));
          ili9488_draw_filled_circle(x, ILI9488_LCD_HEIGHT - plot.filtrado / scale, 4 );
        }
        else{
          ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
          ili9488_draw_filled_circle(x, ILI9488_LCD_HEIGHT - plot.raw / scale, 2 );
        }
      }
    }
  }
}

void task_adc(void){

  adcData adc;
  t_plot plot;

  xQueueADC  = xQueueCreate( 200, sizeof( adcData ) );

  if (xSemaphore == NULL)
  printf("falha em criar o semaforo \n");

  // configura ADC e TC para controlar a leitura
  config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);
  TC_init(TC0, ID_TC1, 1, 100);


  /* Cria buffers para filtragem e faz a inicializacao do filtro. */
  float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
  float32_t inputF32[BLOCK_SIZE + NUM_TAPS - 1];
  float32_t outputF32[BLOCK_SIZE + NUM_TAPS - 1];
  arm_fir_instance_f32 S;
  arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], BLOCK_SIZE);

  uint i = 0;

  while(1){
    if (xQueueReceive( xQueueADC, &(adc), 100)) {
      if(i <= NUM_TAPS){
        inputF32[i++] = (float) adc.value;
        } else{
        arm_fir_f32(&S, &inputF32[0], &outputF32[0], BLOCK_SIZE);
        plot.raw = (int) inputF32[0];
        plot.filtrado = (int) outputF32[0];
        xQueueSend(xQueuePlot, &plot, 0);
        i = 0;
      }
    }
  }
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{
  /* Initialize the USART configuration struct */
  const usart_serial_options_t usart_serial_options = {
    .baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
    .charlength   = USART_SERIAL_CHAR_LENGTH,
    .paritytype   = USART_SERIAL_PARITY,
    .stopbits     = USART_SERIAL_STOP_BIT
  };

  sysclk_init(); /* Initialize system clocks */
  board_init();  /* Initialize board */

  /* Initialize stdio on USART */
  stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);

  /* Create task to handler touch */
  if (xTaskCreate(task_mxt, "mxt", TASK_MXT_STACK_SIZE, NULL, TASK_MXT_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  }

  /* Create task to handler LCD */
  if (xTaskCreate(task_lcd, "lcd", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  }

  if (xTaskCreate(task_adc, "adc", TASK_FIR_STACK_SIZE, NULL, TASK_FIR_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test FIR task\r\n");
  }

  /* Start the scheduler. */
  vTaskStartScheduler();

  while(1){

  }


  return 0;
}
