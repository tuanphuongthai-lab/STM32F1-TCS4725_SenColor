#include <Arduino.h>
#include <libmaple/rcc.h>
#include <libmaple/gpio.h>
#include <libmaple/timer.h>
#include <libmaple/exti.h>
#include <libmaple/nvic.h>
#include <string.h>


/* ================================================================
   CODE BY tuan Phuong
   UART MỀM + GY-33 - KHỚP VỚI GIAO DIỆN C#
   ---------------------------------------------------------------
   UART MỀM (giao tiếp máy tính):
   • PA0 = RX, PA1 = TX @ 9600 baud
   • Protocol: @ ... &
   
   UART CỨNG (GY-33):
   • Serial2: PA2 = TX, PA3 = RX @ 9600 baud
   
   LED OUTPUTS:
   • PB12 = RED, PB13 = GREEN, PB14 = BLUE, PA4 = YELLOW
   
   KÝ TỰ GỬI:
   • 'O' = Đo nền thành công
   • 'F' = Đo nền thất bại
   • 'R','G','B','Y','N' = Màu phát hiện
   ================================================================ */

// ==================== UART MỀM CONFIG ====================
#define SOFT_TX_PIN PA1
#define SOFT_RX_PIN PA0
#define BAUD_RATE   9600
#define BIT_US      (1000000UL / BAUD_RATE)

// Buffer sizes
#define TX_QUEUE_SIZE 2048
#define RX_BUFFER_SIZE 2048
#define FRAME_MAX_LEN 2048

// ==================== LED PINS ====================
#define LED_RED    PB12
#define LED_GREEN  PB13
#define LED_BLUE   PB14
#define LED_YELLOW PA4

// ==================== GY-33 SETTINGS ====================
// (PA2=TX, PA3=RX)
uint8_t cmd_disable_auto[3] = {0xA5,0x00,0xA5};
uint8_t cmd_read_rgbc[3]    = {0xA5,0x51,0xF6};
uint8_t cmd_led_on[3]       =  {0XA5,0x60,0x05};
uint8_t gy33_rx_buf[13];

uint16_t R_base = 0, G_base = 0, B_base = 0;
float rN_filt = 0.0f, gN_filt = 0.0f, bN_filt = 0.0f;
const float alpha = 0.8f; // Công thức: output_mới = output_cũ * (1-0.8) + input * 0.8
char current_color = 'N';
uint32_t lastColorReadTime = 0; // Thời điểm đọc màu lần cuối (milliseconds)
bool baselineMeasured = false;  // Flag = đã đo nền chưa? (false = chưa)

// ==================== UART MỀM VARIABLES ====================
volatile uint8_t txQueue[TX_QUEUE_SIZE];  // Hàng đợi gửi = mảng 2048 bytes
volatile uint16_t txHead = 0;
volatile uint16_t txTail = 0;
volatile bool txBusy = false;  // Flag = đang gửi byte? (true = đang gửi)
volatile uint8_t txByte = 0;
volatile uint8_t txBitIndex = 0;  // Chỉ số bit hiện tại (0-9)

volatile uint8_t rxBuffer[RX_BUFFER_SIZE];  // Bộ đệm nhận = mảng 2048 bytes
volatile uint16_t rxHead = 0;   // Con trỏ ghi dữ liệu nhận
volatile uint16_t rxTail = 0;  // Con trỏ đọc dữ liệu nhận
volatile uint8_t rxByte = 0;   // Byte đang được nhận 
volatile uint8_t rxBitIndex = 0;  // Chỉ số bit hiện tại (0-8, 9 = hoàn tất)
volatile bool rxActive = false;   // Flag = đang nhận byte? (true = đang nhận)
// Bộ đệm frame = mảng 2048 bytes
// Lưu toàn bộ frame từ @ đến &
char frameBuffer[FRAME_MAX_LEN];
uint16_t frameIndex = 0;    // Vị trí hiện tại trong frameBuffer
bool frameComplete = false; // Flag = frame hoàn tất? (true = có dấu &)


// ==================== TÍNH KÍCH THƯỚC HÀNG ĐỢI ============================

inline uint16_t txQueueAvailable() {
  return ((TX_QUEUE_SIZE + txHead - txTail) % TX_QUEUE_SIZE);
}

inline uint16_t txQueueFree() {
  return (TX_QUEUE_SIZE - 1 - txQueueAvailable());
}

inline bool txQueuePush(uint8_t byte) {
  uint16_t next = (txHead + 1) % TX_QUEUE_SIZE;
  if (next == txTail) return false;
  
  noInterrupts();
  txQueue[txHead] = byte;
  txHead = next;
  interrupts();
  return true;
}

inline bool rxBufferPop(uint8_t* byte) {
  noInterrupts();
  if (rxHead == rxTail) {
    interrupts();
    return false;
  }
  *byte = rxBuffer[rxTail];
  rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
  interrupts();
  return true;
}


// ==================== GPIO SETUP ================================

void setupGPIO() {
  rcc_clk_enable(RCC_GPIOA);
  rcc_clk_enable(RCC_GPIOB);
  rcc_clk_enable(RCC_AFIO);
  
  // UART mềm TX (PA1) - output, idle high
  gpio_set_mode(GPIOA, 1, GPIO_OUTPUT_PP); // Push-Pull = có thể xuất cao (1) và thấp (0)
  gpio_write_bit(GPIOA, 1, 1);
  
  // UART mềm RX (PA0) - input with pull-up
  gpio_set_mode(GPIOA, 0, GPIO_INPUT_PU); // bthg la 1 co tin hieu keo xuong 0
  
  // LED pins - output, off initially
  gpio_set_mode(GPIOB, 12, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 13, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 14, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOA, 4, GPIO_OUTPUT_PP);
  
  // ===== TẮT TẤT CẢ LED (KHỞI TẠO) =====
  gpio_write_bit(GPIOB, 12, 0);
  gpio_write_bit(GPIOB, 13, 0);
  gpio_write_bit(GPIOB, 14, 0);
  gpio_write_bit(GPIOA, 4, 0);
}


// ==================== TIMER2 - TX GỬI UART ===============================

extern "C" void TIM2_IRQHandler(void) {
  if (!(TIMER2->regs.gen->SR & TIMER_SR_UIF)) return;
  TIMER2->regs.gen->SR &= ~TIMER_SR_UIF;
  
  if (!txBusy) {
    if (txHead != txTail) {
      txByte = txQueue[txTail];
      txTail = (txTail + 1) % TX_QUEUE_SIZE;
      txBitIndex = 0;
      txBusy = true;
    }
    return;
  }
    // ===== GỬI CÁC BIT =====
  if (txBitIndex == 0) {
    gpio_write_bit(GPIOA, 1, 0);
  } 
  else if (txBitIndex <= 8) {  // Bit 1-8 = data bits
    gpio_write_bit(GPIOA, 1, (txByte & 0x01)); // & 0x01 = lấy bit thấp nhất (LSB),Ghi bit này lên PA1
    txByte >>= 1; // dich phải cho bit ke tiep
  }
  else if (txBitIndex == 9) {
    gpio_write_bit(GPIOA, 1, 1);
  }
  else {              // Bit 10+ = byte hoàn tất
    txBusy = false;  // Đánh dấu = không còn gửi
    return;              // Thoát, lần interrupt tiếp theo sẽ lấy byte mới
  }
  
  txBitIndex++;
}

void setupTimer2() {
  rcc_clk_enable(RCC_TIMER2);
  
  timer_pause(TIMER2);
  timer_set_prescaler(TIMER2, 72 - 1);
  timer_set_reload(TIMER2, BIT_US - 1);
  timer_attach_interrupt(TIMER2, TIMER_UPDATE_INTERRUPT, TIM2_IRQHandler);
  
  nvic_irq_set_priority(NVIC_TIMER2, 0);
  timer_resume(TIMER2);
}


// ==================== TIMER3 - RX ===============================

extern "C" void TIM3_IRQHandler(void) {
  if (!(TIMER3->regs.gen->SR & TIMER_SR_UIF)) return;
  TIMER3->regs.gen->SR &= ~TIMER_SR_UIF;
  
  if (!rxActive) {     //neu ko nhan
    timer_pause(TIMER3);   // dung timer3
    return;
  }
  
  rxBitIndex++;   // Tăng chỉ số bit (0→1→2→...→9)
  
  if (rxBitIndex == 1) {   // da qua start bit 
    timer_set_reload(TIMER3, BIT_US - 1);   // Đặt lại reload về 104µs
  }
  
  if (rxBitIndex <= 8) {
    if (gpio_read_bit(GPIOA, 0)) {
      rxByte |= (1 << (rxBitIndex - 1));
    }
  }
  else if (rxBitIndex == 9) {
    uint16_t next = (rxHead + 1) % RX_BUFFER_SIZE;
    if (next != rxTail) {
      rxBuffer[rxHead] = rxByte;
      rxHead = next;
    }
    
    rxActive = false;
    timer_pause(TIMER3);
  }
}

void setupTimer3() {
  rcc_clk_enable(RCC_TIMER3);
  
  timer_pause(TIMER3);
  timer_set_prescaler(TIMER3, 72 - 1);
  timer_set_reload(TIMER3, BIT_US - 1);
  timer_attach_interrupt(TIMER3, TIMER_UPDATE_INTERRUPT, TIM3_IRQHandler);
  
  nvic_irq_set_priority(NVIC_TIMER3, 1);
}


// ==================== EXTI0 - phat hien START BIT =========================

void exti0_handler(void) {  // ISR của EXTI0 - gọi khi PA0 có cạnh xuống
  if (rxActive) return;
  
  rxActive = true;
  rxBitIndex = 0;
  rxByte = 0;
  TIMER3->regs.gen->CNT = 0;
  timer_set_reload(TIMER3, (BIT_US * 3) / 2 - 1);
  timer_resume(TIMER3);
}

void setupEXTI() {
  exti_attach_interrupt(AFIO_EXTI_0, AFIO_EXTI_PA, exti0_handler, EXTI_FALLING);
  nvic_irq_enable(NVIC_EXTI0);
}


// ==================== UART MỀM API ==============================

void softUART_SendByte(uint8_t b) { // Gửi 1 byte qua UART mềm
  while (!txQueuePush(b));   // Lặp cho tới khi byte được đưa vào hàng đợi
}

void softUART_SendBuffer(const uint8_t* data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    while (!txQueuePush(data[i]));
  }
}


// ==================== GY-33 FUNCTIONS ===========================

void sendCommandGY33(uint8_t *cmd, uint8_t len) {
  Serial2.write(cmd, len);
  delay(50);
}

bool readGY33(uint16_t *R, uint16_t *G, uint16_t *B, uint16_t *C) {
  sendCommandGY33(cmd_read_rgbc, 3);  // Gửi lệnh "đọc RGBC"
  uint32_t timeout = millis() + 400;
  uint8_t idx = 0;
  
  while (millis() < timeout) {
    if (Serial2.available()) {
      gy33_rx_buf[idx++] = Serial2.read();
      if (idx >= 13) break;
    }
  }
  
  if (idx >= 13 && gy33_rx_buf[0]==0x5A && gy33_rx_buf[1]==0x5A && gy33_rx_buf[2]==0x15) {
    *R = (uint16_t)((gy33_rx_buf[4]<<8) | gy33_rx_buf[5]);
    *G = (uint16_t)((gy33_rx_buf[6]<<8) | gy33_rx_buf[7]);
    *B = (uint16_t)((gy33_rx_buf[8]<<8) | gy33_rx_buf[9]);
    *C = (uint16_t)((gy33_rx_buf[10]<<8)| gy33_rx_buf[11]);
    return true;
  }
  return false;
}

void led_off(void) {
  gpio_write_bit(GPIOB, 12, 0);
  gpio_write_bit(GPIOB, 13, 0);
  gpio_write_bit(GPIOB, 14, 0);
  gpio_write_bit(GPIOA, 4, 0);
}

void processColorDetection() {
  // Chỉ đọc màu mỗi 150ms
  if (millis() - lastColorReadTime < 150) return;
  lastColorReadTime = millis();
  
  uint16_t R, G, B, C;
  
  // Đọc dữ liệu từ GY-33
  if (!readGY33(&R, &G, &B, &C)) {
    return;
  }
  
  // Trừ giá trị nền
  R = (R > R_base) ? (R - R_base) : 0;
  G = (G > G_base) ? (G - G_base) : 0;
  B = (B > B_base) ? (B - B_base) : 0;
  
  float sum = (float)R + (float)G + (float)B;
  float rN = (sum > 0.0f) ? (float)R / sum : 0.0f;
  float gN = (sum > 0.0f) ? (float)G / sum : 0.0f;
  float bN = (sum > 0.0f) ? (float)B / sum : 0.0f;
  
  // Kiểm tra ánh sáng quá yếu
  const uint16_t minLight = 120;
  if (C < minLight || sum < 50.0f) {
    led_off();
    if (current_color != 'N') {
      current_color = 'N';
      softUART_SendByte('N');  // Chỉ gửi ký tự 'N'
    }
    return;
  }
  
  // Cập nhật EMA filter (de khong gay nhieu = bach ket hop voi gia tri truoc do ->led se ko nhap nhay)
  rN_filt = rN_filt*(1.0f - alpha) + rN*alpha;
  gN_filt = gN_filt*(1.0f - alpha) + gN*alpha;
  bN_filt = bN_filt*(1.0f - alpha) + bN*alpha;
  
  // Xác định màu LED, ưu tiên vàng
  led_off();
  char new_color = 'N';
  
  if (gN >= 0.40f && gN <= 0.48f && rN >= 0.32f && rN <= 0.47f && bN < 0.22f) {
    new_color = 'Y';
    gpio_write_bit(GPIOA, 4, 1); // YELLOW
  }
  else if (rN > 0.40f && rN > gN && rN > bN) {
    new_color = 'R';
    gpio_write_bit(GPIOB, 12, 1); // RED
  }
  else if (bN > 0.40f && bN > rN && bN > gN) {
    new_color = 'B';
    gpio_write_bit(GPIOB, 14, 1); // BLUE
  }
  else if (gN > 0.40f && gN > rN && gN > bN) {
    new_color = 'G';
    gpio_write_bit(GPIOB, 13, 1); // GREEN
  }
  
  // Chỉ gửi khi màu thay đổi
  if (new_color != current_color) {
    current_color = new_color;
    softUART_SendByte((uint8_t)new_color);  // Chỉ gửi 1 ký tự R/G/B/Y/N
  }
}


// ==================== SETUP =====================================

void setup() {
  // Setup UART mềm
  setupGPIO();
  setupTimer2();
  setupTimer3();
  setupEXTI();

  
  // Setup Serial debug (USB)
  Serial.begin(9600);
  delay(500);
  
  // Setup Serial2 cho GY-33 (PA2=TX, PA3=RX)
  Serial2.begin(9600);
  delay(100);
  
  Serial.println(F("\n=== STM32 UART MỀM + GY-33 ==="));
  Serial.println(F("PA0=RX | PA1=TX | PA2=GY33-TX | PA3=GY33-RX"));
  Serial.println(F("LED: PB12=R PB13=G PB14=B PA4=Y\n"));
  
  // Khởi tạo GY-33
  sendCommandGY33(cmd_disable_auto, 3);
  delay(200);
  sendCommandGY33(cmd_led_on, 3);
  delay(200);
  
  // Đo nền
  uint16_t R, G, B, C;
  Serial.println(F("Đo nền..."));
  delay(800);
  
  if (readGY33(&R, &G, &B, &C)) {
    R_base = R; 
    G_base = G; 
    B_base = B;
    baselineMeasured = true;
    
    Serial.print(F("Nền: R="));
    Serial.print(R_base);
    Serial.print(F(" G="));
    Serial.print(G_base);
    Serial.print(F(" B="));
    Serial.println(B_base);
    
    // Gửi tín hiệu 'O' = Đo nền thành công
    softUART_SendByte('O');
  } else {
    Serial.println(F("Đo nền thất bại"));
    baselineMeasured = false;
    
    // Gửi tín hiệu 'F' = Đo nền thất bại
    softUART_SendByte('F');
  }
  
  lastColorReadTime = millis();
  Serial.println(F("Sẵn sàng!"));
}


// ==================== MAIN LOOP =================================

void loop() {
  // ============ 1. RX BUFFER -> FRAME (UART mềm) ============
  uint8_t byte;
  while (rxBufferPop(&byte)) {
    char c = byte;
    
    if (c == '@') {
      frameIndex = 0;
      frameComplete = false;
    }
    
    if (frameIndex < FRAME_MAX_LEN - 1) {
      frameBuffer[frameIndex++] = c;
      
      if (c == '&') {
        frameBuffer[frameIndex] = '\0';
        frameComplete = true;
        break;
      }
    } else {
      frameIndex = 0;
    }
  }
  
  // ============ 2. PROCESS FRAME - ECHO BACK ============
  if (frameComplete) {
    frameComplete = false;
    
    // Echo toàn bộ frame về UART mềm (bao gồm @ và &)
    softUART_SendBuffer((uint8_t*)frameBuffer, frameIndex);
    
    // Debug qua Serial
    Serial.print(F("Echo: "));
    Serial.print(frameIndex);
    Serial.println(F(" bytes"));
    
    frameIndex = 0;
  }
  
  // ============ 3. ĐỌC MÀU TỪ GY-33 ============
  if (baselineMeasured) {
    processColorDetection();
  }
}