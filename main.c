#include "stm32f4xx.h"
#include <string.h>

// Bộ đệm nhận UART
char rx_buffer[20];
int rx_index = 0;
char last_command[20] = {0};

volatile uint8_t button_state = 1;
volatile uint8_t led_red = 0, led_yellow = 0, led_green = 0;

// UART2 (PA2: TX, PA3: RX)
void USART2_Init(void) {
    RCC->AHB1ENR |= (1 << 0);
    RCC->APB1ENR |= (1 << 17);
    GPIOA->MODER &= ~((3U << 4) | (3U << 6));
    GPIOA->MODER |= (2U << 4) | (2U << 6);
    GPIOA->AFR[0] |= (7U << 8) | (7U << 12);
    USART2->CR1 = 0;
    USART2->BRR = 16000000 / 9600;
    USART2->CR1 |= (1 << 2) | (1 << 3) | (1 << 5) | (1 << 13);
    NVIC_EnableIRQ(USART2_IRQn);
}

// LED (PA5, PA6, PA7)
void LED_Init(void) {
    RCC->AHB1ENR |= (1 << 0);
    GPIOA->MODER &= ~((3U << 10) | (3U << 12) | (3U << 14));
    GPIOA->MODER |= (1U << 10) | (1U << 12) | (1 << 14);
}

// Cấu hình nút bấm (PA0)
void Button_Init(void) {
    RCC->AHB1ENR |= (1 << 0);
    GPIOA->MODER &= ~(3U << 0);
    GPIOA->PUPDR |= (1U << 0);
}

// Cấu hình ngắt ngoài cho PA0
void EXTI_Init(void) {
    RCC->APB2ENR |= (1 << 14);
    SYSCFG->EXTICR[0] &= ~(0xF << 0);
    EXTI->IMR |= (1 << 0);
    EXTI->FTSR |= (1 << 0);
    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);
}

// Cấu hình Timer 2 cho chu kỳ 200ms
void TIM2_Init(void) {
    RCC->APB1ENR |= (1 << 0);
    TIM2->PSC = 15999;
    TIM2->ARR = 199;
    TIM2->CNT = 0;
    TIM2->DIER |= (1 << 0);
    NVIC_EnableIRQ(TIM2_IRQn);
}

// Gửi chuỗi qua UART
void UART_SendString(const char *str) {
    while (*str) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *str++;
    }
}

// Xử lý mã nhận được từ UART
void handle_command(void) {
    rx_buffer[rx_index] = '\0';
    strcpy(last_command, rx_buffer);
    UART_SendString("Received: ");
    UART_SendString(last_command);
    UART_SendString("\r\n");

    // Toggle trạng thái LED được chọn và cập nhật GPIO, giữ nguyên các LED khác
    if (strcmp(last_command, "123R") == 0) {
        led_red = !led_red; // Toggle trạng thái LED đỏ
        if (led_red) {
            GPIOA->ODR |= (1 << 5); // Bật LED đỏ
            UART_SendString("RED LED ON\r\n");
        } else {
            GPIOA->ODR &= ~(1 << 5); // Tắt LED đỏ
            UART_SendString("RED LED OFF\r\n");
        }
    } else if (strcmp(last_command, "123Y") == 0) {
        led_yellow = !led_yellow; // Toggle trạng thái LED vàng
        if (led_yellow) {
            GPIOA->ODR |= (1 << 6); // Bật LED vàng
            UART_SendString("YELLOW LED ON\r\n");
        } else {
            GPIOA->ODR &= ~(1 << 6); // Tắt LED vàng
            UART_SendString("YELLOW LED OFF\r\n");
        }
    } else if (strcmp(last_command, "123G") == 0) {
        led_green = !led_green; // Toggle trạng thái LED xanh lá
        if (led_green) {
            GPIOA->ODR |= (1 << 7); // Bật LED xanh lá
            UART_SendString("GREEN LED ON\r\n");
        } else {
            GPIOA->ODR &= ~(1 << 7); // Tắt LED xanh lá
            UART_SendString("GREEN LED OFF\r\n");
        }
    } else {
        UART_SendString("Unknown command\r\n");
    }

    rx_index = 0;
}

// Xử lý ngắt UART
void USART2_IRQHandler(void) {
    if (USART2 -> SR & USART_SR_RXNE) {
        if (USART2 -> SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
            USART2 -> DR;
            UART_SendString("UART error\r\n");
            rx_index = 0;
        } else {
            char c = USART2->DR;
            if (c == '\r' || c == '\n') {
                handle_command();
            } else if (rx_index < sizeof(rx_buffer) - 1) {
                rx_buffer[rx_index++] = c;
            } else {
                UART_SendString("Buffer overflow\r\n");
                rx_index = 0;
            }
        }
    }
}

// Xử lý ngắt ngoài từ nút bấm
void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1 << 0)) {
        EXTI->PR = (1 << 0);
        UART_SendString("Button pressed, state: ");
        char state_str[2] = {button_state + '0', '\0'};
        UART_SendString(state_str);
        UART_SendString("\r\n");


        if (led_red || led_yellow || led_green) {
            if (button_state == 1) {
                button_state = 2;
                TIM2->CR1 &= ~(1 << 0);
                if (led_red) GPIOA->ODR |= (1 << 5);
                if (led_yellow) GPIOA->ODR |= (1 << 6);
                if (led_green) GPIOA->ODR |= (1 << 7);
                UART_SendString("Switched to continuous mode\r\n");
            } else if (button_state == 2) {
                button_state = 1;
                TIM2->CR1 |= (1 << 0);
                UART_SendString("Switched to blinking mode\r\n");
            }
        } else {
            UART_SendString("No LED selected, no action\r\n");
        }
    }
}

// Xử lý ngắt Timer 2 (nháy LED)
void TIM2_IRQHandler(void) {
    if (TIM2->SR & (1 << 0)) {
        TIM2->SR &= ~(1 << 0);
        if (led_red) GPIOA->ODR ^= (1 << 5);
        if (led_yellow) GPIOA->ODR ^= (1 << 6);
        if (led_green) GPIOA->ODR ^= (1 << 7);
    }
}

// Hàm chính
int main(void) {
    SystemCoreClockUpdate();
    LED_Init();
    Button_Init();
    EXTI_Init();
    TIM2_Init();
    USART2_Init();


    TIM2->CR1 &= ~(1 << 0);
    UART_SendString("STM32 connected to PuTTY\r\n");

    while (1) {
        __WFI();
    }
}
