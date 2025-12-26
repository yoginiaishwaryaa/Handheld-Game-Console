//________________________-Final code with systick and interrupt-________________________

#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define PCF8574_ADDR (0x27 << 1)  // I2C 7-bit address shifted

// --- Joystick ADC Configuration ---
#define JOY_X_PIN     0     // PA0 (ADC1_IN0) - Horizontal
#define JOY_Y_PIN     1     // PA1 (ADC1_IN1) - Vertical
#define JOY_BTN_PIN   8     // PA8 - Joystick Button (active low with pull-up)

// --- Menu Selection Button ---
#define MENU_BTN_PIN  9     // PA9 - Menu Selection Button (active low with pull-up)

// --- LED Indicator ---
#define LED_PIN      13     // PC13 - LED indicator

// --- SysTick Variables ---
volatile uint32_t sys_tick_ms = 0;

// --- Button interrupt flags ---
volatile uint8_t joy_button_pressed = 0;
volatile uint8_t menu_button_pressed = 0;

// --- SysTick Handler (ISR) ---
void SysTick_Handler(void) {
    sys_tick_ms++;
}

// --- EXTI9_5 Handler for PA8 (Joystick Button) and PA9 (Menu Button) ---
void EXTI9_5_IRQHandler(void) {
    // Check if PA8 (Joystick button) triggered the interrupt
    if (EXTI->PR & (1 << JOY_BTN_PIN)) {
        joy_button_pressed = 1;
        EXTI->PR |= (1 << JOY_BTN_PIN);  // Clear pending bit
    }
    
    // Check if PA9 (Menu button) triggered the interrupt
    if (EXTI->PR & (1 << MENU_BTN_PIN)) {
        menu_button_pressed = 1;
        EXTI->PR |= (1 << MENU_BTN_PIN);  // Clear pending bit
    }
}

// --- SysTick Initialization with NVIC (1ms tick) ---
void SysTick_Init(void) {
    // STM32F401CCU6 runs at 84 MHz
    // SysTick reload value for 1ms = (84000000 / 1000) - 1 = 83999
    SysTick->LOAD = 84000 - 1;
    SysTick->VAL = 0;  // Clear current value
    
    // Configure SysTick
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |  // Use processor clock
                    SysTick_CTRL_TICKINT_Msk |     // Enable SysTick interrupt
                    SysTick_CTRL_ENABLE_Msk;       // Enable SysTick timer
    
    // Set SysTick interrupt priority (optional - default is lowest priority)
    NVIC_SetPriority(SysTick_IRQn, 1);  // Priority 1 (lower than buttons)
}

// --- Get current tick count ---
uint32_t millis(void) {
    return sys_tick_ms;
}

// --- Delay function using SysTick ---
void delay_ms(uint32_t ms) {
    uint32_t start = millis();
    while ((millis() - start) < ms);
}

// --- Microsecond delay (kept as NOP-based for precision) ---
void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us * 8; i++) __NOP();
}

// --- ADC Initialization ---
void ADC1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 clock

    ADC->CCR &= ~(3 << 16);
    ADC->CCR |=  (1 << 16); // Prescaler PCLK2/4

    ADC1->CR2 &= ~(1 << 0); // ADC off
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    // Longest sample time for stability
    ADC1->SMPR2 |= (7 << (3 * JOY_X_PIN)) | (7 << (3 * JOY_Y_PIN));

    ADC1->CR2 |= (1 << 0); // Turn on ADC
}

// --- ADC Read Function ---
uint16_t ADC_Read(uint8_t channel) {
    ADC1->SQR3 = channel;        // Select channel
    ADC1->CR2 |= (1 << 30);      // Start conversion
    while (!(ADC1->SR & (1 << 1))); // Wait for EOC
    return (uint16_t)ADC1->DR;
}

// --- GPIO Init for Joystick with EXTI interrupts ---
void Joystick_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // Enable GPIOC clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock for EXTI
    
    // PA0, PA1: Analog inputs for ADC
    GPIOA->MODER |= (3 << (JOY_X_PIN*2));  // Analog mode
    GPIOA->MODER |= (3 << (JOY_Y_PIN*2));  // Analog mode
    
    // PA8: Digital input for joystick button with interrupt
    GPIOA->MODER &= ~(3 << (JOY_BTN_PIN*2)); // Input mode
    GPIOA->PUPDR &= ~(3 << (JOY_BTN_PIN*2));
    GPIOA->PUPDR |=  (1 << (JOY_BTN_PIN*2)); // Pull-up
    
    // PA9: Digital input for menu selection button with interrupt
    GPIOA->MODER &= ~(3 << (MENU_BTN_PIN*2)); // Input mode
    GPIOA->PUPDR &= ~(3 << (MENU_BTN_PIN*2));
    GPIOA->PUPDR |=  (1 << (MENU_BTN_PIN*2)); // Pull-up
    
    // Configure EXTI for PA8 (Joystick button)
    SYSCFG->EXTICR[2] &= ~(0xF << 0);  // Clear EXTI8 configuration
    SYSCFG->EXTICR[2] |= (0 << 0);     // Select PA8 for EXTI8
    
    // Configure EXTI for PA9 (Menu button)
    SYSCFG->EXTICR[2] &= ~(0xF << 4);  // Clear EXTI9 configuration
    SYSCFG->EXTICR[2] |= (0 << 4);     // Select PA9 for EXTI9
    
    // Configure EXTI8 and EXTI9 for falling edge (button press)
    EXTI->IMR |= (1 << JOY_BTN_PIN) | (1 << MENU_BTN_PIN);  // Unmask interrupts
    EXTI->FTSR |= (1 << JOY_BTN_PIN) | (1 << MENU_BTN_PIN); // Falling edge trigger
    EXTI->RTSR &= ~((1 << JOY_BTN_PIN) | (1 << MENU_BTN_PIN)); // Disable rising edge
    
    // Enable EXTI9_5 interrupt in NVIC
    NVIC_SetPriority(EXTI9_5_IRQn, 0);  // Highest priority for button response
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    
    // PC13: LED indicator output
    GPIOC->MODER &= ~(3 << (LED_PIN*2));
    GPIOC->MODER |=  (1 << (LED_PIN*2));  // Output mode
    GPIOC->ODR |= (1 << LED_PIN);         // LED off initially (active low)
    
    ADC1_Init();
}

// --- Joystick Functions ---
uint8_t Joy_Up(void) {
    uint16_t y = ADC_Read(JOY_Y_PIN);
    return (y < 1000); // Threshold for UP
}

uint8_t Joy_Down(void) {
    uint16_t y = ADC_Read(JOY_Y_PIN);
    return (y > 3000); // Threshold for DOWN
}

uint8_t Joy_Select(void) {
    return joy_button_pressed;
}

uint8_t Menu_Select(void) {
    return menu_button_pressed;
}

// --- Clear button flags ---
void Clear_Joy_Button(void) {
    joy_button_pressed = 0;
}

void Clear_Menu_Button(void) {
    menu_button_pressed = 0;
}

// --- I2C Init ---
void I2C1_Init(void) {
    RCC->AHB1ENR |= (1 << 1);
    RCC->APB1ENR |= (1 << 21);
    GPIOB->MODER |= (2 << 12) | (2 << 14);
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);
    GPIOB->PUPDR |= (1 << 12) | (1 << 14);
    GPIOB->AFR[0] |= (4 << 24) | (4 << 28);
    I2C1->CR1 = (1 << 0);
    I2C1->CR2 = 42;
    I2C1->CCR = 210;
    I2C1->TRISE = 43;
}

void I2C1_Write(uint8_t data) {
    while (I2C1->SR2 & 2);
    I2C1->CR1 |= (1 << 8);
    while (!(I2C1->SR1 & 1));
    I2C1->DR = PCF8574_ADDR;
    while (!(I2C1->SR1 & 2)); 
		(void)I2C1->SR2;
    while (!(I2C1->SR1 & 0x80));
    I2C1->DR = data;
    while (!(I2C1->SR1 & 4));
    I2C1->CR1 |= (1 << 9);
}

// --- LCD Functions ---
void LCD_Send(uint8_t value, uint8_t rs) {
    uint8_t hi = value & 0xF0;
    uint8_t lo = (value << 4) & 0xF0;
    uint8_t mode = rs ? 0x09 : 0x08;
    I2C1_Write(hi | mode | 0x04); 
    I2C1_Write(hi | mode);
    I2C1_Write(lo | mode | 0x04); 
    I2C1_Write(lo | mode);
}

void LCD_Init(void) {
    delay_ms(15);
    LCD_Send(0x33, 0);
    LCD_Send(0x32, 0);
    LCD_Send(0x28, 0);
    LCD_Send(0x0C, 0);  // Display ON, Cursor OFF, Blink OFF
    LCD_Send(0x06, 0);
    LCD_Send(0x01, 0);
    delay_ms(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row > 3) row = 3;
    LCD_Send(0x80 | (col + row_offsets[row]), 0);
}

void LCD_Print(char* s) {
    while (*s) LCD_Send(*s++, 1);
}

void LCD_Clear(void) {
    LCD_Send(0x01, 0);
    delay_ms(1);
}

void LCD_CreateChar(uint8_t location, uint8_t charmap[]) {
    location &= 0x7;
    LCD_Send(0x40 | (location << 3), 0);
    for (int i = 0; i < 8; i++) LCD_Send(charmap[i], 1);
}

// --- Welcome Screen ---
void LCD_Welcome(void) {
    LCD_Clear();
    
    LCD_SetCursor(0, 6);
    LCD_Print("WELCOME!");
    
    LCD_SetCursor(1, 2);
    LCD_Print("STM32F401 GAMING");
    
    LCD_SetCursor(2, 5);
    LCD_Print("CONSOLE");
    
    LCD_SetCursor(3, 3);
    LCD_Print("Loading...");
    
    delay_ms(500);
}

// --- Menu System ---
int LCD_Menu(void) {
    int choice = 0;
    bool input_handled = false;
    uint32_t last_refresh = 0;
    
    // Initial display
    LCD_Clear();
    LCD_SetCursor(0, 3);
    LCD_Print("SELECT A GAME");
    
    while(1) {
        // Only refresh menu when choice changes
        if(last_refresh != choice) {
            LCD_SetCursor(1, 0);
            if(choice == 0) {
                LCD_Print("> 1. Memory Game    ");
            } else {
                LCD_Print("  1. Memory Game    ");
            }
            
            LCD_SetCursor(2, 0);
            if(choice == 1) {
                LCD_Print("> 2. Platform Runner");
            } else {
                LCD_Print("  2. Platform Runner");
            }
            
            LCD_SetCursor(3, 2);
            LCD_Print("Move:Joy Sel:Btn");
            
            last_refresh = choice;
        }
        
        // Handle joystick input with debounce
        if(Joy_Up() && !input_handled) {
            input_handled = true;
            if(choice > 0) choice--;
            delay_ms(100);
        }
        else if(Joy_Down() && !input_handled) {
            input_handled = true;
            if(choice < 1) choice++;
            delay_ms(100);
        }
        else if(Menu_Select() && !input_handled) {
            // LED indicator ON when button pressed
            GPIOC->ODR &= ~(1 << LED_PIN); // LED ON (active low)
            input_handled = true;
            delay_ms(150);
            Clear_Menu_Button();  // Clear interrupt flag
            GPIOC->ODR |= (1 << LED_PIN);  // LED OFF
            return choice;
        }
        
        // Reset input flag when joystick returns to center
        if(!Joy_Up() && !Joy_Down() && !Menu_Select()) {
            input_handled = false;
        }
        
        delay_ms(5); // Very fast polling
    }
}

// ---------------- LED Matrix Configuration ----------------
uint16_t rows[8] = {2,3,4,5,6,7,0,1};       // PA2�PA7, PB0�1
uint16_t cols[8] = {2,3,4,5,8,9,10,12};     // PB2�5, PB8�12

// ---------------- Game Settings ----------------
#define SEQ_LEN 3

uint8_t led_sequence[SEQ_LEN];
uint8_t player_sequence[SEQ_LEN];

// ---------------- LED Matrix GPIO Init ----------------
void LED_Matrix_Init(void) {
    // Enable GPIO A,B clocks
    RCC->AHB1ENR |= (1 << 0) | (1 << 1);

    // --- ROWS (PA2�PA7, PB0�1) ---
    for (int i = 0; i < 6; i++) { // PA2�PA7
        GPIOA->MODER &= ~(3 << (rows[i]*2));
        GPIOA->MODER |=  (1 << (rows[i]*2)); // output
    }
    for (int i = 6; i < 8; i++) { // PB0�1
        GPIOB->MODER &= ~(3 << (rows[i]*2));
        GPIOB->MODER |=  (1 << (rows[i]*2)); // output
    }

    // --- COLUMNS (PB2�5, PB8�12) ---
    for (int i = 0; i < 8; i++) {
        GPIOB->MODER &= ~(3 << (cols[i]*2));
        GPIOB->MODER |=  (1 << (cols[i]*2));   // output
        GPIOB->ODR |= (1 << cols[i]);          // off (active low)
    }
    
    // Re-initialize joystick analog inputs (PA0, PA1)
    GPIOA->MODER &= ~(3 << (JOY_X_PIN*2));
    GPIOA->MODER |=  (3 << (JOY_X_PIN*2)); // Analog mode
    GPIOA->MODER &= ~(3 << (JOY_Y_PIN*2));
    GPIOA->MODER |=  (3 << (JOY_Y_PIN*2)); // Analog mode
    
    // Re-initialize joystick button (PA8) with interrupt
    GPIOA->MODER &= ~(3 << (JOY_BTN_PIN*2)); // Input mode
    GPIOA->PUPDR &= ~(3 << (JOY_BTN_PIN*2));
    GPIOA->PUPDR |=  (1 << (JOY_BTN_PIN*2)); // Pull-up
    
    // Re-enable EXTI for PA8
    EXTI->IMR |= (1 << JOY_BTN_PIN);  // Unmask interrupt
    
    // Re-initialize menu button (PA9) with interrupt
    GPIOA->MODER &= ~(3 << (MENU_BTN_PIN*2)); // Input mode
    GPIOA->PUPDR &= ~(3 << (MENU_BTN_PIN*2));
    GPIOA->PUPDR |=  (1 << (MENU_BTN_PIN*2)); // Pull-up
    
    // Re-enable EXTI for PA9
    EXTI->IMR |= (1 << MENU_BTN_PIN);  // Unmask interrupt
}

// ---------------- LED Control ----------------
void LED_AllOff(void) {
    GPIOA->ODR &= ~((1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7));
    GPIOB->ODR &= ~((1<<0)|(1<<1));
    for (int i = 0; i < 8; i++) GPIOB->ODR |= (1 << cols[i]);  // columns off (active low)
}

void LED_AllOn(void) {
    GPIOA->ODR |= ((1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7));
    GPIOB->ODR |= ((1<<0)|(1<<1));
    for (int i = 0; i < 8; i++) GPIOB->ODR &= ~(1 << cols[i]); // all cols active
}

void LED_Set(uint8_t r, uint8_t c) {
    LED_AllOff();
    if (r < 6) GPIOA->ODR |= (1 << rows[r]);
    else        GPIOB->ODR |= (1 << rows[r]);
    GPIOB->ODR &= ~(1 << cols[c]); // active low
}

// ---------------- Simple Random Generator ----------------
uint8_t rand8(void) {
    static uint32_t seed = 12345;
    seed = seed * 1103515245 + 12345;
    return (seed >> 16) & 0xFF;
}

// ---------------- Heart Shape Pattern (Steady with Fast Multiplexing) ----------------
void HeartPattern(void) {
    uint8_t heart[8][8] = {
        {0,1,1,0,0,1,1,0},
        {1,1,1,1,1,1,1,1},
        {1,1,1,1,1,1,1,1},
        {1,1,1,1,1,1,1,1},
        {0,1,1,1,1,1,1,0},
        {0,0,1,1,1,1,0,0},
        {0,0,0,1,1,0,0,0},
        {0,0,0,0,0,0,0,0}
    };

    // Display heart for 800ms with fast multiplexing (appears steady)
    uint32_t start_time = millis();
    while((millis() - start_time) < 800) {
        for (int r = 0; r < 8; r++) {
            // Turn off all rows first
            GPIOA->ODR &= ~((1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7));
            GPIOB->ODR &= ~((1<<0)|(1<<1));
            
            // Turn off all columns
            for (int c = 0; c < 8; c++) {
                GPIOB->ODR |= (1 << cols[c]);
            }

            // Enable current row
            if (r < 6) 
                GPIOA->ODR |= (1 << rows[r]);
            else        
                GPIOB->ODR |= (1 << rows[r]);

            // Light up only the "1" columns for this row
            for (int c = 0; c < 8; c++) {
                if (heart[r][c])
                    GPIOB->ODR &= ~(1 << cols[c]); // active low
            }

            delay_us(400); // Very short delay per row (fast multiplexing)
        }
    }

    LED_AllOff();
}

// ---------------- Main LED Matrix Game Logic ----------------
// Returns 1 if round completed successfully, 0 if early button press occurred
uint8_t game_loop(void) {
    // Clear any stray button presses
    Clear_Joy_Button();
    
    // Display instruction before each round
    LCD_SetCursor(1, 0);
    LCD_Print("Watch the matrix    ");
    LCD_SetCursor(2, 0);
    LCD_Print("closely...          ");
    delay_ms(1000);
    
    // --- Generate Random LED Sequence ---
    for (int i = 0; i < SEQ_LEN; i++) {
        led_sequence[i] = rand8() % 64;
    }

    // --- Show Sequence ---
    for (int i = 0; i < SEQ_LEN; i++) {
        // Check if user pressed button too early
        if (Joy_Select()) {
            Clear_Joy_Button();
            LED_AllOff();
            LCD_SetCursor(1, 0);
            LCD_Print("Too early! Retry    ");
            LCD_SetCursor(2, 0);
            LCD_Print("this round...       ");
            delay_ms(800);  // Reduced from 1500ms
            return 0;  // Indicate round needs to be retried
        }
        
        uint8_t r = led_sequence[i] / 8;
        uint8_t c = led_sequence[i] % 8;
        LED_Set(r, c);
        delay_ms(150);
        LED_AllOff();
        delay_ms(50);
    }
    
    // Clear the LCD for player input phase
    LCD_SetCursor(1, 0);
    LCD_Print("Your turn! Use joy  ");
    LCD_SetCursor(2, 0);
    LCD_Print("Press to select     ");

    // --- Player Cursor ---
    uint8_t cursor_r = 0, cursor_c = 0;

    for (int i = 0; i < SEQ_LEN; i++) {
        while (1) {
            LED_Set(cursor_r, cursor_c);

            uint16_t x = ADC_Read(JOY_X_PIN);
            uint16_t y = ADC_Read(JOY_Y_PIN);
            delay_ms(5);

            // LEFT / RIGHT
            if (x < 1000 && cursor_c > 0) {
                cursor_c--;
                delay_ms(50);
            } else if (x > 3000 && cursor_c < 7) {
                cursor_c++;
                delay_ms(50);
            }

            // UP / DOWN
            if (y < 1000 && cursor_r > 0) {
                cursor_r--;
                delay_ms(50);
            } else if (y > 3000 && cursor_r < 7) {
                cursor_r++;
                delay_ms(50);
            }

            // Joystick Button Press (PA8) - using interrupt flag
            if (Joy_Select()) {
                player_sequence[i] = cursor_r * 8 + cursor_c;
                delay_ms(150);
                Clear_Joy_Button();  // Clear interrupt flag
                break;
            }
        }
    }

    // --- Check Result ---
    uint8_t win = 1;
    for (int i = 0; i < SEQ_LEN; i++) {
        if (player_sequence[i] != led_sequence[i]) {
            win = 0;
            break;
        }
    }

    // --- Feedback (steady display for 0.8s) ---
    if (win) {
        HeartPattern();   // Display steady heart pattern (fast multiplex)
    } else {
        LED_AllOn();      // Turn on all LEDs steadily (no multiplex needed)
        delay_ms(250);    // Keep them on
        LED_AllOff();
    }

    delay_ms(10); // minimal gap before next round
    return 1;  // Round completed successfully
}

// --- Memory Game (LED Matrix Game) ---
void Game_Memory(void) {
    // Initialize LED Matrix
    LED_Matrix_Init();
    LED_AllOff();
    
    // Show info on LCD
    LCD_Clear();
    LCD_SetCursor(0, 2);
    LCD_Print("MEMORY GAME");
    LCD_SetCursor(1, 1);
    LCD_Print("Watch LED Pattern");
    LCD_SetCursor(2, 1);
    LCD_Print("Repeat with Joy");
    LCD_SetCursor(3, 1);
    LCD_Print("Starting...");
    
    delay_ms(800);
    LCD_Clear();
    
    // Run 3 rounds of the game
    for(int round = 0; round < 3; round++) {
        LCD_SetCursor(0, 3);
        char round_str[21];
        snprintf(round_str, 21, "Round %d/3", round + 1);
        LCD_Print(round_str);
        
        // Keep retrying this round until completed successfully
        uint8_t round_complete = 0;
        while(!round_complete) {
            round_complete = game_loop();  // Returns 1 on success, 0 on early press
        }
    }  
    
    // Game complete
    LED_AllOff();
    LCD_Clear();
    LCD_SetCursor(1, 2);
    LCD_Print("GAME COMPLETE!");
    LCD_SetCursor(3, 1);
    LCD_Print("Press to Return");
    
    // Wait for button press
    while(!Menu_Select()) {
        delay_ms(50);
    }
    Clear_Menu_Button();  // Clear interrupt flag
}

// --- Platform Runner Game ---
void Game_PlatformRunner(void) {
    // Create custom characters for runner
    uint8_t run1[8] = {
        0b00100,
        0b00100,
        0b01110,
        0b10101,
        0b00100,
        0b01010,
        0b01000,
        0b10000
    };
    
    uint8_t run2[8] = {
        0b00100,
        0b00100,
        0b01110,
        0b10101,
        0b00100,
        0b00101,
        0b00010,
        0b00001
    };
    
    uint8_t wall_block[8] = {
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111
    };
    
    LCD_CreateChar(0, run1);
    LCD_CreateChar(1, run2);
    LCD_CreateChar(2, wall_block);
    
    // Wall pattern with single blocks and more spacing
    char walls[120] = "                    \x02         \x02          \x02         \x02          \x02         \x02          \x02         \x02          \x02         \x02";
    
    int scroll_pos = 0;
    int runner_x = 4;
    uint8_t run_pose = 0;
    int runner_row = 2;
    int jump_state = 0;
    uint8_t button_prev = 0;
    int wall_len = strlen(walls);
    int score = 0;
    int game_speed = 45; // ms between frames (even faster starting speed)
    int game_over = 0;
    uint8_t prev_obstacle = 0;
    uint8_t scoring_enabled = 0;
    
    char buffer[21];
    uint32_t last_frame_time = millis();
    
    LCD_Clear();
    
    while(1) {
        // Use SysTick timing for consistent frame rate
        uint32_t current_time = millis();
        if((current_time - last_frame_time) < game_speed) {
            continue; // Skip frame if not enough time has passed
        }
        last_frame_time = current_time;
        
        // --- Check for game over condition ---
        if (game_over) {
            // Display GAME OVER screen
            LCD_Clear();
            
            LCD_SetCursor(1, 4);
            LCD_Print("GAME OVER!");
            
            LCD_SetCursor(2, 3);
            char score_str[21];
            snprintf(score_str, 21, "Score: %d", score);
            LCD_Print(score_str);
            
            LCD_SetCursor(3, 1);
            LCD_Print("Press to Return");
            
            // Wait for button press to return
            delay_ms(200);
            while(!Menu_Select()) delay_ms(50);
            Clear_Menu_Button();  // Clear interrupt flag
            return;
        }
        
        // Button handling for jump
        if(Menu_Select() && !button_prev && jump_state == 0) {
            button_prev = 1;
            jump_state = 1;
            Clear_Menu_Button();  // Clear interrupt flag
        }
        
        // Reset button state when not pressed (using actual pin state for release detection)
        if(!(GPIOA->IDR & (1 << MENU_BTN_PIN))) {
            button_prev = 0;
        }
        
        // Extended jump logic (6 frames in air)
        if (jump_state == 1) {
            runner_row = 1;
            jump_state = 2;
        } else if (jump_state == 2) {
            jump_state = 3;
        } else if (jump_state == 3) {
            jump_state = 4;
        } else if (jump_state == 4) {
            jump_state = 5;
        } else if (jump_state == 5) {
            jump_state = 6;
        } else if (jump_state == 6) {
            runner_row = 2;
            jump_state = 0;
        }
        
        // Fill wall buffer
        for(int i = 0; i < 20; i++) {
            buffer[i] = walls[(scroll_pos + i) % wall_len];
        }
        buffer[20] = '\0';
        
        // --- Enable scoring after safe zone ---
        if (scroll_pos >= 20 && !scoring_enabled) {
            scoring_enabled = 1;
            prev_obstacle = 0;
        }
        
        // --- Score increment when obstacle passes ---
        if (scoring_enabled) {
            uint8_t current_obstacle = (buffer[runner_x] == '\x02') ? 1 : 0;
            
            if (prev_obstacle && !current_obstacle) {
                score++;
            }
            prev_obstacle = current_obstacle;
        }
        
        // --- Collision detection ---
        if (runner_row == 2 && buffer[runner_x] == '\x02') {
            game_over = 1;
            continue;
        }
        
        // Row 0: Score
        LCD_SetCursor(0, 0);
        char score_display[21];
        snprintf(score_display, 21, "Score: %d", score);
        LCD_Print(score_display);
        for (int i = strlen(score_display); i < 20; i++) {
            LCD_Send(' ', 1);
        }
        
        // Row 1: Runner when jumping
        LCD_SetCursor(1, 0);
        for(int i = 0; i < 20; i++) {
            if(i == runner_x && runner_row == 1) {
                LCD_Send(run_pose, 1);
            } else {
                LCD_Send(' ', 1);
            }
        }
        
        // Row 2: Runner on ground or walls
        LCD_SetCursor(2, 0);
        for(int i = 0; i < 20; i++) {
            if(i == runner_x && runner_row == 2) {
                LCD_Send(run_pose, 1);
            } else {
                LCD_Send(buffer[i], 1);
            }
        }
        
        // Row 3: Ground
        LCD_SetCursor(3, 0);
        LCD_Print("\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02");
        
        // Update animation
        run_pose = (run_pose == 0) ? 1 : 0;
        scroll_pos = (scroll_pos + 1) % wall_len;
        
        // Increase difficulty (speed up more aggressively)
        if(score % 10 == 0 && score > 0 && game_speed > 20) {
            game_speed -= 2;
        }
    }
}

int main(void) {
    // --- Initialize SysTick first ---
    SysTick_Init();
    
    // --- Initialize peripherals ---
    I2C1_Init();
    LCD_Init();
    Joystick_Init();
    
    // --- Show welcome screen ---
    LCD_Welcome();

    while (1) {
        // --- Show main menu and get user choice ---
        int selected = LCD_Menu();

        // --- Reinitialize LCD before launching games ---
        LCD_Clear();
        delay_ms(100);

        if (selected == 0) {
            // Memory Game Selected
            Game_Memory();
        } 
        else if (selected == 1) {
            // Platform Runner Selected
            Game_PlatformRunner();
        }

        // --- Return to Menu after game completes ---
        LCD_Clear();
        LCD_SetCursor(1, 3);
        LCD_Print("Returning...");
        delay_ms(500);
    }
}

