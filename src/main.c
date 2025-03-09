#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define F_CPU 16000000UL
#define DS18B20_PIN PD5

#define DS18B20_CMD_CONVERT_TEMP 0x44
#define DS18B20_CMD_READ_SCRATCHPAD 0xBE
#define DS18B20_CMD_WRITE_SCRATCHPAD 0x4E
#define DS18B20_CMD_SKIP_ROM 0xCC

// Function to initialize UART communication
void serial_init(int baud_rate)
{
    unsigned int ubrr = F_CPU / 16 / baud_rate - 1;
    // Set the baud rate
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    // Enable transmitter
    UCSR0B = (1 << TXEN0);
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Timeout after 1000 iterations (~1 second)
#define UART_TIMEOUT 1000

void uart_transmit(unsigned char data)
{
    unsigned int timeout = UART_TIMEOUT;

    // Wait for the transmit buffer to be empty, with a timeout
    while (!(UCSR0A & (1 << UDRE0)))
    {
        if (--timeout == 0)
        {
            // Timeout reached, exit the function
            return;
        }
    }

    // Put the data into the buffer, sends the data
    UDR0 = data;
}

// Function to send a string via UART
void serial_print(const char *str)
{
    while (*str)
    {
        uart_transmit(*str++);
        _delay_ms(1);
    }
}

// Function to initialize the DS18B20 communication (set the pin as output or input)
void DS18B20_Init(void)
{
    DDRD |= (1 << DS18B20_PIN); // Set DS18B20_PIN as output
}

// Function to send a single bit to the DS18B20
void DS18B20_WriteBit(uint8_t bit)
{
    DDRD |= (1 << DS18B20_PIN);   // Set pin as output
    PORTD &= ~(1 << DS18B20_PIN); // Pull pin low
    _delay_us(2);                 // Brief low period

    if (bit)
    {
        PORTD |= (1 << DS18B20_PIN); // Release line (pull high)
        _delay_us(60);
    }
    else
    {
        _delay_us(60);               // Keep line low longer for a "0"
        PORTD |= (1 << DS18B20_PIN); // Release line
    }
}

// Function to read a single bit from the DS18B20
uint8_t DS18B20_ReadBit(void)
{
    uint8_t bit = 0;

    // Set pin as output and pull it low for 1µs to signal a read
    DDRD |= (1 << DS18B20_PIN);   // Set pin as output
    PORTD &= ~(1 << DS18B20_PIN); // Pull low
    _delay_us(1);                 // Wait for 1µs

    // Set pin as input and wait for at least 15µs to allow line to stabilize
    DDRD &= ~(1 << DS18B20_PIN); // Set as input
    _delay_us(15);               // Increased wait for line to settle

    // Now read the bit from the bus
    bit = (PIND & (1 << DS18B20_PIN)) ? 1 : 0; // Read the bit

    // Wait for the remaining 45µs before releasing the line
    _delay_us(45); // Allow the bus to recover

    return bit;
}

// Function to send a byte (8 bits) to the DS18B20
void DS18B20_WriteByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        DS18B20_WriteBit(byte & 0x01); // Send LSB first
        byte >>= 1;
    }
}

// Function to read a byte (8 bits) from the DS18B20
uint8_t DS18B20_ReadByte(void)
{
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        uint8_t bit = DS18B20_ReadBit(); // Read a bit
        byte |= (bit << i);              // Shift the bit into place
    }

    return byte;
}

// Function to reset the DS18B20 and check for presence pulse
uint8_t DS18B20_Reset(void)
{
    // Pull the line low for 480µs (reset)
    DDRD |= (1 << DS18B20_PIN);
    PORTD &= ~(1 << DS18B20_PIN);
    _delay_us(480); // Reset pulse duration

    // Release the line and wait for 70µs for the presence pulse
    DDRD &= ~(1 << DS18B20_PIN);
    _delay_us(70);

    uint8_t presence = (PIND & (1 << DS18B20_PIN)) == 0; // Check for presence pulse (should be low)

    _delay_us(410); // Wait before releasing the line

    return presence; // 0 means no presence detected
}

void DS18B20_WaitForConversion(void)
{
    while (!DS18B20_ReadBit()) // Read sensor response
    {
        _delay_ms(10); // Small delay instead of fixed 800ms
    }
}

// Function to read the temperature from DS18B20
uint16_t DS18B20_ReadTemperature(void)
{
    uint16_t temperature = 0;

    // Reset the DS18B20 and check for presence
    if (!DS18B20_Reset())
    {
        serial_print("DS18B20 NOT FOUND!\n");
        return 0xFFFF; // Return 0xFFFF to indicate error
    }

    DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM); // New: send Skip ROM command
    DS18B20_WriteByte(DS18B20_CMD_CONVERT_TEMP);
    _delay_ms(800); // Wait 800ms for conversion to complete (minimum)

    // Reset the DS18B20 again before reading the scratchpad
    if (!DS18B20_Reset())
    {
        serial_print("DS18B20 not found!\n");
        return 0xFFFF; // Return 0xFFFF to indicate error
    }

    // Send Skip ROM command then "Read Scratchpad" command to read temperature data
    DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM); // New: send Skip ROM command
    DS18B20_WriteByte(DS18B20_CMD_READ_SCRATCHPAD);

    // Read the two bytes for temperature (low byte first, then high byte)
    uint16_t tempLow = DS18B20_ReadByte();
    uint16_t tempHigh = DS18B20_ReadByte();

    // Check if the read bytes are valid
    if (tempLow == 0xFF && tempHigh == 0xFF)
    {
        serial_print("Error: Read bytes are 0xFF\n");
        return 0xFFFF; // Return error
    }

    // Combine the low and high byte into a 16-bit value
    temperature = (tempHigh << 8) | tempLow;

    // Return the temperature
    return temperature;
}

int is_on = 0;

void init_led(void)
{
    DDRB = DDRB | (1 << DDB5);
}

void toggle_led(void)
{
    if (is_on)
    {
        PORTB = PORTB & ~(1 << PORTB5);
        is_on = 0;
    }
    else
    {
        PORTB = PORTB | (1 << PORTB5);
        is_on = 1;
    }
}

int main(void)
{
    serial_init(9600); // Initialize UART
    DS18B20_Init();    // Initialize DS18B20
    init_led();        // Arduino LED

    char buffer[50];
    uint16_t temperature;

    while (1)
    {
        toggle_led();
        // Read the temperature
        temperature = DS18B20_ReadTemperature();

        // If temperature is valid, send it over UART
        if (temperature != 0xFFFF) // Check for error (0xFFFF indicates error)
        {
            // Convert to signed integer
            int16_t tempRaw = (int16_t)temperature;

            // Integer part (shift right by 4)
            int8_t tempInt = tempRaw >> 4;

            // Fractional part (x0.0625 scale)
            uint8_t tempFrac = (tempRaw & 0x0F) * 625 / 1000;

            // Send formatted temperature over UART
            sprintf(buffer, "Temp: %d.%01d C\n", tempInt, tempFrac);
            serial_print(buffer);
        }
        else
        {
            // Error message in case of a failed temperature read
            serial_print("Error reading temperature\n");
        }
        toggle_led();
        // Update rate
        _delay_ms(1000);
    }

    return 0;
}
