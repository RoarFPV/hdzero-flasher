#include <SPI.h>
#include <Arduino.h>
#include <SerialFlash.h>

#include "flash.h"
#include "xmodem.h"

/* HDZERO Pinout
1 3.3V
2 ground
3 SPI CS
4 SPI CLK
5 SPI DI
6 SPI DIO
7 RTSN_5680
*/

#define RESET_PIN 0
#define FLASH_SPI_MISO 22
#define FLASH_SPI_MOSI 23
#define FLASH_SPI_SCLK 2
#define CS_PIN 17





#define UART1_TX 33
#define UART1_RX 32

#define UART2_TX 13
#define UART2_RX 25

bool spi_is_init = false;
SPIClass FLASH_SPI_PORT(HSPI);

HardwareSerial* uarts[] = { &Serial, &Serial1, &Serial2};

void setup()
{
  pinMode(RESET_PIN, OUTPUT);

  Serial.begin(115200);

  Serial1.begin(115200, SERIAL_8N1, UART1_TX, UART1_RX);
  Serial2.begin(115200, SERIAL_8N1, UART2_TX, UART2_RX);
}

void spi_init()
{
  if (spi_is_init)
  {
    return;
  }

  digitalWrite(RESET_PIN, LOW);
  delay(10);

  FLASH_SPI_PORT.begin(FLASH_SPI_SCLK, FLASH_SPI_MISO, FLASH_SPI_MOSI);
  FLASH_SPI_PORT.setFrequency(10000000);
  SerialFlash.begin(FLASH_SPI_PORT, CS_PIN);
  spi_is_init = true;
}

void print_chip_info()
{
  uint8_t id[64];
  SerialFlash.readID(id);
  Serial.print(F("JEDEC ID: "));
  Serial.print(id[0], HEX);
  Serial.print(' ');
  Serial.print(id[1], HEX);
  Serial.print(' ');
  Serial.println(id[2], HEX);

  Serial.print(F("Part Number: "));
  Serial.println(flash_part_number(id));

  Serial.print(F("Memory Size: "));
  const uint32_t chipsize = SerialFlash.capacity(id);
  Serial.print(chipsize);
  Serial.println(F(" bytes"));

  Serial.print(F("Block Size: "));
  const uint32_t blocksize = SerialFlash.blockSize();
  Serial.print(blocksize);
  Serial.println(F(" bytes"));
}

uint8_t monitor = 0;

bool update_serial_monitor()
{
  if(monitor < 1 || monitor> 3)
  {
    monitor = 0;
    return false;
  }

  if(Serial.available() > 0)
  {
    const uint8_t input = Serial.read();
    
    switch (input)
    {
    case 'e':
      monitor =0;
      return false;
    
    case '1':
    case '2':
    
    monitor = atoi((const char*)&input);
    Serial.printf("SM: Entering Serial Monitor %u Mode\n", monitor);
    break;

    default:
      break;
    }
  }

  auto targetSerial = uarts[monitor];
  if( targetSerial == nullptr)
    return false;

  while( targetSerial->available() > 0 )
  {
    Serial.write(targetSerial->read());
  }

  return true;
}

void loop()
{
  if(update_serial_monitor())
    return;


  while (Serial.available() == 0)
  {
    delay(1);
  }

  const uint8_t input = Serial.read();
  switch (input)
  {
  case 'p':
    spi_init();
    print_chip_info();
    break;

  case 'f':
    spi_init();
    xmodem_receive();
    // Fallthrough

  case 'e':
    Serial.println("Exiting...");
    spi_is_init = false;
    FLASH_SPI_PORT.end();
    delay(10);

    digitalWrite(RESET_PIN, HIGH);
    break;

  case '1':
  case '2':
    monitor = atoi((const char*)&input);
    Serial.printf("Entering Serial Monitor %u Mode\n", monitor);
    break;

  case '\r':
  case '\n':
    break;

  default:
    Serial.printf("Unknown command %c\r\n", input);
    break;
  }
}
