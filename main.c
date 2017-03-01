/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include <stdio.h>
#include <stdint.h>
#include "stm32f4_delay.h"
#include "hardware.h"

#define LCD_DC      GPIO_Pin_14
#define LCD_CE      GPIO_Pin_13
#define LCD_RST     GPIO_Pin_15
#define LCD_DIN     GPIO_Pin_3
#define LCD_CLK     GPIO_Pin_10
#define LCD_CLK_LINE  GPIOB

//Deklaracja tablicy znaków
const uint8_t font_ASCII[][5] = { { 0x00, 0x00, 0x00, 0x00, 0x00 } // 20
                , { 0x00, 0x00, 0x5f, 0x00, 0x00 } // 21 !
                , { 0x00, 0x07, 0x00, 0x07, 0x00 } // 22 "
                , { 0x14, 0x7f, 0x14, 0x7f, 0x14 } // 23 #
                , { 0x24, 0x2a, 0x7f, 0x2a, 0x12 } // 24 $
                , { 0x23, 0x13, 0x08, 0x64, 0x62 } // 25 %
                , { 0x36, 0x49, 0x55, 0x22, 0x50 } // 26 &
                , { 0x00, 0x05, 0x03, 0x00, 0x00 } // 27 '
                , { 0x00, 0x1c, 0x22, 0x41, 0x00 } // 28 (
                , { 0x00, 0x41, 0x22, 0x1c, 0x00 } // 29 )
                , { 0x14, 0x08, 0x3e, 0x08, 0x14 } // 2a *
                , { 0x08, 0x08, 0x3e, 0x08, 0x08 } // 2b +
                , { 0x00, 0x50, 0x30, 0x00, 0x00 } // 2c ,
                , { 0x08, 0x08, 0x08, 0x08, 0x08 } // 2d -
                , { 0x00, 0x60, 0x60, 0x00, 0x00 } // 2e .
                , { 0x20, 0x10, 0x08, 0x04, 0x02 } // 2f /
                , { 0x3e, 0x51, 0x49, 0x45, 0x3e } // 30 0
                , { 0x00, 0x42, 0x7f, 0x40, 0x00 } // 31 1
                , { 0x42, 0x61, 0x51, 0x49, 0x46 } // 32 2
                , { 0x21, 0x41, 0x45, 0x4b, 0x31 } // 33 3
                , { 0x18, 0x14, 0x12, 0x7f, 0x10 } // 34 4
                , { 0x27, 0x45, 0x45, 0x45, 0x39 } // 35 5
                , { 0x3c, 0x4a, 0x49, 0x49, 0x30 } // 36 6
                , { 0x01, 0x71, 0x09, 0x05, 0x03 } // 37 7
                , { 0x36, 0x49, 0x49, 0x49, 0x36 } // 38 8
                , { 0x06, 0x49, 0x49, 0x29, 0x1e } // 39 9
                , { 0x00, 0x36, 0x36, 0x00, 0x00 } // 3a :
                , { 0x00, 0x56, 0x36, 0x00, 0x00 } // 3b ;
                , { 0x08, 0x14, 0x22, 0x41, 0x00 } // 3c <
                , { 0x14, 0x14, 0x14, 0x14, 0x14 } // 3d =
                , { 0x00, 0x41, 0x22, 0x14, 0x08 } // 3e >
                , { 0x02, 0x01, 0x51, 0x09, 0x06 } // 3f ?
                , { 0x32, 0x49, 0x79, 0x41, 0x3e } // 40 @
                , { 0x7e, 0x11, 0x11, 0x11, 0x7e } // 41 A
                , { 0x7f, 0x49, 0x49, 0x49, 0x36 } // 42 B
                , { 0x3e, 0x41, 0x41, 0x41, 0x22 } // 43 C
                , { 0x7f, 0x41, 0x41, 0x22, 0x1c } // 44 D
                , { 0x7f, 0x49, 0x49, 0x49, 0x41 } // 45 E
                , { 0x7f, 0x09, 0x09, 0x09, 0x01 } // 46 F
                , { 0x3e, 0x41, 0x49, 0x49, 0x7a } // 47 G
                , { 0x7f, 0x08, 0x08, 0x08, 0x7f } // 48 H
                , { 0x00, 0x41, 0x7f, 0x41, 0x00 } // 49 I
                , { 0x20, 0x40, 0x41, 0x3f, 0x01 } // 4a J
                , { 0x7f, 0x08, 0x14, 0x22, 0x41 } // 4b K
                , { 0x7f, 0x40, 0x40, 0x40, 0x40 } // 4c L
                , { 0x7f, 0x02, 0x0c, 0x02, 0x7f } // 4d M
                , { 0x7f, 0x04, 0x08, 0x10, 0x7f } // 4e N
                , { 0x3e, 0x41, 0x41, 0x41, 0x3e } // 4f O
                , { 0x7f, 0x09, 0x09, 0x09, 0x06 } // 50 P
                , { 0x3e, 0x41, 0x51, 0x21, 0x5e } // 51 Q
                , { 0x7f, 0x09, 0x19, 0x29, 0x46 } // 52 R
                , { 0x46, 0x49, 0x49, 0x49, 0x31 } // 53 S
                , { 0x01, 0x01, 0x7f, 0x01, 0x01 } // 54 T
                , { 0x3f, 0x40, 0x40, 0x40, 0x3f } // 55 U
                , { 0x1f, 0x20, 0x40, 0x20, 0x1f } // 56 V
                , { 0x3f, 0x40, 0x38, 0x40, 0x3f } // 57 W
                , { 0x63, 0x14, 0x08, 0x14, 0x63 } // 58 X
                , { 0x07, 0x08, 0x70, 0x08, 0x07 } // 59 Y
                , { 0x61, 0x51, 0x49, 0x45, 0x43 } // 5a Z
                , { 0x00, 0x7f, 0x41, 0x41, 0x00 } // 5b [
                , { 0x02, 0x04, 0x08, 0x10, 0x20 } // 5c
                , { 0x00, 0x41, 0x41, 0x7f, 0x00 } // 5d ]
                , { 0x04, 0x02, 0x01, 0x02, 0x04 } // 5e ^
                , { 0x40, 0x40, 0x40, 0x40, 0x40 } // 5f _
                , { 0x00, 0x01, 0x02, 0x04, 0x00 } // 60 `
                , { 0x20, 0x54, 0x54, 0x54, 0x78 } // 61 a
                , { 0x7f, 0x48, 0x44, 0x44, 0x38 } // 62 b
                , { 0x38, 0x44, 0x44, 0x44, 0x20 } // 63 c
                , { 0x38, 0x44, 0x44, 0x48, 0x7f } // 64 d
                , { 0x38, 0x54, 0x54, 0x54, 0x18 } // 65 e
                , { 0x08, 0x7e, 0x09, 0x01, 0x02 } // 66 f
                , { 0x0c, 0x52, 0x52, 0x52, 0x3e } // 67 g
                , { 0x7f, 0x08, 0x04, 0x04, 0x78 } // 68 h
                , { 0x00, 0x44, 0x7d, 0x40, 0x00 } // 69 i
                , { 0x20, 0x40, 0x44, 0x3d, 0x00 } // 6a j
                , { 0x7f, 0x10, 0x28, 0x44, 0x00 } // 6b k
                , { 0x00, 0x41, 0x7f, 0x40, 0x00 } // 6c l
                , { 0x7c, 0x04, 0x18, 0x04, 0x78 } // 6d m
                , { 0x7c, 0x08, 0x04, 0x04, 0x78 } // 6e n
                , { 0x38, 0x44, 0x44, 0x44, 0x38 } // 6f o
                , { 0x7c, 0x14, 0x14, 0x14, 0x08 } // 70 p
                , { 0x08, 0x14, 0x14, 0x18, 0x7c } // 71 q
                , { 0x7c, 0x08, 0x04, 0x04, 0x08 } // 72 r
                , { 0x48, 0x54, 0x54, 0x54, 0x20 } // 73 s
                , { 0x04, 0x3f, 0x44, 0x40, 0x20 } // 74 t
                , { 0x3c, 0x40, 0x40, 0x20, 0x7c } // 75 u
                , { 0x1c, 0x20, 0x40, 0x20, 0x1c } // 76 v
                , { 0x3c, 0x40, 0x30, 0x40, 0x3c } // 77 w
                , { 0x44, 0x28, 0x10, 0x28, 0x44 } // 78 x
                , { 0x0c, 0x50, 0x50, 0x50, 0x3c } // 79 y
                , { 0x44, 0x64, 0x54, 0x4c, 0x44 } // 7a z
                , { 0x00, 0x08, 0x36, 0x41, 0x00 } // 7b {
                , { 0x00, 0x00, 0x7f, 0x00, 0x00 } // 7c |
                , { 0x00, 0x41, 0x36, 0x08, 0x00 } // 7d }
                , { 0x10, 0x08, 0x08, 0x10, 0x08 } // 7e ~
                , { 0x78, 0x46, 0x41, 0x46, 0x78 } // 7f DEL
};

uint32_t ADCValue[1] = { 0 };

uint8_t lcd_buffer[84 * 48 / 8];

//Wyslanie komendy na wyswietlacz
static uint8_t SPISend(uint8_t byte);

//Funkcja pozwalajaca na przesylanie danych do ukladu
static void NOKIACMD(uint8_t cmd);

//Inicjalicja wyswietlacza
void NokiaInit(void);

//Czyszczenie wyswietlacza
void LCDClear(void);

void LCDDraw(int row, int col, const char* text);

//Inicjalizacja portów GPIO
void GPIOInit_Nokia(void);

//Inicjalizacja SPI
void SPIInit_Nokia(void);
void LCDCopy(void);
//Inicjaizacja ADC
void ADCInit(void);
//Inicjaizacja DMA
void DMAInit(void);

int main(void) {

        //Inicjalizacja funkcji
        ADCInit();
        DMAInit();
        systickInit(1000);
        GPIOInit_Nokia();
        SPIInit_Nokia();
        NokiaInit();
        board_leds_init();
        while (1)
                ;
}
void board_leds_init(void) {
        GPIO_InitTypeDef GPIO_InitStructure;
        // always do this with an auto structure as it is undefined
        GPIO_StructInit(&GPIO_InitStructure);
        RCC_AHB1PeriphClockCmd(LED_PORT_CLOCK, ENABLE);
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = GREEN_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(LED_PORT, &GPIO_InitStructure);
        GPIO_ResetBits(LED_PORT, GREEN_PIN);
}

void ADCInit() {
        //Definicja struktury
        ADC_InitTypeDef ADCInit;
        GPIO_InitTypeDef GPIOInit;

        //Konfiguracja zegarów
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
        //Zegar dla portu ADC
        RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN, ENABLE);

        //Konfiguracja pinu analogowego
        //Kanal 12 podlaczony do PC2
        GPIOInit.GPIO_Pin = GPIO_Pin_2;
        GPIOInit.GPIO_Mode = GPIO_Mode_AN;
        GPIOInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIOInit.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(GPIOC, &GPIOInit);

        //Konfiguracja ADC
        ADC_DeInit();
        //Przetworzone dane wyrównane do prawej
        ADCInit.ADC_DataAlign = ADC_DataAlign_Right;
        //Dane wejsciowe przetwarzane w 12 bitowy numer
        //z duza dokladnoscia, max 4096
        ADCInit.ADC_Resolution = ADC_Resolution_12b;
        //Konwersacja jest ciagla przetwarzanie
        //wiecej niz jeden raz
        ADCInit.ADC_ContinuousConvMode = ENABLE;
        ADCInit.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
        //Bez wyzwalania zewnetrznego
        ADCInit.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
        //Liczba konwersji
        ADCInit.ADC_NbrOfConversion = 16;
        //Pomiar jednego kanalu, skanowanie kanalow zostalo wylaczone
        ADCInit.ADC_ScanConvMode = DISABLE;
        //Inicjalizacja ADC
        ADC_Init(ADC1, &ADCInit);

        //Wybranie kanalu z którego bedzie odczytywane
        //ADC kanal 12, GPIOC2, Czestotliwosc probkowania = 1Mhz
        ADC_RegularChannelConfig( ADC1, ADC_Channel_12, 1,
        ADC_SampleTime_144Cycles);

        ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

        //Wlaczenie DMA dla ADC1
        ADC_DMACmd(ADC1, ENABLE);

        //Wlaczenie konwersji ADC
        ADC_Cmd(ADC1, ENABLE);

        //Rozpoczecie konwersji
        ADC_SoftwareStartConv(ADC1);
}

//Konfiguracja DMA
void DMAInit(void) {
        DMA_InitTypeDef DMAInit;

        DMA_StructInit(&DMAInit);

        //Wlaczenie zegara taktujacego
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

        //Wlaczenie ustawien standardowych
        DMA_DeInit(DMA2_Stream4);
        DMAInit.DMA_Channel = DMA_Channel_0;
        //deklaracja zrodla danych, rejestr ADC1 DR
        DMAInit.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
        //Adres poczatku bloku do przeslania,
        DMAInit.DMA_Memory0BaseAddr = (uint32_t) &ADCValue[0];
        //Kierunek transferu danych z urzadzenia do pamieci
        DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
        //Rozmiar bufora
        DMAInit.DMA_BufferSize = 1;
        //Wylaczenie automatycznego zwiekszania adresu ADC
        DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        //Wylaczenie obslugi transferu z pamieci do pamieci
        DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
        //Rozmiar przesylanych danych ADC, 16b
        DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        //Rozmiar przesylanych danych po stronie pamieci
        DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        //Tryb pracy ciagly
        DMAInit.DMA_Mode = DMA_Mode_Circular;
        //Ustawienie priorytetu na wysoki
        DMAInit.DMA_Priority = DMA_Priority_High;

        //Wylaczenie trybu fifom, automatycznie wlaczony tryb bezposredni
        DMAInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMAInit.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;

        //Wyzwalanie pojedyncze
        DMAInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMAInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

        //Inicjalizacja DMA
        DMA_Init(DMA2_Stream4, &DMAInit);
        //Wlaczenie DMA
        DMA_Cmd(DMA2_Stream4, ENABLE);
}

//Wyslanie komendy na wyswietlacz
static uint8_t SPISend(uint8_t byte) {
        //Petla while czeka az bufor nadawczy bedzie wolny
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
                ;
        GPIO_ResetBits(GPIOC, LCD_CE);
        SPI_I2S_SendData(SPI2, byte);

        // poczekaj na dane w buforze odbiorczym
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
                ;
        GPIO_SetBits(GPIOC, LCD_CE);
        return SPI_I2S_ReceiveData(SPI2);
}

//Funkcja pozwalajaca na przesylanie danych do ukladu
static void NOKIACMD(uint8_t cmd) {
        //Ustawienie stanu niskiego na CE i DC
        GPIO_ResetBits(GPIOC, LCD_CE | LCD_DC);
        //Przeslanie komendy
        SPISend(cmd);
        //Ustawienie stanu wysokiego na ChipSelect
        GPIO_SetBits(GPIOC, LCD_CE);
}

//Inicjalicja wyswietlacza
void NokiaInit(void) {
        //Sygnal niski na RST, reset wyswietlacza
        GPIO_ResetBits(GPIOC, LCD_RST);

        //Stan wysoki na pin reset
        GPIO_SetBits(GPIOC, LCD_RST);

        //Wlaczenie zasilania wyswietlacza
        //Function SET 00100001
        NOKIACMD(0x21);

        //Napiecie zasilania matrycy VOP 11001000
        NOKIACMD(0xc8);

        //Wybór krzywej kompensacji temperatury
        //00001100
        NOKIACMD(0x06);

        //Wspólczynnik multipleksowania 00010011
        NOKIACMD(0x13);

        //Wlaczenie zasilania matrycy 00100000
        NOKIACMD(0x20);

        //Standardowy tryb pracy 00001100
        //Do wyboru sa dwa tryby pracy
        NOKIACMD(0x0c);

        //Zresetowanie licznika wierszy
        NOKIACMD(0x40);

        //Zresetowanie liczniki kolumn
        NOKIACMD(0x80);
}

//Czyszczenie wyswietlacza
void LCDClear(void) {
        //void * memset ( void * buffer, int c, size_t num );
        //Wypelnia kolejne bajty w pamieci ustalona wartoscia
        //W tym przypadku jest to zero, czyszczony jest wyswietlacz
        //Ustawienie stanu niskiego na CE i DC
        memset(lcd_buffer, 0, (84 * 48 / 8));
}

void LCDDraw(int row, int col, const char* text) {
        //Deklaracja zmiennych
        int i;
        uint8_t* pbuf = &lcd_buffer[row * 84 + col];

        //Dopóki bedzie jakis tekst do wyswietlania oraz bedzie mozliwosc
        //wpisania danych wtedy funkcja bedzie wykonywana
        while ((*text) && (pbuf < &lcd_buffer[(84 * 48 / 8) - 6])) {
                //Wskaznik na wartosc, zostaje ona zwiekszona o 1
                int ch = *text++;
                //Deklaracja zmiennej w kodzie ASCII
                //z wskaznikiem na jej adres.
                //Liczenie od 0 odejmnowanie znaku spacji
                const uint8_t* font = &font_ASCII[ch - ' '][0];
                //Przejscie w tablicy po poszczególnych
                //wartosciach podanego znaku
                for (i = 0; i < 5; i++)
                        *pbuf++ = *font++;
                *pbuf++ = 0;
        }

        i = 0;

        //Ustawienie lini DC na wysoki
        GPIO_SetBits(GPIOC, LCD_DC);
        //Wybranie urzadzenia, niski na CE
        GPIO_ResetBits(GPIOC, LCD_CE);

        for (i = 0; i < (84 * 48 / 8); i++)
                SPISend(lcd_buffer[i]);

        GPIO_SetBits(GPIOC, LCD_CE);
}

//Inicjalizacja portów GPIO
void GPIOInit_Nokia(void) {
        GPIO_InitTypeDef GPIOInit;

        //Wlaczenie zegarów
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

        //Definicja funkcji alternatywnych dla pinów MOSI oraz CLK
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);

        //Inicjalizacja pinów MOSI
        GPIO_StructInit(&GPIOInit);
        GPIOInit.GPIO_Pin = LCD_DIN;
        GPIOInit.GPIO_Mode = GPIO_Mode_AF;
        GPIOInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIOInit.GPIO_OType = GPIO_OType_PP;
        GPIOInit.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(GPIOC, &GPIOInit);

        //Inicjalizacja SCK
        GPIO_StructInit(&GPIOInit);
        GPIOInit.GPIO_Pin = LCD_CLK;
        GPIOInit.GPIO_Mode = GPIO_Mode_AF;
        GPIOInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIOInit.GPIO_OType = GPIO_OType_PP;
        GPIOInit.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(LCD_CLK_LINE, &GPIOInit);

        //Inicjalizacja MISO, niewykorzystywane
        GPIOInit.GPIO_Pin = GPIO_Pin_2;
        GPIOInit.GPIO_Mode = GPIO_Mode_AF;
        GPIOInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIOInit.GPIO_OType = GPIO_OType_PP;
        GPIOInit.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(GPIOB, &GPIOInit);

        //Wlaczenie pozostalych linii
        GPIOInit.GPIO_Pin = LCD_DC | LCD_CE | LCD_RST;
        GPIOInit.GPIO_Mode = GPIO_Mode_OUT;
        GPIOInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIOInit.GPIO_OType = GPIO_OType_PP;
        GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIOInit);

        //Ustawienie stanów wysokich na liniach CE oraz RST.
        GPIO_SetBits(GPIOC, LCD_CE | LCD_RST);
}

//Inicjalizacja SPI
void SPIInit_Nokia(void) {
        SPI_InitTypeDef SPIInit;

        //Wlaczenie zegara dla SPI
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

        SPI_StructInit(&SPIInit);
        //Transmisja z wykorzystaniem dwóch linii
        SPIInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        //Tryb pracy mikrokontrolera
        SPIInit.SPI_Mode = SPI_Mode_Master;
        //Stan sygnalu taktujacego przy braku transmisji
        SPIInit.SPI_CPOL = SPI_CPOL_Low;
        //Aktywne zbocze sygnalu taktujacego
        SPIInit.SPI_CPHA = SPI_CPHA_1Edge;
        //Wylaczenie sprzetowej obslugi linii CS
        SPIInit.SPI_NSS = SPI_NSS_Soft;
        //Rozmiar danych
        SPIInit.SPI_DataSize = SPI_DataSize_8b;
        //Co idzie pierwsze w transmisji
        SPIInit.SPI_FirstBit = SPI_FirstBit_MSB;
        //Szybkosc transmisji
        SPIInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
        //Wylaczenie SPI
        SPI2->CR1 &= ~SPI_CR1_SPE;
        //Inicjalizacja SPI
        SPI_Init(SPI2, &SPIInit);
        //Wlaczenie SPI
        SPI_Cmd(SPI2, ENABLE);
}
void systickInit(uint16_t frequency) {
        RCC_ClocksTypeDef RCC_Clocks;
        RCC_GetClocksFreq(&RCC_Clocks);
        (void) SysTick_Config(RCC_Clocks.HCLK_Frequency / frequency);
}

void SysTick_Handler(void) {
        static uint8_t counter;
        if (counter++ > (200 - 1)) {
                counter = 0;
                calculations();
        }
}
void ToggleLed(void) {
        static int i = 0;
        if (i == 0) {
                i++;
                GPIOD->BSRRL = 0x1000;
        } else {
                GPIOD->BSRRH = 0x1000;
                i--;
        }
}

void calculations(void) {
        //Deklaracja zmiennych
        char zrodlo[20];
        int handler = 0;
        int i = 0;
        //Migaj Dioda
        ToggleLed();
        //Narysuj PROJEKT KP
        LCDDraw(1, 1, "PROJEKT KP");
        //Narysuj ODCZYT Z ADC
        LCDDraw(2, 1, "Odczyt z ADC:");
        //Pobranie 10 wartosci z DMA
        for (i = 0; i < 10; i++) {
                handler += ADCValue[0];
        }

        //Wyciagniecie sredniej arytmetycznej
        handler = (handler / 10);

        //Wyswietlenie wyniku napiecie na plytce
        sprintf(zrodlo, "A:%d", handler);
        LCDDraw(3, 1, zrodlo);

        sprintf(zrodlo, "V:%.2f", ((3.3 / 4096.0) * (handler)));
        LCDDraw(4, 1, zrodlo);

        //Wyzerowanie zmiennej
        handler = 0;

        LCDClear();
}
