#include "ad9833.h"

#define FCLK 25000000 // 设置晶振频率
#define RealFreDat 268435456.0 / FCLK // 总的公式为 Fout=（Fclk/2的28次方）*28位寄存器的值
#define delay_ms(ms) HAL_Delay(ms)
/**
 * @brief Writes the value to a register.
 *
 * @param -  regValue - The value to write to the register.
 *
 * @return  None.
 **/
void AD9833_SetRegisterValue(SPI_HandleTypeDef hspi, uint16_t regValue)
{
    HAL_SPI_Transmit(&hspi, (uint8_t *)&regValue, 1, 100);
}

/**
 * @brief Initializes the SPI communication peripheral and resets the part.
 *
 * @return 1.
 **/
unsigned char AD9833_Init(SPI_HandleTypeDef hspi)
{
    AD9833_SetRegisterValue(hspi, AD9833_REG_CMD | AD9833_RESET);
    return (1);
}

/**
 * @brief Sets the Reset bit of the AD9833.
 *
 * @return None.
 **/
void AD9833_Reset(SPI_HandleTypeDef hspi)
{
    AD9833_SetRegisterValue(hspi, AD9833_REG_CMD | AD9833_RESET);
    delay_ms(10);
}

/**
 * @brief Clears the Reset bit of the AD9833.
 *
 * @return None.
 **/
void AD9833_ClearReset(SPI_HandleTypeDef hspi)
{
    AD9833_SetRegisterValue(hspi, AD9833_REG_CMD);
    delay_ms(10);
}

/**
 * @brief Writes to the frequency registers.
 *
 * @param -  reg - Frequence register to be written to.
 * @param -  val - The value to be written.
 *
 * @return  None.
 **/
void AD9833_SetFrequency(SPI_HandleTypeDef hspi, unsigned short reg, float fout, unsigned short type)
{
    unsigned short freqHi = reg;
    unsigned short freqLo = reg;
    unsigned long val = RealFreDat * fout;
    freqHi |= (val & 0xFFFC000) >> 14;
    freqLo |= (val & 0x3FFF);
    AD9833_SetRegisterValue(hspi, AD9833_B28 | type);
    AD9833_SetRegisterValue(hspi, freqLo);
    AD9833_SetRegisterValue(hspi, freqHi);
}

/**
 * @brief Writes to the phase registers.
 *
 * @param -  reg - Phase register to be
 * written to.
 * @param -  val - The value to be
 * written.
 *
 * @return  None.
 **/
void AD9833_SetPhase(SPI_HandleTypeDef hspi, unsigned short reg, unsigned short val)
{
    unsigned short phase = reg;
    phase |= val;
    AD9833_SetRegisterValue(hspi, phase);
}

/**
 * @brief Selects the Frequency,Phase and
 * Waveform type.
 *
 * @param -  freq  - Frequency register
 * used.
 * @param -  phase - Phase register used.
 * @param -  type  - Type of waveform to
 * be output.
 *
 * @return  None.
 * AD9833_Setup(1000,0,AD9833_OUT_SINUS);
 **/
void AD9833_Setup(SPI_HandleTypeDef hspi, unsigned short freq, unsigned short phase, unsigned short type)
{
    unsigned short val = 0;

    val = freq | phase | type;
    AD9833_SetRegisterValue(hspi, val);
}

/**
 * @brief Sets the type of waveform to be
 * output.
 *
 * @param -  type - type of waveform to be
 * output.
 *
 * @return  None.
 **/
void AD9833_SetWave(SPI_HandleTypeDef hspi, unsigned short type)
{
    AD9833_SetRegisterValue(hspi, type);
}

/**
 * @brief Sets frequency (Hz), phase
 * (degrees), and waveform in one call.
 *
 * @param -  freq_hz   - Desired output
 * frequency in Hz.
 * @param -  phase_deg - Desired output
 * phase in degrees (0-360).
 * @param -  type      - Type of waveform
 * to be output (e.g., AD9833_OUT_SINUS).
 *
 * @return  None.
 **/
void AD9833_SetOutput(SPI_HandleTypeDef hspi, float freq_hz, float phase_deg, unsigned short type)
{
    unsigned short phase_word = 0;

    if (phase_deg >= 360.0f || phase_deg < 0.0f)
    {
        while (phase_deg >= 360.0f)
        {
            phase_deg -= 360.0f;
        }
        while (phase_deg < 0.0f)
        {
            phase_deg += 360.0f;
        }
    }

    phase_word = (unsigned short)(phase_deg * (4096.0f / 360.0f) + 0.5f);
    phase_word &= 0x0FFF;

    AD9833_SetFrequency(hspi, AD9833_REG_FREQ0, freq_hz, type);
    AD9833_SetPhase(hspi, AD9833_REG_PHASE0, phase_word);
    AD9833_Setup(hspi, AD9833_FSEL0, AD9833_PSEL0, type);
}

void AD9833_SetFrequencyQuick(SPI_HandleTypeDef hspi, float fout, unsigned short type)
{

    // AD9833_Setup(AD9833_FSEL0, AD9833_PSEL0, type);
    AD9833_SetFrequency(hspi, AD9833_REG_FREQ0, fout, type); // 400 kHz
}
