#include "hardware/imu.h"

namespace hardware
{
    IMU::IMU(float sampling_period, float gyro_factor)
        : sampling_period(sampling_period),
          gyro_factor(gyro_factor),
          theta(0),
          offset_gz(0) {}

    uint8_t IMU::read_byte(uint8_t reg)
    {
        uint8_t rx_data[2];
        uint8_t tx_data[2];

        tx_data[0] = reg | 0x80;
        tx_data[1] = 0x00; // dummy

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

        return rx_data[1];
    }

    void IMU::write_byte(uint8_t reg, uint8_t data)
    {
        uint8_t rx_data[2];
        uint8_t tx_data[2];

        tx_data[0] = reg & 0x7F;
        //   tx_data[0] = reg | 0x00;
        tx_data[1] = data; // write data

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CSピン立ち下げ
        HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CSピン立ち上げ
    }

    void IMU::Initialize()
    {
        uint8_t who_am_i;

        HAL_Delay(100);                          // wait start up
        who_am_i = read_byte(0x75);              // read who am i
        printf("who_am_i = 0x%x\r\n", who_am_i); // check who am i value
        HAL_Delay(10);
        while (who_am_i != 0x70)
        {
            who_am_i = read_byte(0x75);
            printf("who_am_i = 0x%x\r\n", who_am_i);
            HAL_Delay(20);
        }

        HAL_Delay(50);
        write_byte(0x68, 0x00); // set pwr_might (20MHz)
        HAL_Delay(50);
        write_byte(0x1A, 0x00); // set config (FSYNCはNC)
        HAL_Delay(50);
        write_byte(0x1B, 0x18); // set gyro config (2000dps)
        HAL_Delay(50);
    }

    void IMU::CalcOffset()
    {
        int16_t gz_raw;
        float gz_sum = 0;
        for (int i = 0; i < 1000; i++)
        {
            // H:8bit shift, Link h and l
            gz_raw = (int16_t)((uint16_t)(read_byte(0x47) << 8) | (uint16_t)read_byte(0x48));
            // printf("%d\r\n", gz_raw);
            gyro_z = (float)(gz_raw / gyro_factor) * M_PI / 180.0f; // dps to rad/sec

            gz_sum += gyro_z;
            HAL_Delay(1);
        }
        offset_gz = gz_sum / 1000.0f;
    }

    void IMU::Update()
    {
        UpdateGyro();
        // UpdateAcc();
    }

    void IMU::UpdateGyro()
    {
        int16_t gz_raw;

        // H:8bit shift, Link h and l
        gz_raw = (int16_t)((uint16_t)(read_byte(0x47) << 8) | (uint16_t)read_byte(0x48));

        gyro_z = (float)(gz_raw / gyro_factor) * M_PI / 180.0f - offset_gz; // dps to deg/sec
        theta += gyro_z * sampling_period;

        static int a = 0;
        if (a % 200 == 0)
        {
            printf("%f\r\n", theta);
        }
        a++;
    }

    float IMU::GetAngle()
    {
        return theta;
    }

    float IMU::GetAngularVelocity()
    {
        return gyro_z;
    }

} // namespace hardware