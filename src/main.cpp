#include "mbed.h"
#include "FlashIAP.h"
#include <array>
#include <vector>
#include <algorithm>

#define CTRL_REG1 0x20                   // Control register 1 address
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1 // Configuration: ODR=100Hz, Enable X/Y/Z axes, power on
#define CTRL_REG4 0x23                   // Control register 4 address
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0 // Configuration: High-resolution, 2000dps sensitivity
#define SPI_FLAG 1
#define OUT_X_L 0x28
#define DEG_TO_RAD (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)
// Flash memory settings
#define FLASH_ADDRESS 0x08020000 // Base address of Sector 5
#define FLASH_SIZE 131072        // Sector size in bytes (128KB)
FlashIAP flash;

EventFlags flags;

using GyroData = std::array<float, 3>;

SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
uint8_t write_buf[32], read_buf[32];

void spi_cb(int event)
{
    flags.set(SPI_FLAG); // Set the SPI_FLAG to signal that transfer is complete
}

template <typename T>
bool storeToFlash(const T *data, size_t num_elements, uint32_t flash_address, size_t flash_size)
{
    FlashIAP flash;

    // Calculate total data size
    size_t data_size = num_elements * sizeof(T);

    // Check if data size fits in the allocated flash region
    if (data_size > flash_size)
    {
        printf("Error: Data size exceeds allocated flash space.\n");
        return false;
    }

    // Align data size to the flash block size
    const size_t block_size = flash.get_page_size(); // Minimum writable block size
    if (data_size % block_size != 0)
    {
        data_size = ((data_size / block_size) + 1) * block_size; // Align to block size
    }

    // Initialize flash
    int result = flash.init();
    if (result != 0)
    {
        printf("Error: Failed to initialize flash.\n");
        return false;
    }

    // Erase the target flash region
    result = flash.erase(flash_address, flash_size);
    if (result != 0)
    {
        printf("Error: Failed to erase flash sector.\n");
        flash.deinit();
        return false;
    }

    // Write data to flash
    result = flash.program(data, flash_address, data_size);
    if (result != 0)
    {
        printf("Error: Failed to write to flash memory.\n");
        flash.deinit();
        return false;
    }

    // Deinitialize flash
    flash.deinit();

    printf("Data successfully stored to flash.\n");
    return true;
}

void initializeGyroscope(SPI &spi, uint8_t *write_buf, uint8_t *read_buf)
{
    // Configure Control Register 1 (CTRL_REG1)
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Configure Control Register 4 (CTRL_REG4)
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);
}

// Function to read gyroscope data
GyroData readGyroscopeData(SPI &spi, uint8_t *write_buf, uint8_t *read_buf)
{
    uint16_t raw_gx, raw_gy, raw_gz;
    GyroData gyroData;
    // Prepare to read gyroscope output starting at OUT_X_L
    write_buf[0] = OUT_X_L | 0x80 | 0x40; // Read mode + auto-increment

    // Perform SPI transfer to read 6 bytes (X, Y, Z axis data)
    spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Extract and Convert Raw Data
    raw_gx = (((uint16_t)read_buf[2]) << 8) | read_buf[1];
    raw_gy = (((uint16_t)read_buf[4]) << 8) | read_buf[3];
    raw_gz = (((uint16_t)read_buf[6]) << 8) | read_buf[5];

    // Debug and Teleplot Output
    /* printf("RAW Angular Speed -> gx: %d deg/s, gy: %d deg/s, gz: %d deg/s\n", raw_gx, raw_gy, raw_gz);
    printf(">x_axis: %d|g\n", raw_gx);
    printf(">y_axis: %d|g\n", raw_gy);
    printf(">z_axis: %d|g\n", raw_gz); */

    // Convert Raw Data to Angular Velocity

    gyroData[0] = raw_gx * DEG_TO_RAD;
    gyroData[1] = raw_gy * DEG_TO_RAD;
    gyroData[2] = raw_gz * DEG_TO_RAD;

    return gyroData;

    // Print converted values (angular velocity in rad/s)
    /* printf("Angular Speed -> gx: %.5f rad/s, gy: %.5f rad/s, gz: %.5f rad/s\n", gx, gy, gz); */
}

// Function to trim gyro data (remove values below threshold)
std::vector<GyroData> trim_gyro_data(const std::vector<GyroData> &data, float threshold = 0.01f)
{
    auto is_below_threshold = [threshold](const GyroData &d)
    {
        return std::all_of(d.begin(), d.end(), [threshold](float v)
                           { return fabs(v) < threshold; });
    };

    auto first_valid = std::find_if_not(data.begin(), data.end(), is_below_threshold);
    auto last_valid = std::find_if_not(data.rbegin(), data.rend(), is_below_threshold).base();

    return (first_valid < last_valid) ? std::vector<GyroData>(first_valid, last_valid) : std::vector<GyroData>();
}

// Function to store gyro data to flash memory
void storeGyroDataToFlash(const std::vector<GyroData> &data)
{
    size_t data_size = data.size() * sizeof(GyroData);

    printf("\t/******************************************/\n");
    printf("\t/************data to be stored*************/\n");
    printf("\t/******************************************/\n");

    for (const auto &gyro : data)
    {
        printf("gx: %.5f, gy: %.5f, gz: %.5f\n", gyro[0], gyro[1], gyro[2]);
    }

    // Ensure the data size fits within the flash block
    if (data_size > FLASH_SIZE)
    {
        printf("Error: Data too large to store in flash memory.\n");
        return;
    }

    flash.init();
    flash.erase(FLASH_ADDRESS, FLASH_SIZE);               // Erase flash sector
    flash.program(data.data(), FLASH_ADDRESS, data_size); // Write data to flash
    flash.deinit();
}

/* std::vector<GyroData> readGyroDataFromFlash()
{
    flash.init();

    // Calculate the number of entries stored
    size_t data_size = FLASH_SIZE / sizeof(GyroData);
    std::vector<GyroData> data(data_size);

    // Copy data from flash
    memcpy(data.data(), reinterpret_cast<const void *>(FLASH_ADDRESS), FLASH_SIZE);

    flash.deinit();

    // Debug retrieved data
    printf("Data retrieved from flash:\n");
    for (const auto &gyro : data)
    {
        printf("gx: %.5f, gy: %.5f, gz: %.5f\n", gyro[0], gyro[1], gyro[2]);
    }

    return data;
} */

std::vector<GyroData> readGyroDataFromFlash()
{
    FlashIAP flash;
    flash.init();

    // Calculate the number of entries stored
    size_t data_size = FLASH_SIZE / sizeof(GyroData);
    std::vector<GyroData> data(data_size);

    // Copy data from flash
    memcpy(data.data(), reinterpret_cast<const void *>(FLASH_ADDRESS), data.size() * sizeof(GyroData));

    flash.deinit();

    printf("Data retrieved from flash:\n");
    for (const auto &gyro : data)
    {
        printf("gx: %.5f, gy: %.5f, gz: %.5f\n", gyro[0], gyro[1], gyro[2]);
    }

    return data;
}

InterruptIn button(BUTTON1); // On-board button
DigitalOut led1(LED1);       // On-board LED1
DigitalOut led2(LED2);       // On-board LED2

Timer debounceTimer;
Timer doubleClickTimer;

volatile int clickCount = 0;        // Click counter
volatile bool processClick = false; // Flag to indicate processing of clicks

void on_button_press()
{
    debounceTimer.reset(); // Reset debounce timer
    clickCount++;          // Increment click counter
    doubleClickTimer.reset();
    doubleClickTimer.start(); // Start the timer to detect double clicks
}

void process_clicks()
{
    if (clickCount > 0 && doubleClickTimer.elapsed_time() > 500ms) // Wait 500ms for potential second click
    {
        if (clickCount == 1)
        {
            // Single click detected
            printf("Single click detected\n");
            for (int i = 0; i < 50; i++)
            {
                led1 = !led1;
                ThisThread::sleep_for(100ms);
            }
            led1 = 0; // Turn off LED1
            led2 = 1; // Turn on LED2 for 0.5 seconds
            ThisThread::sleep_for(500ms);
            led2 = 0; // Turn off LED2
        }
        else if (clickCount == 2)
        {
            // Double click detected
            printf("Double click detected\n");

            std::vector<GyroData> gyro_data_buffer;

            // Start recording data for 5 seconds
            Timer recordingTimer;
            recordingTimer.start();

            while (recordingTimer.elapsed_time() < 5s)
            {
                // Collect gyroscope data during 5 seconds
                GyroData gyro = readGyroscopeData(spi, write_buf, read_buf);
                gyro_data_buffer.push_back(gyro);

                // Optional: Blink LED2 while recording
                led2 = !led2;
                ThisThread::sleep_for(100ms);
            }

            led2 = 0; // Turn off LED2

            // Print all collected gyroscope data
            printf("Recorded gyro data:\n");
            for (const auto &gyro : gyro_data_buffer)
            {
                printf("gx: %.5f, gy: %.5f, gz: %.5f\n", gyro[0], gyro[1], gyro[2]);
            }

            // Store data to flash
            printf("Storing gyro data to flash memory...\n");
            bool success = storeToFlash(gyro_data_buffer.data(), gyro_data_buffer.size(), FLASH_ADDRESS, FLASH_SIZE);
            if (success)
            {
                printf("Gyro data stored successfully in flash.\n");
            }
            else
            {
                printf("Failed to store gyro data in flash.\n");
            }

            // Turn on LED1 to indicate completion
            led1 = 1;
            std::vector<GyroData> retrieved_data = readGyroDataFromFlash();
            printf("Retrieved data matches recorded data.\n");
            ThisThread::sleep_for(500ms);
            led1 = 0;
        }
        // Reset state after processing
        clickCount = 0;
        processClick = false;
    }
}

int main()
{
    // Start the timers
    debounceTimer.start();
    doubleClickTimer.start();

    // Attach the button press handler
    button.rise(&on_button_press);

    // SPI Initialization

    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Buffers for SPI data transfer

    // Initialize the gyroscope
    initializeGyroscope(spi, write_buf, read_buf);

    while (true)
    {
        // Process clicks when necessary
        if (clickCount > 0)
        {
            process_clicks();
        }
    }
}