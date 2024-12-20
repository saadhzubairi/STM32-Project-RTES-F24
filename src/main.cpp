#include "mbed.h"
#include "FlashIAP.h"
#include <array>
#include <vector>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <numeric>
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/TS_DISCO_F429ZI.h"
#include "display.h"

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
LCD_DISCO_F429ZI lcd; // LCD object
TS_DISCO_F429ZI ts;   // Touch screen object

EventFlags flags;

using GyroData = std::array<float, 3>;

SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
uint8_t write_buf[32], read_buf[32];

std::vector<GyroData> gyroTemp;

/*
to purge this use:
memset(&gyro, 0, sizeof(gyro));
 */

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
        printf("[-] Error: Data size exceeds allocated flash space.\n");
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
        printf("[-] Error: Failed to initialize flash.\n");
        return false;
    }

    // Erase the target flash region
    result = flash.erase(flash_address, flash_size);
    if (result != 0)
    {
        printf("[-] Error: Failed to erase flash sector.\n");
        flash.deinit();
        return false;
    }

    // Write data to flash
    result = flash.program(data, flash_address, data_size);
    if (result != 0)
    {
        printf("[-] Error: Failed to write to flash memory.\n");
        flash.deinit();
        return false;
    }

    // Deinitialize flash
    flash.deinit();

    printf("[+] Data successfully stored to flash.\n");
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
/* void storeGyroDataToFlash(const std::vector<GyroData> &data)
{
    size_t data_size = data.size() * sizeof(GyroData);



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
} */

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

    /* printf("Data retrieved from flash:\n");
    for (const auto &gyro : data)
    {
        printf("gx: %.5f, gy: %.5f, gz: %.5f\n", gyro[0], gyro[1], gyro[2]);
    } */

    return data;
}

InterruptIn button(BUTTON1); // On-board button
DigitalOut led1(LED1);       // On-board LED1
DigitalOut led2(LED2);       // On-board LED2

Timer debounceTimer;
Timer doubleClickTimer;

volatile int clickCount = 0;        // Click counter
volatile bool processClick = false; // Flag to indicate processing of clicks
bool changeScreenColor = false;

void on_button_press()
{
    debounceTimer.reset(); // Reset debounce timer
    clickCount++;          // Increment click counter
    doubleClickTimer.reset();
    doubleClickTimer.start(); // Start the timer to detect double clicks

    // change the lcd color if button is pressed
    changeScreenColor = true;
}

float dynamicTimeWarping(const std::vector<float> &seq1, const std::vector<float> &seq2)
{
    size_t n = seq1.size();
    size_t m = seq2.size();

    if (n > gyroTemp.size() || m > gyroTemp.size())
    {
        printf("[-] Error: Sequence size exceeds maximum DTW buffer size.\n");
        return FLT_MAX; // Return a large value to indicate failure
    }

    std::vector<float> prev(m + 1, FLT_MAX);
    std::vector<float> curr(m + 1, FLT_MAX);
    prev[0] = 0;

    for (size_t i = 1; i <= n; ++i)
    {
        curr[0] = FLT_MAX;
        for (size_t j = 1; j <= m; ++j)
        {
            float dist = fabs(seq1[i - 1] - seq2[j - 1]);
            curr[j] = dist + std::min({prev[j], curr[j - 1], prev[j - 1]});
        }
        std::swap(prev, curr);
    }

    return prev[m] / std::max(n, m);
}

bool compareGyroDataUsingDTW(const std::vector<GyroData> &gyroTemp, const std::vector<GyroData> &flashData, float dtw_threshold = 1.0f)
{
    // Ensure the flash data has at least as many elements as gyroTemp
    if (flashData.size() < gyroTemp.size())
    {
        printf("[-] Error: Flash data has fewer elements than the gyroscope data.\n");
        return false;
    }
    else
    {
        printf("[.] Comparing using DTW for gyroTemp size %d\n", gyroTemp.size());
    }

    // Extract only the first gyroTemp.size() elements from flashData for comparison
    std::vector<GyroData> flashSubset(flashData.begin(), flashData.begin() + gyroTemp.size());

    /* printf("Extracting data\n"); */

    // Extract axis-specific data
    std::vector<float> tempX, tempY, tempZ;
    std::vector<float> flashX, flashY, flashZ;

    
    printf("[.] Processing...\n");

    for (size_t i = 0; i < gyroTemp.size() - 1; ++i)
    {
        tempX.push_back(gyroTemp[i][0]);
        tempY.push_back(gyroTemp[i][1]);
        tempZ.push_back(gyroTemp[i][2]);

        flashX.push_back(flashSubset[i][0]);
        flashY.push_back(flashSubset[i][1]);
        flashZ.push_back(flashSubset[i][2]);
    }

    // Compute DTW distance for each axis
    float dtwX = dynamicTimeWarping(tempX, flashX);
    float dtwY = dynamicTimeWarping(tempY, flashY);
    float dtwZ = dynamicTimeWarping(tempZ, flashZ);


    printf("[.] DTW distances -> X: %.5f, Y: %.5f, Z: %.5f\n", dtwX, dtwY, dtwZ);

    // Combine distances and check threshold
    float combined_dtw = (dtwX + dtwY + dtwZ) / 3.0f;
    if (combined_dtw <= dtw_threshold)
    {
        printf("[+] Gyro data matches flash data based on DTW. Average DTW distance = %.5f is within threshold = %.5f\n", combined_dtw, dtw_threshold);
        return true;
    }
    else
    {
        printf("[-] Mismatch: Average DTW distance = %.5f exceeds threshold = %.5f\n", combined_dtw, dtw_threshold);
        return false;
    }
}

void processAndValidateInput()
{
    printf("[.] Validating...\n");
    UpdateInfo(lcd, "matching");
    led1 = 1;
    led2 = 1;
    // Retrieve data from flash memory
    std::vector<GyroData> flashData = readGyroDataFromFlash();
    // Compare gyroTemp with the first gyroTemp.size() elements of flashData using DTW
    bool valid = compareGyroDataUsingDTW(gyroTemp, flashData);
    if (valid)
    {
        led2 = 0; // Turn off LED2
        led1 = 1; // Turn on LED2 for 0.5 seconds
        printf("[o] correct!");
        UpdateInfo(lcd, "correct, device unlocked!");
    }
    else
    {
        led1 = 0; // Turn off LED2
        led2 = 1; // Turn on LED2 for 0.5 seconds
        printf("[x] incorrect!");
        UpdateInfo(lcd, "incorrect, device unlocked failed! Press button twice to record gesture. Or press once to match");
    }
    gyroTemp.clear();
    ThisThread::sleep_for(1000ms);
}

void process_clicks()
{
    led1 = 0;
    led2 = 0;
    if (clickCount > 0 && doubleClickTimer.elapsed_time() > 500ms) // Wait 500ms for potential second click
    {
        if (clickCount == 1)
        {
            // Single click detected
            printf("[.] Single click detected\n");
            UpdateInfo(lcd, "recording");

            Timer recordingTimer;
            recordingTimer.start();

            std::vector<GyroData> gyro_data_buffer;

            while (recordingTimer.elapsed_time() < 5s)
            {

                GyroData gyro = readGyroscopeData(spi, write_buf, read_buf);
                gyroTemp.push_back(gyro);

                led1 = !led1;
                ThisThread::sleep_for(25ms);
            }

            processAndValidateInput();
            recordingTimer.stop();
        }
        else if (clickCount == 2) // Record and store data to flash
        {
            // Double click detected
            printf("[.] Double click detected\n");
            UpdateInfo(lcd, "recording");

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
                ThisThread::sleep_for(25ms);
            }

            led2 = 0; // Turn off LED2
            UpdateInfo(lcd, "recorded, storing");
            // Print all collected gyroscope data
            printf("[.] Recorded gyro data:\n");
            /* for (const auto &gyro : gyro_data_buffer)
            {
                printf("gx: %.5f, gy: %.5f, gz: %.5f\n", gyro[0], gyro[1], gyro[2]);
            } */

            // Store data to flash
            printf("[.] Storing gyro data to flash memory...\n");
            bool success = storeToFlash(gyro_data_buffer.data(), gyro_data_buffer.size(), FLASH_ADDRESS, FLASH_SIZE);
            if (success)
            {
                printf("[+] Gyro data stored successfully in flash.\n");
                UpdateInfo(lcd, "Stored successfully, press button once to match");
            }
            else
            {
                printf("[-] Failed to store gyro data in flash.\n");
            }
            // Turn on LED1 to indicate completion
            led1 = 1;
            /* std::vector<GyroData> retrieved_data = readGyroDataFromFlash();
            printf("Retrieved data matches recorded data.\n"); */
            ThisThread::sleep_for(500ms);
            led1 = 0;
            recordingTimer.stop();
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

    UpdateInfo(lcd, "press the button twice to record");
    while (true)
    {

        // Process clicks when necessary
        if (clickCount > 0)
        {
            process_clicks();
            if (debounceTimer.elapsed_time() < 100ms)
            {
                changeScreenColor = true;
            }
            else
            {
                changeScreenColor = false;
            }
        }

        if (changeScreenColor)
        {
            ClearScreen(lcd, LCD_COLOR_BLUE);
        }
        else
        {
            ClearScreen(lcd, LCD_COLOR_WHITE);
        }
    }
}
