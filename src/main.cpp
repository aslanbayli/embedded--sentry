#include <mbed.h>
#include <vector>
#include <array>
#include <limits>
#include <cmath>
#include <math.h>

#include "gyro.h"          
#include "constants.h"

#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/TS_DISCO_F429ZI.h"

// Event flags
#define KEY_FLAG 1
#define UNLOCK_FLAG 2
#define ERASE_FLAG 4
#define DATA_READY_FLAG 8

// LCD font size for text display
#define FONT_SIZE 16

// Threshold for determining successful unlock; decrease if unlock is difficult (must be positive)
#define CORRELATION_THRESHOLD 0.3f

InterruptIn rotIntPin(PA_2, PullDown);
InterruptIn usrBtn(BUTTON1, PullDown);

DigitalOut greenLed(LED1);
DigitalOut redLed(LED2);

LCD_DISCO_F429ZI display;      // LCD control object
TS_DISCO_F429ZI touchScreen;   // Touch screen control object

EventFlags evtFlags; // Event flags to communicate between threads

Timer sysTimer; // General-purpose timer

/*******************************************************************************
 * Function Prototypes for LCD and Touch Screen Operations
 * ****************************************************************************/
void renderButton(int x, int y, int width, int height, const char *label);
bool checkButtonTouch(int touch_x, int touch_y, int button_x, int button_y, int button_width, int button_height);

/*******************************************************************************
 * Function Prototypes for Data Processing
 * ****************************************************************************/
float calcEuclideanDist(const array<float, 3> &a, const array<float, 3> &b);
float calcDTW(const vector<array<float, 3>> &s, const vector<array<float, 3>> &t);
void removeZeroData(vector<array<float, 3>> &data);
float calcCorrelation(const vector<float> &a, const vector<float> &b);
array<float, 3> calcCorrelationVecs(vector<array<float, 3>> &vec1, vector<array<float, 3>> &vec2);

/*******************************************************************************
 * Function Prototypes for Threads
 * ****************************************************************************/
void rotationThread();
void touchThread();

/*******************************************************************************
 * Function Prototypes for Flash Operations
 * ****************************************************************************/
bool flashStoreRotData(vector<array<float, 3>> &gestureKey, uint32_t flash_address);
vector<array<float, 3>> flashReadRotData(uint32_t flash_address, size_t data_size);

/*******************************************************************************
 * Function Prototype for Filters
 * ****************************************************************************/
float movAvgFilter(float input, float dispBuf[], size_t N, size_t &index, float &sum);

/*******************************************************************************
 * ISR Callback Functions
 * ****************************************************************************/
void onButtonPress() // ISR for user button press
{
    evtFlags.set(ERASE_FLAG);
}

// ISR for rotation sensor data-ready interrupt
void onRotDataReady() 
{
    evtFlags.set(DATA_READY_FLAG);
}

/*******************************************************************************
 * Global Variables
 * ****************************************************************************/
vector<array<float, 3>> gestureKey;     // Holds the recorded gesture key
vector<array<float, 3>> unlockRecord;   // Holds the recorded attempt for unlocking

const int btn1X = 60;
const int btn1Y = 80;
const int btn1Width = 120;
const int btn1Height = 50;
const char *btn1Label = "RECORD";

const int btn2X = 60;
const int btn2Y = 180;
const int btn2Width = 120;
const int btn2Height = 50;
const char *btn2Label = "UNLOCK";

const int msgX = 5;
const int msgY = 30;
const char *welcomeMsg = "GESTURE UNLOCKER";

const int txtX = 5;
const int txtY = 270;
const char *txt0 = "NO KEY RECORDED";
const char *txt1 = "LOCKED";

int calcError = 0;

/*******************************************************************************
 * @brief main function
 * ****************************************************************************/
int main()
{
    display.Clear(LCD_COLOR_BLACK);

    // Draw the "RECORD" button
    renderButton(btn1X, btn1Y, btn1Width, btn1Height, btn1Label);

    // Draw the "UNLOCK" button
    renderButton(btn2X, btn2Y, btn2Width, btn2Height, btn2Label);

    // Display initial message
    display.DisplayStringAt(msgX, msgY, (uint8_t *)welcomeMsg, CENTER_MODE);

    // Setup interrupts
    usrBtn.rise(&onButtonPress);
    rotIntPin.rise(&onRotDataReady);

    // Setup initial LED and text state
    if (gestureKey.empty())
    {
        redLed = 0;
        greenLed = 1;
        display.DisplayStringAt(txtX, txtY, (uint8_t *)txt0, CENTER_MODE);
    }
    else
    {
        redLed = 1;
        greenLed = 0;
        display.DisplayStringAt(txtX, txtY, (uint8_t *)txt1, CENTER_MODE);
    }

    // Create thread for rotation sensor operations
    Thread rotationKeyThread;
    rotationKeyThread.start(callback(rotationThread));

    // Create thread for touch screen operations
    Thread tsThread;
    tsThread.start(callback(touchThread));

    // Keep main thread alive
    while (1)
    {
        ThisThread::sleep_for(100ms);
    }
}

/*******************************************************************************
 *
 * @brief Thread handling rotation sensor-based gesture recording/unlocking
 *
 * ****************************************************************************/
void rotationThread()
{
    // Initialize parameters for the rotation sensor
    RotationSensor_Init_Params initParams;
    initParams.sampling_rate_conf = ODR_200HZ_CUTOFF_50HZ;  
    initParams.irq_conf = INT2_DATA_READY;                  
    initParams.scale_conf = FULL_SCALE_500_DPS;             

    // Holds the raw rotation sensor data
    RotationSensor_RawValues rawVals;

    // Buffer used to show status messages on the LCD
    char dispBuf[50];

    // Handle the possible scenario where rotation data-ready line might be high at start-up
    if (!(evtFlags.get() & DATA_READY_FLAG) && (rotIntPin.read() == 1))
    {
        evtFlags.set(DATA_READY_FLAG);
    }

    while (1)
    {
        vector<array<float, 3>> tempKey; // Temporary storage of recorded data

        auto eventReceived = evtFlags.wait_any(KEY_FLAG | UNLOCK_FLAG | ERASE_FLAG);

        if (eventReceived & ERASE_FLAG)
        {
            // Erasing the stored key and any unlock attempt data
            sprintf(dispBuf, "Erasing key...");
            display.SetTextColor(LCD_COLOR_BLACK);
            display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
            display.SetTextColor(LCD_COLOR_BLUE);
            display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
            gestureKey.clear();
            
            sprintf(dispBuf, "Key erased.");
            display.SetTextColor(LCD_COLOR_BLACK);
            display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
            display.SetTextColor(LCD_COLOR_BLUE);
            display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
            unlockRecord.clear();

            // Update LEDs and status
            greenLed = 1;
            redLed = 0;
            sprintf(dispBuf, "All data cleared.");
            display.SetTextColor(LCD_COLOR_BLACK);
            display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
            display.SetTextColor(LCD_COLOR_BLUE);
            display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
        }

        if (eventReceived & (KEY_FLAG | UNLOCK_FLAG))
        {
            // Prompt user to hold on
            sprintf(dispBuf, "Please wait...");
            display.SetTextColor(LCD_COLOR_BLACK);
            display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
            display.SetTextColor(LCD_COLOR_BLUE);
            display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);

            ThisThread::sleep_for(1s);

            // Inform user about calibration
            sprintf(dispBuf, "Calibrating sensor...");
            display.SetTextColor(LCD_COLOR_BLACK);
            display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
            display.SetTextColor(LCD_COLOR_BLUE);
            display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);

            // Initialize sensor for rotation measurement
            InitializeRotationSensor(&initParams, &rawVals);

            // Countdown before recording starts
            sprintf(dispBuf, "Recording in 3...");
            display.SetTextColor(LCD_COLOR_BLACK);
            display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
            display.SetTextColor(LCD_COLOR_BLUE);
            display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
            ThisThread::sleep_for(1s);
            sprintf(dispBuf, "Recording in 2...");
            display.SetTextColor(LCD_COLOR_BLACK);
            display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
            display.SetTextColor(LCD_COLOR_BLUE);
            display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
            ThisThread::sleep_for(1s);
            sprintf(dispBuf, "Recording in 1...");
            display.SetTextColor(LCD_COLOR_BLACK);
            display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
            display.SetTextColor(LCD_COLOR_BLUE);
            display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
            ThisThread::sleep_for(1s);

            sprintf(dispBuf, "Capturing data...");
            display.SetTextColor(LCD_COLOR_BLACK);
            display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
            display.SetTextColor(LCD_COLOR_BLUE);
            display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
            
            // Start collecting rotation data for a fixed duration
            sysTimer.start();
            while (sysTimer.elapsed_time() < 5s)
            {
                // Wait for sensor data-ready signal
                evtFlags.wait_all(DATA_READY_FLAG);

                // Read and convert sensor data
                FetchCalibratedRotationData();

                // Store converted values
                tempKey.push_back({RawToDPS(rawVals.x_axis_value), 
                                   RawToDPS(rawVals.y_axis_value), 
                                   RawToDPS(rawVals.z_axis_value)});
                
                ThisThread::sleep_for(50ms); // about 20Hz sampling
            }
            sysTimer.stop();
            sysTimer.reset();

            // Remove leading and trailing zeros from data
            removeZeroData(tempKey);

            sprintf(dispBuf, "Data capture complete.");
            display.SetTextColor(LCD_COLOR_BLACK);
            display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
            display.SetTextColor(LCD_COLOR_BLUE);
            display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
        }

        // Determine if we were recording a new key or attempting unlock
        if (eventReceived & KEY_FLAG)
        {
            if (gestureKey.empty())
            {
                sprintf(dispBuf, "Saving new key...");
                display.SetTextColor(LCD_COLOR_BLACK);
                display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
                display.SetTextColor(LCD_COLOR_BLUE);
                display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);

                // Save the newly recorded key
                gestureKey = tempKey;
                tempKey.clear();

                // Update LEDs to indicate key presence
                redLed = 1;
                greenLed = 0;

                sprintf(dispBuf, "Key saved successfully.");
                display.SetTextColor(LCD_COLOR_BLACK);
                display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
                display.SetTextColor(LCD_COLOR_BLUE);
                display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
            }
            else
            {
                sprintf(dispBuf, "Replacing old key...");
                display.SetTextColor(LCD_COLOR_BLACK);
                display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
                display.SetTextColor(LCD_COLOR_BLUE);
                display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);

                ThisThread::sleep_for(1s);
                
                // Clear old key and save the new one
                gestureKey.clear();
                gestureKey = tempKey;

                sprintf(dispBuf, "New key saved.");
                display.SetTextColor(LCD_COLOR_BLACK);
                display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
                display.SetTextColor(LCD_COLOR_BLUE);
                display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);

                tempKey.clear();

                // Update LEDs to indicate key presence
                redLed = 1;
                greenLed = 0;
            }
        }
        else if (eventReceived & UNLOCK_FLAG)
        {
            evtFlags.clear(UNLOCK_FLAG);

            sprintf(dispBuf, "Attempting unlock...");
            display.SetTextColor(LCD_COLOR_BLACK);
            display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
            display.SetTextColor(LCD_COLOR_BLUE);
            display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);

            unlockRecord = tempKey; 
            tempKey.clear();

            if (gestureKey.empty())
            {
                sprintf(dispBuf, "No key to match.");
                display.SetTextColor(LCD_COLOR_BLACK);
                display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
                display.SetTextColor(LCD_COLOR_BLUE);
                display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);

                unlockRecord.clear();

                // LEDs indicate locked state since no key is saved
                greenLed = 1;
                redLed = 0;
            }
            else
            {
                int unlockCount = 0; 

                array<float, 3> correlationResult = calcCorrelationVecs(gestureKey, unlockRecord); 
                if (calcError != 0)
                {
                    printf("Error in correlation calculation: vector size mismatch.\n");
                }
                else
                {
                    printf("Correlations: x = %f, y = %f, z = %f\n", correlationResult[0], correlationResult[1], correlationResult[2]);
                    
                    for (size_t i = 0; i < correlationResult.size(); i++)
                    {
                        if (correlationResult[i] > CORRELATION_THRESHOLD)
                        {
                            unlockCount++;
                        }
                    }
                }

                if (unlockCount == 3)
                {
                    sprintf(dispBuf, "UNLOCK SUCCESS");
                    display.SetTextColor(LCD_COLOR_BLACK);
                    display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
                    display.SetTextColor(LCD_COLOR_BLUE);
                    display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
                    
                    greenLed = 1;
                    redLed = 0;

                    unlockRecord.clear();
                    unlockCount = 0;
                }
                else
                {
                    sprintf(dispBuf, "UNLOCK FAILED");
                    display.SetTextColor(LCD_COLOR_BLACK);
                    display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
                    display.SetTextColor(LCD_COLOR_BLUE);
                    display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);

                    greenLed = 0;
                    redLed = 1;

                    unlockRecord.clear();
                    unlockCount = 0;
                }
            }
        }
        ThisThread::sleep_for(100ms);
    }
}

/*******************************************************************************
 *
 * @brief Thread handling touch screen interactions
 *
 * ****************************************************************************/
void touchThread()
{
    TS_StateTypeDef touchState;

    if (touchScreen.Init(display.GetXSize(), display.GetYSize()) != TS_OK)
    {
        printf("Touch screen initialization failed!\r\n");
        return;
    }

    char dispBuf[50];

    while (1)
    {
        touchScreen.GetState(&touchState);
        if (touchState.TouchDetected)
        {
            int touch_x = touchState.X;
            int touch_y = touchState.Y;

            // Check if the touch is within the UNLOCK button area
            if (checkButtonTouch(touch_x, touch_y, btn2X, btn2Y, btn1Width, btn1Height))
            {
                sprintf(dispBuf, "Start recording...");
                display.SetTextColor(LCD_COLOR_BLACK);
                display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
                display.SetTextColor(LCD_COLOR_BLUE);
                display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
                ThisThread::sleep_for(1s);
                evtFlags.set(KEY_FLAG);
            }

            // Check if the touch is within the RECORD button area
            if (checkButtonTouch(touch_x, touch_y, btn1X, btn1Y, btn2Width, btn2Height))
            {
                sprintf(dispBuf, "Start unlock attempt...");
                display.SetTextColor(LCD_COLOR_BLACK);
                display.FillRect(0, txtY, display.GetXSize(), FONT_SIZE);
                display.SetTextColor(LCD_COLOR_BLUE);
                display.DisplayStringAt(txtX, txtY, (uint8_t *)dispBuf, CENTER_MODE);
                ThisThread::sleep_for(1s);
                evtFlags.set(UNLOCK_FLAG);
            }
        }
        ThisThread::sleep_for(10ms);
    }
}

/*******************************************************************************
 *
 * @brief Store rotation-based gesture data to flash memory
 *
 * ****************************************************************************/
bool flashStoreRotData(vector<array<float, 3>> &gestureKey, uint32_t flash_address)
{
    FlashIAP flash;
    flash.init();

    uint32_t data_size = gestureKey.size() * sizeof(array<float, 3>);

    flash.erase(flash_address, data_size);

    int write_result = flash.program(gestureKey.data(), flash_address, data_size);

    flash.deinit();

    return (write_result == 0);
}

/*******************************************************************************
 *
 * @brief Read stored rotation-based gesture data from flash
 *
 * ****************************************************************************/
vector<array<float, 3>> flashReadRotData(uint32_t flash_address, size_t data_size)
{
    vector<array<float, 3>> gestureKey(data_size);

    FlashIAP flash;
    flash.init();

    flash.read(gestureKey.data(), flash_address, data_size * sizeof(array<float, 3>));

    flash.deinit();

    return gestureKey;
}

/*******************************************************************************
 *
 * @brief Draw a rectangular button on the display
 *
 * ****************************************************************************/
void renderButton(int x, int y, int width, int height, const char *label)
{
    display.SetTextColor(LCD_COLOR_BLUE);
    display.FillRect(x, y, width, height);
    display.DisplayStringAt(x + width / 2 - strlen(label) * 19, y + height / 2 - 8, (uint8_t *)label, CENTER_MODE);
}

/*******************************************************************************
 *
 * @brief Verify if a touch point is inside a given button
 *
 * ****************************************************************************/
bool checkButtonTouch(int touch_x, int touch_y, int button_x, int button_y, int button_width, int button_height)
{
    return (touch_x >= button_x && touch_x <= button_x + button_width &&
            touch_y >= button_y && touch_y <= button_y + button_height);
}

/*******************************************************************************
 *
 * @brief Compute the Euclidean distance between two 3D points
 *
 * ****************************************************************************/
float calcEuclideanDist(const array<float, 3> &a, const array<float, 3> &b)
{
    float sum = 0;
    for (size_t i = 0; i < 3; ++i)
    {
        sum += (a[i] - b[i]) * (a[i] - b[i]);
    }
    return sqrt(sum);
}

/*******************************************************************************
 *
 * @brief Compute the DTW (Dynamic Time Warping) distance between two sequences
 *
 * ****************************************************************************/
float calcDTW(const vector<array<float, 3>> &s, const vector<array<float, 3>> &t)
{
    vector<vector<float>> dtw_matrix(s.size() + 1, vector<float>(t.size() + 1, numeric_limits<float>::infinity()));

    dtw_matrix[0][0] = 0;

    for (size_t i = 1; i <= s.size(); ++i)
    {
        for (size_t j = 1; j <= t.size(); ++j)
        {
            float cost = calcEuclideanDist(s[i - 1], t[j - 1]);
            dtw_matrix[i][j] = cost + min({dtw_matrix[i - 1][j], dtw_matrix[i][j - 1], dtw_matrix[i - 1][j - 1]});
        }
    }

    return dtw_matrix[s.size()][t.size()];
}

/*******************************************************************************
 *
 * @brief Remove leading/trailing segments of negligible rotation data
 *
 * ****************************************************************************/
void removeZeroData(vector<array<float, 3>> &data)
{
    float threshold = 0.00001;
    auto startPtr = data.begin();
    while (startPtr != data.end() && fabs((*startPtr)[0]) <= threshold && fabs((*startPtr)[1]) <= threshold && fabs((*startPtr)[2]) <= threshold)
    {
        startPtr++;
    }
    if (startPtr == data.end())
        return;      
    auto leftPtr = startPtr; 
    startPtr = data.end() - 1;
    while (startPtr != data.begin() && fabs((*startPtr)[0]) <= threshold && fabs((*startPtr)[1]) <= threshold && fabs((*startPtr)[2]) <= threshold)
    {
        startPtr--;
    }
    auto rightPtr = startPtr; 

    auto replacePtr = data.begin();
    for (; replacePtr != leftPtr && leftPtr <= rightPtr; replacePtr++, leftPtr++)
    {
        *replacePtr = *leftPtr;
    }
    if (leftPtr > rightPtr)
    {
        data.erase(replacePtr, data.end());
    }
    else
    {
        data.erase(rightPtr + 1, data.end());
    }
}

/*******************************************************************************
 *
 * @brief Compute correlation between two equal-length numerical sequences
 *
 * ****************************************************************************/
float calcCorrelation(const vector<float> &a, const vector<float> &b)
{
    if (a.size() != b.size())
    {
        calcError = -1;
        return 0.0f;
    }

    float sum_a = 0, sum_b = 0, sum_ab = 0, sq_sum_a = 0, sq_sum_b = 0;

    for (size_t i = 0; i < a.size(); ++i)
    {
        sum_a += a[i];
        sum_b += b[i];
        sum_ab += a[i] * b[i];
        sq_sum_a += a[i] * a[i];
        sq_sum_b += b[i] * b[i];
    }

    size_t n = a.size();

    float numerator = n * sum_ab - sum_a * sum_b; 
    float denominator = sqrt((n * sq_sum_a - sum_a * sum_a) * (n * sq_sum_b - sum_b * sum_b));

    return numerator / denominator;
}

/*******************************************************************************
 *
 * @brief Calculate correlation values for x, y, z dimensions of two datasets
 *
 * ****************************************************************************/
array<float, 3> calcCorrelationVecs(vector<array<float, 3>> &vec1, vector<array<float, 3>> &vec2)
{
    array<float, 3> result;

    for (int i = 0; i < 3; i++)
    {
        vector<float> a;
        vector<float> b;

        for (const auto &arr : vec1)
        {
            a.push_back(arr[i]);
        }
        for (const auto &arr : vec2)
        {
            b.push_back(arr[i]);
        }

        if (a.size() > b.size())
        {
            a.resize(b.size(), 0);
        }
        else if (b.size() > a.size())
        {
            b.resize(a.size(), 0);
        }

        result[i] = calcCorrelation(a, b);
    }

    return result;
}
