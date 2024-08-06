#include <Arduino.h>
#include <math.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* Thermistor Definitions */
#define THERMISTOR  34
#define BIAS        54.0

/* Cooler PWM Definitions */
#define COOLER      25
#define CHANNEL     0
#define FREQUENCY   5000
#define RESOLUTION  8

/* OLED Display Definitions */
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* I2C Definitions */
#define I2C_SDA 21
#define I2C_SCL 22

/* Tasks Definitions */
#define STACKSIZE              10000
#define PRIORITY_TEMP_TASK     1
#define PRIORITY_CONTROL_TASK  1
#define PRIORITY_DISPLAY_TASK  0
#define COLLECTION_TIME        1 // in seconds

/* PID Configuration */
double Setpoint, Input, Output;
double Kp = 50.0, Ki = 15.0, Kd = 0.0; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

/* Global Variables */
double last_temp = 0.0;
double current_temp = 0.0;
double gradient = 0.0;

/* Mutex for protection */
SemaphoreHandle_t xMutex;

/* To Read Thermistor Function */
double getTemperature(int pin) 
{
  int RawADC = analogRead(pin);

  // Check analog reading
  if (RawADC == 0) {
    Serial.println("Error: RawADC is 0, cannot calculate resistance.");
    return NAN;
  }

  long Resistance;
  double Temp;

  // Calculate thermistor resistance
  Resistance = ((40960000 / RawADC) - 10000);

  // Check calculated resistance
  if (Resistance <= 0) {
    Serial.println("Error: Calculated resistance is non-positive.");
    return NAN;
  }

  // Use the Steinhart-Hart equation to calculate temperature
  Temp = log(Resistance);
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius

  return (Temp + BIAS);  // Return temperature in Celsius
}

/* Temperature Task */
void temperatureTask(void *pvParameters) 
{
  while (1) 
  {
    // Read the new temperature from the thermistor
    double new_temp = getTemperature(THERMISTOR);
    if (!isnan(new_temp)) 
    {
      // Update the shared variables with the new temperature and gradient
      xSemaphoreTake(xMutex, portMAX_DELAY);
      last_temp = current_temp;
      current_temp = new_temp;
      gradient = (current_temp - last_temp) / COLLECTION_TIME;
      xSemaphoreGive(xMutex);

      // Print the current temperature and gradient
      Serial.print("Temperature: ");
      Serial.print(current_temp);
      Serial.println(" *C");

      Serial.print("Temperature Gradient: ");
      Serial.print(gradient);
      Serial.println(" *C/s");
    } 
    else 
    {
      Serial.println("Error: Failed to read temperature.");
    }

    // Delay for the specified collection time
    vTaskDelay(pdMS_TO_TICKS(COLLECTION_TIME*1000));
  }
}

/* Control Task */
void controlTask(void *pvParameters) 
{
  // Set up the cooler PWM
  ledcSetup(CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(COOLER, CHANNEL);

  // Set the desired temperature gradient setpoint
  Setpoint = 0.3;

  // Configure the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(70, 255);

  while (1) 
  {
    // Acquire the mutex to access the shared gradient variable
    xSemaphoreTake(xMutex, portMAX_DELAY);
    Input = gradient;
    xSemaphoreGive(xMutex);

    // Compute the PID output
    myPID.Compute();

    // Set the PWM value based on the PID output
    ledcWrite(CHANNEL, Output);

    // Print the PWM value
    Serial.print("PWM Value: ");
    Serial.println(Output);  
    
    // Delay for the specified collection time
    vTaskDelay(pdMS_TO_TICKS(COLLECTION_TIME*1000));
  }
}

/* Display Task */
void displayTask(void *pvParameters) 
{
  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.display();

  // Graph settings
  const int graphWidth = SCREEN_WIDTH - 10;
  const int graphHeight = 20;
  const int graphYPos = SCREEN_HEIGHT - graphHeight - 10;

  // Array to store last temperature values for the graph
  float lastTemperatureValues[graphWidth];
  memset(lastTemperatureValues, 0, sizeof(lastTemperatureValues));
  int index = 0;

  // Y-axis range for the graph
  const float Y_MAX = 40.0;
  const float Y_MIN = 0.0;
  const float Y_RANGE = Y_MAX - Y_MIN;

  while (1)
  {
    display.clearDisplay();

    // Protect the shared variable with the mutex
    xSemaphoreTake(xMutex, portMAX_DELAY);
    float temperature = current_temp;
    float grad = gradient;
    xSemaphoreGive(xMutex);

    // Display the temperature gradient
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Temp Gradient: ");
    display.println(grad);

    // Add the current temperature to the temperature array
    lastTemperatureValues[index] = temperature;
    index = (index + 1) % graphWidth;

    // Draw the temperature trend graph
    int lastY = graphYPos + graphHeight;
    for (int i = 0; i < graphWidth; i++) 
    {
      int x = (index + i) % graphWidth;
      int y = graphYPos + graphHeight - (int)((lastTemperatureValues[x] - Y_MIN) / Y_RANGE * graphHeight);
      display.drawPixel(i + 5, y, SSD1306_WHITE);
      if (i > 0) {
        display.drawLine(i + 4, lastY, i + 5, y, SSD1306_WHITE);
      }
      lastY = y;
    }

    // Update the display
    display.display();

    vTaskDelay(pdMS_TO_TICKS(COLLECTION_TIME*1000));
  }
}

void setup() 
{
  Serial.begin(115200);

  // Create a mutex to protect shared resources
  xMutex = xSemaphoreCreateMutex();

  // creating tasks
  xTaskCreatePinnedToCore(temperatureTask, "Temperature Task", STACKSIZE, NULL, PRIORITY_TEMP_TASK, NULL, 1);
  xTaskCreatePinnedToCore(controlTask, "Control Task", STACKSIZE, NULL, PRIORITY_CONTROL_TASK, NULL, 0);
  xTaskCreatePinnedToCore(displayTask, "DisplayTask", STACKSIZE, NULL, PRIORITY_DISPLAY_TASK, NULL, 0);
}

void loop() 
{
  // free
}
