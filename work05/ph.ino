#define SensorPin A2
#define Offset 0.66
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth 40

int pHArray[ArrayLenth];
int pHArrayIndex = 0;

void setup(void)
{
  Serial.begin(9600);
  Serial.println("PH-4502C 传感器已启动");
}

void loop(void)
{
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue, voltage;

  if (millis() - samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++] = analogRead(SensorPin);
    if (pHArrayIndex == ArrayLenth) pHArrayIndex = 0;

    voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
    pHValue = -5.7 * voltage + 21.7 + Offset;
    samplingTime = millis();
  }

  if (millis() - printTime > printInterval)
  {
    Serial.print("电压：");
    Serial.print(voltage, 2);
    Serial.print(" V  |  PH值：");
    Serial.println(pHValue, 2);

    printTime = millis();
  }
}

double avergearray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;

  if (number <= 0) return 0;
  if (number < 5) {
    for (i = 0; i < number; i++) amount += arr[i];
    avg = amount / number;
    return avg;
  }

  min = arr[0] < arr[1] ? arr[0] : arr[1];
  max = arr[0] > arr[1] ? arr[0] : arr[1];

  for (i = 2; i < number; i++) {
    if (arr[i] < min) {
      amount += min;
      min = arr[i];
    } else if (arr[i] > max) {
      amount += max;
      max = arr[i];
    } else {
      amount += arr[i];
    }
  }

  avg = (double)amount / (number - 2);
  return avg;
}