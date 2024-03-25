/***************************************************
 DFRobot Gravity: Analog Spear Tip pH Sensor / Meter Kit (For Soil And Food Applications)
 <https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_Spear_Tip_pH_Sensor_/_Meter_Kit_(For_Soil_And_Food_Applications)_SKU:_SEN0249>

 ***************************************************
 This product is used to measure the pH value of the semisolid,such as meat,fruit,moist soil and so on.
 This sample code reads the pH value.

 Created 2017-9-10
 By Jason <jason.ling@dfrobot.com@dfrobot.com>

 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

 /***********Notice and Trouble shooting***************
 1. This code is tested on Arduino Uno and Leonardo with Arduino IDE 1.0.5 r2 and 1.8.2.
 2. More details, please click this link: <https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_Spear_Tip_pH_Sensor_/_Meter_Kit_(For_Soil_And_Food_Applications)_SKU:_SEN0249>
 ****************************************************/

#define PHSensorPin A2    //dissolved oxygen sensor analog output pin to arduino mainboard
#define VREF 5.0    //for arduino uno, the ADC reference is the AVCC, that is 5.0V(TYP)
#define OFFSET 0.0  //zero drift compensation

#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the analog value in the array, readed from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;

float averageVoltage,phValue;

void setup()
{
    Serial.begin(115200);
    pinMode(PHSensorPin,INPUT);
}

void loop()
{
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 30U)     //every 30 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(PHSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT)
         analogBufferIndex = 0;
   }
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 1000U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      {
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      }
      averageVoltage = (getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / (1024.0 * 40)) -0.05; // read the value more stable by the median filtering algorithm
      phValue = 7 - ((averageVoltage)/59.16) + OFFSET;
      Serial.print("Voltage:");
      Serial.print(averageVoltage,3);
      Serial.print("   pH value:");
      Serial.println(phValue,3);

   }
}

int getMedianNum(int bArray[], int iFilterLen)
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
      bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++)
      {
      for (i = 0; i < iFilterLen - j - 1; i++)
          {
        if (bTab[i] > bTab[i + 1])
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}