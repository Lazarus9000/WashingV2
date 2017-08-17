#include "ArduinoJson.h"
#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A0
const uint16_t samples = 32;//64; //This value MUST ALWAYS be a power of 2
double samplingFrequency = 38500;

unsigned int delayTime = 0;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

int incomingAudio;//storage for A0 data

//https://hekilledmywire.wordpress.com/2011/03/16/using-the-adc-tutorial-part-5/
//Consider implementing adc_init and read_adc from this source, to sample fast from several sources

void adc_init(void){
 
 //ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));    //16Mhz/128 = 125Khz the ADC reference clock
 //ADMUX |= (1<<REFS0);                //Voltage reference from Avcc (5v)
 //ADCSRA |= (1<<ADEN);                //Turn on ADC
 //ADCSRA |= (1<<ADSC);                //Do an initial conversion because this one is the slowest and to ensure that everything is up and running
}
 
void read_adc(uint8_t channel){
 ADMUX &= 0xF0;                    //Clear the older channel that was read
 ADMUX |= channel;                //Defines the new ADC channel to be read
 ADCSRA |= (1<<ADSC);                //Starts a new conversion

 for(uint16_t i =0;i<samples;i++)
  {
    vReal[i] = ADCH;//double(analogRead(CHANNEL));
    //if(samplingFrequency<=1000)
    //  delay(delayTime);
    //else
    //Consider replacing delay with a check on how much time has progressed 
    //make a new sample if millis() - timeSinceLastSample > sampleFrequency
      delayMicroseconds(delayTime);
  }
  
  //while(ADCSRA & (1<<ADSC));            //Wait until the conversion is done
  //return ADCW;                    //Returns the ADC value of the chosen channel
}

void setup()
{
  //Faster sampling of ADC from https://www.instructables.com/id/Arduino-Audio-Input/
  //set up continuous sampling of analog pin 0 (you don't need to understand this part, just know how to use it in the loop())
  
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
  
  if(samplingFrequency<=1000)
    delayTime = 1000/samplingFrequency;
  else
    delayTime = 1000000/samplingFrequency;
  Serial.begin(115200);
  //Serial.println("Ready");

  //Set up json object (from JsonGeneratorExample)
  // Memory pool for JSON object tree.
  //
  // Inside the brackets, 200 is the size of the pool in bytes.
  // If the JSON object is more complex, you need to increase that value.
  // See https://bblanchon.github.io/ArduinoJson/assistant/
  

  // StaticJsonBuffer allocates memory on the stack, it can be
  // replaced by DynamicJsonBuffer which allocates in the heap.
  //
  // DynamicJsonBuffer  jsonBuffer(200);

  // Create the root of the object tree.
  //
  // It's a reference to the JsonObject, the actual bytes are inside the
  // JsonBuffer with all the other nodes of the object tree.
  // Memory is freed when jsonBuffer goes out of scope.
  
  // It's also possible to create the array separately and add it to the
  // JsonObject but it's less efficient.
  
}

void sample(int channel) {
  memset(vReal,0,sizeof(vReal));
  memset(vImag,0,sizeof(vImag));
  
  read_adc(channel);

  //Serial.print("#S|LOGTEST|[");
  /* Print the results of the sampling according to time */
  //Serial.println("Data:");
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  //Serial.println("Weighed data:");
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  //Serial.println("Computed Real values:");
  //PrintVector(vReal, samples, SCL_INDEX);
  //Serial.println("Computed Imaginary values:");
  //PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  //Serial.println("Computed magnitudes:");
  //PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  StaticJsonBuffer<400> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonArray& data1 = root.createNestedArray("data1");
  PrintVector(vReal, samples, SCL_FREQUENCY, data1);
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);

  root["peakFreq"] = x;
  //Serial.print(x, 2);
  //Serial.println("]#");
  if(channel == 1) {
    Serial.print("{\"FFT1\":");
    data1.printTo(Serial);
    //Serial.print(",\"peakFreq\":");
    //root["peakFreq"].printTo(Serial);
    Serial.print("};{\"FFT2\":''}\n");
  }

  if(channel == 2) {
    Serial.print("{\"FFT1\":''};{\"FFT2\":");
    data1.printTo(Serial);
    //Serial.print(",\"peakFreq\":");
    //root["peakFreq"].printTo(Serial);
    Serial.print("}\n");
  }

  if(channel > 2) {
    Serial.print("Invalid channel\n");
  }
}

void loop()
{

  sample(1);

  //switch analog input to other mic
  
  delay(2000);

  sample(2);

}

void PrintVector(double *vData, uint8_t bufferSize, uint8_t scaleType, JsonArray& out)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    //Serial.print(abscissa, 6);
    //Serial.print(",");
    out.add(vData[i]);
    //Serial.print(vData[i], 2);
    //Serial.print(",");
    //Serial.println();
  }
  //Serial.println();
}

