#include <FastLED.h>
#include <Adafruit_NeoPixel.h>

/** BASIC CONFIGURATION  **/
#define BRIGHTNESS          160
#define FRAMES_PER_SECOND  120
 
//The pin that controls the LEDs
#define LED_PIN 2
//The pin that we read sensor values form
#define MIC_PIN A0
#define switchPin 5

uint8_t led_volts = 5;
uint32_t max_milliamps = 2400;

int newborn = 0; //indicates if in newborn mode, so color palette can be black and white

////////////////////////////////////////////////////////////////////////////////
////////////////MATRIX PATTERN SETUP

const uint8_t kMatrixWidth  = 16;
const uint8_t kMatrixHeight = 16;
const bool    kMatrixSerpentineLayout = true;

//The amount of LEDs in the setup
#define NUM_LEDS (kMatrixWidth * kMatrixHeight)
#define NUM_LEDS_HALF (NUM_LEDS / 2)
#define NUM_LEDS_QUARTER (NUM_LEDS / 4)
#define MAX_DIMENSION ((kMatrixWidth>kMatrixHeight) ? kMatrixWidth : kMatrixHeight)


// The 16 bit version of our coordinates
static uint16_t x;
static uint16_t y;
static uint16_t z;

// We're using the x/y dimensions to map to the x/y pixels on the matrix.  We'll
// use the z-axis for "time".  speed determines how fast time moves forward.  Try
// 1 for a very slow moving effect, or 60 for something that ends up looking like
// water.
uint16_t speed = 6; // speed is set dynamically once we've started up

// Scale determines how far apart the pixels in our noise matrix are.  Try
// changing these values around to see how it affects the motion of the display.  The
// higher the value of scale, the more "zoomed out" the noise iwll be.  A value
// of 1 will be so zoomed in, you'll mostly see solid colors.
uint16_t scale = 30; // scale is set dynamically once we've started up

// This is the array that we keep our computed noise values in
uint8_t noise[MAX_DIMENSION][MAX_DIMENSION];


CRGBPalette16 currentPalette( PartyColors_p );
uint8_t       colorLoop = 1;

// x,y, & time values
uint32_t v_time,hue_time,hxy;

// how many octaves to use for the brightness and hue functions
uint8_t octaves=1;
uint8_t hue_octaves=3;

// the 'distance' between x/y points for the hue noise
int hue_scale=1;

// how fast we move through time & hue noise
int time_speed=100;
int hue_speed=10;

// adjust these values to move along the x or y axis between frames
int x_speed=331;
int y_speed=1111;

// the 'distance' between points on the x and y axis
int xscale=57771;
int yscale=57771;


////////////////////////////////////////////////////////////////////////////////
////////////////VISUALIZER SETUP

//Confirmed microphone low value, and max value
#define MIC_LOW 380.0
#define MIC_HIGH 460.0
/** Other macros */
//How many previous sensor values effects the operating average?
#define AVGLEN 5
//How many previous sensor values decides if we are on a peak/HIGH (e.g. in a song)
#define LONG_SECTOR 20

//Mneumonics
#define HIGH 3
#define NORMAL 2

//How long do we keep the "current average" sound, before restarting the measuring
#define MSECS 30 * 1000
#define CYCLES MSECS / DELAY

/*Sometimes readings are wrong or strange. How much is a reading allowed
to deviate from the average to not be discarded? **/
#define DEV_THRESH 0.8

//Arduino loop delay
#define DELAY 1

float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve);
void insert(int val, int *avgs, int len);
int compute_average(int *avgs, int len);
void visualize_music();
void led_mode2();
void led_mode3();
void led_mode4();
void led_mode5();
void led_off();
int oldMode = 0;  // assume switch closed because of pull-down resistor
const unsigned long debounceTime = 1000;  // milliseconds

//How many LEDs to we display
int height = NUM_LEDS;

/*Not really used yet. Thought to be able to switch between sound reactive
mode, and general gradient pulsing/static color*/
int mode = 0;

//Showing different colors based on the mode.
int songmode = NORMAL;

//Average sound measurement the last CYCLES
unsigned long song_avg;

//The amount of iterations since the song_avg was reset
int iter = 0;

//The speed the LEDs fade to black if not relit
float fade_scale = 1.2;

//Led array
CRGB leds[NUM_LEDS];

/*Short sound avg used to "normalize" the input values.
We use the short average instead of using the sensor input directly */
int avgs[AVGLEN] = {-1};

//Longer sound avg
int long_avg[LONG_SECTOR] = {-1};

//Keeping track how often, and how long times we hit a certain mode
struct time_keeping {
  unsigned long times_start;
  short times;
};

//How much to increment or decrement each color every cycle
struct color {
  int r;
  int g;
  int b;
};

struct time_keeping high;
struct color Color; 

//heigh_and_condition variables;
  int sensor_value, mapped, avg, longavg;

//peak dot parameters
byte
  peak      = 0,      // Used for falling dot
  dotCount  = 0;      // Frame counter for delaying dot-falling speed
#define PEAK_FALL 1  // Rate of peak falling dot
#define PEAK_FALL_split 0  // Rate of peak falling dot

//Vu7 Parameters
#define NSAMPLES 64
unsigned int samplearray[NSAMPLES];
unsigned long samplesum = 0;
unsigned int sampleavg = 0;
int samplecount = 0;
unsigned int sample = 0;
unsigned long oldtime = 0;
unsigned long newtime = 0;

//vu ripple
uint8_t colour; 
uint8_t myfade = 255;                                         // Starting brightness.
#define maxsteps 24                                           // Case statement wouldn't allow a variable.
int peakspersec = 0;
int peakcount = 0;
uint8_t bgcol = 0;   
int thisdelay = 20; 
int color;
int center = 0;
int center_2 = 0;
int step = -1;
float fadeRate = 0.80;
int diff;
int width=45;

//center burst FHT parameters
CRGBPalette16 targetPalette;
TBlendType    currentBlending;                                // NOBLEND or LINEARBLEND
uint8_t maxChanges = 48;



// FOR SYLON ETC
uint8_t thisbeat =  23;
uint8_t thatbeat =  28;
uint8_t thisfade =   2;                                     // How quickly does it fade? Lower = slower fade rate.
uint8_t thissat = 255;                                     // The saturation, where 255 = brilliant colours.
uint8_t thisbri = 255; 
int    myhue =   0;

//FOR JUGGLE
uint8_t numdots = 4;                                          // Number of dots in use.
uint8_t faderate = 2;                                         // How long should the trails be. Very low value = longer trails.
uint8_t hueinc = 16;                                          // Incremental change in hue between each dot.
uint8_t thishue = 0;                                          // Starting hue.
uint8_t curhue = 0; 
uint8_t thisbright = 255;                                     // How bright should the LED/display be.
uint8_t basebeat = 5; 
uint8_t max_bright = 255;

// Twinkle
float redStates[NUM_LEDS];
float blueStates[NUM_LEDS];
float greenStates[NUM_LEDS];
float Fade = 0.96;
//unsigned int sample;

////For Fire
//#define SPARKING 50
//#define COOLING  55
//bool gReverseDirection = false;
//#define FRAMES_PER_SECOND 60

//For Fireplace
CRGBPalette16 gPal;
#define FPS 48
#define FPS_DELAY 1000/FPS
#define COOLING 20  
#define HOT 100
#define MAXHOT HOT*kMatrixHeight


uint8_t gHue = 0; // rotating "base color" used by many of the patterns



///Cine-lights fireplace / torch setup
///not working now -- go back and fix later

//uint16_t cycle_wait = 1; // 0..255
//
//byte flame_min = 100; // 0..255
//byte flame_max = 220; // 0..255
//
//byte random_spark_probability = 2; // 0..100
//byte spark_min = 200; // 0..255
//byte spark_max = 255; // 0..255
//
//byte spark_tfr = 40; // 0..256 how much energy is transferred up for a spark per cycle
//uint16_t spark_cap = 200; // 0..255: spark cells: how much energy is retained from previous cycle
//
//uint16_t up_rad = 40; // up radiation
//uint16_t side_rad = 35; // sidewards radiation
//uint16_t heat_cap = 0; // 0..255: passive cells: how much energy is retained from previous cycle
//
//byte red_bg = 0;
//byte green_bg = 0;
//byte blue_bg = 0;
//byte red_bias = 10;
//byte green_bias = 0;
//byte blue_bias = 0;
//int red_energy = 180;
//int green_energy = 20; // 145;
//int blue_energy = 0;
//byte upside_down = 0; // if set, flame (or rather: drop) animation is upside down. Text remains as-is
//
//#define ledsPerLevel kMatrixWidth
//#define levels kMatrixHeight
//
//byte currentEnergy[NUM_LEDS]; // current energy level
//byte nextEnergy[NUM_LEDS]; // next energy level
//byte energyMode[NUM_LEDS]; // mode how energy is calculated for this point
//
//enum {
//  torch_passive = 0, // just environment, glow from nearby radiation
//  torch_nop = 1, // no processing
//  torch_spark = 2, // slowly looses energy, moves up
//  torch_spark_temp = 3, // a spark still getting energy from the level below
//};


//soundmems setup
#define qsuba(x, b)  ((x>b)?x-b:0)                            // Analog Unsigned subtraction macro. if result <0, then => 0
int wavebright = 20;


///////Setup Function run when turned on

void setup() {
    analogReference(EXTERNAL);                                  // Connect 3.3V to AREF pin for any microphones using 3.3V

  Serial.begin(9600);
  pinMode (switchPin, INPUT);
  //Set all lights to make sure all are working as expected
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  set_max_power_in_volts_and_milliamps (led_volts, max_milliamps);
  
  for (int i = 0; i < NUM_LEDS; i++) 
    leds[i] = CRGB(70, 70, 70);
  FastLED.show(); 
  delay(1000);  
  
  //bootstrap average with some low values
  for (int i = 0; i < AVGLEN; i++) {  
    insert(250, avgs, AVGLEN);
  }

  //Initial values
  high.times = 0;
  high.times_start = millis();
  Color.r = 0;  
  Color.g = 0;
  Color.b = 1;

   // Initialize our coordinates to some random values
  x = random16();
  y = random16();
  z = random16();

  //pallette for fireplace
    gPal = HeatColors_p; 

 

}

// Lists of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { blobby_flow, blobby_flow, blobby_flow, XYmatrix, rainbow, rainbowWithGlitter, confetti, blobby_flow_gaps, blobby_flow, blobby_flow, blobby_flow, blobby_flow, sinelon, juggle, bpm, matrix_code, blobby_flow };  //non-reactive patterns
SimplePatternList musicPatterns = { visualize_music_4, visualize_music_2, center_burst_matrix, center_burst_FHT, vu10, center_burst_VU, blur_and_beat, visualize_music_4, visualize_music_2, center_burst_FHT, visualize_music_3, blur_and_spark, center_burst_matrix, vu7, center_burst_quad_pulse}; //music-reactive patterns



uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t musicCurrentPatternNumber = 0; // Index number of which pattern is current


void loop ()
  {
  // see if switch is open or closed
  int switchState = digitalRead (switchPin);
 
  // has it changed since last time?
  if (switchState == 0)
    {
      delay(300);
    if (oldMode == 4)
       {
        oldMode = 0;
//        Serial.print("oldMode is ");
//      Serial.println(oldMode);
 
       }  // end if switchState is LOW
      else {
        oldMode++;
        if (oldMode == 1) 
          { newborn = 1;}
        else newborn = 0;
//        Serial.print("oldMode is ");
//      Serial.println(oldMode);
   
      }
    }  // end of state change

  switch(oldMode) {
    case 0:
      rotate_all_patterns();
      break;
      
    case 1:
      rotate_music_patterns();
      break;
      
    case 2:
      Fireplace();
      //matrix_code();
      //center_burst_quad_pulse(); 
      //matrix_sinelon();
      break;
      
    case 3:
      matrix_code();
      //Fireplace();
      //matrix_sinelon();
      break;
      
    case 4:
      nightlight();
      //all2();
      break;
      
//    case 5:
//      center_burst_FHT();
//      break;
//      
//    case 6:
//      vu10();
//      break;
//      
//    case 7:
//      visualize_music_4();
//      break;
//      
//    case 8:
//      led_off();
//      break;
//   
//    case 9:
//      rotate_all_patterns();
//      break;
//      
//      
    default:
      break;
  }
    delay(DELAY);       // delay in between reads for stability
}


//////////////////////////////////////////////
//All red nightlight mode

void nightlight() {
      Serial.println("running led_mode3");

   for (int i =0; i < NUM_LEDS; i++) 
    leds[i] = CHSV (0, 250, 200);
  

FastLED.show();
delay(5);
}


//void led_mode3() {
//      Serial.println("running led_mode3");
//
//   for (int i =0; i < NUM_LEDS; i++) 
//    leds[i] = CRGB(0, 240, 0);
//  
//
//FastLED.show();
//delay(5);
//}
//
//void led_mode4() {
//
//      Serial.println("running led_mode4");
//
//   for (int i =0; i < NUM_LEDS; i++) 
//    leds[i] = CRGB(0, 0, 240);
//  
//
//FastLED.show();
//delay(5);
//}
//
//void led_mode5() {
//      Serial.println("running led_mode5");
//
//   for (int i =0; i < NUM_LEDS; i++) 
//    leds[i] = CRGB(240, 0, 0);
//  
//
//FastLED.show();
//delay(5);
//}
//

void led_off() {
      Serial.println("turning off LEDs");

   for (int i =0; i < NUM_LEDS; i++) 
    leds[i] = CRGB(0, 0, 0);
  

FastLED.show();
delay(5);
}



///////////////////////////////////////////////
//Begin Music Reactive patterns and functions

void rotate_music_patterns() {

  musicPatterns[musicCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND); 

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  EVERY_N_SECONDS( 15 ) { nextMusicPattern(); } // change patterns periodically

}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextMusicPattern() {

  reset_variables();
  // add one to the current pattern number, and wrap around at the end
  musicCurrentPatternNumber = (musicCurrentPatternNumber + 1) % ARRAY_SIZE( musicPatterns);
}


/**Funtion to check if the lamp should either enter a HIGH mode,
or revert to NORMAL if already in HIGH. If the sensors report values
that are higher than 1.1 times the average values, and this has happened
more than 30 times the last few milliseconds, it will enter HIGH mode. 
TODO: Not very well written, remove hardcoded values, and make it more
reusable and configurable.  */
void check_high(int avg) {
  if (avg > (song_avg/iter * 1.1))  {
    if (high.times != 0) {
      if (millis() - high.times_start > 200.0) {
        high.times = 0;
        songmode = NORMAL;
      } else {
        high.times_start = millis();  
        high.times++; 
      }
    } else {
      high.times++;
      high.times_start = millis();

    }
  }
  if (high.times > 30 && millis() - high.times_start < 50.0)
    songmode = HIGH;
  else if (millis() - high.times_start > 200) {
    high.times = 0;
    songmode = NORMAL;
  }
}

void height_and_condition () {
  
    //Actual sensor value
  sensor_value = analogRead(MIC_PIN);
  
  //If 0, discard immediately. Probably not right and save CPU.
  if (sensor_value == 0)
    return;

  //Discard readings that deviates too much from the past avg.
  mapped = (float)fscale(MIC_LOW, MIC_HIGH, MIC_LOW, (float)MIC_HIGH, (float)sensor_value, 2.0);
  avg = compute_average(avgs, AVGLEN);

  if (((avg - mapped) > avg*DEV_THRESH)) //|| ((avg - mapped) < -avg*DEV_THRESH))
    return;
  
  //Insert new avg. values
  insert(mapped, avgs, AVGLEN); 
  insert(avg, long_avg, LONG_SECTOR); 

  //Compute the "song average" sensor value
  song_avg += avg;
  iter++;
  if (iter > CYCLES) {  
    song_avg = song_avg / iter;
    iter = 1;
  }
    
  longavg = compute_average(long_avg, LONG_SECTOR);

  //Check if we enter HIGH mode 
  check_high(longavg);  

  if (songmode == HIGH) {
    fade_scale = 3;
    Color.r = 5;
    Color.g = 3;
    Color.b = -1;
  }
  else if (songmode == NORMAL) {
    fade_scale = 2;
    Color.r = -1;
    Color.b = 2;
    Color.g = 1;
  }

  //Decides how many of the LEDs will be lit
  height = fscale(MIC_LOW, MIC_HIGH, 0.0, (float)NUM_LEDS, (float)avg, -1);
  //height_2 = fscale(MIC_LOW, MIC_HIGH, 0.0, (float)NUM_LEDS_2, (float)avg, -1);
}




//void visualize_music() {
//
//  height_and_condition();
//  
//  /*Set the different leds. Control for too high and too low values.
//          Fun thing to try: Dont account for overflow in one direction, 
//    some interesting light effects appear! */
//  for (int i = 0; i < NUM_LEDS; i++) 
//    //The leds we want to show
//    if (i < height) {
//      if (leds[i].r + Color.r > 255)
//        leds[i].r = 255;
//      else if (leds[i].r + Color.r < 0)
//        leds[i].r = 0;
//      else
//        leds[i].r = leds[i].r + Color.r;
//          
//      if (leds[i].g + Color.g > 255)
//        leds[i].g = 255;
//      else if (leds[i].g + Color.g < 0)
//        leds[i].g = 0;
//      else 
//        leds[i].g = leds[i].g + Color.g;
//
//      if (leds[i].b + Color.b > 255)
//        leds[i].b = 255;
//      else if (leds[i].b + Color.b < 0)
//        leds[i].b = 0;
//      else 
//        leds[i].b = leds[i].b + Color.b;  
//      
//    //All the other LEDs begin their fading journey to eventual total darkness
//    } else {
//      leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
//    }
//
//    //keep peak at top
//  if(height > peak)     peak   = height; // Keep 'peak' dot at top
//  if(peak > 0 && peak < NUM_LEDS) leds[peak] = CHSV(200, 200, 200);  // set peak dot
//  FastLED.show(); 
//
//   if(++dotCount >= PEAK_FALL) { //fall rate 
//      
//      if(peak > 0) peak--;
//      dotCount = 0;
//    } 
//}




/////////////////////////////////////////////////////////////////////////////////////
//Center out spread visualizer

void visualize_music_2() {
  
  height_and_condition();
  height = height / 2;

  /*Set the different leds. Control for too high and too low values.
          Fun thing to try: Dont account for overflow in one direction, 
    some interesting light effects appear! */
  for (int i = 0; i < NUM_LEDS_HALF; i++) 
    //The leds we want to show
    if (i < height) {
      if (leds[NUM_LEDS_HALF-i-1].r + Color.r > 255) {
        leds[NUM_LEDS_HALF-i-1].r = 255;
        leds[NUM_LEDS_HALF+i].r = 255;
      }
      else if (leds[NUM_LEDS_HALF-i-1].r + Color.r < 0) {
        leds[NUM_LEDS_HALF-i-1].r = 0;
        leds[NUM_LEDS_HALF+i].r = 0;
      }
      else {
        leds[NUM_LEDS_HALF-i-1].r = leds[NUM_LEDS_HALF-i-1].r + Color.r;
        leds[NUM_LEDS_HALF+i].r = leds[NUM_LEDS_HALF+i].r + Color.r;
      }
          
      if (leds[NUM_LEDS_HALF-i-1].g + Color.g > 255) {
        leds[NUM_LEDS_HALF-i-1].g = 255;
        leds[NUM_LEDS_HALF+i].g = 255;
      }
      else if (leds[NUM_LEDS_HALF-i-1].g + Color.g < 0) {
        leds[NUM_LEDS_HALF-i-1].g = 0;
        leds[NUM_LEDS_HALF+i].g = 0;
      }
      else {
        leds[NUM_LEDS_HALF-i-1].g = leds[NUM_LEDS_HALF-i-1].g + Color.g;
        leds[NUM_LEDS_HALF+i].g = leds[NUM_LEDS_HALF+i].g + Color.g;
      }


      if (leds[NUM_LEDS_HALF-i-1].b + Color.b > 255) {
        leds[NUM_LEDS_HALF-i-1].b = 255;
        leds[NUM_LEDS_HALF+i].b = 255;
      }
      else if (leds[NUM_LEDS_HALF-i-1].b + Color.b < 0) {
        leds[NUM_LEDS_HALF-i-1].b = 0;
        leds[NUM_LEDS_HALF+i].b = 0;
      }
      else {
        leds[NUM_LEDS_HALF-i-1].b = leds[NUM_LEDS_HALF-i-1].b + Color.b;  
        leds[NUM_LEDS_HALF+i].b = leds[NUM_LEDS_HALF+i].b + Color.b;  
      }
      
    //All the other LEDs begin their fading journey to eventual total darkness
    } else {
      leds[NUM_LEDS_HALF-i-1] = CRGB(leds[NUM_LEDS_HALF-i-1].r/fade_scale, leds[NUM_LEDS_HALF-i-1].g/fade_scale, leds[NUM_LEDS_HALF-i-1].b/fade_scale);
      leds[NUM_LEDS_HALF+i] = CRGB(leds[NUM_LEDS_HALF+i].r/fade_scale, leds[NUM_LEDS_HALF+i].g/fade_scale, leds[NUM_LEDS_HALF+i].b/fade_scale);

    }

  //keep peak at top for small LED array
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 0 && peak < NUM_LEDS_HALF) {
    leds[NUM_LEDS_HALF-peak] = CHSV(200, 200, 200);  // set peak dot
    leds[NUM_LEDS_HALF+peak-1] = CHSV(200, 200, 200);  // set peak dot
  }

    addGlitter(100);

  FastLED.show(); 

    // Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL_split) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    } 
    
}


/////////////////////////////////////////////////////////////////////////////////////
//quadrasect spread out VU w/ falling dot

void visualize_music_3() {
  
  height_and_condition();
  height = height / 4;


  /*Set the different leds. Control for too high and too low values.
          Fun thing to try: Dont account for overflow in one direction, 
    some interesting light effects appear! */
  //Small LED Array 
  for (int i = 0; i < NUM_LEDS_QUARTER; i++) 
    //The leds we want to show
    if (i < height) {
      if (leds[i].r + Color.r > 255) {
        leds[i].r = 255;
        leds[NUM_LEDS_QUARTER+i].r = 255;
        leds[NUM_LEDS_HALF+i].r = 255;
        leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].r = 255;
      }
      else if (leds[i].r + Color.r < 0) {
        leds[i].r = 0;
        leds[NUM_LEDS_QUARTER+i].r = 0;
        leds[NUM_LEDS_HALF+i].r = 0;
        leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].r = 0;
      }
      else {
        leds[i].r = leds[i].r + Color.r;
        leds[NUM_LEDS_QUARTER+i].r = leds[NUM_LEDS_QUARTER+i].r + Color.r;
        leds[NUM_LEDS_HALF+i].r = leds[NUM_LEDS_HALF+i].r + Color.r;
        leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].r = leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].r + Color.r;
      }
          
      if (leds[i].g + Color.g > 255) {
        leds[i].g = 255;
        leds[NUM_LEDS_QUARTER+i].g = 255;
        leds[NUM_LEDS_HALF+i].g = 255;
        leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].g = 255;
      }
      else if (leds[i].g + Color.g < 0) {
        leds[i].g = 0;
        leds[NUM_LEDS_QUARTER+i].g = 0;
        leds[NUM_LEDS_HALF+i].g = 0;
        leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].g = 0;
      }
      else {
        leds[i].g = leds[i].g + Color.g;
        leds[NUM_LEDS_QUARTER+i].g = leds[NUM_LEDS_QUARTER+i].g + Color.g;
        leds[NUM_LEDS_HALF+i].g = leds[NUM_LEDS_HALF+i].g + Color.g;
        leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].g = leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].g + Color.g;
      }


      if (leds[i].b + Color.b > 255) {
        leds[i].b = 255;
        leds[NUM_LEDS_QUARTER+i].b = 255;
        leds[NUM_LEDS_HALF+i].b = 255;
        leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].b = 255;
      }
      else if (leds[i].b + Color.b < 0) {
        leds[i].b = 0;
        leds[NUM_LEDS_QUARTER+i].b = 0;
        leds[NUM_LEDS_HALF+i].b = 0;
        leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].b = 0;
      }
      else {
        leds[i].b = leds[i].b + Color.b;
        leds[NUM_LEDS_QUARTER+i].b = leds[NUM_LEDS_QUARTER+i].b + Color.b;
        leds[NUM_LEDS_HALF+i].b = leds[NUM_LEDS_HALF+i].b + Color.b;
        leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].b = leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].b + Color.b; 
      }
      
    //All the other LEDs begin their fading journey to eventual total darkness
    } else {
      leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
      leds[NUM_LEDS_QUARTER+i] = CRGB(leds[NUM_LEDS_QUARTER+i].r/fade_scale, leds[NUM_LEDS_QUARTER+i].g/fade_scale, leds[NUM_LEDS_QUARTER+i].b/fade_scale);
      leds[NUM_LEDS_HALF+i] = CRGB(leds[NUM_LEDS_HALF+i].r/fade_scale, leds[NUM_LEDS_HALF+i].g/fade_scale, leds[NUM_LEDS_HALF+i].b/fade_scale);
      leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i] = CRGB(leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].r/fade_scale, leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].g/fade_scale, leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+i].b/fade_scale);
    }

  //peak for small LED array
  //keep peak at top
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 0 && peak < NUM_LEDS_QUARTER) {
    leds[peak] = CHSV(200, 200, 200);  // set peak dot
    leds[NUM_LEDS_QUARTER+peak] = CHSV(200, 200, 200);  // set peak dot
    leds[NUM_LEDS_HALF+peak] = CHSV(200, 200, 200);
    leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+peak] = CHSV(200, 200, 200);
  }

  addGlitter(80);
  
  FastLED.show(); 

    // Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL_split) { //fall rate 
      if(peak > 0) peak--;
      dotCount = 0;
    } 
   
}


/////////////////////////////////////////////////////////////////////////////////////
//outside in spread visualizer

void visualize_music_4() {
  
  height_and_condition();
  height = height / 2;

  /*Set the different leds. Control for too high and too low values.
          Fun thing to try: Dont account for overflow in one direction, 
    some interesting light effects appear! */
  // Update Small LED array
  for (int i = 0; i < NUM_LEDS_HALF; i++) 
    //The leds we want to show
    if (i < height) {
      if (leds[i].r + Color.r > 255) {
        leds[i].r = 255;
        leds[NUM_LEDS-1-i].r = 255;
      }
      else if (leds[i].r + Color.r < 0) {
        leds[i].r = 0;
        leds[NUM_LEDS-1-i].r = 0;
      }
      else {
        leds[i].r = leds[i].r + Color.r;
        leds[NUM_LEDS-1-i].r = leds[NUM_LEDS-1-i].r + Color.r;
      }
          
      if (leds[i].g + Color.g > 255) {
        leds[i].g = 255;
        leds[NUM_LEDS-1-i].g = 255;
      }
      else if (leds[i].g + Color.g < 0) {
        leds[i].g = 0;
        leds[NUM_LEDS-1-i].g = 0;
      }
      else {
        leds[i].g = leds[i].g + Color.g;
        leds[NUM_LEDS-1-i].g = leds[NUM_LEDS-1-i].g + Color.g;
      }


      if (leds[i].b + Color.b > 255) {
        leds[i].b = 255;
        leds[NUM_LEDS-1-i].b = 255;
      }
      else if (leds[i].b + Color.b < 0) {
        leds[i].b = 0;
        leds[NUM_LEDS-1-i].b = 0;
      }
      else {
        leds[i].b = leds[i].b + Color.b;  
        leds[NUM_LEDS-1-i].b = leds[NUM_LEDS-1-i].b + Color.b;  
      }
      
    //All the other LEDs begin their fading journey to eventual total darkness
    } else {
      leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
      leds[NUM_LEDS-1-i] = CRGB(leds[NUM_LEDS-1-i].r/fade_scale, leds[NUM_LEDS-1-i].g/fade_scale, leds[NUM_LEDS-1-i].b/fade_scale);

    }

  //keep peak at top for small LED array
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 0 && peak < NUM_LEDS_HALF) {
    leds[peak] = CHSV(gHue, 200, 200);  // set peak dot
    leds[NUM_LEDS-1-peak] = CHSV(gHue, 200, 200);  // set peak dot
  }

  addGlitter(80);
  FastLED.show(); 

    // Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL_split) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    } 
    EVERY_N_MILLISECONDS(20) {gHue++;}
   
}



/////////////////////////////////////////////////////////////////////////////
//flash ripple and background pulse visualizer

void vu7() {

  EVERY_N_MILLISECONDS(1000) {
    peakspersec = peakcount;                                       // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;                                                 // Reset the counter every second.
  }

  //soundmems();
  soundrip();

  EVERY_N_MILLISECONDS(20) {
   ripple3();
  }

  // show_at_max_brightness_for_power();
  FastLED.show();
  delay(3);
} // loop()


//void soundmems() {                                                  // Rolling average counter - means we don't have to go through an array each time.
//  newtime = millis();
//  int tmp = analogRead(MIC_PIN);
//  sample = abs(tmp);
//
//  //int potin = map(analogRead(POT_PIN), 0, 1023, 0, 90);
//
//  samplesum = samplesum + sample - samplearray[samplecount];        // Add the new sample and remove the oldest sample in the array 
//  sampleavg = samplesum / NSAMPLES;                                 // Get an average
//  samplearray[samplecount] = sample;                                // Update oldest sample in the array with new sample
//  samplecount = (samplecount + 1) % NSAMPLES;                       // Update the counter for the array
//
//  //if (newtime > (oldtime + 200)) digitalWrite(13, LOW);             // Turn the LED off 200ms after the last peak.
//
//  if ((sample > (sampleavg + 30)) && (newtime > (oldtime + 50)) ) { // Check for a peak, which is 30 > the average, but wait at least 60ms for another.  
//    //note: hardcoded 30 instead of potin in above line
//    step = -1;
//    peakcount++;
//    //digitalWrite(13, HIGH);
//    oldtime = newtime;
//  }
//}  // soundmems()

void soundmems() {

  int tmp = analogRead(MIC_PIN) - 512;
  sample = abs(tmp);

} // soundmems()


void ripple3() {
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CHSV(bgcol, 255, sampleavg*2);  // Set the background colour.

  switch (step) {

    case -1:                                                          // Initialize ripple variables.
      center = random(NUM_LEDS);
      colour = (peakspersec*10) % 255;                                             // More peaks/s = higher the hue colour.
      step = 0;
      bgcol = bgcol+8;
      break;

    case 0:
      leds[center] = CHSV(colour, 255, 255);                          // Display the first pixel of the ripple.
      step ++;
      break;

    case maxsteps:                                                    // At the end of the ripples.
      // step = -1;
      break;

    default:                                                             // Middle of the ripples.
      leds[(center + step + NUM_LEDS) % NUM_LEDS] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds[(center - step + NUM_LEDS) % NUM_LEDS] += CHSV(colour, 255, myfade/step*2);
      step ++;                                                         // Next step.
      break;  
  } // switch step

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Slow blending change of hue with VU meter dot over it, combine with vu10() to make work

///Need to figure out what is making, all lights go on periodically and fix

void blur_and_beat() {

  height_and_condition();
  height = height / 4;
  
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
 
  uint8_t blurAmount = dim8_raw( beatsin8(3,64, 192) );       // A sinewave at 3 Hz with values ranging from 64 to 192.
  blur1d( leds, NUM_LEDS, blurAmount);                        // Apply some blurring to whatever's already on the strip, which will eventually go black.
  
  uint8_t  i = beatsin8(  9, 0, NUM_LEDS-1);
  uint8_t  j = beatsin8( 7, 0, NUM_LEDS-1);
  uint8_t  k = beatsin8(  5, 0, NUM_LEDS-1);

  
  // The color of each point shifts over time, each at a different speed.
  uint16_t ms = millis();  
  leds[(i+j)/2] = CHSV( ms / 29, 200, 255);
  leds[(j+k)/2] = CHSV( ms / 41, 200, 255);
  leds[(k+i)/2] = CHSV( ms / 73, 200, 255);
  leds[(k+i+j)/3] = CHSV( ms / 53, 200, 255);


  //VU Dot overlay on blur
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 2 && peak < NUM_LEDS_QUARTER) {
    
    for (int j = 0; j < 3; j++) {
      leds[peak-j] = CHSV(200, 200, 200);  // set peak dot
      leds[NUM_LEDS_QUARTER+peak-j] = CHSV(200, 200, 200);  // set peak dot
      leds[NUM_LEDS_HALF+peak-j] = CHSV(200, 200, 200);
      leds[NUM_LEDS_HALF+NUM_LEDS_QUARTER+peak-j] = CHSV(200, 200, 200);
    }
  }

 
  FastLED.show();

  if(++dotCount >= PEAK_FALL_split-5) { //fall rate 
      if(peak > 0) peak--;
      dotCount = 0;
    } 
   
  
  //delay(2);
} // loop()



//////////////////////////////////////////////////////////////////////////////////////////////////////////
// blur with ripple spark overlay

void blur_and_spark() {

  height_and_condition();
  height = height / 4;

  EVERY_N_MILLISECONDS(1000) {
    peakspersec = peakcount;                                  // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;                                            // Reset the counter every second.
  }
  
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);


  uint8_t blurAmount = dim8_raw( beatsin8(3,64, 192) );       // A sinewave at 3 Hz with values ranging from 64 to 192.
  blur1d( leds, NUM_LEDS, blurAmount);                        // Apply some blurring to whatever's already on the strip, which will eventually go black.
  
  uint8_t  i = beatsin8(  9, 0, NUM_LEDS-1);
  uint8_t  j = beatsin8( 7, 0, NUM_LEDS-1);
  uint8_t  k = beatsin8(  5, 0, NUM_LEDS-1);

  // The color of each point shifts over time, each at a different speed.
  uint16_t ms = millis();  
  leds[(i+j)/2] = CHSV( ms / 29, 200, 255);
  leds[(j+k)/2] = CHSV( ms / 41, 200, 255);
  leds[(k+i)/2] = CHSV( ms / 73, 200, 255);
  leds[(k+i+j)/3] = CHSV( ms / 53, 200, 255);


  soundrip_2();

  EVERY_N_MILLISECONDS(30) {
   rippled_2();
  }


  FastLED.show();

} // loop()


void soundrip_2() {                                            // Rolling average counter - means we don't have to go through an array each time.

  newtime = millis();
  int tmp = analogRead(MIC_PIN);
  sample = abs(tmp);

  //int potin = map(analogRead(POT_PIN), 0, 1023, 0, 60) + 20;

  samplesum = samplesum + sample - samplearray[samplecount];  // Add the new sample and remove the oldest sample in the array 
  sampleavg = samplesum / NSAMPLES;                           // Get an average
  samplearray[samplecount] = sample;                          // Update oldest sample in the array with new sample
  samplecount = (samplecount + 1) % NSAMPLES;                 // Update the counter for the array

  if (newtime > (oldtime + 200)) digitalWrite(13, LOW);       // Turn the LED off 200ms after the last peak.
EVERY_N_MILLISECONDS(1000) {
    uint8_t width_adj = random8(15);
    width = 12 + (width_adj * 8);
  }

    if ((sample > (sampleavg + width)) && (newtime > (oldtime + 30)) )  { // Check for a peak, which is 30 > the average, but wait at least 60ms for another.
    step = -1;
    peakcount++;
    //digitalWrite(13, HIGH);
    oldtime = newtime;
  }
  
}  // soundmems()



void rippled_2() {

  fadeToBlackBy(leds, NUM_LEDS, 44);                          // 8 bit, 1 = slow, 255 = fast

  switch (step) {

    case -1:                                                  // Initialize ripple variables.
      center = random(NUM_LEDS);
      colour = (peakspersec*10) % 255;                        // More peaks/s = higher the hue colour.
      step = 0;
      break;

    case 0:
      leds[center] = CHSV(colour, 255, 255);                  // Display the first pixel of the ripple.
      step ++;
      break;

    case maxsteps:                                            // At the end of the ripples.
      // step = -1;
      break;

    default:                                                  // Middle of the ripples.
      leds[(center + step + NUM_LEDS) % NUM_LEDS] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds[(center - step + NUM_LEDS) % NUM_LEDS] += CHSV(colour, 255, myfade/step*2);
      
      step ++;                                                // Next step.
      break;  
  } // switch step
  
} // ripple_2()


///////////////////////////////////////////////////////
//Center Burst  Visualizer

void center_burst_FHT() {
  
  EVERY_N_MILLISECONDS(100) {                                 // AWESOME palette blending capability.
     
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);
  }


  EVERY_N_SECONDS(5) {                                        // Change the palette every 5 seconds.
    for (int i = 0; i < 16; i++) {
      targetPalette[i] = CHSV(random8(), 255, 255);
    }
  }

  EVERY_N_MILLISECONDS(18) {
    fadeToBlackBy(leds, NUM_LEDS, 4);                            // 8 bit, 1 = slow, 255 = fast    
    sndwave();
    soundmems();
  }

  FastLED.show();
}

void sndwave() {
  
  leds[NUM_LEDS/2] = ColorFromPalette(currentPalette, sample, sample*2, currentBlending); 

  for (int i = NUM_LEDS - 1; i > NUM_LEDS/2; i--) {       //move to the left
    leds[i] = leds[i - 1];
  }

  for (int i = 0; i < NUM_LEDS/2; i++) {                  // move to the right
    leds[i] = leds[i + 1];
  }

} // sndwave()




///////////////////////////////////////////////////////

void center_burst_VU() {
  
  EVERY_N_MILLISECONDS(100) {                                 // AWESOME palette blending capability.
     
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);
  }


  EVERY_N_MILLISECONDS(25) {
    fadeToBlackBy(leds, NUM_LEDS, 15);                            // 8 bit, 1 = slow, 255 = fast    
    sndwave();
    soundmems();
  }
  
  height_and_condition();
  height = height / 2;

  for (int i = 0; i < NUM_LEDS_HALF-48; i++) 
    //The leds we want to show
    if (i < height) {
      if (leds[i].r + Color.r > 255) {
        leds[i].r = 255;
        leds[NUM_LEDS-1-i].r = 255;
      }
      else if (leds[i].r + Color.r < 0) {
        leds[i].r = 0;
        leds[NUM_LEDS-1-i].r = 0;
      }
      else {
        leds[i].r = leds[i].r + Color.r;
        leds[NUM_LEDS-1-i].r = leds[NUM_LEDS-1-i].r + Color.r;
      }
          
      if (leds[i].g + Color.g > 255) {
        leds[i].g = 255;
        leds[NUM_LEDS-1-i].g = 255;
      }
      else if (leds[i].g + Color.g < 0) {
        leds[i].g = 0;
        leds[NUM_LEDS-1-i].g = 0;
      }
      else {
        leds[i].g = leds[i].g + Color.g;
        leds[NUM_LEDS-1-i].g = leds[NUM_LEDS-1-i].g + Color.g;
      }

      if (leds[i].b + Color.b > 255) {
        leds[i].b = 255;
        leds[NUM_LEDS-1-i].b = 255;
      }
      else if (leds[i].b + Color.b < 0) {
        leds[i].b = 0;
        leds[NUM_LEDS-1-i].b = 0;
      }
      else {
        leds[i].b = leds[i].b + Color.b;  
        leds[NUM_LEDS-1-i].b = leds[NUM_LEDS-1-i].b + Color.b;  
      }
      
    //All the other LEDs begin their fading journey to eventual total darkness
    } else {
      leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
      leds[NUM_LEDS-1-i] = CRGB(leds[NUM_LEDS-1-i].r/fade_scale, leds[NUM_LEDS-1-i].g/fade_scale, leds[NUM_LEDS-1-i].b/fade_scale);

    }

    //keep peak at top for small LED array
    if(height > peak)     peak   = height; // Keep 'peak' dot at top
    if(peak > 0 && peak < NUM_LEDS_HALF) {
      leds[peak] = CHSV(gHue, 200, 200);  // set peak dot
      leds[NUM_LEDS-1-peak] = CHSV(gHue, 200, 200);  // set peak dot
    }
    
  EVERY_N_SECONDS(5) {                                        // Change the palette every 5 seconds.
    for (int i = 0; i < 16; i++) {
      targetPalette[i] = CHSV(random8(), 255, 255);
    }
  }

  FastLED.show();


  if(++dotCount >= PEAK_FALL_split) { //fall rate 
      if(peak > 0) peak--;
      dotCount = 0;
  }
}

///////////////////////////////////////////////////////
//Center Burst and quad pulse overlay


void center_burst_quad_pulse() {

   
  EVERY_N_MILLISECONDS(100) {                                 // AWESOME palette blending capability.
     
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);
  }


  EVERY_N_SECONDS(5) {                                        // Change the palette every 5 seconds.
    for (int i = 0; i < 16; i++) {
      targetPalette[i] = CHSV(random8(), 255, 255);
    }
  }

  EVERY_N_MILLISECONDS(25) {
    fadeToBlackBy(leds, NUM_LEDS, 10);                            // 8 bit, 1 = slow, 255 = fast    
    sndwave();
    soundmems();
  }

  height_and_condition();
  height = height / 2;


  //keep peak at top for small LED array
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 2 && peak < NUM_LEDS_HALF) {
    for (int j = 0; j < 3; j++) {
      leds[peak-j] = CHSV(200, 200, 200);  // set peak dot
      leds[NUM_LEDS-1-peak+j] = CHSV(200, 200, 200);  // set peak dot
    }
  }


  FastLED.show(); 

    // Every few frames, make the peak pixel drop by 1:

    if(++dotCount >= PEAK_FALL_split-4) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
  
  delay(2);
}



///////////////////////////////////////////////////////////////////
//whole matrix center burst

void center_burst_matrix() {
  
   EVERY_N_MILLISECONDS(100) {                                 // AWESOME palette blending capability.
     
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);
   }


  EVERY_N_SECONDS(5) {                                        // Change the palette every 5 seconds.
    for (int i = 0; i < 16; i++) {
      targetPalette[i] = CHSV(random8(), 255, 255);
    }
  }

  EVERY_N_MILLISECONDS(25) {
    fadeToBlackBy(leds, NUM_LEDS, 50);                            // 8 bit, 1 = slow, 255 = fast    
    sndwave_matrix();
    soundmems();
  }

  addGlitter(60);
  FastLED.show();
}

void sndwave_matrix() {
  
  for (int i = 0; i < kMatrixWidth; i++) {
  
    leds[XY(i, kMatrixHeight/2)] =    ColorFromPalette(currentPalette, sample, sample*2, currentBlending); 
    
    for(int j = kMatrixHeight - 1; j > kMatrixHeight/2; j--) {
      leds[XY(i,j)] = leds[XY(i,j-1)]; //move to the left 
    }

    for(int j = 0 ; j < kMatrixHeight/2; j++) {
      leds[XY(i,j)] = leds[XY(i,j+1)]; // move to the right;
    }
  }

} // sndwave_matrix()

///////////////////////////////////////////////////////////////////
// Ripple with Black background and variable width


void vu10() {

  EVERY_N_MILLISECONDS(1000) {
    peakspersec = peakcount;                                  // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;                                            // Reset the counter every second.
  }

  soundrip();

  EVERY_N_MILLISECONDS(20) {
   rippled();
  }

   FastLED.show();
  delay(3);
} // loop()


void soundrip() {                                            // Rolling average counter - means we don't have to go through an array each time.

  newtime = millis();
  int tmp = analogRead(MIC_PIN);
  sample = abs(tmp);

  //int potin = map(analogRead(POT_PIN), 0, 1023, 0, 60) + 20;

  samplesum = samplesum + sample - samplearray[samplecount];  // Add the new sample and remove the oldest sample in the array 
  sampleavg = samplesum / NSAMPLES;                           // Get an average
  samplearray[samplecount] = sample;                          // Update oldest sample in the array with new sample
  samplecount = (samplecount + 1) % NSAMPLES;                 // Update the counter for the array

 // if (newtime > (oldtime + 400)) digitalWrite(13, LOW);       // Turn the LED off 400ms after the last peak.


  
  EVERY_N_MILLISECONDS(1000) {
    uint8_t width_adj = random8(15);
    width = 12 + (width_adj * 8);
  }
  if ((sample > (sampleavg + width)) && (newtime > (oldtime + 30)) ) { // Check for a peak, which is 30 > the average, but wait at least 60ms for another.
    step = -1;
    peakcount++;
    oldtime = newtime;
  }
  
}  // soundmems()



void rippled() {

  fadeToBlackBy(leds, NUM_LEDS, 44);                          // 8 bit, 1 = slow, 255 = fast

  switch (step) {

    case -1:                                                  // Initialize ripple variables.
      center = random(NUM_LEDS);
      colour = (peakspersec*10) % 255;                        // More peaks/s = higher the hue colour.
      step = 0;
      break;

    case 0:
      leds[center] = CHSV(colour, 255, 255);                  // Display the first pixel of the ripple.
      step ++;
      break;

    case maxsteps:                                            // At the end of the ripples.
      // step = -1;
      break;

    default:                                                  // Middle of the ripples.
      leds[(center + step + NUM_LEDS) % NUM_LEDS] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds[(center - step + NUM_LEDS) % NUM_LEDS] += CHSV(colour, 255, myfade/step*2);
      step ++;                                                // Next step.
      break;  
  } // switch step
  
} // ripple()





//Compute average of a int array, given the starting pointer and the length
int compute_average(int *avgs, int len) {
  int sum = 0;
  for (int i = 0; i < len; i++)
    sum += avgs[i];

  return (int)(sum / len);

}

//Insert a value into an array, and shift it down removing
//the first value if array already full 
void insert(int val, int *avgs, int len) {
  for (int i = 0; i < len; i++) {
    if (avgs[i] == -1) {
      avgs[i] = val;
      return;
    }  
  }

  for (int i = 1; i < len; i++) {
    avgs[i - 1] = avgs[i];
  }
  avgs[len - 1] = val;
}

//Function imported from the arduino website.
//Basically map, but with a curve on the scale (can be non-uniform).
float fscale( float originalMin, float originalMax, float newBegin, float
    newEnd, float inputValue, float curve){

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){ 
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd; 
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {   
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange); 
  }

  return rangedValue;
}


//End Music Reactive Functions
//////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////
//FASTLED MATRIX FUNCTIONS
//////////////////////////////////////////////////////////////////////////

void blobby_flow_gaps() {
  // fill the led array 2/16-bit noise values
  fill_2dnoise16(LEDS.leds(), kMatrixWidth, kMatrixHeight, kMatrixSerpentineLayout,
                octaves,x,xscale,y,yscale,v_time,
                hue_octaves,hxy,hue_scale,hxy,hue_scale,hue_time, false);

  LEDS.show();

  // adjust the intra-frame time values
  x += x_speed;
  y += y_speed;
  v_time += time_speed;
  hue_time += hue_speed;
   delay(50);
}


void blobby_flow() {

 
  if (newborn ==1) {   
    SetupBlackAndWhiteStripedPalette(); 
    ChangeSettingsPeriodically();  
  } else {
    // Periodically choose a new palette, speed, and scale
     ChangePaletteAndSettingsPeriodically();
  }


  // generate noise data
  fillnoise8();
  
  // convert the noise data to colors in the LED array
  // using the current palette
  mapNoiseToLEDsUsingPalette();

  LEDS.show();
  // delay(10);
}



// Fill the x/y array of 8-bit noise values using the inoise8 function.
void fillnoise8() {
  // If we're runing at a low "speed", some 8-bit artifacts become visible
  // from frame-to-frame.  In order to reduce this, we can do some fast data-smoothing.
  // The amount of data smoothing we're doing depends on "speed".
  uint8_t dataSmoothing = 0;
  if( speed < 50) {
    dataSmoothing = 200 - (speed * 4);
  }
  
  for(int i = 0; i < MAX_DIMENSION; i++) {
    int ioffset = scale * i;
    for(int j = 0; j < MAX_DIMENSION; j++) {
      int joffset = scale * j;
      
      uint8_t data = inoise8(x + ioffset,y + joffset,z);

      // The range of the inoise8 function is roughly 16-238.
      // These two operations expand those values out to roughly 0..255
      // You can comment them out if you want the raw noise data.
      data = qsub8(data,16);
      data = qadd8(data,scale8(data,39));

      if( dataSmoothing ) {
        uint8_t olddata = noise[i][j];
        uint8_t newdata = scale8( olddata, dataSmoothing) + scale8( data, 256 - dataSmoothing);
        data = newdata;
      }
      
      noise[i][j] = data;
    }
  }
  
  z += speed;
  
  // apply slow drift to X and Y, just for visual variation.
  x += speed / 8;
  y -= speed / 16;
}

void mapNoiseToLEDsUsingPalette()
{
  static uint8_t ihue=0;
  
  for(int i = 0; i < kMatrixWidth; i++) {
    for(int j = 0; j < kMatrixHeight; j++) {
      // We use the value at the (i,j) coordinate in the noise
      // array for our brightness, and the flipped value from (j,i)
      // for our pixel's index into the color palette.

      uint8_t index = noise[j][i];
      uint8_t bri =   noise[i][j];

      // if this palette is a 'loop', add a slowly-changing base value
      if( colorLoop) { 
        index += ihue;
      }

      // brighten up, as the color palette itself often contains the 
      // light/dark dynamic range desired
      if( bri > 127 ) {
        bri = 255;
      } else {
        bri = dim8_raw( bri * 2);
      }

      CRGB color = ColorFromPalette( currentPalette, index, bri);
      leds[XY(i,j)] = color;
    }
  }
  
  ihue+=1;
}



// There are several different palettes of colors demonstrated here.
//
// FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.
//
// Additionally, you can manually define your own color palettes, or you can write
// code that creates color palettes on the fly.

// 1 = 5 sec per palette
// 2 = 10 sec per palette
// etc
#define HOLD_PALETTES_X_TIMES_AS_LONG 1

void ChangePaletteAndSettingsPeriodically()
{
  EVERY_N_MILLISECONDS(100) {
  uint8_t secondHand = ((millis() / 1000) / HOLD_PALETTES_X_TIMES_AS_LONG) % 60;
  static uint8_t lastSecond = 99;
  
  if( lastSecond != secondHand) {
    lastSecond = secondHand;
    if( secondHand ==  0)  { 
      currentPalette = RainbowColors_p;  
      speed = 6;       
//      speed = 20; 
      scale = 30; 
      colorLoop = 1; }
    if( secondHand ==  5)  { 
      SetupPurpleAndGreenPalette();             
     speed = 4; 
//     speed = 10; 
      scale = 50; 
      colorLoop = 1; }
    if( secondHand == 10)  { 
      SetupRandomPalette();          
      speed = 4; 
//      speed = 20;  
      scale = 30; 
      colorLoop = 1; }
    if( secondHand == 15)  { 
      currentPalette = ForestColors_p;          
      speed =  12; 
      scale =120; 
      colorLoop = 0; }
    if( secondHand == 20)  { 
      SetupRandomPalette();          
      speed =  6; 
      scale = 30; 
      colorLoop = 1; }
    if( secondHand == 25)  { 
      currentPalette = LavaColors_p;            
      speed =  8; 
      scale = 50; 
      colorLoop = 0; }
    if( secondHand == 30)  { 
       SetupRandomPalette();        
      speed = 4; 
//      speed = 20; 
      scale = 40; 
      colorLoop = 0; }
    if( secondHand == 35)  { 
      currentPalette = PartyColors_p;           
      speed = 8; 
//      speed = 20; 
      scale = 30; 
      colorLoop = 1; }
    if( secondHand == 40)  { 
      SetupBlackAndWhiteStripedPalette();                   
     speed = 8; 
//      speed = 20; 
      scale = 20; 
      colorLoop = 1; }
    if( secondHand == 45)  { 
      SetupRandomPalette();                     
     speed = 5; 
//     speed = 50; 
      scale = 25; 
      colorLoop = 1; }
    if( secondHand == 50)  { 
      currentPalette = CloudColors_p;                       
     speed = 8; 
//     speed = 90; 
      scale = 90; 
      colorLoop = 1; }
    if( secondHand == 55)  { 
      currentPalette = OceanColors_p;   
     speed = 8; 
//     speed = 30; 
      scale = 20; 
      colorLoop = 1; }
  }
  }
}

void ChangeSettingsPeriodically()
{
  EVERY_N_MILLISECONDS(100) {
  uint8_t secondHand = ((millis() / 1000) / HOLD_PALETTES_X_TIMES_AS_LONG) % 60;
  static uint8_t lastSecond = 99;
  
  if( lastSecond != secondHand) {
    lastSecond = secondHand;
    if( secondHand ==  0)  { 
      speed = 3;       
//      speed = 20; 
      scale = 30; 
      colorLoop = 1; }
    if( secondHand ==  5)  { 
     speed = 2; 
//     speed = 10; 
      scale = 50; 
      colorLoop = 1; }
    if( secondHand == 10)  { 
      speed = 2; 
//      speed = 20;  
      scale = 30; 
      colorLoop = 1; }
    if( secondHand == 15)  { 
      speed =  6; 
      scale =120; 
      colorLoop = 0; }
    if( secondHand == 20)  { 
      speed =  3; 
      scale = 30; 
      colorLoop = 0; }
    if( secondHand == 25)  { 
      currentPalette = LavaColors_p;            
      speed =  4; 
      scale = 50; 
      colorLoop = 0; }
    if( secondHand == 30)  { 
      speed = 4; 
//      speed = 20; 
      scale = 90; 
      colorLoop = 0; }
    if( secondHand == 35)  { 
      speed = 4; 
//      speed = 20; 
      scale = 30; 
      colorLoop = 1; }
    if( secondHand == 40)  { 
     speed = 4; 
//      speed = 20; 
      scale = 20; 
      colorLoop = 1; }
    if( secondHand == 45)  { 
     speed = 4; 
//     speed = 50; 
      scale = 50; 
      colorLoop = 1; }
    if( secondHand == 50)  { 
     speed = 8; 
//     speed = 90; 
      scale = 90; 
      colorLoop = 1; }
    if( secondHand == 55)  { 
     speed = 4; 
//     speed = 30; 
      scale = 20; 
      colorLoop = 1; }
  }
  }
}

// This function generates a random palette that's a gradient
// between four different colors.  The first is a dim hue, the second is 
// a bright hue, the third is a bright pastel, and the last is 
// another bright hue.  This gives some visual bright/dark variation
// which is more interesting than just a gradient of different hues.
void SetupRandomPalette()
{
  currentPalette = CRGBPalette16( 
                      CHSV( random8(), 255, 32), 
                      CHSV( random8(), 255, 255), 
                      CHSV( random8(), 128, 255), 
                      CHSV( random8(), 255, 255)); 
}

// This function sets up a palette of black and white stripes,
// using code.  Since the palette is effectively an array of
// sixteen CRGB colors, the various fill_* functions can be used
// to set them up.
void SetupBlackAndWhiteStripedPalette()
{
  // 'black out' all 16 palette entries...
  fill_solid( currentPalette, 16, CRGB::Black);
  // and set every fourth one to white.
  currentPalette[0] = CRGB::White;
  currentPalette[4] = CRGB::White;
  currentPalette[8] = CRGB::White;
  currentPalette[12] = CRGB::White;

}

// This function sets up a palette of purple and green stripes.
void SetupPurpleAndGreenPalette()
{
  CRGB purple = CHSV( HUE_PURPLE, 255, 255);
  CRGB green  = CHSV( HUE_GREEN, 255, 255);
  CRGB black  = CRGB::Black;
  
  currentPalette = CRGBPalette16( 
    green,  green,  black,  black,
    purple, purple, black,  black,
    green,  green,  black,  black,
    purple, purple, black,  black );
}


//
// Mark's xy coordinate mapping code.  See the XYMatrix for more information on it.
//
uint16_t XY( uint8_t x, uint8_t y)
{
  uint16_t i;
  if( kMatrixSerpentineLayout == false) {
    i = (y * kMatrixWidth) + x;
  }
  if( kMatrixSerpentineLayout == true) {
    if( y & 0x01) {
      // Odd rows run backwards
      uint8_t reverseX = (kMatrixWidth - 1) - x;
      i = (y * kMatrixWidth) + reverseX;
    } else {
      // Even rows run forwards
      i = (y * kMatrixWidth) + x;
    }
  }
  return i;
}



////////////////////////////////////
///Blur Visualize

void blur() {


  EVERY_N_MILLISECONDS(40) {
  uint8_t blurAmount = dim8_raw( beatsin8(3,64, 192) );       // A sinewave at 3 Hz with values ranging from 64 to 192.
  blur1d( leds, NUM_LEDS, blurAmount);                        // Apply some blurring to whatever's already on the strip, which will eventually go black.
  
  uint8_t  i = beatsin8(  9, 0, NUM_LEDS-1);
  uint8_t  j = beatsin8( 7, 0, NUM_LEDS-1);
  uint8_t  k = beatsin8(  5, 0, NUM_LEDS-1);

  
  // The color of each point shifts over time, each at a different speed.
  uint16_t ms = millis();  
  leds[(i+j)/2] = CHSV( ms / 29, 200, 255);
  leds[(j+k)/2] = CHSV( ms / 41, 200, 255);
  leds[(k+i)/2] = CHSV( ms / 73, 200, 255);
  leds[(k+i+j)/3] = CHSV( ms / 53, 200, 255);
 
  FastLED.show();
  }
 
} // loop()



//////Matrix Sinelon
void matrix_sinelon() {
    fadeToBlackBy( leds, NUM_LEDS, 25);
    int pos = beatsin16( 19, 0, kMatrixWidth-1 );

    for(int i = 0; i < kMatrixWidth; i++) {
      leds[XY(i, pos)] += CHSV( gHue, 240, 140);
    }

    for(int i = 0; i < kMatrixHeight; i++) {
      leds[XY(pos, i)] += CHSV( gHue+100, 240, 140);
    }
    FastLED.show();
  EVERY_N_MILLISECONDS( 50 ) { gHue++; } // slowly cycle the "base color" through the rainbow

}



//////////////////////////////////////////////////////
//////////////////Rotate Non-Reactive Visualizers

void rotate_all_patterns() {

  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND); 

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  EVERY_N_SECONDS( 20 ) { nextPattern(); } // change patterns periodically

}


void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

///////////////////////////////////////////////////////////////////
//FASTLED XYMATRIX PATTERN EXAMPLE

void XYmatrix()
{
    uint32_t ms = millis();
    int32_t yHueDelta32 = ((int32_t)cos16( ms * (27/1) ) * (350 / kMatrixWidth));
    int32_t xHueDelta32 = ((int32_t)cos16( ms * (39/1) ) * (310 / kMatrixHeight));
    DrawOneFrame( ms / 65536, yHueDelta32 / 32768, xHueDelta32 / 32768);
    if( ms < 5000 ) {
      FastLED.setBrightness( scale8( BRIGHTNESS, (ms * 256) / 5000));
    } else {
      FastLED.setBrightness(BRIGHTNESS);
    }
    FastLED.show();
}

void DrawOneFrame( byte startHue8, int8_t yHueDelta8, int8_t xHueDelta8)
{
  byte lineStartHue = startHue8;
  for( byte y = 0; y < kMatrixHeight; y++) {
    lineStartHue += yHueDelta8;
    byte pixelHue = lineStartHue;      
    for( byte x = 0; x < kMatrixWidth; x++) {
      pixelHue += xHueDelta8;
      leds[ XY(x, y)]  = CHSV( pixelHue, 255, 255);
    }
  }
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
//Put in black and white for newborn baby mode

void newborn_mode() {
   SetupBlackAndWhiteStripedPalette();       
}


/////////////////////////////////////////////////
/////////////////////////////////////////////////
//Fireplace visualizer

void Fireplace () {
  
  random16_add_entropy( random() ); // We chew a lot of entropy

  static unsigned int spark[kMatrixHeight]; // base heat
  CRGB stack[kMatrixHeight][kMatrixWidth];        // stacks that are cooler
 
  // 1. Generate sparks to re-heat
  for( int i = 0; i < kMatrixHeight; i++) {
    if (spark[i] < HOT ) {
      int base = HOT * 2;
      spark[i] = random16( base, MAXHOT );
    }
  }
  
  // 2. Cool all the sparks
  for( int i = 0; i < kMatrixHeight; i++) {
    spark[i] = qsub8( spark[i], random8(0, COOLING) );
  }
  
  // 3. Build the stack
  /*    This works on the idea that pixels are "cooler"
        as they get further from the spark at the bottom */
  for( int i = 0; i < kMatrixHeight; i++) {
    unsigned int heat = constrain(spark[i], HOT, MAXHOT);
    for( int j = kMatrixWidth-1; j >= 0; j--) {
      /* Calculate the color on the palette from how hot this
         pixel is */
      byte index = constrain(heat, 0, HOT);
      stack[i][j] = ColorFromPalette( gPal, index );
      
      /* The next higher pixel will be "cooler", so calculate
         the drop */
      unsigned int drop = random8(0,HOT);
      if (drop > heat) heat = 0; // avoid wrap-arounds from going "negative"
      else heat -= drop;
 
      heat = constrain(heat, 0, MAXHOT);
    }
  }
  
  // 4. map stacks to led array
  for( int i = 0; i < kMatrixHeight; i++) {
    for( int j = 0; j < kMatrixWidth; j++) {
     leds[ XY(j, i) ] = stack[i][j];
    }
  }
  FastLED.show();
  FastLED.delay(FPS_DELAY); 
}

////////////////////////////////////////////////////
//FastLED Matrix Code Pattern

void matrix_code() {
  EVERY_N_MILLIS(75) // falling speed
  {
    // move code downward
    // start with lowest row to allow proper overlapping on each column
    for (int8_t row=kMatrixHeight-1; row>=0; row--)
    {
      for (int8_t col=0; col<kMatrixWidth; col++)
      {
        if (leds[getIndex(row, col)] == CRGB(175,255,175))
        {
          leds[getIndex(row, col)] = CRGB(27,130,39); // create trail
          if (row < kMatrixHeight-1) leds[getIndex(row+1, col)] = CRGB(175,255,175);
        }
      }
    }

    // fade all leds
    for(int i = 0; i < NUM_LEDS; i++) {
      if (leds[i].g != 255) leds[i].nscale8(192); // only fade trail
    }

    // check for empty screen to ensure code spawn
    bool emptyScreen = true;
    for(int i = 0; i < NUM_LEDS; i++) {
      if (leds[i])
      {
        emptyScreen = false;
        break;
      }
    }

    // spawn new falling code
    if (random8(3) == 0 || emptyScreen) // lower number == more frequent spawns
    {
      int8_t spawnX = random8(kMatrixWidth);
      leds[getIndex(0, spawnX)] = CRGB(175,255,175 );
    }

    FastLED.show();
  }
}

// convert x/y cordinates to LED index on zig-zag grid
uint16_t getIndex(uint16_t x, uint16_t y)
{
  uint16_t index;
  if (y == 0)
  {
    index = x;
  }
  else if (y % 2 == 0)
  {
    index = y * kMatrixWidth + x;
  }
  else
  {
    index = ((y * kMatrixWidth) + (kMatrixWidth-1)) - x;
  }
  return index;
}



////////////////////////////////////////////////////
//Cine-lights Torch Functions
///not working now -- go back and fix later
//
//
//inline void reduce(byte &aByte, byte aAmount, byte aMin = 0)
//{
//  int r = aByte-aAmount;
//  if (r<aMin)
//    aByte = aMin;
//  else
//    aByte = (byte)r;
//}
//
//
//inline void increase(byte &aByte, byte aAmount, byte aMax = 255)
//{
//  int r = aByte+aAmount;
//  if (r>aMax)
//    aByte = aMax;
//  else
//    aByte = (byte)r;
//}
//
//uint16_t random2(uint16_t aMinOrMax, uint16_t aMax = 0)
//{
//  if (aMax==0) {
//    aMax = aMinOrMax;
//    aMinOrMax = 0;
//  }
//  uint32_t r = aMinOrMax;
//  aMax = aMax - aMinOrMax + 1;
//  r += rand() % aMax;
//  return r;
//}
//
//void resetEnergy()
//{
//  for (int i=0; i<NUM_LEDS; i++) {
//    currentEnergy[i] = 0;
//    nextEnergy[i] = 0;
//    energyMode[i] = torch_passive;
//  }
//}
//
//void calcNextEnergy()
//{
//  int i = 0;
//  for (int y=0; y<levels; y++) {
//    for (int x=0; x<ledsPerLevel; x++) {
//      byte e = currentEnergy[i];
//      byte m = energyMode[i];
//      switch (m) {
//        case torch_spark: {
//          // loose transfer up energy as long as the is any
//          reduce(e, spark_tfr);
//          // cell above is temp spark, sucking up energy from this cell until empty
//          if (y<levels-1) {
//            energyMode[i+ledsPerLevel] = torch_spark_temp;
//          }
//          break;
//        }
//        case torch_spark_temp: {
//          // just getting some energy from below
//          byte e2 = currentEnergy[i-ledsPerLevel];
//          if (e2<spark_tfr) {
//            // cell below is exhausted, becomes passive
//            energyMode[i-ledsPerLevel] = torch_passive;
//            // gobble up rest of energy
//            increase(e, e2);
//            // loose some overall energy
//            e = ((int)e*spark_cap)>>8;
//            // this cell becomes active spark
//            energyMode[i] = torch_spark;
//          }
//          else {
//            increase(e, spark_tfr);
//          }
//          break;
//        }
//        case torch_passive: {
//          e = ((int)e*heat_cap)>>8;
//          increase(e, ((((int)currentEnergy[i-1]+(int)currentEnergy[i+1])*side_rad)>>9) + (((int)currentEnergy[i-ledsPerLevel]*up_rad)>>8));
//        }
//        default:
//          break;
//      }
//      nextEnergy[i++] = e;
//    }
//  }
//}
//
//const uint8_t energymap[32] = {0, 64, 96, 112, 128, 144, 152, 160, 168, 176, 184, 184, 192, 200, 200, 208, 208, 216, 216, 224, 224, 224, 232, 232, 232, 240, 240, 240, 240, 248, 248, 248};
//
//void calcNextColors()
//{
//  for (int i=0; i<NUM_LEDS; i++) {
//    int ei; // index into energy calculation buffer
//    if (upside_down)
//      ei = NUM_LEDS-i;
//    else
//      ei = i;
//    uint16_t e = nextEnergy[ei];
//    currentEnergy[ei] = e;
//    if (e>250)
//      leds[i] = CRGB(170, 170, e); // blueish extra-bright spark
//    else {
//      if (e>0) {
//        // energy to brightness is non-linear
//        byte eb = energymap[e>>3];
//        byte r = red_bias;
//        byte g = green_bias;
//        byte b = blue_bias;
//        increase(r, (eb*red_energy)>>8);
//        increase(g, (eb*green_energy)>>8);
//        increase(b, (eb*blue_energy)>>8);
//        leds[i] = CRGB(r, g, b);
//      }
//      else {
//        // background, no energy
//        leds[i] = CRGB(red_bg, green_bg, blue_bg);
//      }
//    }
//  }
//}
//
//void injectRandom()
//{
//  // random flame energy at bottom row
//  for (int i=0; i<ledsPerLevel; i++) {
//    currentEnergy[i] = random2(flame_min, flame_max);
//    energyMode[i] = torch_nop;
//  }
//  // random sparks at second row
//  for (int i=ledsPerLevel; i<2*ledsPerLevel; i++) {
//    if (energyMode[i]!=torch_spark && random2(100)<random_spark_probability) {
//      currentEnergy[i] = random2(spark_min, spark_max);
//      energyMode[i] = torch_spark;
//    }
//  }
//}
//
//uint16_t torch() {
//  injectRandom();
//  calcNextEnergy();
//  calcNextColors();
//  return 1;
//}

////////////////////////////////////////////////////
//Reset variables - use when switching patterns

void reset_variables() {

  peakspersec =0;
  peakcount =0;
  newtime=0;
  oldtime =0;
  sample=0;
  samplesum=0;
  sampleavg=0;
  samplecount = 0;
  height = NUM_LEDS;
  iter = 0;
  peak      = 0;
  dotCount  = 0;
  bgcol = 0;
  center = 0;
  center_2 = 0;
  step = -1;
  width=45;
  gHue = 0;

  for (int i = 0; i < NUM_LEDS; i++) 
    leds[i] = CRGB(0, 0, 0);

  for (int i = 0; i < AVGLEN; i++) {  
    insert(250, avgs, AVGLEN);
  }

  high.times = 0;
  high.times_start = millis();
  Color.r = 0;  
  Color.g = 0;
  Color.b = 1;

  for (int i = 0; i < NSAMPLES; i++) {
    samplearray[i]=0;
  }

}


