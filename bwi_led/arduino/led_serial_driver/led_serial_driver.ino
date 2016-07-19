#include <PololuLedStrip.h>

// Sets control pin for led strip
PololuLedStrip<12> ledStrip;

typedef struct rgb_color_data{
  unsigned char r, g, b;
  uint8_t index;
} rgb_color_data;

union stream_data {
  rgb_color rgb;
  rgb_color_data rgbd;
};

// Default led count
int led_count = 60;

// Max supported led count is 180
rgb_color colors[180];

void setup() {
  Serial.begin(115200);
  Serial.println("Ready to receive colors."); 
  Serial.println("Default LED Count");
  Serial.println(led_count);
}

void improvedSerial() {
  uint8_t header = Serial.read();
  
  // Flushes colors to led strip
  if (header == 'f') {
    ledStrip.write(colors, led_count);
    
  // Sets led colors on strip
  } else if (header == 's') {
    union stream_data data;
    Serial.readBytes((char*)&data, 4);
    colors[data.rgbd.index] = data.rgb;
    
  // Clears Led Strip
  } else if (header == 'c') {
    for (int i = 0; i < led_count; i++) {
       colors[i] = (rgb_color){0,0,0};
    }
    ledStrip.write(colors, led_count);
    
  // Sets led_count value, 0-255
  } else if (header == 'l') {
    uint8_t count;
    Serial.readBytes((uint8_t*)&count, 1);
    led_count = count;
  }
}

void loop() {
  if (Serial.available()){
    improvedSerial();
  }
}
