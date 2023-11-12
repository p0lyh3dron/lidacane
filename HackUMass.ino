#include <TFLI2C.h>
#include <Wire.h>

#include <BluetoothA2DPSink.h>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

BluetoothA2DPSink _sink;

void read_data_stream(const uint8_t *data, uint32_t length) {
    // process all data
    int16_t *values = (int16_t*) data;
    for (int j=0; j<length/2; j+=2){
      // print the 2 channel values
      Serial.print(values[j]);
      Serial.print(",");
      Serial.println(values[j+1]);
    }
}
                                                                  
void setup() {
  // initialize serial communication:
  Serial.begin(115200);
  Wire.begin();
  pinMode(2, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(5, INPUT_PULLUP);

  i2s_pin_config_t my_pin_config = {
      .mck_io_num = I2S_PIN_NO_CHANGE,
      .bck_io_num = I2S_PIN_NO_CHANGE,
      .ws_io_num = I2S_PIN_NO_CHANGE,
      .data_out_num = 25,
      .data_in_num = I2S_PIN_NO_CHANGE
  };

        
  const i2s_config_t i2s_config = {
      .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
      .sample_rate = 44100, // corrected by info from bluetooth
      .bits_per_sample = (i2s_bits_per_sample_t) 16, /* the DAC module will only take the 8bits from MSB */
      .channel_format =  I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false
  };
  
  /* Enable for BT.  */

  /* _sink.set_pin_config(my_pin_config);
     _sink.set_i2s_config(i2s_config);
     _sink.set_stream_reader(read_data_stream);
     _sink.start("Map Cane Assist");
     _sink.set_volume(255);
  */
}

int pulse_min_cm = 200;
int pulse_max_cm = 25;

float T_f = 1.0;
float T_c = 0.2;
float timer = 0.0;
int prev = 0;
int cur = 0;
float pulse_len = 0.0;
float hold_len  = 0.0;
int all_enabled = 1;
int spk_enabled = 1;
int holding = 0;

TFLI2C _lidar;

void haptic_pulse(int strength) {
  pulse_len = 0.1;

  digitalWrite(2, HIGH);
  analogWrite(33, strength);
  
  if (spk_enabled) {
    tone(25, 1000);
  }
}

void haptic_unpulse() {
  noTone(25);
  analogWrite(33, 0);
  digitalWrite(2, LOW);
}

void loop() {
  cur = millis();
  int16_t distance, flux, temp;
  _lidar.getData(distance, flux, temp, TFL_DEF_ADR);

  if (distance == 0) {
    distance = 900;
  }

  int pot = analogRead(26) + 25;

  int upper = pulse_max_cm * 4 * pot / 4000;
  int lower = pulse_min_cm * 4 * pot / 4000;

  Serial.println(pot);

  distance = MIN(lower, distance);
  distance = MAX(upper, distance);

  float factor = (float)(lower - upper - (distance - upper)) / (lower - upper);

  float T = T_f + (T_c - T_f) * factor;

  timer     += 1/T * (cur - prev) / 1000.0;
  pulse_len -= (cur - prev) / 1000.0;

  if (timer > 2*T) {
    if (all_enabled) {
      haptic_pulse(255);
    }

    timer -= 2*T;
  }

  if (pulse_len <= 0) {
    if (all_enabled) {
      haptic_unpulse();
    }
  }

  if (digitalRead(5) == 0) {
    if (holding == false) {
      holding = true;
      spk_enabled = !spk_enabled;
    }
    hold_len += (cur - prev) / 1000.0;
  } else {
    holding = false;
    hold_len = 0.0;
  }

  if (hold_len >= 1.0) {
    hold_len = 0.0;

    all_enabled = !all_enabled;
  }

  prev = cur;

  delay(30);
}