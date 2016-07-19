#pragma once

#include <time.h>
#include <unistd.h>
#include <string.h>
#include "ros/ros.h"
#include "serial/serial.h"
#include "rgbhsv.h"

class LedCOM{
  private:
    static const uint8_t CLEAR_MSG[1];
    static const uint8_t FLUSH_MSG[1];
    serial::Serial* serial_conn;
    void write(const uint8_t[], uint32_t);

  public:
    void connect(const std::string, const unsigned long);
    void setRGB(const uint8_t, const uint8_t, const uint8_t, const uint8_t);
    void setHSV(const uint8_t, float, float, float);
    void clear();
    void flush();
    void setLEDCount(const uint8_t);
};

const uint8_t LedCOM::CLEAR_MSG[] = {'c'};
const uint8_t LedCOM::FLUSH_MSG[] = {'f'};

void LedCOM::write(const uint8_t data[], const uint32_t data_size) {
  serial_conn->write(data, data_size);
  serial_conn->flushOutput();
}

void LedCOM::connect(const std::string port, const unsigned long baud) {

  try{
    serial_conn = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
  }
  catch(const serial::IOException &e)
  {
      cerr << "EXCEPTION CAUGHT: serial::Serial could not open a connection." << endl;
      cerr << endl << "Original exception: " << e.what() << endl;
      cerr << endl << "Ensure device is connected and using port, " << port << ", with baud setting, " << baud << "." << endl;
      cerr << endl << "Retrying to open connection after waiting 30 seconds." << endl;
      sleep(2);
      LedCOM::connect(port, baud);
  }
}

void LedCOM::setRGB(const uint8_t index, const uint8_t r, const uint8_t g, const uint8_t b){
  uint8_t data[] = {'s',r,g,b,index};
  write(data, 5);
}

void LedCOM::setHSV(const uint8_t index, float h, float s, float v) {
  float rf, gf, bf;

  HSVtoRGB(rf,gf,bf,h,s,v);

  uint8_t r = 255 * rf;
  uint8_t g = 255 * gf;
  uint8_t b = 255 * bf;

  uint8_t data[] = {'s',r,g,b,index};
  write(data, 5);
}

void LedCOM::clear() { write(CLEAR_MSG, 1); }

void LedCOM::flush() { write(FLUSH_MSG, 1); }

void LedCOM::setLEDCount(const uint8_t led_count) {
  uint8_t data[] = {'l',led_count};
  write(data, 2);
}