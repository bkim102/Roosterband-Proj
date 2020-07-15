/*****************************************************************************
  BH1792GLC.ino

 Copyright (c) 2017 ROHM Co.,Ltd.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
******************************************************************************/
#include <avr/sleep.h>
#include <Wire.h>
#include <FlexiTimer2.h>

extern "C" {
#include <bh1792.h>
#include <movingAverage.h>
#include <pwCalc.h>
}


#include <SPI.h>
#include <ADXL362.h>

bool vibrating;
bool touching;
ADXL362 xl;

int beatTime = 1;
int32_t idleCount;

int16_t temp;
int16_t XValue, YValue, ZValue, Temperature, Xprev, Yprev, Zprev, Xdiff, Ydiff, Zdiff;

bh1792_t      m_bh1792;
bh1792_data_t m_bh1792_dat;

float32_t hr_prev1;
float32_t hr_prev2;
int8_t bpm;
int8_t bpmAvg;
int32_t avgCount; 
 
int32_t hr_count; 
int32_t hr_countDiff2;
int32_t hr_countDet;

float32_t pul_wave;

static MAParamU8  s_maPrm_bpm;

uint8_t hr_calc(uint16_t  on_data);
void timer_isr(void);
void bh1792_isr(void);
int32_t i2c_write(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size);
int32_t i2c_read(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size);
void error_check(int32_t ret, String msg);

void setup() {

  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  vibrating  = false;
  touching = false;
  xl.begin(10);                   // Setup SPI protocol, issue device soft reset
  xl.beginMeasure();              // Switch ADXL362 to measure mode  
  Xprev = 0;
  Yprev = 0;
  Zprev = 0;
  idleCount = 0;
  
  int32_t ret = 0;
  bpmAvg = 0;
  avgCount  = 0;

  // Sleep Mode
  set_sleep_mode(SLEEP_MODE_IDLE);

  // Serial Port
  Serial.begin(115200);
  while (!Serial);

  // I2C
  Wire.begin();
  Wire.setClock(400000L);

  // BH1792
  m_bh1792.fnWrite      = i2c_write;
  m_bh1792.fnRead       = i2c_read;
  ret = bh1792_Init(&m_bh1792);
  error_check(ret, "bh1792_Init");

  m_bh1792.prm.sel_adc  = BH1792_PRM_SEL_ADC_GREEN;
  m_bh1792.prm.msr      = BH1792_PRM_MSR_SINGLE;//BH1792_PRM_MSR_1024HZ;
  m_bh1792.prm.led_en   = (BH1792_PRM_LED_EN1_0 << 1) | BH1792_PRM_LED_EN2_0;
  m_bh1792.prm.led_cur1 = BH1792_PRM_LED_CUR1_MA(1);
  m_bh1792.prm.led_cur2 = BH1792_PRM_LED_CUR2_MA(0);
  m_bh1792.prm.ir_th    = 0xFFFC;
  m_bh1792.prm.int_sel  = BH1792_PRM_INT_SEL_SGL;//BH1792_PRM_INT_SEL_WTM;
  ret = bh1792_SetParams();
  error_check(ret, "bh1792_SetParams");


  hr_prev1 = 0;
  hr_prev2 = 0;
  bpm = 0;
  hr_count = 0; 
  hr_countDiff2 = 0;
  hr_countDet = 0; 
  ma_InitU8(4U, &s_maPrm_bpm);
  pwCalc_Init();

  Serial.println(F("GDATA(@LED_ON),GDATA(@LED_OFF)"));

  ret = bh1792_StartMeasure();
  error_check(ret, "bh1792_StartMeasure");

  attachInterrupt(0, bh1792_isr, LOW);

  FlexiTimer2::stop();
  if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
    FlexiTimer2::set(2000, 5.0/10000, timer_isr);    // 1Hz timer
  } else {
    FlexiTimer2::set(250, 1.0/8000, timer_isr);      // 32Hz timer
  }
  FlexiTimer2::start();
}

void loop() 
{
  sleep_mode();
}

void timer_isr(void) {
  int32_t ret = 0;
  uint8_t tmp_eimsk;

  tmp_eimsk = EIMSK;
  EIMSK = 0;
  interrupts();

  if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
    ret = bh1792_SetSync();
    error_check(ret, "bh1792_SetSync");

    if (m_bh1792.sync_seq < 3) {
      if (m_bh1792.sync_seq == 1) {
        tmp_eimsk = 0;
      } else {
        ret = bh1792_ClearFifoData();
        error_check(ret, "bh1792_ClearFifoData");

        tmp_eimsk = bit(INT0);
      }
    }
  } else {
    ret = bh1792_StartMeasure();
    error_check(ret, "bh1792_StartMeasure");
  }

  xl.readXYZTData(XValue, YValue, ZValue, Temperature);      
  Xdiff = Xprev - XValue;
  Ydiff = Yprev - YValue;
  Zdiff = Zprev - ZValue;
  
  Xprev = XValue;
  Yprev = YValue;
  Zprev = ZValue;

  if( abs(Xdiff) < (int16_t)150 && abs(Ydiff) < (int16_t)150 && abs(Zdiff) < (int16_t)150 )
  idleCount++ ;
  else
  idleCount = 0;

  
  sleepCheck();
  

  
  noInterrupts();
  EIMSK |= tmp_eimsk;
}

void bh1792_isr(void) {
  int32_t ret = 0;
  uint8_t i   = 0;

  EIMSK = 0;
  interrupts();

  ret = bh1792_GetMeasData(&m_bh1792_dat);
  error_check(ret, "bh1792_GetMeasData");

  if(m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) 
  {
    for (i = 0; i < m_bh1792_dat.fifo_lev; i++) 
    {
      hr_calc(m_bh1792_dat.fifo[i].on);
      Serial.print(bpm, DEC);
      Serial.print(F(","));
      Serial.println(m_bh1792_dat.fifo[i].off, DEC);
    }
  } 
  
  else 
  {
    if(m_bh1792.prm.sel_adc == BH1792_PRM_SEL_ADC_GREEN) 
    {

      
      if(m_bh1792_dat.green.off < 1000.0f)
      {
        touching = true;
        pwCalc( &m_bh1792_dat.green , &pul_wave); 
        hr_calc(pul_wave);
      }
      else
      {
        touching = false;
        hr_reset();
      }

      if(bpm > 0)
      {
        if(avgCount < 32*60*beatTime)
        {
          bpmAvg =  (bpmAvg * avgCount + bpm)/(avgCount + 1);
          avgCount ++; 
        }
        else
         {
          bpmAvg =  (bpmAvg + bpm)/2;
          avgCount = 1;
         }
      }
      
      Serial.println(bpm, DEC);
    } 
    else 
    {
      hr_calc(m_bh1792_dat.ir.on);
      Serial.print(bpm, DEC);
      Serial.print(F(","));
      Serial.println(m_bh1792_dat.ir.off, DEC);
    }
  }

  noInterrupts();
  EIMSK = bit(INT0);
}

// Note:  I2C access should be completed within 0.5ms
int32_t i2c_write(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size)
{
  byte rc;

  if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
    if((slv_adr != BH1792_SLAVE_ADDR) || (reg_adr != BH1792_ADDR_MEAS_SYNC)) {
      while(FlexiTimer2::count == 1999);
    }
  }

  Wire.beginTransmission(slv_adr);
  Wire.write(reg_adr);
  Wire.write(reg, reg_size);
  rc = Wire.endTransmission(true);

  return rc;
}

// Note:  I2C access should be completed within 0.5ms
int32_t i2c_read(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size)
{
  byte    rc;
  uint8_t cnt;

  if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
    while(FlexiTimer2::count == 1999);
  }

  Wire.beginTransmission(slv_adr);
  Wire.write(reg_adr);
  rc = Wire.endTransmission(false);
  if (rc == 0) {
    Wire.requestFrom((int32_t)slv_adr, (int32_t)reg_size, true);
    cnt = 0;
    while(Wire.available()) {
      reg[cnt] = Wire.read();
      cnt++;
    }
    if(cnt < reg_size) {
      rc = 4;
    }
  }

  return rc;
}

void error_check(int32_t ret, String msg)
{
  if(ret < 0) {
    msg = "Error: " + msg;
    msg += " function";
    Serial.println(msg);
    Serial.print("ret = ");
    Serial.println(ret, DEC);
    if(ret == BH1792_I2C_ERR) {
      Serial.print("i2c_ret = ");
      Serial.println(m_bh1792.i2c_err, DEC);
    }
    while(1);
  }
}

uint8_t hr_calc(uint16_t data)
{

    float32_t bpm_candidate = 0.0F;
    float32_t diff1         = 0.0F;
    float32_t diff2         = 0.0F;
    int32_t   tmp           = 0;
    
    hr_count++;
    
    if (hr_count> 1) {
        diff1 = data - hr_prev1;
    }
    else {
        diff1 = 0.0F;
    }
   hr_prev1 = data;
    
    if (hr_count > 2) {
        diff2 = diff1 - hr_prev2;
    }
    else {
        diff2 = 0.0F;
    }
    hr_prev2 = diff1;
    
    tmp = hr_count - hr_countDet;
    if (tmp > 0) {
        bpm_candidate = 60.0F / (0.03125F * (float32_t)tmp);
    }
    else {
        bpm_candidate = 0.0F;
    }
    
    if (bpm_candidate < 40.0f) {
        hr_countDet = 0;
    }
    
    if ((diff1 < 0.0F) && (diff2 >= 0.0F) && (hr_countDiff2 >= 3)) {
        if(hr_countDet != 0) {
            if (bpm_candidate <= 240.0f) {
                bpm_candidate += 0.5F;
                bpm = ma_AverageU8((uint8_t)bpm_candidate, &s_maPrm_bpm);
            }
        }
        hr_countDet = hr_count;
    }
    
    if (diff2 < 0.0F) {
        hr_countDiff2 = hr_countDiff2 + 1;
    }
    else {
        hr_countDiff2 = 0;
    }
  
}

void hr_reset()
{
  hr_prev1 = 0;
  hr_prev2 = 0;
  bpm = 0;
  hr_count = 0; 
  hr_countDiff2 = 0;
  hr_countDet = 0; 
  ma_ClearU8(&s_maPrm_bpm);
}


void sleepCheck()
{
  if ( idleCount  > (int32_t)(32*60*1) && (bpmAvg - bpm) > (int8_t)17.81  )  //bpmAvg - bpm) > 17.81
  {
      vibrating = true;
      digitalWrite(3, HIGH);
  }
  else if ( idleCount  > (int32_t)(32*60*3) && (bpmAvg - bpm) > (int8_t)12  )  //bpmAvg - bpm) > 17.81
  {
      vibrating = true;
      digitalWrite(3, HIGH);
  }
  else if (idleCount  > (int32_t)(32*60*15)   )  //bpmAvg - bpm) > 17.81
  {
      vibrating = true;
      digitalWrite(3, HIGH);
  }

  else
  {
  if(!vibrating)
  digitalWrite(3, LOW);
  }
  
}
