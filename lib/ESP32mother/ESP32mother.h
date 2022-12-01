#ifndef ESP32MOTHER_H_
#define ESP32MOTHER_H_
#include "Setup.h"
#include "Param.h"
#include <Arduino.h>
#include "WiFi.h"
#include "math.h"
#include <FS.h>
#include <SPIFFS.h>
#include <FreeRTOS.h>
#include "Timer.h"
#include "stprint.h"

#include "ESP_peripheral.h"
#include "PID.h"

#if (ESP_OTAE == 1)
#include <ArduinoOTA.h>
#endif


#if defined __cplusplus
extern "C" {
#endif
extern i2c_err_t I2C_Error;

extern float Vx;
extern float Vy;
extern float Angular;

class ESP32Mather
{
    private:
        //task number
        int tasksel=0;
        //system flag
        bool EMS=false;
        bool LOWV=false;
        bool wificonnect=false;
    public:
        ESP32Mather();
        int setup();
        void update();
        bool EMARGENCYSTOP();
        void BTError();
        void Error();
        void Start();
        void Start_();
};
extern ESP32Mather ESP32M;

//使用可能なクラスのオブジェクト

//I2CとSPIの関数ポインタ
extern fptr_vv_t ESPtask_i2c_init;
extern fptr_vv_t ESPtask_i2c_while;

void OTAsetup(); //ESP32M.setup()の内部で実行される

#if defined __cplusplus
}
#endif
#endif