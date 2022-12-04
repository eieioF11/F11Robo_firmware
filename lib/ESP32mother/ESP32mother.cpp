#include "ESP32mother.h"
#include "Setup.h"
#include "Mbed.h"

ESP32Mather ESP32M;

bool OTAFLAG=false;

float Vx=0.0f;
float Vy=0.0f;
float Angular=0.0f;

int mode,mdc=0;
int sel=0,melodysel=0;
uint16_t stledinterval=500;

String sdprint="/";

/*beep&melody*/

void melodytask(void *arg)
{
    int stc=0,ledsec=0;
    portTickType lt = xTaskGetTickCount();
    while(1)
    {
        if(stc>100)
        {
            switch(melodysel)
            {
                case 1:mel1();break;
                case 2:mel2();break;
                case 3:mel3();break;
                case Onbeep:if(ONbeep())melodysel=0;break;
                case ConectBeep:if(conectbeep(StatusLED))melodysel=0;break;
                case UnmountBeep:if(unmountbeep(StatusLED))melodysel=0;break;
                case ErrorBeep:if(errorbeep())melodysel=0;break;
                case StartBeep:if(startbeep())melodysel=0;break;
                case XPStartBeep:if(winxpstartbeep())melodysel=0;break;
                case BTErrorBeep:if(BTerrorbeep())melodysel=0;break;
                case Onbeep-1:
                default:melodysel=0;break;
            }
        }
        else stc++;
        if(melodysel!=ConectBeep&&melodysel!=UnmountBeep&&!OTAFLAG)
        {
            if(ledsec>(stledinterval/2))
                digitalWrite(StatusLED, HIGH);
            else
                digitalWrite(StatusLED, LOW);
            if(ledsec>stledinterval)ledsec=0;
            else ledsec++;
        }
        vTaskDelayUntil(&lt,5/portTICK_RATE_MS);
    }
}

/*I2CTask*/
#if(I2CPORT == ON)
fptr_vv_t ESPtask_i2c_init = NULL;
fptr_vv_t ESPtask_i2c_while = [](){};

void ESPtask_i2c(void *arg)
{
    sensor_ini=true;
    pinMode(ESP_SDA,PULLUP);
    pinMode(ESP_SCL,PULLUP);
    Wire.begin(ESP_SDA,ESP_SCL,400000);
    if(ESPtask_i2c_init != NULL)
        ESPtask_i2c_init();
    portTickType lt = xTaskGetTickCount();
    while(1)
    {
        sensor_ini=false;
        vTaskDelayUntil(&lt, (sensor_interval*100)/portTICK_RATE_MS);
        if(!OTAFLAG)
            ESPtask_i2c_while();
    }
}
#endif

/*OTA*/
int otac=0;
String otastatus;
void OTAsetup()
{
    #if (ESP_OTAE == 1)
    ArduinoOTA
    .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";
        otastatus="Start updating"+ type;
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println(otastatus);
        wheel->Stop();
        digitalWrite(StatusLED, HIGH);
        digitalWrite(STBY, HIGH);
        OTAFLAG=true;
        ESP32M.Start();
    })
    .onEnd([]() {
        digitalWrite(StatusLED, LOW);
        digitalWrite(STBY, LOW);
        Serial.println(otastatus);
        OTAFLAG=false;
    })
    .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        if(otac>10)
        {
            digitalWrite(StatusLED, HIGH);
            digitalWrite(STBY, LOW);
        }
        else
        {
            digitalWrite(StatusLED, LOW);
            digitalWrite(STBY, HIGH);
        }
        otac++;
        if(otac>20)otac=0;
    })
    .onError([](ota_error_t error) {
        digitalWrite(StatusLED, HIGH);
        OTAFLAG=false;
        char buff[255];
        sprintf(buff,"Error[%u]: ", error);
        otastatus+=String(buff)+":";
        if (error == OTA_AUTH_ERROR) otastatus+="Auth Failed";
        else if (error == OTA_BEGIN_ERROR) otastatus+="Begin Failed";
        else if (error == OTA_CONNECT_ERROR) otastatus+="Connect Failed";
        else if (error == OTA_RECEIVE_ERROR) otastatus+="Receive Failed";
        else if (error == OTA_END_ERROR) otastatus+="End Failed";
        Serial.println(otastatus);
    });
    ArduinoOTA.begin();
    #endif
}

/*Other*/

ESP32Mather::ESP32Mather()
{
}
int ESP32Mather::setup()
{
    /*Digitalpin setup*/
    pinMode(STBY, OUTPUT);
    pinMode(SD_CHECK, INPUT);
    pinMode(SD_MOUNT, INPUT);
    pinMode(stsw, INPUT_PULLUP);
    pinMode(StatusLED, OUTPUT);
    digitalWrite(StatusLED, HIGH);
    digitalWrite(STBY, LOW);
    Vmonitor_setup();
    tone(beep,310);
    /*Motor Initialize*/
    motorsetup();
    /*OTA Initialize*/
    #if (ESP_OTAE == 1)
    OTAsetup();
    otastatus=" ";
    #endif
    /*Encoder Initialize*/
    #if (MOTORMODE ==PID_M)
    ((ESPMotor*)md[0])->Reset();
    ((ESPMotor*)md[1])->Reset();
    xTaskCreatePinnedToCore(enctask,"Encoder task",1024,NULL,2,NULL,0);
    #endif
    /*I2CTask Initialize*/
    #if(I2CPORT == ON)
    xTaskCreatePinnedToCore(ESPtask_i2c,"I2C task",8096,NULL,3,NULL,0);
    #endif
    /*Odmetry update Task Initialize*/
    #if(I2CPORT == ON && MOTORMODE != DUTY_M)
    xTaskCreatePinnedToCore(Odmetryupdate,"Odmetry task",8096,NULL,0,NULL,1);
    #endif
    /*melody*/
    xTaskCreatePinnedToCore(melodytask,"Melody task",1024,NULL,0,NULL,1);

    tone(beep,246);
    delay(100);
    noTone(beep);
    digitalWrite(StatusLED,LOW);
    return 0;
}
void ESP32Mather::update()
{
    /*Variable declaration*/
    static bool mflag=false;
    bool sel=false;
    EMS=Emergency_Stop();
    if((LOWV=LowV())==true&&SDCH()==0)
    {
        if(!mflag)
            melodysel=UnmountBeep;
        mflag=true;
    }
    /*OTA*/
    #if (ESP_OTAE == 1)
    ArduinoOTA.handle();
    #endif
    if(OTAFLAG)
        return;

    if(EMS)
    {
        digitalWrite(STBY, LOW);
    }

    //delay(Delta_T*100);//Execution interval
}

void ESP32Mather::BTError()
{
    melodysel=BTErrorBeep;
}

void ESP32Mather::Error()
{
    melodysel=ErrorBeep;
}
void ESP32Mather::Start()
{
    melodysel=StartBeep;
}
void ESP32Mather::Start_()
{
    melodysel=XPStartBeep;
}

bool ESP32Mather::EMARGENCYSTOP()
{
    return EMS;
}

/*
FreeRTOS memo
core 0 Background task execution
core 1 Main task execution
High priority  Large value
Low priority   Small value
setup and loop task priority 1
*/