#ifndef __hciemulator_h
#define __hciemulator_h

#include <Arduino.h>
#include <Stream.h>
#include <SoftwareSerial.h>

#define LL_OFF 0
#define LL_ERROR 1
#define LL_WARN 2
#define LL_INFO 3
#define LL_DEBUG 4

#define DEFAULTLOGLEVEL LL_WARN

#define DEVICEID 0x02
#define BROADCASTID 0x00
#define SIMULATEKEYPRESSDELAYMS 100 


// Modbus states that a baud rate higher than 19200 must use a fixed 750 us 
// for inter character time out and 1.75 ms for a frame delay.
// For baud rates below 19200 the timeing is more critical and has to be calculated.
// E.g. 9600 baud in a 10 bit packet is 960 characters per second
// In milliseconds this will be 960characters per 1000ms. So for 1 character
// 1000ms/960characters is 1.04167ms per character and finaly modbus states an
// intercharacter must be 1.5T or 1.5 times longer than a normal character and thus
// 1.5T = 1.04167ms * 1.5 = 1.5625ms. A frame delay is 3.5T.  
#define T1_5 750
#define T3_5  4800 //1750  


enum DoorState : uint8_t {    
    DOOR_OPEN_POSITION =  0x20,    
    DOOR_CLOSE_POSITION = 0x40,
    DOOR_HALF_POSITION =  0x80,
    DOOR_MOVE_CLOSEPOSITION = 0x02,
    DOOR_MOVE_OPENPOSITION = 0x01,
    
};

struct SHCIState{
    bool valid;
    bool lampOn;
    uint8_t doorState; // see DoorState
    uint8_t doorCurrentPosition;
    uint8_t doorTargetPosition;
    uint8_t reserved;
};

enum StateMachine: uint8_t{
    WAITING,

    STARTOPENDOOR,
    STARTOPENDOOR_RELEASE,

    STARTOPENDOORHALF,
    STARTOPENDOORHALF_RELEASE,

    STARTCLOSEDOOR,
    STARTCLOSEDOOR_RELEASE,

    STARTSTOPDOOR,
    STARTSTOPDOOR_RELEASE,

    STARTTOGGLELAMP,
    STARTTOGGLELAMP_RELEASE,

    STARTVENTPOSITION,
    STARTVENTPOSITION_RELEASE
};

class HCIEmulator {
public:
    typedef std::function<void(const SHCIState&)> callback_function_t;

    HCIEmulator(Stream * port);

    void poll();

    void openDoor();
    void openDoorHalf();
    void closeDoor();
    void stopDoor();
    void toggleLamp();
    void turnOnLamp();
    void turnOffLamp();
    void ventilationPosition();

    const SHCIState& getState() {   
        if(micros()-m_recvTime > 2000000){
            // 2 sec without statusmessage 
            m_state.valid = false;
        }
        return m_state;
    };
    
    unsigned long getMessageAge(){
        return micros()-m_recvTime;
    }

    int getLogLevel();
    void setLogLevel(int level);

    void onStatusChanged(callback_function_t handler);

protected:
    void processFrame();
    void processDeviceStatusFrame();
    void processDeviceBusScanFrame();
    void processBroadcastStatusFrame();

private:
    callback_function_t m_statusCallback;    
    Stream *m_port;
    SHCIState m_state;
    StateMachine m_statemachine;

    unsigned long m_recvTime;
    unsigned long m_lastStateTime;
    unsigned long m_lastSendTime;

    size_t m_rxlen;
    size_t m_txlen;

    unsigned char m_rxbuffer[255];
    unsigned char m_txbuffer[255];

    bool m_skipFrame;
};


#endif