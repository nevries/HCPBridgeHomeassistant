static const char* TAG = "hciemulator";
#include "hciemulator.h"
#define CHECKCHANGEDSET(Target,Value,Flag) if((Target)!=(Value)){Target=Value;Flag=true;}
int hciloglevel = DEFAULTLOGLEVEL;

#define Log(Level,Message) LogCore(Level,Message)
#define Log3(Level,Message,Buffer, Len) LogCore(Level,Message,Buffer,Len)
//LOGLEVEL
void LogCore(int Level, const char* msg, const unsigned char * data=NULL, size_t datalen=0){
    if(Level>hciloglevel){
        return;
    }
    if(data!=NULL && datalen>0){ 
        String newmsg(msg);
        char str[4];
        for (size_t i = 0; i < datalen; i++){
            snprintf(str,sizeof(str),"%02x ", data[i]);
            newmsg+=str;
        }
        msg = newmsg.c_str();        
    }
    switch (Level)
    {
    case LL_ERROR:
        ESP_LOGE(TAG, "%s", msg);
        break;
    
    case LL_WARN:
        ESP_LOGW(TAG, "%s", msg);
        break;
    
    case LL_INFO:
        ESP_LOGI(TAG, "%s", msg);
        break;
    
    case LL_DEBUG:
        ESP_LOGD(TAG, "%s", msg);
        break;

    default:
        break;
    }
}


int HCIEmulator::getLogLevel(){
    return hciloglevel;
    
}
void HCIEmulator::setLogLevel(int level){
    hciloglevel=level;
}

//modbus crc calculation borrowed from:
//https://github.com/yaacov/ArduinoModbusSlave
#define MODBUS_CRC_LENGTH 2
#define readCRC(arr, length) word(arr[(length - MODBUS_CRC_LENGTH) + 1], arr[length - MODBUS_CRC_LENGTH])
#define readUInt16(arr, index) word(arr[index], arr[index + 1])
/**
 * Calculate the CRC of the passed byte array from zero up to the passed length.
 *
 * @param buffer The byte array containing the data.
 * @param length The length of the byte array.
 *
 * @return The calculated CRC as an unsigned 16 bit integer.
 * 
 * Calculate and add the CRC.
 *       uint16_t crc = Modbus::calculateCRC(_responseBuffer, _responseBufferLength - MODBUS_CRC_LENGTH);
 *       _responseBuffer[_responseBufferLength - MODBUS_CRC_LENGTH] = crc & 0xFF;
 *       _responseBuffer[(_responseBufferLength - MODBUS_CRC_LENGTH) + 1] = crc >> 8;
 * 
 * 
 * #define MODBUS_FRAME_SIZE 4
 * #define MODBUS_CRC_LENGTH 2
 * uint16_t crc = readCRC(_requestBuffer, _requestBufferLength);
 * #define readUInt16(arr, index) word(arr[index], arr[index + 1])
 * #define readCRC(arr, length) word(arr[(length - MODBUS_CRC_LENGTH) + 1], arr[length - MODBUS_CRC_LENGTH])
 */
uint16_t calculateCRC(uint8_t *buffer, int length)
{
    int i, j;
    uint16_t crc = 0xFFFF;
    uint16_t tmp;

    // Calculate the CRC.
    for (i = 0; i < length; i++)
    {
        crc = crc ^ buffer[i];
        for (j = 0; j < 8; j++)
        {
            tmp = crc & 0x0001;
            crc = crc >> 1;
            if (tmp)
            {
                crc = crc ^ 0xA001;
            }
        }
    }
    return crc;
}

HCIEmulator::HCIEmulator(Stream * port) { 
    m_state.valid = false; 
    m_statemachine=WAITING;
    m_rxlen = m_txlen = 0;
    m_recvTime=m_lastStateTime=0;
    m_skipFrame=false;
    m_port = port;
    m_statusCallback = NULL;
    setLogLevel(DEFAULTLOGLEVEL);
};

void HCIEmulator::poll(){
    
    if(m_port==NULL) return;

    // receive Data
    if(m_port->available() >0)
    {
        m_rxlen+= m_port->readBytes((char*)(m_rxbuffer+m_rxlen), _min((int)(255-m_rxlen),m_port->available()));
        if(m_rxlen > 254)
        {
            Log(LL_ERROR,"RX Bufferoverflow, skip next Frame");   
            Log3(LL_DEBUG,"Buffer Data: ", m_rxbuffer, m_rxlen);
            m_rxlen=0;               
            m_skipFrame = true;
        }
        m_recvTime = micros();             
    }        
    

    // check frame, process frame
    if(m_rxlen>0 && (micros()-m_recvTime > T3_5)) 
    {
        // check last action timeout -> reset > then 2sec
        if(m_statemachine!= WAITING && m_lastStateTime+2000<millis()){
            m_statemachine = WAITING;
        }

        if(!m_skipFrame){

            processFrame();

            // send response
            if(m_txlen > 0){
                
                // fix crc
                uint16_t crc = calculateCRC(m_txbuffer, m_txlen - MODBUS_CRC_LENGTH);
                m_txbuffer[m_txlen - MODBUS_CRC_LENGTH] = crc & 0xFF;
                m_txbuffer[(m_txlen - MODBUS_CRC_LENGTH) + 1] = crc >> 8;                

                // send data
                m_lastSendTime = micros()-m_recvTime;

                //Log(LL_DEBUG, ("ST:"+String(m_lastSendTime)).c_str());
                
                m_port->write(m_txbuffer, m_txlen);
                Log3(LL_DEBUG,"Response: ", m_txbuffer, m_txlen);
                m_txlen = 0;
            }                
        }

        m_skipFrame = false;
        m_rxlen=0;            
    }    
}

void HCIEmulator::processFrame(){
    m_txlen = 0;    // clear send buffer

    if(m_rxlen<5) {
        Log(LL_ERROR,"Frame skipped, invalid frame len");      
        Log3(LL_ERROR,"Data:", m_rxbuffer,m_rxlen);
        return;
    }

    // check device id, pass only device id 2 and 0 (broadcast)
    if(m_rxbuffer[0] != BROADCASTID && m_rxbuffer[0] != DEVICEID){      
        Log(LL_DEBUG,"Frame skipped, unsupported device id");      
        Log3(LL_DEBUG,"Data:", m_rxbuffer,m_rxlen);
        return;
    }

    // check crc
    uint16_t crc = readCRC(m_rxbuffer, m_rxlen);        
    if(crc != calculateCRC(m_rxbuffer,m_rxlen-MODBUS_CRC_LENGTH)){
        Log3(LL_ERROR,"Frame skipped, wrong crc", m_rxbuffer,m_rxlen);
        return;
    }

    Log3(LL_DEBUG,"Incomming Data: ", m_rxbuffer, m_rxlen);

    // dispatch modbus function
    switch(m_rxbuffer[1]){ 
        case 0x10:{  //  Write Multiple registers                            
            if(m_rxlen == 0x1b && m_rxbuffer[0] == BROADCASTID)
            {
                processBroadcastStatusFrame();
                return;
            }  
            break;
        }
        
        case 0x17:{ // Read/Write Multiple registers
            if(m_rxbuffer[0] == DEVICEID){
                switch(m_rxlen){
                    case 0x11:{
                        processDeviceStatusFrame();
                        return;
                    }

                    case 0x13:
                        processDeviceBusScanFrame();
                        return;;
                }
            }
            break;                               
        }
    }
    Log3(LL_ERROR,"Frame skipped, unexpected data: ", m_rxbuffer, m_rxlen);
}

const unsigned char ResponseTemplate_Fcn17_Cmd03_L08 []= {0x02,0x17,0x10,0x3E,0x00,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x74,0x1B};
const unsigned char ResponseTemplate_Fcn17_Cmd04_L02 []= {0x02,0x17,0x04,0x0F,0x00,0x04,0xFD,0x0A,0x72};
void HCIEmulator::processDeviceStatusFrame(){
    if(m_rxlen==0x11){  
        unsigned char counter = m_rxbuffer[11];
        unsigned char cmd = m_rxbuffer[12];
        if(m_rxbuffer[5] == 0x08){
                // expose internal state
            //      00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16
            //0011: 02 17 9C B9 00 08 9C 41 00 02 04 3E 03 00 00 EB CC
            //res=> 02 17 10 3E 00 03 01 00 00 00 00 00 00 00 00 00 00 00 00 74 1B                 
            memcpy(m_txbuffer, ResponseTemplate_Fcn17_Cmd03_L08, sizeof(ResponseTemplate_Fcn17_Cmd03_L08));  
            m_txbuffer[0] = m_rxbuffer[0];
            m_txbuffer[3] = counter;
            m_txbuffer[5] = cmd;
            m_txlen = sizeof(ResponseTemplate_Fcn17_Cmd03_L08);   

            
            switch(m_statemachine)
            {
                // open Door
                case STARTOPENDOOR:
                    m_txbuffer[7]= 0x02;
                    m_txbuffer[8]= 0x10;
                    m_statemachine = STARTOPENDOOR_RELEASE;
                    m_lastStateTime = millis();
                    break;
                case STARTOPENDOOR_RELEASE:
                    if(m_lastStateTime+SIMULATEKEYPRESSDELAYMS<millis()){
                        m_txbuffer[7]= 0x01;
                        m_txbuffer[8]= 0x10;
                        m_statemachine = WAITING; 
                    }
                    break;

                // close Door
                case STARTCLOSEDOOR:
                    m_txbuffer[7]= 0x02;
                    m_txbuffer[8]= 0x20;
                    m_statemachine = STARTCLOSEDOOR_RELEASE;
                    m_lastStateTime = millis();
                    break;
                case STARTCLOSEDOOR_RELEASE:
                    if(m_lastStateTime+SIMULATEKEYPRESSDELAYMS<millis()){
                        m_txbuffer[7]= 0x01;
                        m_txbuffer[8]= 0x20;
                        m_statemachine = WAITING; 
                    }
                    break;   

                // stop Door
                case STARTSTOPDOOR:
                    m_txbuffer[7]= 0x02;
                    m_txbuffer[8]= 0x40;
                    m_statemachine = STARTSTOPDOOR_RELEASE;
                    m_lastStateTime = millis();
                    break;
                case STARTSTOPDOOR_RELEASE:
                    if(m_lastStateTime+SIMULATEKEYPRESSDELAYMS<millis()){
                        m_txbuffer[7]= 0x01;
                        m_txbuffer[8]= 0x40;
                        m_statemachine = WAITING; 
                    }
                    break;

                // Ventilation
                case STARTVENTPOSITION:
                    m_txbuffer[7]= 0x02;
                    m_txbuffer[9]= 0x40;
                    m_statemachine = STARTVENTPOSITION_RELEASE;
                    m_lastStateTime = millis();
                    break;
                case STARTVENTPOSITION_RELEASE:
                    if(m_lastStateTime+SIMULATEKEYPRESSDELAYMS<millis()){
                        m_txbuffer[7]= 0x01;
                        m_txbuffer[9]= 0x40;
                        m_statemachine = WAITING; 
                    }
                    break;        


                // Half Position
                case STARTOPENDOORHALF:
                    m_txbuffer[7]= 0x02;
                    m_txbuffer[9]= 0x04;
                    m_statemachine = STARTOPENDOORHALF_RELEASE;
                    m_lastStateTime = millis();
                    break;

                case STARTOPENDOORHALF_RELEASE:
                    if(m_lastStateTime+SIMULATEKEYPRESSDELAYMS<millis()){
                        m_txbuffer[7]= 0x01;
                        m_txbuffer[9]= 0x04;
                        m_statemachine = WAITING; 
                    }
                    break;                                          

                // Toggle Lamp
                case STARTTOGGLELAMP:
                    m_txbuffer[7]= 0x10;
                    m_txbuffer[9]= 0x02;
                    m_statemachine = STARTTOGGLELAMP_RELEASE;
                    m_lastStateTime = millis();
                    break;
                case STARTTOGGLELAMP_RELEASE:
                    if(m_lastStateTime+SIMULATEKEYPRESSDELAYMS<millis()){
                        m_txbuffer[7]= 0x08;
                        m_txbuffer[9]= 0x02;
                        m_statemachine = WAITING; 
                    }
                    break;  
                    
                case WAITING:
                    break;            
            }                                 
            return;             
        }
        else if(m_rxbuffer[5] == 0x02){
            //      00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16
            //0011: 02 17 9C B9 00 02 9C 41 00 02 04 0F 04 17 00 7B 21
            //res=> 02 17 04 0F 00 04 FD 0A 72     
            memcpy(m_txbuffer, ResponseTemplate_Fcn17_Cmd04_L02, sizeof(ResponseTemplate_Fcn17_Cmd04_L02));  
            m_txbuffer[0] = m_rxbuffer[0];
            m_txbuffer[3] = counter;
            m_txbuffer[5] = cmd;
            m_txlen = sizeof(ResponseTemplate_Fcn17_Cmd04_L02);   
            return;
        }
    }

    Log3(LL_ERROR,"Frame skipped, unexpected data: ", m_rxbuffer, m_rxlen); 
}

const unsigned char ResponseTemplate_Fcn17_Cmd02_L05 []= {0x02,0x17,0x0a,0x00,0x00,0x02,0x05,0x04,0x30,0x10,0xff,0xa8,0x45,0x0e,0xdf};
void HCIEmulator::processDeviceBusScanFrame(){
    //      00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16
    //0013: 02 17 9C B9 00 05 9C 41 00 03 06 00 02 00 00 01 02 f8 35
    //res=> 02 17 0a 00 00 02 05 04 30 10 ff a8 45 0e df
    unsigned char counter = m_rxbuffer[11];
    unsigned char cmd = m_rxbuffer[12];
    memcpy(m_txbuffer, ResponseTemplate_Fcn17_Cmd02_L05, sizeof(ResponseTemplate_Fcn17_Cmd02_L05));  
    m_txbuffer[0] = m_rxbuffer[0];
    m_txbuffer[3] = counter;
    m_txbuffer[5] = cmd;
    m_txlen = sizeof(ResponseTemplate_Fcn17_Cmd02_L05); 

    Log(LL_INFO,"Busscan received");  
}

void HCIEmulator::processBroadcastStatusFrame(){
    //001B: 00 10 9D 31 00 09 12 64 00 00 00 40 60 00 00 00 00 00 00 00 00 00 01 00 00 CA 22
    bool hasChanged = false;
    CHECKCHANGEDSET(m_state.lampOn,m_rxbuffer[20] == 0x14,hasChanged);      
    CHECKCHANGEDSET(m_state.doorCurrentPosition,m_rxbuffer[10],hasChanged);
    CHECKCHANGEDSET(m_state.doorTargetPosition, m_rxbuffer[9],hasChanged);
    CHECKCHANGEDSET(m_state.doorState, m_rxbuffer[11],hasChanged);
    CHECKCHANGEDSET(m_state.reserved, m_rxbuffer[17],hasChanged);
    CHECKCHANGEDSET(m_state.valid, true,hasChanged);   
    
    if(hasChanged){
        Log3(LL_INFO,"New State: ",m_rxbuffer,m_rxlen);
        if(m_statusCallback != NULL){
            m_statusCallback(m_state);
        }  
    }             
}    

void HCIEmulator::openDoor(){
    if(m_statemachine != WAITING){
        return;
    }
    m_lastStateTime = millis();
    m_statemachine = STARTOPENDOOR;
}

void HCIEmulator::openDoorHalf(){
    if(m_statemachine != WAITING){
        return;
    }
    m_lastStateTime = millis();
    m_statemachine = STARTOPENDOORHALF;
}

void HCIEmulator::closeDoor(){
    if(m_statemachine != WAITING){
        return;
    }
    m_lastStateTime = millis();    
    m_statemachine = STARTCLOSEDOOR;
}

void HCIEmulator::stopDoor(){
    if(m_statemachine != WAITING){
        return;
    }
    // STARTSTOPDOOR actually would start a move if the door is not moving, which
    // is not intended by a call to "stopDoor". Hence, we check for movement,
    // anf if none, we won't trigger the command
    if(!(m_state.doorState == DOOR_MOVE_CLOSEPOSITION || m_state.doorState == DOOR_MOVE_OPENPOSITION)){    
        ESP_LOGD(TAG, "Close command while not moving");
        return;
    }
    m_lastStateTime = millis();    
    m_statemachine = STARTSTOPDOOR;
}

void HCIEmulator::toggleLamp(){
    if(m_statemachine != WAITING){
        return;
    }
    m_lastStateTime = millis();
    m_statemachine = STARTTOGGLELAMP;
}

void HCIEmulator::turnOffLamp(){
    if(m_state.lampOn) {
        toggleLamp();
    }
}

void HCIEmulator::turnOnLamp(){
    if( ! m_state.lampOn) {
        toggleLamp();
    }
}

void HCIEmulator::ventilationPosition(){
    if(m_statemachine != WAITING){
        return;
    }
    m_lastStateTime = millis();
    m_statemachine = STARTVENTPOSITION;
}

void HCIEmulator::onStatusChanged(callback_function_t handler) {
    m_statusCallback = handler;
}