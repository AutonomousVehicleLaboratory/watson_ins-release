/* 
 * watson_ins_driver.h
 *
 * Copyright 2019 AVL
 */
#ifndef WATSON_INS_H
#define WATSON_INS_H
#include <serial/serial.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"

// NOTE: Change this data structure depending on the enabled Watson channels.
// Yes, I know, unfortunately, for sake of code compactness, the driver will
// have to be recompiled everytime the INS output channels are modified.
//
// This struct is overlayed on top of the received buffer from the INS, and the
// string fields can then be directly accessed.
//
struct ins_data_t {
  char       dataType[2]; // 0
  char      timestamp[9]; // 1
  char           roll[7]; // 4
  char          pitch[6]; // 3
  char            yaw[6]; // 2
  char         xAccel[6]; // 5
  char         yAccel[6]; // 6
  char         zAccel[6]; // 7
  char   forwardAccel[6]; // 8, NOTE: not used
  char   lateralAccel[6]; // 9, NOTE: not used
  char  verticalAccel[6]; // 10, NOTE: not used
  char     xAngleRate[6]; // 11
  char     yAngleRate[6]; // 12
  char     zAngleRate[6]; // 13
  char     hedingRate[6]; // 14, NOTE: not used
  char     forwardVel[7]; // 15, NOTE: not used
  char            lat[10];// 16
  char            lon[11];// 17
  char            alt[5]; // 18
  char            _cr[1]; // 19
};

class WatsonINSDriver {
  public:
    WatsonINSDriver(std::string path, int baud, int timeout_ms);  
    ~WatsonINSDriver();
    void read(sensor_msgs::Imu &imuMsg,
              sensor_msgs::NavSatFix &fixMsg,
              bool &validImu, bool &validFix);

  private:
    static const int readRetryCount = 10;
    serial::Serial* serDev;
    int gpsMsgSeq;
    struct ins_data_t insData;

    bool validCmd(void);
    void parseIMU(sensor_msgs::Imu &imuMsg);
    void parseGPS(sensor_msgs::NavSatFix &gpsMsg);
};
#endif /* WATSON_INS_H */
