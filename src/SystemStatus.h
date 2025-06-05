#ifndef SYSTEM_STATUS_H
#define SYSTEM_STATUS_H

enum SystemStatus {
  STATUS_MENU,
  STATUS_HOMING,
  STATUS_JOINT,
  STATUS_KINEMATIC,
  STATUS_IDLE,
  STATUS_ERROR
};

void setStatusLED(SystemStatus s);

#endif // SYSTEM_STATUS_H
