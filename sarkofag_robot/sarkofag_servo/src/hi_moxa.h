/* TODO:
 *
 * inicjalizacja struktur servo_data w konstruktorze hi_moxa
 * przekazanie do konstruktora hi_moxa danych o ilosci i numerach portow
 */

#ifndef __HI_MOXA_H
#define __HI_MOXA_H

#define USLEEP_US 500000

#include "hi_moxa_combuf.h"

#include <stdint.h>
#include <termios.h>
//#include <ctime>

#include <string>
#include <vector>

namespace hi_moxa
{

const int BAUD = B921600; // 921600;
const int WRITE_BYTES = 10;
const int READ_BYTES = 8;
const char INIT_PORT_CHAR = 50;

const int MOXA_SERVOS_NR = 8;

const long COMMCYCLE_TIME_NS = 2000000;

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------


class HI_moxa
{

public:

  HI_moxa(unsigned int numberOfDrivers); // Konstruktor
  ~HI_moxa();

  virtual void init(std::vector<std::string> ports);
  virtual void insertSetValue(int drive_offset, double set_value);
  virtual int getCurrent(int drive_offset);
  virtual double getIncrement(int drive_offset);
  virtual long int getPosition(int drive_offset);
  virtual uint64_t readWriteHardware(void); // Obsluga sprzetu
  virtual void resetCounters(void); // Zerowanie licznikow polozenia
  virtual void startSynchro(int drive_offset);
  virtual void finishSynchro(int drive_offset);
  virtual bool isInSynchroArea(int drive);

  virtual bool isImpulseZero(int drive_offset);
  virtual void resetPosition(int drive_offset);

  bool isRobotSynchronized();

protected:
private:

  int fd[MOXA_SERVOS_NR], fd_max;
  unsigned int last_drive_number;
  struct servo_St servo_data[MOXA_SERVOS_NR];
  struct termios oldtio[MOXA_SERVOS_NR];

  bool robot_synchronized;
  bool power_fault;

}; // koniec: class hardware_interface

} // namespace hi_moxa

#endif // __HI_MOXA_H
