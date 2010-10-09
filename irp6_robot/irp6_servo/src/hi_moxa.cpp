#include "hi_moxa.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>

#include <exception>
#include <stdexcept>
#include <cstring>

#include <iostream>

namespace hi_moxa
{

HI_moxa::HI_moxa(unsigned int numberOfDrivers)
{
  last_drive_number = numberOfDrivers;
}

HI_moxa::~HI_moxa()
{
  for (unsigned int i = 0; i < last_drive_number; i++)
  {
    if (fd[i] > 0)
    {
      tcsetattr(fd[i], TCSANOW, &oldtio[i]);
      close(fd[i]);
    }
  }
}

void HI_moxa::init(std::vector<std::string> ports)
{

  if (ports.size() != last_drive_number)
    throw(std::runtime_error("ports size invalid !!!"));

  // inicjalizacja zmiennych
  for (unsigned int i = 0; i < last_drive_number; i++)
  {
    servo_data[i].first_hardware_read = true;
    servo_data[i].command_params = 0;
  }

  fd_max = 0;
  for (unsigned int i = 0; i < last_drive_number; i++)
  {
    // std::cout << "[info] opening port : " << ports[i] << std::endl;
    fd[i] = open(ports[i].c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd[i] < 0)
    {
      std::cout << "unable to open: " << ports[i] << std::endl;
      throw(std::runtime_error("unable to open device!!!"));
    }
    else
    {
      if (fd[i] > fd_max)
        fd_max = fd[i];
    }
    tcgetattr(fd[i], &oldtio[i]);

    // set up new settings
    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = CS8 | CLOCAL | CREAD | CSTOPB;
    newtio.c_iflag = INPCK; //IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0)
    {
      tcsetattr(fd[i], TCSANOW, &oldtio[i]);
      close(fd[i]);
      fd[i] = -1;
      throw(std::runtime_error("unable to set baudrate !!!"));
    }
    // activate new settings
    tcflush(fd[i], TCIFLUSH);
    tcsetattr(fd[i], TCSANOW, &newtio);
  }

  resetCounters();
}

void HI_moxa::insertSetValue(int drive_offset, double set_value)
{
  int drive_number;
  drive_number = drive_offset;
  servo_data[drive_number].buf[0] = 0x00;
  servo_data[drive_number].buf[1] = 0x00;
  servo_data[drive_number].buf[2] = 0x00;
  servo_data[drive_number].buf[3] = 0x00;
  servo_data[drive_number].buf[4] = START_BYTE;
  servo_data[drive_number].buf[5] = COMMAND_MODE_PWM | servo_data[drive_number].command_params;
  struct pwm_St* temp = (pwm_St*) &(servo_data[drive_number].buf[6]);
  //temp->pwm = set_value * (300.0 / 255.0);
  temp->pwm = set_value * (1000.0 / 255.0);

#ifdef T_INFO_FUNC
  std::cout << "[func] HI_moxa::insert_set_value(" << drive_offset << ", " << set_value << ")" << std::endl;
#endif
}

int HI_moxa::getCurrent(int drive_offset)
{
  int ret;

  ret = servo_data[drive_offset].drive_status.current;

#ifdef T_INFO_FUNC
  std::cout << "[func] HI_moxa::get_current(" << drive_offset << ") = " << ret << std::endl;
#endif
  return ret;
}

double HI_moxa::getIncrement(int drive_offset)
{
  double ret;
  ret = servo_data[drive_offset].current_position_inc;

#ifdef T_INFO_FUNC
  std::cout << "[func] HI_moxa::get_increment(" << drive_offset << ") = " << ret << std::endl;
#endif
  return ret;
}

long int HI_moxa::getPosition(int drive_offset)
{
  int ret;

  ret = servo_data[drive_offset].current_absolute_position;

#ifdef T_INFO_FUNC
  std::cout << "[func] HI_moxa::get_position(" << drive_offset << ") = " << ret << std::endl;
#endif
  return ret;
}

uint64_t HI_moxa::readWriteHardware(void)
{
  static int64_t receive_attempts = 0, receive_timeouts = 0;
  static int error_msg_power_stage = 0;
  bool hardware_read_ok = true;
  bool all_hardware_read = true;
  unsigned int bytes_received[MOXA_SERVOS_NR];
  fd_set rfds;
  uint64_t ret = 0;
  unsigned int drive_number;

  for (drive_number = 0; drive_number < last_drive_number; drive_number++)
  {
    write(fd[drive_number], servo_data[drive_number].buf, WRITE_BYTES);
    bytes_received[drive_number] = 0;
  }

  receive_attempts++;

  while (1)
  {
    FD_ZERO(&rfds);
    for (drive_number = 0; drive_number < last_drive_number; drive_number++)
    {
      if (bytes_received[drive_number] < READ_BYTES)
      {
        FD_SET(fd[drive_number], &rfds);
      }
    }

    // timeout
    struct timeval timeout;
    timeout.tv_sec = (time_t) 0;
    timeout.tv_usec = 500;
    int select_retval = select(fd_max + 1, &rfds, NULL, NULL, &timeout);
    if (select_retval == 0)
    {
      receive_timeouts++;
      hardware_read_ok = false;
      throw(std::runtime_error("communication timeout !!!"));
      break;
    }
    else
    {
      all_hardware_read = true;
      for (drive_number = 0; drive_number < last_drive_number; drive_number++)
      {
        if (FD_ISSET(fd[drive_number], &rfds))
        {
          bytes_received[drive_number]
          += read(fd[drive_number], (char*) (&(servo_data[drive_number].drive_status))
                  + bytes_received[drive_number], READ_BYTES - bytes_received[drive_number]);
        }
        if (bytes_received[drive_number] < READ_BYTES)
        {
          all_hardware_read = false;
        }
      }

      if (all_hardware_read)
        break;
    }
  }

  // Wypelnienie pol odebranymi danymi
  for (drive_number = 0; drive_number < last_drive_number; drive_number++)
  {

    // Wypelnienie pol odebranymi danymi
    if (bytes_received[drive_number] >= READ_BYTES)
    {
      servo_data[drive_number].previous_absolute_position = servo_data[drive_number].current_absolute_position;
      servo_data[drive_number].current_absolute_position = servo_data[drive_number].drive_status.position;
    }

    // W pierwszym odczycie danych z napedu przyrost pozycji musi byc 0.
    if (servo_data[drive_number].first_hardware_read && hardware_read_ok)
    {
      servo_data[drive_number].previous_absolute_position = servo_data[drive_number].current_absolute_position;
      servo_data[drive_number].first_hardware_read = false;
    }

    servo_data[drive_number].current_position_inc = (double) (servo_data[drive_number].current_absolute_position
        - servo_data[drive_number].previous_absolute_position);
  }

  robot_synchronized = false;
  power_fault = false;
  for (drive_number = 0; drive_number < last_drive_number; drive_number++)
  {
    if (servo_data[drive_number].drive_status.powerStageFault != 0)
    {
      power_fault = true;
    }
    if (servo_data[drive_number].drive_status.isSynchronized != 0)
    {
      robot_synchronized = true;
    }
  }

  for (drive_number = 0; drive_number < last_drive_number; drive_number++)
  {
    if (servo_data[drive_number].drive_status.sw1 != 0)
      ret |= (uint64_t) (UPPER_LIMIT_SWITCH << (5 * (drive_number))); // Zadzialal wylacznik "gorny" krancowy
    if (servo_data[drive_number].drive_status.sw2 != 0)
      ret |= (uint64_t) (LOWER_LIMIT_SWITCH << (5 * (drive_number))); // Zadzialal wylacznik "dolny" krancowy
    if (servo_data[drive_number].drive_status.swSynchr != 0)
      ret |= (uint64_t) (SYNCHRO_SWITCH_ON << (5 * (drive_number))); // Zadzialal wylacznik synchronizacji
    if (servo_data[drive_number].drive_status.synchroZero != 0)
      ret |= (uint64_t) (SYNCHRO_ZERO << (5 * (drive_number))); // Impuls zera rezolwera
    if (servo_data[drive_number].drive_status.overcurrent != 0)
      ret |= (uint64_t) (OVER_CURRENT << (5 * (drive_number))); // Przekroczenie dopuszczalnego pradu
  }

  return ret;
}

void HI_moxa::resetCounters(void)
{

  for (int i = 0; i < last_drive_number; i++)
  {

    servo_data[i].current_absolute_position = 0L;
    servo_data[i].previous_absolute_position = 0L;
    servo_data[i].current_position_inc = 0.0;

  }
}

void HI_moxa::startSynchro(int drive_offset)
{
  servo_data[drive_offset].command_params |= COMMAND_PARAM_SYNCHRO;
}

void HI_moxa::finishSynchro(int drive_offset)
{
  servo_data[drive_offset].command_params &= 0;
}

bool HI_moxa::isImpulseZero(int drive_offset)
{
  return servo_data[drive_offset].drive_status.synchroZero;
}

void HI_moxa::resetPosition(int drive_offset)
{
  int drive_number;
  drive_number = drive_offset;
  servo_data[drive_number].current_absolute_position = 0L;
  servo_data[drive_number].previous_absolute_position = 0L;
  servo_data[drive_number].current_position_inc = 0.0;
  servo_data[drive_number].first_hardware_read = true;
}

bool HI_moxa::isRobotSynchronized()
{
  return robot_synchronized;
}

bool HI_moxa::isInSynchroArea(int drive)
{
  return servo_data[drive].drive_status.swSynchr;
}

}

