/*
 * protonek.cc
 *
 *  Created on: Sep 5, 2009
 *      Author: konradb3
 */

#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstring>

#include "elektron.hpp"

using namespace std;

double
ang_nor_rad(double rad)
{
        static double TWO_PI = 2.0*M_PI;
        for (;;) {
                if (rad >= M_PI)
                        rad -= TWO_PI;
                else if (rad <= -M_PI)
                        rad += TWO_PI;
                else
                        return (rad);
        }
}

Protonek::Protonek(const std::string& port, int baud)
{
	connected = false;
	
	llpos = 0;
	lrpos = 0;

	xpos = 0;
	ypos = 0;
	apos = 0;

	setvel.start = 'x';
	setvel.cmd = 'a';
	setvel.lvel = 0;
	setvel.rvel = 0;

	odom_initialized = false;

	robot_axle_length = AXLE_LENGTH;
	m_per_tick = M_PI * WHEEL_DIAM / ENC_TICKS;
	enc_ticks = ENC_TICKS;

	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd >= 0)
	{
		tcgetattr(fd, &oldtio);

		// set up new settings
		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD | CSTOPB;
		newtio.c_iflag = INPCK; //IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		if (cfsetispeed(&newtio, baud) < 0 || cfsetospeed(&newtio, baud) < 0) 
		{
			fprintf(stderr, "Failed to set serial baud rate: %d\n", baud);
			tcsetattr(fd, TCSANOW, &oldtio);
			close(fd);
			fd = -1;
			return;
		}
		// activate new settings
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &newtio);
		connected = true;
	}
}

Protonek::~Protonek()
{
  // restore old port settings
  if (fd > 0)
	  tcsetattr(fd, TCSANOW, &oldtio);
  close(fd);
}

void Protonek::update()
{
  unsigned int ret = 0;
  tcflush(fd, TCIFLUSH);
  write(fd, &setvel, sizeof(setvel));

  while(ret < sizeof(getdata))
    ret += read(fd, ((char*)&getdata)+ret, sizeof(getdata)-ret);

}

void Protonek::setVelocity(double lvel, double rvel)
{
  setvel.lvel = (int16_t)(lvel*(1/m_per_tick)*0.1); // Convert SI units to internal units
  setvel.rvel = (int16_t)(rvel*(1/m_per_tick)*0.1);
  if(setvel.rvel > 120)
	setvel.rvel = 120;
  else if(setvel.rvel < -120)
	setvel.rvel = -120;

  if(setvel.lvel > 120)
	setvel.lvel = 120;
  else if(setvel.lvel < -120)
	setvel.lvel = -120;
}

void Protonek::getVelocity(double &lvel, double &rvel)
{
  lvel = (double)(getdata.lvel) * m_per_tick * 10;
  rvel = (double)(getdata.rvel) * m_per_tick * 10;
}

void Protonek::updateOdometry()
{
  int lpos = getdata.lpos + getdata.lindex * enc_ticks;
  int rpos = getdata.rpos + getdata.rindex * enc_ticks;

  double linc = (double)(lpos - llpos) * m_per_tick;
  double rinc = (double)(rpos - lrpos) * m_per_tick;

  llpos = lpos;
  lrpos = rpos;
  if (odom_initialized == true)
  {
  apos += atan((linc - rinc)/robot_axle_length);
  apos = ang_nor_rad(apos);
  double dist = (rinc + linc)/2;

  xpos += dist * cos(apos);
  ypos += dist * sin(apos);
  }else odom_initialized = true;

}

void Protonek::getOdometry(double &x, double &y, double &a)
{
  x = xpos;
  y = ypos;
  a = apos;
}

void Protonek::setOdometry(double x, double y,double a)
{
  xpos = x;
  ypos = y;
  apos = a;
}

void Protonek::getRawOdometry(double &linc, double &rinc)
{
  int lpos = getdata.lpos + getdata.lindex * enc_ticks;
  int rpos = getdata.rpos + getdata.rindex * enc_ticks;

  linc = (lpos - llpos) * m_per_tick;
  rinc = (rpos - lrpos) * m_per_tick;

  llpos = lpos;
  lrpos = rpos;
}

bool Protonek::isConnected()
{
	return connected;
}
