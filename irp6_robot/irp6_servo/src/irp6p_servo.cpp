/*
 * FakeServo.cpp
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#include <ocl/Component.hpp>

#include <iostream>

#include "irp6p_servo.h"

const double synchro_position =  2.7649785262774484;

IRP6pServo::IRP6pServo(const std::string& name) :
    RTT::TaskContext(name, PreOperational), setpoint_port("setpoint"), jointState_port(
      "jointState"), numberOfJoints_prop("numberOfJoints", "", 0), hi_(number_of_drives)
{
  this->ports()->addPort(setpoint_port);
  this->ports()->addPort(jointState_port);

  this->addProperty(numberOfJoints_prop);
}

IRP6pServo::~IRP6pServo()
{

}

bool IRP6pServo::configureHook()
{
  std::vector<std::string> ports;
  ports.push_back("/dev/ttyMI0");
  ports.push_back("/dev/ttyMI1");
  ports.push_back("/dev/ttyMI2");
  ports.push_back("/dev/ttyMI3");
  ports.push_back("/dev/ttyMI4");
  ports.push_back("/dev/ttyMI5");

  try
  {
    hi_.init(ports);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
    return false;
  }

  jointState.resize(number_of_drives);
  return true;
}

bool IRP6pServo::startHook()
{
  try
  {
    for (unsigned int i = 0; i < number_of_drives; i++)
      hi_.insertSetValue(i, 0);
    hi_.readWriteHardware();

    if (hi_.isRobotSynchronized())
    {
      for (unsigned int i = 0; i < number_of_drives; i++)
        pos_old_[i] = hi_.getPosition(i);
      state = SERVOING;
    }
    else
    {
      std::cout << "robot not synchronized" << std::endl;
      //state = NOT_SYNCHRONIZED;
      synchro_state_ = MOVE_TO_SYNCHRO_AREA;
      state = SYNCHRONIZING;
    }
  }
  catch (const std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
  reg.reset();

  return true;
}

void IRP6pServo::updateHook()
{
  double motor_pos[number_of_drives];

  switch (state)
  {
  case NOT_SYNCHRONIZED :
    for (unsigned int i = 0; i < number_of_drives; i++)
      pos_inc_[i] = 0;
    break;
  case SERVOING :
    if (setpoint_port.read(setpoint) == RTT::NewData)
    {/*
      int64_t pos = ((setpoint[0].position - synchro_position) / GEAR_RATIO) * ENC_RES/(2*M_PI); // radians to encoder units
      pos_inc_ = pos - pos_old_;
      pos_old_ = pos; */
    }
    else
    {
      for (unsigned int i = 0; i < number_of_drives; i++)
        pos_inc_[i] = 0;
    }
    break;
  case SYNCHRONIZING :
    switch (synchro_state_)
    {
    case MOVE_TO_SYNCHRO_AREA :
      /*   if (hi_.isInSynchroArea(0))
         {
           std::cout << "MOVE_TO_SYNCHRO_AREA cmp" << std::endl;
           pos_inc_ = 0;
           synchro_state_ = STOP;
         }
         else
         {
           pos_inc_ = 20;
         }*/
      break;
    case STOP :
      std::cout << "STOP cmp" << std::endl;
      synchro_state_ = MOVE_FROM_SYNCHRO_AREA;
      hi_.startSynchro(0);
      break;
    case MOVE_FROM_SYNCHRO_AREA :
      if (hi_.isImpulseZero(0))
      {
        /*   std::cout << "MOVE_FROM_SYNCHRO_AREA cmp" << std::endl;
           hi_.finishSynchro(0);
           hi_.resetPosition(0);
           reg.reset();
           pos_inc_ = 0;
           state = SERVOING;
           pos_old_ = 0;*/
      }
      else
      {
        //     pos_inc_ = -5;
      }
      break;
    }
    break;
  }

  // do servo

  //int pwm = reg.doServo(pos_inc_, hi_.getIncrement(0));
  try
  {
    //hi_.insertSetValue(0, pwm);
    hi_.readWriteHardware();
  }
  catch (const std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }

  for (unsigned int i = 0; i < number_of_drives; i++)
    motor_pos[i] = ((double)hi_.getPosition(i) / (ENC_RES/(2*M_PI)));
  double joint_pos[number_of_drives];
  mp2i(motor_pos, joint_pos);
  
  for (unsigned int i = 0; i < number_of_drives; i++)
    jointState[i].position = joint_pos[i];

  jointState_port.write(jointState);
}

void IRP6pServo::mp2i(const double* motors, double* joints)
{
  // zmienne pomocnicze
  double c, d, l;
  double sinus, cosinus;
  double M1, M2;

  const double sl123 = 7.789525e+04;
  const double mi1 = 6.090255e+04;
  const double ni1 = -2.934668e+04;

  const double mi2 = -4.410000e+04;
  const double ni2 = -5.124000e+04;

// Przelicznik polozenia walu silnika napedowego kolumny w radianach
// na kat obrotu kolumny (wspolrzedna wewnetrzna) w radianach
  joints[0] = (motors[0] - synchro_motor_position[0]) / gear[0] + theta[0];

// Przelicznik polozenia walu silnika napedowego ramienia dolnego w radianach
// na kat obrotu ramienia (wspolrzedna wewnetrzna) w radianach
  l = (motors[1] - synchro_motor_position[1]) / gear[1] + theta[1];
  M1 = mi1 * mi1 + ni1 * ni1;
  c = l * l - sl123;
  d = sqrt(M1 - c * c);
  cosinus = (mi1 * c - ni1 * d) / M1;
  sinus = -(ni1 * c + mi1 * d) / M1;
  joints[1] = atan2(sinus, cosinus);

// Przelicznik polozenia walu silnika napedowego ramienia gornego w radianach
// na kat obrotu ramienia (wspolrzedna wewnetrzna) w radianach
  l = (motors[2] - synchro_motor_position[2]) / gear[2] + theta[2];
  M2 = mi2 * mi2 + ni2 * ni2;
  c = l * l - sl123;
  d = sqrt(M2 - c * c);
  cosinus = (mi2 * c - ni2 * d) / M2;
  sinus = -(ni2 * c + mi2 * d) / M2;
  joints[2] = atan2(sinus, cosinus);

// Przelicznik polozenia walu silnika napedowego obrotu kisci T w radianach
// na kat pochylenia kisci (wspolrzedna wewnetrzna) w radianach
  joints[3] = (motors[3] - synchro_motor_position[3]) / gear[3];

// Przelicznik polozenia walu silnika napedowego obrotu kisci V w radianach
// na kat obrotu kisci (wspolrzedna wewnetrzna) w radianach
  joints[4] = (motors[4] - synchro_motor_position[4] - (motors[3]
                             - synchro_motor_position[3])) / gear[4] + theta[4];

// Przelicznik polozenia walu silnika napedowego obrotu kisci N w radianach
// na kat obrotu kisci (wspolrzedna wewnetrzna) w radianach
  joints[5] = (motors[5] - synchro_motor_position[5]) / gear[5] + theta[5];

  joints[2] -= joints[1] + M_PI_2;
  joints[3] -= joints[2] + joints[1] + M_PI_2;

}

Regulator::Regulator()
{

}

Regulator::~Regulator()
{

}

int Regulator::doServo(int pos_inc_new, int pos_inc)
{

  // algorytm regulacji dla serwomechanizmu
  // position_increment_old - przedostatnio odczytany przyrost polozenie
  //                         (delta y[k-2] -- mierzone w impulsach)
  // position_increment_new - ostatnio odczytany przyrost polozenie
  //                         (delta y[k-1] -- mierzone w impulsach)
  // step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
  //                         regulacji (przyrost wartosci zadanej polozenia --
  //                         delta r[k-2] -- mierzone w impulsach)
  // step_new               - nastepna wartosc zadana dla jednego kroku
  //                         regulacji (przyrost wartosci zadanej polozenia --
  //                         delta r[k-1] -- mierzone w radianach)
  // set_value_new          - wielkosc kroku do realizacji przez HIP
  //                         (wypelnienie PWM -- u[k]): czas trwania jedynki
  // set_value_old          - wielkosc kroku do realizacji przez HIP
  //                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
  // set_value_very_old     - wielkosc kroku do realizacji przez HIP
  //                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

  double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji

  step_new_pulse = pos_inc_new;
  position_increment_new = pos_inc;

  // Przyrost calki uchybu
  delta_eint = delta_eint_old + 1.008 * (step_new_pulse - position_increment_new) - 0.992 * (step_old_pulse
               - position_increment_old);

  double a = 0.548946716233 / 2;
  double b0 = 1.576266 / 2; //9.244959545156;
  double b1 = 1.468599 / 2; //8.613484947882;

  // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
  set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * delta_eint - b1 * delta_eint_old;

  // ograniczenie na sterowanie
  if (set_value_new > MAX_PWM)
    set_value_new = MAX_PWM;
  if (set_value_new < -MAX_PWM)
    set_value_new = -MAX_PWM;

  // przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
  position_increment_old = position_increment_new;
  delta_eint_old = delta_eint;
  step_old_pulse = step_new_pulse;
  set_value_very_old = set_value_old;
  set_value_old = set_value_new;

  return ((int) set_value_new);
}

void Regulator::reset()
{
  position_increment_old = 0.0;
  position_increment_new = 0.0;
  step_old_pulse = 0.0;
  step_new = 0.0;
  set_value_new = 0.0;
  set_value_old = 0.0;
  set_value_very_old = 0.0;
  delta_eint = 0.0;
  delta_eint_old = 0.0;
}

ORO_CREATE_COMPONENT( IRP6pServo )
