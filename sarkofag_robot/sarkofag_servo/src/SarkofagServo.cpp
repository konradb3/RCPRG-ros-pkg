/*
 * FakeServo.cpp
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#include <ocl/Component.hpp>

#include <iostream>

#include "SarkofagServo.h"

const double synchro_position =  2.7649785262774484;

SarkofagServo::SarkofagServo(const std::string& name) :
    RTT::TaskContext(name, PreOperational), setpoint_port("setpoint"), jointState_port(
      "jointState"), numberOfJoints_prop("numberOfJoints", "", 0), hi_(1)
{

  this->ports()->addPort(setpoint_port);
  this->ports()->addPort(jointState_port);

  this->addProperty(numberOfJoints_prop);
}

SarkofagServo::~SarkofagServo()
{

}

bool SarkofagServo::configureHook()
{
  std::vector<std::string> ports;
  ports.push_back("/dev/ttyMI0");

  try
  {
    hi_.init(ports);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
    return false;
  }

  jointState.resize(1);
  return true;
}

bool SarkofagServo::startHook()
{
  try
  {
    hi_.insertSetValue(0, 0);
    hi_.readWriteHardware();

    if (hi_.isRobotSynchronized())
    {
      pos_old_ = hi_.getPosition(0);
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

void SarkofagServo::updateHook()
{
  switch (state)
  {
  case NOT_SYNCHRONIZED :
    pos_inc_ = 0;
    break;
  case SERVOING :
    if (setpoint_port.read(setpoint) == RTT::NewData)
    {
      int64_t pos = ((setpoint[0].position - synchro_position) / GEAR_RATIO) * ENC_RES/(2*M_PI); // radians to encoder units
      pos_inc_ = pos - pos_old_;
      pos_old_ = pos;
    }
    else
    {
      pos_inc_ = 0;
    }
    break;
  case SYNCHRONIZING :
    switch (synchro_state_)
    {
    case MOVE_TO_SYNCHRO_AREA :
      if (hi_.isInSynchroArea(0))
      {
        std::cout << "MOVE_TO_SYNCHRO_AREA cmp" << std::endl;
        pos_inc_ = 0;
        synchro_state_ = STOP;
      }
      else
      {
        pos_inc_ = 20;
      }
      break;
    case STOP :
      std::cout << "STOP cmp" << std::endl;
      synchro_state_ = MOVE_FROM_SYNCHRO_AREA;
      hi_.startSynchro(0);
      break;
    case MOVE_FROM_SYNCHRO_AREA :
      if (hi_.isImpulseZero(0))
      {
        std::cout << "MOVE_FROM_SYNCHRO_AREA cmp" << std::endl;
        hi_.finishSynchro(0);
        hi_.resetPosition(0);
        reg.reset();
        pos_inc_ = 0;
        state = SERVOING;
        pos_old_ = 0;
      }
      else
      {
        pos_inc_ = -5;
      }
      break;
    }
    break;
  }

  // do servo

  int pwm = reg.doServo(pos_inc_, hi_.getIncrement(0));
  try
  {
    hi_.insertSetValue(0, pwm);
    hi_.readWriteHardware();
  }
  catch (const std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
//  std::cout << hi_.getPosition(0) << std::endl;
  jointState[0].position = (((double)hi_.getPosition(0) / (ENC_RES/(2*M_PI))) * GEAR_RATIO) + synchro_position;
//  std::cout << jointState[0].position << std::endl;
//  std::cout << synchro_position << std::endl;

  jointState_port.write(jointState);
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

ORO_CREATE_COMPONENT( SarkofagServo )
