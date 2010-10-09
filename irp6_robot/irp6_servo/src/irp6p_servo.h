/*
 * FakeServo.h
 *
 *  Created on: 22-09-2010
 *      Author: konrad
 */

#ifndef FAKESERVO_H_
#define FAKESERVO_H_

#include <string>
#include <vector>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include "Setpoint.hpp"
#include "JointState.hpp"

#include "hi_moxa.h"

const unsigned int number_of_drives = 6;
const double gear[6] = {-158.0, 2*M_PI/5.0, 2*M_PI/5.0, -128.0, -128.0*0.6, 288.8845};
const double synchro_motor_position[6] = {-15.9, -5.0, -8.527, 151.31, 432.25, 791.0};
const double theta[6] = {0.0, 2.203374e+02, 1.838348e+02, 1.570796e+00, 0.0, 0.0};

class Regulator
{
public:
  Regulator();
  ~Regulator();

  int doServo(int, int);
  void reset();
private:
  double position_increment_old; // przedosatnio odczytany przyrost polozenie (delta y[k-2]
  // -- mierzone w impulsach)
  double position_increment_new; // ostatnio odczytany przyrost polozenie (delta y[k-1]
  // -- mierzone w impulsach)
  double step_old_pulse; // poprzednia wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-2]
  // -- mierzone w radianach)
  double step_new; // nastepna wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-1]
  // -- mierzone w radianach)
  double step_old; // poprzednia wartosc zadana dla jednego kroku regulacji
  // (przyrost wartosci zadanej polozenia -- delta r[k-1]
  // -- mierzone w radianach)

  double set_value_new; // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k])
  double set_value_old; // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k-1])
  double set_value_very_old; // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k-2])
  double delta_eint; // przyrost calki uchybu
  double delta_eint_old; // przyrost calki uchybu w poprzednim kroku
};

typedef enum { NOT_SYNCHRONIZED, SERVOING, SYNCHRONIZING } State;
typedef enum { MOVE_TO_SYNCHRO_AREA, STOP, MOVE_FROM_SYNCHRO_AREA } SynchroState;

const double ENC_RES = 4000.0;
const double GEAR_RATIO = 1.0/150.0;
const int MAX_PWM = 2048;

class IRP6pServo: public RTT::TaskContext
{
public:
  IRP6pServo(const std::string& name);
  virtual ~IRP6pServo();

  bool configureHook();
  bool startHook();
  void updateHook();
protected:
  RTT::InputPort<std::vector<Setpoint> > setpoint_port;
  RTT::OutputPort<std::vector<JointState> > jointState_port;

  RTT::Property<int> numberOfJoints_prop;
private:
  void mp2i(const double* motors, double* joints);

  std::vector<Setpoint> setpoint;
  std::vector<JointState> jointState;

  int numberOfJoints;

  hi_moxa::HI_moxa hi_;
  Regulator reg;

  State state;
  SynchroState synchro_state_;
  int64_t pos_inc_[number_of_drives];
  int64_t pos_old_[number_of_drives];

};

#endif /* FAKESERVO_H_ */
