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

#include "oro_servo_msgs/Setpoints.h"
#include "oro_servo_msgs/ServoStates.h"

#include "hi_moxa.h"

const double DT = 0.002;
const unsigned int NUMBER_OF_DRIVES = 6;
const double GEAR[6] = {-158.0, 2*M_PI/5.0, 2*M_PI/5.0, -128.0, -128.0*0.6, 288.8845};
const double SYNCHRO_MOTOR_POSITION[6] = {-15.9, -5.0, -8.527, 151.31, 432.25, 791.0};
const double THETA[6] = {0.0, 2.203374e+02, 1.838348e+02, 1.570796e+00, 0.0, 0.0};

const double 	SYNCHRO_JOINT_POSITION[6] = { SYNCHRO_MOTOR_POSITION[0] - GEAR[0] * THETA[0],
                                            SYNCHRO_MOTOR_POSITION[1] - GEAR[1] * THETA[1],
	                                          SYNCHRO_MOTOR_POSITION[2] - GEAR[2] * THETA[2],
	                                          SYNCHRO_MOTOR_POSITION[3] - GEAR[3] * THETA[3],
	                                          SYNCHRO_MOTOR_POSITION[4] - GEAR[4] * THETA[4] - SYNCHRO_MOTOR_POSITION[3],
	                                          SYNCHRO_MOTOR_POSITION[5] - GEAR[5] * THETA[5] };

const int ENC_RES[6] = {4000, 4000, 4000, 4000, 4000, 2000};

const double LOWER_MOTOR_LIMIT[6] = { -470, -110, -80, -70, -80, -1000};
const double UPPER_MOTOR_LIMIT[6] = { 450, 100, 100, 380, 490, 3000};

const double A[6] = {0.412429378531, 0.655629139073, 0.315789473684, 0.548946716233, 0.391982182628, 0.3};
const double BB0[6] = {2.594932 * 0.6, 1.030178 * 0.6, 1.997464 * 0.6, 1.576266 * 0.4, 1.114648 * 0.4, 1.364 * 0.4};
const double BB1[6] = {2.504769 * 0.6, 0.986142 * 0.6, 1.904138 * 0.6, 1.468599 * 0.4, 1.021348 * 0.4, 1.264 * 0.4};

const int MAX_PWM = 2048;

const double SYNCHRO_STEP_COARSE[6] = {-0.03, -0.03, -0.03, -0.03, -0.03, -0.05};
const double SYNCHRO_STEP_FINE[6] = {0.007, 0.007, 0.007, 0.007, 0.007, 0.05};

class Regulator
{
public:
  Regulator();
  ~Regulator();

  int doServo(int, int);
  void reset();
  void setParam(double a, double b0, double b1);
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

  double a_, b0_, b1_;
};

typedef enum { NOT_SYNCHRONIZED, SERVOING, SYNCHRONIZING } State;
typedef enum { MOVE_TO_SYNCHRO_AREA, STOP, MOVE_FROM_SYNCHRO_AREA } SynchroState;

class IRP6pServo: public RTT::TaskContext
{
public:
  IRP6pServo(const std::string& name);
  virtual ~IRP6pServo();

  bool configureHook();
  bool startHook();
  void updateHook();
protected:
  RTT::InputPort<oro_servo_msgs::Setpoints> setpoint_port;
  RTT::OutputPort<oro_servo_msgs::ServoStates> jointState_port;

  RTT::Property<bool> autoSynchronize_prop;

private:
  void mp2i(const double* motors, double* joints);
  bool i2mp(const double* joints, double* motors);

  bool checkMotorPosition(const double *);

  oro_servo_msgs::Setpoints setpoint;
  oro_servo_msgs::ServoStates jointState;

  hi_moxa::HI_moxa hi_;
  Regulator reg_[NUMBER_OF_DRIVES];

  State state_;
  SynchroState synchro_state_;
  unsigned int synchro_drive_;
  int64_t pos_inc_[NUMBER_OF_DRIVES];
  double motor_pos_old_[NUMBER_OF_DRIVES];

  double joint_pos_[NUMBER_OF_DRIVES];
  double joint_pos_old_[NUMBER_OF_DRIVES];

  int delay_;

};

#endif /* FAKESERVO_H_ */
