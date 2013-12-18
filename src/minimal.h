// -*- C++ -*-
/*!
 * @file  rtc-stack-of-tasks.h * @brief Module for controlling humanoid robot * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef MINIMAL_STACK_OF_TASKS_H
#define MINIMAL_STACK_OF_TASKS_H

# include <sot/core/abstract-sot-external-interface.hh>

#include <boost/array.hpp>

namespace dgsot=dynamicgraph::sot;

/** \brief Config variables
 */
struct robot_config_t
{
  /// \brief Name of the controller to load
  std::string libname;
  /// \brief Robot number of DoFs
  //int nb_dofs;
  /// \brief Number of force sensors
  //int nb_force_sensors;
};

class MinimalStackOfTasks
{
 public:
  MinimalStackOfTasks();
  ~MinimalStackOfTasks();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
  virtual void onInitialize();

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual void onExecute();
//  void saveLog() const;


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /// \brief Config variables 
  robot_config_t robot_config_;

  /// \brief Starting the controller.
  bool started_;
  
  // </rtc-template>

  /// \brief Load the overall SoT structure.
  void LoadSot();

  /// \brief Capture the current time in the argument.
  /// \param t store current time
  void captureTime (timeval& t);

  /// \brief Log time spent between t0 and t1 in the next
  /// free case of timeArray.
  ///
  /// \param t0 begin time
  /// \param t1 end time
  void logTime (const timeval& t0, const timeval& t1);

  /// \brief Write logged times in a file.
  void writeLog ();

 private:

  /// \brief Read config variables
  void readConfig();

  /// \brief the sot-hrp2 controller
  dgsot::AbstractSotExternalInterface * m_sotController;

  /// Map of sensor readings
  std::map<std::string,dgsot::SensorValues> sensorsIn_;
  /// Map of control values
  std::map<std::string,dgsot::ControlValues> controlValues_;
  /// Angular values read by encoders
  std::vector <double> angleEncoder_;
  /// Angular values sent to motors
  std::vector<double> angleControl_;
  /// Forces read by force sensors
  std::vector<double> forces_;
  /// Torques
  std::vector<double> torques_;
  /// Attitude of the robot computed by extended Kalman filter.
  std::vector<double> baseAtt_;
  /// Accelerometer information.
  std::vector<double> accelerometer_;
  /// Gyrometer information.
  std::vector<double> gyrometer_;  

  /// \brief Timestamp matching the beginning of the control
  /// loop.
  timeval t0_;
  /// \brief Timestamp matching the end of the control loop.
  timeval t1_;

//  /// \brief Log time spend during control loops.
//  boost::array<double, 100000> timeArray_;

//  /// \brief First unfilled item in timeArray.
  unsigned int timeIndex_;

  bool initialize_library_;
};

#endif // RTC_STACK_OF_TASKS_H

