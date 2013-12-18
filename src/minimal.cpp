// -*- C++ -*-
/*!
 * @file  MinimalStackOfTasks.cpp * @brief Module for controlling humanoid robot * $Date$
 *
 * $Id$ 
 */
#include "minimal.h"
#include <stdlib.h>
#include <dlfcn.h>
#include <cmath>
#include <fstream>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sys/time.h>
#include <signal.h>
#include <boost/shared_ptr.hpp>

//#define DEBUG
#ifdef DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/rtc-stack-of-tasks-comp"
#define RESETDEBUG5() { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::out);  \
DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app);   \
DebugFile << __FILE__ << ":" \
          << __FUNCTION__ << "(#" \
          << __LINE__ << "):" << x << std::endl; \
          DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app); \
DebugFile << x << std::endl; \
          DebugFile.close();}



MinimalStackOfTasks::MinimalStackOfTasks()
    // <rtc-template block="initializer">
  : initialize_library_(false)
    // </rtc-template>
{
  RESETDEBUG5()
}

MinimalStackOfTasks::~MinimalStackOfTasks()
{
}

std::string filename ("/tmp/rtc-log-time.txt");
std::ofstream logTimeFile (filename.c_str());

//void MinimalStackOfTasks::saveLog() const
//{
////  std::string filename ("/tmp/rtc-log-time.txt");
////  std::ofstream logTime (filename.c_str());
////  if(logTime.is_open())
////  {
////    for(unsigned i=0;i<std::min<unsigned>(timeIndex_, timeArray_.size()); ++i)
////      logTime << i << "   " << "   " << timeArray_[i] << std::endl;
////    logTime.close();
////  }
////  else
////  {
////    ODEBUG5("Unable to open '" << filename <<"' to save the log'");
////  }
//}

void MinimalStackOfTasks::readConfig()
{
  robot_config_.libname="libsot-hrp4-controller.so";
  ODEBUG5("The library to be loaded: " << robot_config_.libname) ;
//  ODEBUG5("Nb dofs:" << robot_config_.nb_dofs);
//  ODEBUG5("Nb force sensors:" << robot_config_.nb_force_sensors);
}

void MinimalStackOfTasks::LoadSot()
{
  char * sLD_LIBRARY_PATH;
  sLD_LIBRARY_PATH=getenv("LD_LIBRARY_PATH");
  ODEBUG5("LoadSot - Start " << sLD_LIBRARY_PATH);
  char * sPYTHONPATH;
  sPYTHONPATH=getenv("PYTHONPATH");
  ODEBUG5("PYTHONPATH:" << sPYTHONPATH );
  sPYTHONPATH=getenv("PYTHON_PATH");
  ODEBUG5("PYTHON_PATH:" << sPYTHONPATH);

  // Load the SotHRP2Controller library.
  void * SotHRP2ControllerLibrary = dlopen(robot_config_.libname.c_str(),
                                           RTLD_GLOBAL | RTLD_NOW);
  if (!SotHRP2ControllerLibrary) {
    ODEBUG5("Cannot load library: " << dlerror() );
    return ;
  }
  ODEBUG5("Success in loading the library:" << robot_config_.libname);
  // reset errors
  dlerror();
  
  // Load the symbols.
  createSotExternalInterface_t * createHRP2Controller =
    (createSotExternalInterface_t *) dlsym(SotHRP2ControllerLibrary, 
                                           "createSotExternalInterface");
  ODEBUG5("createHRPController call "<< std::hex
          << std::setbase(10));
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    ODEBUG5("Cannot load symbol create: " << dlsym_error );
    return ;
  }
  ODEBUG5("Success in getting the controller factory");
  
  // Create hrp2-controller
  try 
    {
      ODEBUG5("exception handled createHRP2Controller call "<< std::hex 
              << std::setbase(10));
      m_sotController = createHRP2Controller();
      ODEBUG5("After createHRP2Controller.");

    } 
  catch (std::exception &e)
    {
      ODEBUG5("Exception: " << e.what());
    }
  ODEBUG5("LoadSot - End");
}


void
MinimalStackOfTasks::captureTime (timeval& t)
{
  gettimeofday (&t, NULL);
}

void
MinimalStackOfTasks::logTime (const timeval& t0, const timeval& t1)
{
  double dt =
    (t1.tv_sec - t0.tv_sec)
    + (t1.tv_usec - t0.tv_usec + 0.) / 1e6;

//  if (timeIndex_ < timeArray_.size())
//    timeArray_[timeIndex_++] = dt;
  logTimeFile << timeIndex_ << "   " << "   " << dt << std::endl;
  ++timeIndex_;
}

void MinimalStackOfTasks::onInitialize()
{
  if (!initialize_library_)
  {
    readConfig();
    LoadSot();
    initialize_library_ = true;
  }
}

void MinimalStackOfTasks::onExecute()
{
//  ODEBUG(m_configsets.getActiveId());
  //
  // Log control loop start time.
  captureTime (t0_);
  
//  fillSensors(sensorsIn_);
  try
    {
      m_sotController->setupSetSensors(sensorsIn_);
      m_sotController->getControl(controlValues_);
    }
  catch (std::exception &e)
    {  ODEBUG5("Exception on Execute: " << e.what());throw e; }
//  ODEBUG("Before reading control");
//  readControl(controlValues_);
//  ODEBUG("After reading control");

  // Log control loop end time and compute time spent.
  captureTime (t1_);
  logTime (t0_, t1_);
  ODEBUG("onExecute - end");
  return;
}


int main ()
{
  boost::shared_ptr<MinimalStackOfTasks> m;
  m.reset(new MinimalStackOfTasks);

  m->onInitialize();
  char tmp;
  std::cout << "Waiting for your input to start..."<<std::endl;
//  std::cin >> tmp;
  sleep(10);
  std::cout << "Starting" <<std::endl;
  while(1){
    m->onExecute();
    usleep(500);
  }
}

