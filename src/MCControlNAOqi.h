#pragma once

#include <mc_control/mc_global_controller.h>
#include <condition_variable>
#include <qi/session.hpp>
#include <thread>

namespace mc_naoqi
{
/**
 * @brief mc_rtc control interface for NAO and PEPPER robots running NAOqi operating system
 */
struct MCControlNAOqi
{
  /**
   * @brief Interface constructor and destructor
   */
  MCControlNAOqi(mc_control::MCGlobalController & controller, const std::string & host, const unsigned int port);

  virtual ~MCControlNAOqi();

  /**
   * @brief Is the interface running
   */
  bool running()
  {
    return interfaceRunning_;
  }

  /**
   * @brief Start or stop the experiment
   *
   * @param state
   *  true: Start controller
   *  false: Stop controller
   */
  void startOrStop(const bool state);
  bool controllerStartedState()
  {
    return controllerStartedState_;
  };

  /**
   * @brief Stop the experimnet
   */
  void stop();

  /**
   * @brief Set the joints stiffness to max value or switch off
   * Connect a preproccess callback to DCM loop
   *
   * @param state
   *  true: Turn on the actuators
   *  false: Turn off the actuators
   */
  void servo(const bool state);

  /**
   * @brief Return a reference to the global mc_rtc controller
   */
  mc_control::MCGlobalController & controller()
  {
    return globalController_;
  }

private:
  /*! Controller state (started or stopped) */
  bool controllerStartedState_ = false;
  std::string controllerButtonText_ = "Start/Stop controller";

  /**
   * @brief Thread that sends mc_rtc controller commands to the robot.
   */
  void control_thread();

  /**
   * @brief Thread that retrieves robot sensor values and sends them to mc_rtc
   */
  void sensor_thread();

  /*! Global mc_rtc controller */
  mc_control::MCGlobalController & globalController_;

  /*! Controller timestep expressed in ms */
  unsigned int timestep_;

  /*! Running the mc_naoqi interface */
  bool interfaceRunning_;

  /*! Servo on/off (joint stiffness 0 if off) */
  bool servoState_ = false;
  std::string servoButtonText_ = "Motors ON/OFF";

  /* Sensor information */
  /*! Encoder values */
  std::vector<double> qIn_;
  /*! ElectricCurrent values */
  std::vector<double> tauIn_;

  /* Connection information */
  /*! Connection host */
  std::string host_;
  /*! Connection port */
  unsigned int port_;
  /*! Connection state */
  std::string connectionState_ = "none";

  /* Handles communication with NAOqi */
  qi::SessionPtr ALBroker_;
  /*! Custom DCM module for fast access to NAOqi memory and actuators */
  qi::AnyObject MCNAOqiDCM_;
  /*! ALLauncher module (check if needed modules are present on robot) */
  qi::AnyObject ALlauncher_;

  /*! Control and sensor threads */
  std::thread controlThread_;
  // Wait for sensor input before starting control
  std::condition_variable controlCV_;
  std::mutex controlMut_;
  std::thread sensorThread_;

  /*!  Maps sensor name to sensor index */
  std::map<std::string, size_t> sensorOrderMap_;
  /*! Sensor values read from the robot memory */
  std::vector<float> sensors_;
  /*! Total number of sensors */
  int numSensors_;
};

} // namespace mc_naoqi
