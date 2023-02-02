// mc_naoqi
#include "MCControlNAOqi.h"

// SpaceVecAlg
#include <SpaceVecAlg/Conversions.h>

#include <mc_rbdyn/rpy_utils.h>

namespace mc_naoqi
{
MCControlNAOqi::MCControlNAOqi(mc_control::MCGlobalController & controller,
								const std::string & host,
								const unsigned int port = 9559)
: globalController_(controller), timestep_(static_cast<unsigned int>(1000 * controller.timestep())),
	interfaceRunning_(true), host_(host), port_(port)
{
	/* Set up interface GUI tab */
	globalController_.controller().gui()->addElement(
		{"NAOqi"}, mc_rtc::gui::Label("Host", [this]() { return this->host_; }),
		mc_rtc::gui::Label("Port", [this]() { return this->port_; }),
		mc_rtc::gui::Label("Connection status", [this]() { return this->connectionState_; }),
		mc_rtc::gui::Button(controllerButtonText_, [this]() { startOrStop(!controllerStartedState_); }),
		mc_rtc::gui::Button(servoButtonText_, [this]() { servo(!servoState_); })
	);
	
	/* Don't start running controller before commanded `start` */
	globalController_.running = false;

	/* Connect to robot (real or simulation) */
	if(host_ != "simulation")
	{
		/* Create Naoqi session */
		ALBroker_ = qi::makeSession();
		/* Try to connect via TCP to the robot */
		mc_rtc::log::info("MCControlNAOqi: Connecting to {} robot on address {}:{}", globalController_.robot().name(),
											host_, port_);
		std::stringstream strstr;
		try
		{
			mc_rtc::log::info("Connecting to {}:{}", host_, port_);
			strstr << "tcp://" << host_ << ":" << port_;
			ALBroker_->connect(strstr.str()).wait();
		}
		catch(const std::exception & e)
		{
			mc_rtc::log::error_and_throw<std::runtime_error>("Cannot connect to session: {}", e.what());
			ALBroker_->close();
		}
		mc_rtc::log::success("Connected to {}", host_);
		connectionState_ = strstr.str() + " OK";

		/* Connect to local robot modules */
		MCNAOqiDCM_ = ALBroker_->service("MCNAOqiDCM");
		/* Check that controller main robot and real robot are of the same type */
		std::string realRobotName = MCNAOqiDCM_.call<std::string>("getRobotName");
		if(realRobotName != globalController_.robot().name())
		{
			mc_rtc::log::error_and_throw<std::runtime_error>(
					"Controller main robot '{}' and the real robot '{}' are not of the same type",
					globalController_.robot().name(), realRobotName);
		}

		/* Map sensor name to sensor index in `sensors` vector */
		std::vector<std::string> sensorsOrder = MCNAOqiDCM_.call<std::vector<std::string>>("getSensorsOrder");
		for(size_t i = 0; i < sensorsOrder.size(); i++)
		{
			mc_rtc::log::info("Sensor[{}]: {}", i, sensorsOrder[i]);
			sensorOrderMap_[sensorsOrder[i]] = i;
		}

		/* Check that actuator order and names is the same everywhere */
		std::vector<std::string> jointsOrder = MCNAOqiDCM_.call<std::vector<std::string>>("getJointOrder");
		for(unsigned int i = 0; i < globalController_.robot().refJointOrder().size(); i++)
		{
			if(globalController_.robot().refJointOrder()[i] != jointsOrder[i]
				 || jointsOrder[i] != sensorsOrder[i].substr(std::string("Encoder").length()))
			{
				mc_rtc::log::error("{} != {} != {}", globalController_.robot().refJointOrder()[i], jointsOrder[i],
													 sensorsOrder[i].substr(std::string("Encoder").length()));
				mc_rtc::log::error_and_throw<std::runtime_error>(
						"Joints reference order does not match! Check the definitions in the remote and local robot modules");
			}
		}
		mc_rtc::log::info("Joints reference order check: OK");
	}
	else
	{
		connectionState_ = "virtual robot";
		mc_rtc::log::warning("Host is '{}'. Running simulation only. No connection to real robot.", host_);
	}

	/* Control thread will run QP (every timestep ms) and send result joint commands to the robot */
	controlThread_ = std::thread(std::bind(&MCControlNAOqi::control_thread, this));

	/* Sensor thread reads the sensor values and passes them along to `mc_rtc` (mc_observers etc) */
	if(host_ != "simulation")
	{
		/* Joints position sensor readings */
		qIn_.resize(globalController_.robot().refJointOrder().size());
		/* Torque for now is just electric current sensor readings */
		tauIn_.resize(globalController_.robot().refJointOrder().size());
		/* Allocate space for reading all sensors from the robot memory */
		numSensors_ = MCNAOqiDCM_.call<int>("numSensors");
		sensors_.resize(numSensors_);
		/* Start sensor thread */
		sensorThread_ = std::thread(std::bind(&MCControlNAOqi::sensor_thread, this));
	}
	else
	{
		/* Running simulation only */
		/* Setting realRobot encoder values same as control robot at the start of controller first run */
		auto & mbc = globalController_.robot().mbc();
		const auto & rjo = globalController_.ref_joint_order();
		for(const auto & jn : rjo)
		{
			if(globalController_.robot().hasJoint(jn))
			{
				for(auto & qj : mbc.q[globalController_.robot().jointIndexByName(jn)])
				{
					qIn_.push_back(qj);
				}
			}
		}
		globalController_.setEncoderValues(qIn_);
		globalController_.realRobot().posW(globalController_.robot().posW());
	}

	mc_rtc::log::info("MCControlNAOqi interface initialized");
}

/* Destructor */
MCControlNAOqi::~MCControlNAOqi()
{
	/* Disconnect callback from DCM */
	if(host_ != "simulation")
	{
		MCNAOqiDCM_.call<void>("stopLoop");
	}
	/* Wait for control thread to finish */
	controlThread_.join();
}

void MCControlNAOqi::control_thread()
{
	/* Wait for first sensor readings from sensor_thread */
	if(host_ != "simulation")
	{
		mc_rtc::log::info("[Control] Waiting for sensor data");
		std::unique_lock<std::mutex> lk(controlMut_);
		controlCV_.wait(lk);
		mc_rtc::log::info("[Control] Got sensor data, ready for control");
	}

	/* Vector of joint angles to be sent to robot actuators */
	std::vector<float> angles;
	angles.resize(globalController_.robot().refJointOrder().size());

	while(interfaceRunning_)
	{
		auto start = std::chrono::high_resolution_clock::now();

		if(globalController_.running)
		{

			/** CONTROL loop **/
			if(globalController_.run())
			{
				/* Get latest QP result */
				const mc_solver::QPResultMsg & res = globalController_.send(0);

				/* Prepare to send desired joint angles as actuator commands */
				for(size_t i = 0; i < globalController_.robot().refJointOrder().size(); ++i)
				{
					const auto & jname = globalController_.robot().refJointOrder()[i];
					angles[i] = static_cast<float>(res.robots_state[0].q.at(jname)[0]);
				}

				/* Send actuator commands to the robot */
				if(host_ != "simulation")
				{
					MCNAOqiDCM_.call<void>("setJointAngles", angles);
				}
			}
		}
		else
		{
			globalController_.run(); // keep running the gui and plugins
		}

		/* Wait until next controller run */
		double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
		if(elapsed * 1000 > timestep_)
		{
			mc_rtc::log::warning("[Control] Loop time {} exeeded timestep of {} ms", elapsed * 1000, timestep_);
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(timestep_ - static_cast<unsigned int>(elapsed * 1000)));
		}
	}
	mc_rtc::log::info("MCControlNAOqi running thread stopped");
}

void MCControlNAOqi::sensor_thread()
{
	while(interfaceRunning_)
	{
		auto start = std::chrono::high_resolution_clock::now();

		/* Get all sensor readings from the robot */
		sensors_ = MCNAOqiDCM_.call<std::vector<float>>("getSensors");
		double elapsed_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
		mc_rtc::log::warning("[getSensors] getSensors done in {} ms", elapsed_time * 1000);
		
		/* Process the sensor readings common to bith robots */
		/* Encoder values */
		const auto & ref_joint_order = globalController_.robot().refJointOrder();
		for(unsigned i = 0; i < ref_joint_order.size(); ++i)
		{
			const auto & jname = ref_joint_order[i];
			qIn_[i] = sensors_[sensorOrderMap_["Encoder" + jname]];
		}

		/* Electric current sensor values */
		for(unsigned i = 0; i < ref_joint_order.size(); ++i)
		{
			const auto & jname = ref_joint_order[i];
			// TODO: multiply these values by current to torque constants of each motor
			tauIn_[i] = sensors_[sensorOrderMap_["ElectricCurrent" + jname]];
		}

		/* Send sensor readings to mc_rtc controller */
		globalController_.setEncoderValues(qIn_);
		globalController_.setJointTorques(tauIn_);

		/* Start control only once the robot state has been read at least once */
		controlCV_.notify_one();
		double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
		if(elapsed * 1000 > timestep_)
		{
			mc_rtc::log::warning("[Sensors] Loop time {} exeeded timestep {} ms", elapsed * 1000, timestep_);
		}
		// else if(timestep_ - elapsed > timestep_ / 2.0 && blinking_)
		// {
		// 	/* Blink if there is enough time after sensors reading until next DCM cicle */
		// 	auto startExtraAnimation = std::chrono::high_resolution_clock::now();
		// 	msTillBlink_ -= int(timestep_ + elapsed * 1000);
		// 	if(msTillBlink_ <= 0)
		// 	{
		// 		MCNAOqiDCM_.post("blink");
		// 		msTillBlink_ = rand() % 6000 + 1000;
		// 	}
		// 	double elapsedExtraAnimation =
		// 			std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - startExtraAnimation).count();
		// 	std::this_thread::sleep_for(std::chrono::milliseconds(timestep_ - static_cast<unsigned int>(elapsed * 1000)
		// 																												- static_cast<unsigned int>(elapsedExtraAnimation * 1000)));
		// }
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(timestep_ - static_cast<unsigned int>(elapsed * 1000)));
		}
	}
}

void MCControlNAOqi::servo(const bool state)
{
	if(host_ != "simulation")
	{
		if(state) // Servo ON
		{
			mc_rtc::log::warning("Turning ON the motors");
			/* Deactivate safety reflexes if ALMotion module is running */
			if(ALlauncher_.call<bool>("isModulePresent", "ALMotion"))
			{
				mc_rtc::log::info("ALMotion module is active on the robot. Disabling safety reflexes...");
				try
				{
					qi::AnyObject al_motion = ALBroker_->service("ALMotion");
					al_motion.call<bool>("setCollisionProtectionEnabled", "Arms", false);
					al_motion.call<void>("setDiagnosisEffectEnabled", false);
					al_motion.call<void>("setSmartStiffnessEnabled", false);
					al_motion.call<void>("setExternalCollisionProtectionEnabled", "All", false);
					al_motion.call<void>("setFallManagerEnabled", false);
					if(globalController_.robot().name() == "pepper")
					{
						al_motion.call<void>("setPushRecoveryEnabled", false);
					}
				}
				catch(const std::exception & e)
				{
					std::cout << "Cannot deactivate safety reflexes " << e.what() << std::endl;
					std::cout << "Try going to http://your_robot_ip/advanced/#/settings to enable the deactivation first"
										<< std::endl;
				}
			}

			/* If controller is not running, set joint angle commands to current joint state from encoders */
			if(!globalController_.running)
			{
				std::vector<float> angles;
				angles.resize(globalController_.robot().refJointOrder().size());
				for(size_t i = 0; i < globalController_.robot().encoderValues().size(); ++i)
				{
					angles[i] = static_cast<float>(globalController_.robot().encoderValues()[i]);
				}
				MCNAOqiDCM_.call<void>("setJointAngles", angles);
			}

			/* Gradually increase stiffness over 1s to prevent initial jerky motion */
			for(int i = 1; i <= 100; ++i)
			{
				MCNAOqiDCM_.call<void>("setStiffness", i / 100.);
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
			servoState_ = true;
			servoButtonText_ = "Motors OFF";
			mc_rtc::log::warning("Motors ON");
			// end of servo ON
		}
		else
		{ /* Servo OFF */
			mc_rtc::log::warning("Turning OFF the motors");
			/* Gradually decrease stiffness over 1s to prevent jerky motion */
			for(int i = 1; i <= 100; ++i)
			{
				MCNAOqiDCM_.call<void>("setStiffness", 1.0 - i / 100.);
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}

			/* Re-activate safety reflexes */
			if(ALlauncher_.call<bool>("isModulePresent", "ALMotion"))
			{
				mc_rtc::log::info("ALMotion module is active on the robot. Re-activating safety reflexes...");
				qi::AnyObject al_motion = ALBroker_->service("ALMotion");
				al_motion.call<bool>("setCollisionProtectionEnabled", "Arms", true);
				al_motion.call<void>("setDiagnosisEffectEnabled", true);
				al_motion.call<void>("setSmartStiffnessEnabled", true);
				al_motion.call<void>("setExternalCollisionProtectionEnabled", "All", true);
				al_motion.call<void>("setFallManagerEnabled", true);
				if(globalController_.robot().name() == "pepper")
				{
					al_motion.call<void>("setPushRecoveryEnabled", true);
				}
				mc_rtc::log::info("Safety reflexes reactivated");
			}
			servoState_ = false;
			servoButtonText_ = "Motors ON";
			mc_rtc::log::warning("Motors OFF");
			// end of servo OFF
		}
	}
	else
	{
		mc_rtc::log::error("Host is virtual robot, cannot turn ON/OFF motors");
	}
}

void MCControlNAOqi::startOrStop(const bool state)
{

	if(state)
	{
		mc_rtc::log::info("Starting experiment");
		/* Initialize controller with values from the encoders */
		mc_rtc::log::info("Initializing controller");
		globalController_.init(qIn_);
		mc_rtc::log::info("Controller initialized with sensor data from encoders");

		/* Start running controller */
		globalController_.running = true;

		controllerStartedState_ = true;
		controllerButtonText_ = "Stop";
		mc_rtc::log::info("Controller stated");
		mc_rtc::log::info("Experiment started");
	}
	else
	{
		mc_rtc::log::info("Stopping experiment");
		/* Stop running controller */
		globalController_.running = false;

		controllerStartedState_ = false;
		controllerButtonText_ = "Start";
		mc_rtc::log::info("Controller Stopped");
		mc_rtc::log::info("Experiment stopped");
	}
}

} // namespace mc_naoqi
