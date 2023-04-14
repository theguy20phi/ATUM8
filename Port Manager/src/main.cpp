#include "main.h"
#include <fstream>
#include <vector>
#include <functional>
#include <unordered_map>

using PortMap = std::unordered_map<std::string, int8_t>;

template <int N>
PortMap getPorts(const std::string &filename,
				 const std::array<std::string, N> &portNames,
				 const std::array<int8_t, N> &defaultPorts)
{
	PortMap ports;
	for (int i = 0; i < N; i++)
		ports[portNames[i]] = defaultPorts[i];
	std::ifstream portsFile{filename};
	if (!portsFile)
		return ports;
	for (std::string portName : portNames)
	{
		std::string port;
		std::getline(portsFile, port);
		ports[portName] = std::stoi(port);
	}
	return ports;
}

PortMap ports;

constexpr int numOfPorts{4};

std::array<std::string, numOfPorts> portNames{"Left Front Drive",
											  "Left Back Drive",
											  "Right Front Drive",
											  "Right Back Drive"};

std::array<int8_t, numOfPorts> defaultPorts{1, 11, -10, -20};

const std::string filename{"/usd/settings.txt"};

void setPorts(std::string filename)
{
	std::ofstream portsFile{filename};
	for (std::string portName : portNames)
		portsFile << std::to_string(ports[portName]) << std::endl;
}

void initialize()
{
	ports = getPorts<numOfPorts>(filename, portNames, defaultPorts);

	initializeLCD();
	portManager = std::make_shared<atum8::Debugger>(atum8::Debugger::LineFns{
		[](int control)
		{ return "PORT MANAGER"; },
		[](int control)
		{
			const int index{0};
			int port = ports[portNames[index]] + control;
			ports[portNames[index]] = ports[portNames[index]] + control;
			return portNames[index] + " " + std::to_string(ports[portNames[index]]);
		},
		[](int control)
		{
			const int index{1};
			int port = ports[portNames[index]] + control;
			ports[portNames[index]] = ports[portNames[index]] + control;
			return portNames[index] + " " + std::to_string(ports[portNames[index]]);
		},
		[](int control)
		{
			const int index{2};
			int port = ports[portNames[index]] + control;
			ports[portNames[index]] = ports[portNames[index]] + control;
			return portNames[index] + " " + std::to_string(ports[portNames[index]]);
		},
		[](int control)
		{
			const int index{3};
			ports[portNames[index]] = ports[portNames[index]] + control;
			return portNames[index] + " " + std::to_string(ports[portNames[index]]);
		}});
}

void opcontrol()
{
	while (true)
	{
		ports = getPorts<numOfPorts>(filename, portNames, defaultPorts);
		portManager->view();
		setPorts(filename);
		pros::delay(100);
	}
}

/* -------------------------------------------------------------------------- */
/*                                   Helpers                                  */
/* -------------------------------------------------------------------------- */

void initializeLCD()
{
	pros::lcd::initialize();
	pros::lcd::set_background_color(lv_color_t{0x000000});
	pros::lcd::set_text_color(255, 255, 255);
	pros::lcd::register_btn0_cb([]()
								{ portManager->control(-1); });
	pros::lcd::register_btn1_cb([]()
								{ portManager->control(0); });
	pros::lcd::register_btn2_cb([]()
								{ portManager->control(1); });
}