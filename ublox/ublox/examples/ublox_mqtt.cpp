

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>

#include "mqtt/async_client.h"

#include "ublox/ublox.h"
#include "ublox/ublox_structures.h"

const std::string CLIENTID("ublox");
const std::string TOPIC("/sensors/Location");
const char*       LWT_PAYLOAD = "Last will and testament.";
const int         QOS         = 1;
const long        TIMEOUT     = 10000L;

typedef std::chrono::system_clock::time_point SysClkTimePoint;

std::shared_ptr<mqtt::async_client> client_;

bool displayPayload_;

std::ofstream outputFile_;

void navPosLlhCallback(ublox::NavPosLLH nav_position,
                       SysClkTimePoint  time_stamp)
{
  // std::cout << "NAV-POSLLH: " <<std::endl <<
  //               "  GPS milliseconds: " << nav_position.iTOW << std::endl <<
  //               "  Latitude: " << nav_position.latitude_scaled * 1.0e-7 <<
  //               std::endl << "  Longitude: " << nav_position.longitude_scaled
  //               * 1.0e-7 << std::endl << "  Height: " << nav_position.height
  //               << std::endl << std::endl;

  std::string payload;
  payload =
    "{\"Latitude\":" + std::to_string(nav_position.latitude_scaled * 1.0e-7) +
    ",\"Longitude\":" + std::to_string(nav_position.longitude_scaled * 1.0e-7) +
    "}";

  if (displayPayload_)
  {
    std::cout << payload << std::endl;
  }

  if (outputFile_.is_open())
  {
    outputFile_ << payload;
  }

  try
  {
    mqtt::message_ptr pubmsg = mqtt::make_message(payload.c_str());
    pubmsg->set_qos(QOS);
    client_->publish(TOPIC, pubmsg)->wait_for_completion(TIMEOUT);
  }
  catch (const mqtt::exception& exc)
  {
    std::cerr << "Error: " << exc.what() << std::endl;
  }
}

int main(int argc, char** argv)
{
  unsigned long baudrate = 115200;

  std::string port("/dev/ttyACM0");

  std::string address("tcp://localhost:1883");

  std::string outputFileName("");

  displayPayload_ = false;

  //    if (argc > 1)
  //    {
  //       port = std::string(argv[1]);
  //    }

  //    if (argc > 2)
  //    {
  //       std::istringstream(argv[2]) >> baudrate;
  //    }

  // if (argc >3)
  // {
  // 	address = std::string(argv[3]);
  // }

  for (int i = 1; i < argc; ++i)
  { /* We will iterate over argv[] to get the parameters stored inside.
     * Note that we're starting on 1 because we don't need to know the
     * path of the program, which is stored in argv[0] */
    if (i != argc)
    {  // Check that we haven't finished parsing already

      std::cout << std::string(argv[i]) << std::endl;

      if (std::string(argv[i]) == "-a")
      {
        // MQTT Broker address
        address = std::string(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "-b")
      {
        // Baud rate
        sscanf(argv[i + 1], "%lu", &baudrate);
      }
      else if (std::string(argv[i]) == "-d")
      {
        // Output rate
        displayPayload_ = true;
      }
      else if (std::string(argv[i]) == "-p")
      {
        // Port name
        port = std::string(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "-s")
      {
        // Save payload to file
        outputFileName = std::string(argv[i + 1]);

        outputFile_.open(outputFileName);

        if (!outputFile_.is_open())
        {
          std::cout << outputFileName << " failed to open." << std::endl;
          return (1);
        }
        else
        {
          std::cout << "File " << outputFileName << " opened." << std::endl;
        }
      }
      else if ((std::string(argv[i]) == "-h") |
               (std::string(argv[i]) == "--help") |
               (std::string(argv[i]) == "-help"))
      {
        std::cout << "Unrecognized input:" << argv[i] << std::endl;
        std::cout << "Input options are:" << std::endl;
        std::cout
          << "\t-a <address>: MQTT broker address (e.g. ''-a "
             "tcp://localhost:1883 (default), or -a somehostname.local'')"
          << std::endl;
        std::cout
          << "\t-b <baud rate>: Baudrate in bps (e.g. ''-b 115200 (default)'')"
          << std::endl;
        std::cout
          << "\t-d: Display MQTT payload to the command prompt (false default)"
          << std::endl;
        std::cout << "\t-p <port name>: Device port name (e.g. ''-s "
                     "/dev/ttyUSB0 (/dev/ttyACM0 default)'')"
                  << std::endl;
        std::cout << "\t-s <file name>: Output file name (e.g. ''-s "
                     "outputFile.txt (default none)'')"
                  << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return (1);
      }
      else
      {
        // Do nothing, iterate to next arg
      }
    }
    //    std::cout << argv[i] << " ";
  }

  try
  {
    client_ = std::make_shared<mqtt::async_client>(address, CLIENTID);

    mqtt::connect_options conopts;

    std::cout << "Connecting..." << std::endl;
    mqtt::itoken_ptr conntok = client_->connect(conopts);

    std::cout << "Waiting for the connection...";
    conntok->wait_for_completion();
    std::cout << "  ...OK" << std::endl;

    ublox::Ublox ubx;

    ubx.set_nav_position_llh_Handler(navPosLlhCallback);

    bool result = ubx.connect(port, baudrate);
    if (result)
    {
      std::cout << "Successfully connected." << std::endl;
    }
    else
    {
      std::cout << "Failed to connect." << std::endl;
      return -1;
    }

    // request position message
    ubx.configureMessageRate(0x01, 0x02, 1);

    // loop forever
    while (1)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (outputFile_.is_open())
    {
      outputFile_.close();
    }

    // Disconnect
    std::cout << "Disconnecting..." << std::endl;
    conntok = client_->disconnect();
    conntok->wait_for_completion();
    std::cout << "  ...OK" << std::endl;

    ubx.disconnect();
  }
  catch (const mqtt::exception& exc)
  {
    std::cerr << "Error: " << exc.what() << std::endl;
    return 1;
  }

  return 0;
}
