#include "sensor_data.h"

using namespace std;

sensor_data::sensor_data()
{
  
}

void sensor_data::Init_IMU()
{
  if(IMU_on == true)
  {
    // Initialize connection and node here
    connection = std::make_unique<mscl::Connection>(mscl::Connection::Serial("/dev/ttyACM0"));
    node = std::make_unique<mscl::InertialNode>(*connection);

    if (!node->features().supportsCategory(mscl::MipTypes::CLASS_AHRS_IMU)) {
      std::cerr << "This node does not support IMU/Filtering data." << std::endl;
      return;
    }

    mscl::MipChannels ahrsChannels = {
      mscl::MipChannel(mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_DELTA_VELOCITY_VEC, mscl::SampleRate::Hertz(1000)),
      mscl::MipChannel(mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_GYRO_VEC, mscl::SampleRate::Hertz(1000))
    };

    node->setActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU, ahrsChannels);
    node->enableDataStream(mscl::MipTypes::CLASS_AHRS_IMU);

    std::cout << "Data stream enabled. Reading data at 1 kHz..." << std::endl;

  }
}

void sensor_data::get_IMU_data()
{

  if(get_IMU_on == false)
  {get_IMU_on = IMU_on;}

  if(get_IMU_on == true)
  {
    mscl::MipDataPackets packets = node->getDataPackets();
    for (const auto& packet : packets)
    {
      mscl::MipDataPoints points = packet.data(); 
      for(int i = 0; i < 6; i ++)
      Trunk_data[i] = points[i].as_double();
    }
  
  }
  

  Trunk_linear_vel[0] = 9.81 * Trunk_data[1];
  Trunk_linear_vel[1] = 9.81 * Trunk_data[0];
  Trunk_linear_vel[2] = 9.81 * Trunk_data[2];
  Trunk_ori[0] = Trunk_data[4]; // Alpha dot 
  Trunk_ori[1] = Trunk_data[3]; // Beta dot
  Trunk_ori[2] = Trunk_data[5]; // Gamma dot
}

void sensor_data::exchange_mutex()
{
  IMU_on = _M_IMU_on;
  _M_Trunk_ori = Trunk_ori;
  _M_Trunk_linear_vel = Trunk_linear_vel;
  
}
