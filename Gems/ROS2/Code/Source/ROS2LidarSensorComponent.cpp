#include "ROS2LidarSensorComponent.h"

using namespace ROS2;

void ROS2LidarSensorComponent::Init()
{
    //ROS2Requests* requests = ROS2Interface::Get();
    auto ros2_node = ROS2Interface::Get()->GetNode();
    //EBUS_EVENT_RESULT(ros2_node, ROS2RequestBus, GetNode); // ...in case of no response
    m_pointCloudPublisher = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
}

void ROS2LidarSensorComponent::Activate()
{
    // TODO - add range validation (Attributes?)
    m_frameTime = m_hz == 0 ? 1 : 1 / m_hz;
    AZ::TickBus::Handler::BusConnect();
}

void ROS2LidarSensorComponent::Deactivate()
{
    AZ::TickBus::Handler::BusDisconnect();
}

void ROS2LidarSensorComponent::Reflect(AZ::ReflectContext* context)
{
    if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
    {
        serialize->Class<ROS2LidarSensorComponent, AZ::Component>()
            ->Version(1)
            ->Field("hz", &ROS2LidarSensorComponent::m_hz)
            ->Field("frameName", &ROS2LidarSensorComponent::m_frameName)
            ->Field("lidarModel", &ROS2LidarSensorComponent::m_lidarModel)
            ;

        if (AZ::EditContext* ec = serialize->GetEditContext())
        {
            ec->Class<ROS2LidarSensorComponent>("Lidar Sensor", "[Simple Lidar component]")
                ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2LidarSensorComponent::m_hz, "Hz", "Lidar data acquisition and publish frequency")
                ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2LidarSensorComponent::m_frameName, "Frame Name", "Lidar ros2 message frame")
                ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2LidarSensorComponent::m_lidarModel, "Lidar Model", "Lidar model")
                ;
        }
    }
}

void ROS2LidarSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
{
    required.push_back(AZ_CRC("TransformService"));
}

void ROS2LidarSensorComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
{
    static float elapsed = 0;
    elapsed += deltaTime;
    if (elapsed < m_frameTime)
        return;

    elapsed -= m_frameTime;
    if (deltaTime > m_frameTime)
    {   // Frequency higher than possible, not catching up, just keep going with each frame.
        elapsed = 0;
    }

    AZStd::vector<AZ::Vector3> results;
    float distance = LidarTemplateUtils::GetTemplate(m_lidarModel).m_maxRange;
    const auto directions = LidarTemplateUtils::PopulateRayDirections(m_lidarModel);
    //AZ_TracePrintf("Lidar Sensor Component", "Populated with %d", int(directions.size()));
    AZ::Vector3 start(0.0f, 0.0f, 5.0f); // TODO - get transform
    m_lidarRaycaster.PerformRaycast(start, directions, distance, results);

    if (results.size() < 1)
    {
        AZ_TracePrintf("Lidar Sensor Component", "No results from raycast");
        return;
    }

    //AZ_TracePrintf("Lidar Sensor Component", "Raycast done, results ready");

    auto message = sensor_msgs::msg::PointCloud2();
    message.header.frame_id = m_frameName.data();
    message.height = 1;
    message.width = results.size();
    message.point_step = sizeof(AZ::Vector3); // TODO - check
    message.row_step = message.width * message.point_step;

    std::vector<std::string> point_field_names = { "x", "y", "z"};
    for (int i = 0; i < point_field_names.size(); i++)
    {
        sensor_msgs::msg::PointField pf;
        pf.name = point_field_names[i];
        pf.offset = i * 4;
        pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
        pf.count = 1;
        message.fields.push_back(pf);
    }

    message.data.resize(message.row_step);
    memcpy(message.data.data(), results.data(), message.data.size());

    m_pointCloudPublisher->publish(message);
}

