/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GeoreferenceLevelComponent.h"
#include "GNSSFormatConversions.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
namespace ROS2
{

    void GeoReferenceLevelComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GeoReferenceLevelComponent>()
                ->Version(1)
                ->Field("EnuOriginAltitude", &GeoReferenceLevelComponent::m_EnuOriginAltitude)
                ->Field("EnuOriginLatitude", &GeoReferenceLevelComponent::m_EnuOriginLatitude)
                ->Field("EnuOriginLongitude", &GeoReferenceLevelComponent::m_EnuOriginLongitude)
                ->Field("EnuOriginLocationEntityId", &GeoReferenceLevelComponent::m_EnuOriginLocationEntityId);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<GeoReferenceLevelComponent>(
                        "GeoReference Level Component", "Component allows to provide georeference level for the level")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2GNSSSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2GNSSSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoReferenceLevelComponent::m_EnuOriginLocationEntityId,
                        "ENU Origin Transform",
                        "ENU (North-East-Up) origin in the level")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoReferenceLevelComponent::m_EnuOriginAltitude,
                        "Origin Altitude",
                        "Origin altitude in meters, WGS84 ellipsoid")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoReferenceLevelComponent::m_EnuOriginLatitude,
                        "Origin Latitude",
                        "Origin latitude in degrees, WGS84 ellipsoid, west is negative")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoReferenceLevelComponent::m_EnuOriginLongitude,
                        "Origin Longitude",
                        "Origin longitude in degrees, WGS84 ellipsoid, south is negative");
            }
        }
    }

    void GeoReferenceLevelComponent::Activate()
    {
        AZ::EntityBus::Handler::BusConnect(m_EnuOriginLocationEntityId);
        GeoreferenceRequestsBus::Handler::BusConnect();
    }

    void GeoReferenceLevelComponent::Deactivate()
    {
        GeoreferenceRequestsBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
    }

    void GeoReferenceLevelComponent::OnEntityActivated(const AZ::EntityId& entityId)
    {
        m_EnuOriginTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(m_EnuOriginTransform, m_EnuOriginLocationEntityId, &AZ::TransformBus::Events::GetWorldTM);
        m_EnuOriginTransform.Invert();
        AZ_Printf("ROS2", "GeoReferenceLevelComponent::OnEntityActivated");
        AZ::EntityBus::Handler::BusDisconnect();
    }

    AZ::Vector3 GeoReferenceLevelComponent::ConvertFromLevelToWSG84(const AZ::Vector3& xyz)
    {
        using namespace ROS2::GNSS;
        const AZ::Vector3 enu = m_EnuOriginTransform.TransformPoint(xyz);
        const AZ::Vector3 ecef = ENUToECEF({ m_EnuOriginLatitude, m_EnuOriginLongitude, m_EnuOriginAltitude }, enu);
        const AZ::Vector3 wsg84 = ECEFToWGS84(ecef);
        return wsg84;
    }

    AZ::Vector3 GeoReferenceLevelComponent::ConvertFromWSG84ToLevel(const AZ::Vector3& latLon)
    {
        using namespace ROS2::GNSS;
        const AZ::Vector3 ecef = WGS84ToECEF(latLon);
        const AZ::Vector3 enu = ECEFToENU({ m_EnuOriginLatitude, m_EnuOriginLongitude, m_EnuOriginAltitude }, ecef);
        const AZ::Vector3 xyz = m_EnuOriginTransform.TransformPoint(enu);
        return xyz;
    };

    AZ::Quaternion GeoReferenceLevelComponent::ConvertFromLevelRotationToENU()
    {
        return m_EnuOriginTransform.GetRotation();
    };

} // namespace ROS2
