/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsManipulationEditorComponent.h"
#include "JointsManipulationComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2/Manipulation/Controllers/JointsPositionControllerRequests.h>
#include <ROS2/Manipulation/JointInfo.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <Source/ArticulationLinkComponent.h>
#include <Source/HingeJointComponent.h>

#include <ROS2/Frame/ROS2FrameSystemBus.h>
#include <AzCore/Console/ILogger.h>

namespace ROS2
{
    JointsManipulationEditorComponent::JointsManipulationEditorComponent()
    {
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_topic = "joint_states";
        m_jointStatePublisherConfiguration.m_frequency = 25.0f;
    }

    void printChildren(const AZStd::set<AZ::EntityId>& frameChildren) {
        if (frameChildren.empty()) {
            AZLOG_ERROR("PSZ empty frame children set");
            return;
        }
        for (const auto &childEntityId : frameChildren) {
            AZStd::string name;
            AZ::ComponentApplicationBus::BroadcastResult(name, &AZ::ComponentApplicationRequests::GetEntityName, childEntityId);
            AZLOG_ERROR("PSZ frame child entityId: '%s', name: '%s'", childEntityId.ToString().c_str(), name.c_str());
        }
    }

    void JointsManipulationEditorComponent::recursiveGetFrameChildren(const AZ::EntityId& frameEntityId) {
        AZ::Name jointName;
        ROS2::ROS2FrameComponentBus::EventResult(jointName, frameEntityId, &ROS2::ROS2FrameComponentBus::Events::GetJointName);
        AZLOG_ERROR("PSZ frame child joint name: '%s'", jointName.GetCStr());
        m_childrenJointNames.push_back(jointName.GetStringView());
        const auto children = ROS2FrameSystemInterface::Get()->GetChildrenEntityId(frameEntityId);
        printChildren(children);
        for (const auto& child : children) {
            recursiveGetFrameChildren(child);
        }
    }

    void JointsManipulationEditorComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();

        if (auto* ros2FrameEditorComponent = GetEntity()->FindComponent<ROS2::ROS2FrameEditorComponent>())
        {
            AZLOG_ERROR("PSZ got ROS2FrameEditoComponent");
            AZ::SystemTickBus::QueueFunction(&ROS2::JointsManipulationEditorComponent::recursiveGetFrameChildren, this, GetEntityId());

            AZStd::string jointNamesStr = "";
            for (const auto& jointName : m_childrenJointNames) {
                jointNamesStr += AZStd::string{jointName} + " ";
            }
            AZLOG_ERROR("PSZ all frame children joint names: '[%s]'", jointNamesStr.c_str());
        }
        else
        {
            AZLOG_ERROR("PSZ NO ROS2FrameEditoComponent :(");
        }
    }

    void JointsManipulationEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<JointsManipulationComponent>(m_jointStatePublisherConfiguration, m_initialPositions);
    }

    void JointsManipulationEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsManipulationEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsManipulationEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsManipulationEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsManipulationEditorComponent, AZ::Component>()
                ->Version(0)
                ->Field("JointStatePublisherConfiguration", &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration)
                ->Field("Initial positions", &JointsManipulationEditorComponent::m_initialPositions);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointsManipulationEditorComponent>("JointsManipulationEditorComponent", "Component for manipulation of joints")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/JointsManipulationEditorComponent.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/JointsManipulationEditorComponent.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration,
                        "Joint State Publisher",
                        "Configuration of Joint State Publisher")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_initialPositions,
                        "Initial positions",
                        "Initial positions of all the joints");
            }
        }
    }
} // namespace ROS2
