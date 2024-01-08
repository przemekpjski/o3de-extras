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
        : m_childChangedHandler([this](AZ::ChildChangeType type, AZ::EntityId child) { OnChildChanged(type, child); })
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

        AZStd::vector<AZStd::string> entityNames; //jointNames;
        auto searchFunc = [&entityNames](AZ::Entity* entity)
        {
            entityNames.push_back(entity->GetName());
        };

        AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationRequests::EnumerateEntities, searchFunc);

        AZStd::string allNames;
        for (auto&& name : entityNames) {
            allNames += name + " ";
        }
        AZLOG_ERROR("PSZ entity names: '%s'", allNames.c_str());
        //auto parentEntityId = ROS2FrameSystemInterface::Get()->GetParentEntityId(GetEntityId());
        AZ::Entity* entity = GetEntity();
        
        AZLOG_ERROR("PSZ JointsManipulationEditorComponent's component id: '%llu'", GetId());
        AZLOG_ERROR("PSZ JointsManipulationEditorComponent's entity id: '%s', name: '%s'", entity->GetId().ToString().c_str(), entity->GetName().c_str());
        auto isEntityTopLevel = ROS2FrameSystemInterface::Get()->IsTopLevel(entity->GetId());
        AZLOG_ERROR("PSZ entity is top level: '%d'", isEntityTopLevel);

        auto transform = entity->GetTransform();
        auto parentEntityId = transform->GetParentId();
        AZStd::string parentEntityName;
        AZ::ComponentApplicationBus::BroadcastResult(parentEntityName, &AZ::ComponentApplicationRequests::GetEntityName, parentEntityId);
        AZLOG_ERROR("PSZ parent entityId: '%s', name: '%s'", parentEntityId.ToString().c_str(), parentEntityName.c_str());

        auto descendants = transform->GetAllDescendants();
        AZLOG_ERROR("PSZ num of descendants: '%lu'", descendants.size());
        for (const auto &descEntityId : descendants) {
            AZStd::string name;
            AZ::ComponentApplicationBus::BroadcastResult(name, &AZ::ComponentApplicationRequests::GetEntityName, descEntityId);
            AZLOG_ERROR("PSZ desc entityId: '%s', name: '%s'", descEntityId.ToString().c_str(), name.c_str());
        }

        // every time this component activates?
        if (AzFramework::TransformComponent* transformComponent = GetEntity()->FindComponent<AzFramework::TransformComponent>())
        {
            //transformComponent->BindChildChangedEventHandler(m_childChangedHandler);
        }
        else
        {
            AZLOG_ERROR("PSZ NO TRANSFORM COMPONENT!!!");
        }
        transform->BindChildChangedEventHandler(m_childChangedHandler);

        if (auto* ros2FrameEditorComponent = GetEntity()->FindComponent<ROS2::ROS2FrameEditorComponent>())
        {
            AZLOG_ERROR("PSZ got ROS2FrameEditoComponent");
            /*const auto thisJointName = ros2FrameEditorComponent->GetJointName();
            AZLOG_ERROR("PSZ this frame name: '%s'", thisJointName.GetCStr());
            const auto frameChildren = ros2FrameEditorComponent->GetFrameChildren();
            printChildren(frameChildren);
            AZStd::vector<AZ::EntityId> frameChildrenVec(frameChildren.begin(), frameChildren.end());
            int depth = 20;
            while (!frameChildrenVec.empty() && depth) {
                AZStd::vector<AZ::EntityId> grandchildrenVec;
                for (const auto& frameChildId : frameChildrenVec) {
                    AZ::Name jointName;
                    ROS2::ROS2FrameComponentBus::EventResult(jointName, frameChildId, &ROS2::ROS2FrameComponentBus::Events::GetJointName);
                    AZLOG_ERROR("PSZ frame child joint name: '%s'", jointName.GetCStr());
                    m_childrenJointNames.push_back(jointName.GetStringView());
                    const auto grandChildrenSet = ROS2FrameSystemInterface::Get()->GetChildrenEntityId(frameChildId);
                    printChildren(grandChildrenSet);
                    grandchildrenVec.append_range(grandChildrenSet);
                }
                frameChildrenVec.swap(grandchildrenVec);
                --depth;
            }*/
            AZ::SystemTickBus::QueueFunction(&ROS2::JointsManipulationEditorComponent::recursiveGetFrameChildren, this, GetEntityId());
            //recursiveGetFrameChildren(GetEntityId());

            //TODO print m_childrenJointNames
            AZStd::string jointNamesStr = "";
            for (const auto& jointName : m_childrenJointNames) {
                jointNamesStr += AZStd::string{jointName} + " ";
            }
            //AZLOG_ERROR("PSZ all frame children joint names: '[%s]', remaining depth is: %d", jointNamesStr.c_str(), depth);
            AZLOG_ERROR("PSZ all frame children joint names: '[%s]'", jointNamesStr.c_str());
        }
        else
        {
            AZLOG_ERROR("PSZ NO ROS2FrameEditoComponent :(");
        }
    }

    void JointsManipulationEditorComponent::Deactivate()
    {
        auto descendants = GetTransform()->GetAllDescendants();
        AZLOG_ERROR("PSZ [Deactivate] num of descendants: '%lu'", descendants.size());
        for (const auto &descEntityId : descendants) {
            AZStd::string name;
            AZ::ComponentApplicationBus::BroadcastResult(name, &AZ::ComponentApplicationRequests::GetEntityName, descEntityId);
            AZLOG_ERROR("PSZ [Deactivate] desc entityId: '%s', name: '%s'", descEntityId.ToString().c_str(), name.c_str());
        }

        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    void JointsManipulationEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<JointsManipulationComponent>(m_jointStatePublisherConfiguration, m_initialPositions, m_positionCommandTopic);
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
                ->Version(1)
                ->Field("JointStatePublisherConfiguration", &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration)
                ->Field("Initial positions", &JointsManipulationEditorComponent::m_initialPositions)
                ->Field("Position command topic", &JointsManipulationEditorComponent::m_positionCommandTopic);

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
                        "Initial positions of all the joints")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_positionCommandTopic,
                        "Position command topic",
                        "Topic on which position commands are received");
            }
        }
    }

    void JointsManipulationEditorComponent::OnChildChanged(AZ::ChildChangeType type, AZ::EntityId child)
    {
        AZStd::string entityName;
        AZ::ComponentApplicationBus::BroadcastResult(entityName, &AZ::ComponentApplicationRequests::GetEntityName, child);
        switch (type)
        {
            case AZ::ChildChangeType::Added:
                AZLOG_ERROR("PSZ OnChildChanged ADDED childId: '%s', name: '%s'", child.ToString().c_str(), entityName.c_str());
                m_childrenEntityNames.push_back(std::move(entityName));
                break;
            case AZ::ChildChangeType::Removed:
                AZLOG_ERROR("PSZ OnChildChanged REMOVED childId: '%s', name: '%s'", child.ToString().c_str(), entityName.c_str());
                //TODO
                break;
            default:
                ;
        }
    }
} // namespace ROS2
