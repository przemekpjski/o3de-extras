/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Name/Name.h>

#include "JointStatePublisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <rclcpp/subscription.hpp>

namespace ROS2
{
    //! Component responsible for controlling a hierarchical system of joints such as robotic arm with Articulations or Hinge Joints.
    //! This manipulator component uses simple joint position interface. For trajectory control, see JointsTrajectoryComponent.
    class JointsManipulationComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public JointsManipulationRequestBus::Handler
    {
    public:
        JointsManipulationComponent();
        JointsManipulationComponent(
            const PublisherConfiguration& configuration,
            const AZStd::vector<AZStd::pair<AZStd::string, JointPosition>>& initialPositions,
            const AZStd::string& positionCommandZTopic
        );
        ~JointsManipulationComponent() = default;
        AZ_COMPONENT(JointsManipulationComponent, "{3da9abfc-0028-4e3e-8d04-4e4440d2e319}", AZ::Component);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

        // JointsManipulationRequestBus::Handler overrides ...
        //! @see ROS2::JointsManipulationRequestBus::GetJoints
        ManipulationJoints GetJoints() override;
        //! @see ROS2::JointsManipulationRequestBus::GetJointPosition
        AZ::Outcome<JointPosition, AZStd::string> GetJointPosition(const AZStd::string& jointName) override;
        //! @see ROS2::JointsManipulationRequestBus::GetJointVelocity
        AZ::Outcome<JointVelocity, AZStd::string> GetJointVelocity(const AZStd::string& jointName) override;
        //! @see ROS2::JointsManipulationRequestBus::GetAllJointsPositions
        JointsPositionsMap GetAllJointsPositions() override;
        //! @see ROS2::JointsManipulationRequestBus::GetAllJointsVelocities
        JointsVelocitiesMap GetAllJointsVelocities() override;
        //! @see ROS2::JointsManipulationRequestBus::GetJointEffort
        AZ::Outcome<JointEffort, AZStd::string> GetJointEffort(const AZStd::string& jointName) override;
        //! @see ROS2::JointsManipulationRequestBus::GetAllJointsEfforts
        JointsEffortsMap GetAllJointsEfforts() override;
        //! @see ROS2::JointsManipulationRequestBus::SetMaxJointEffort
        AZ::Outcome<void, AZStd::string> SetMaxJointEffort(const AZStd::string& jointName, JointEffort maxEffort);
        //! @see ROS2::JointsManipulationRequestBus::MoveJointsToPositions
        AZ::Outcome<void, AZStd::string> MoveJointsToPositions(const JointsPositionsMap& positions) override;
        //! @see ROS2::JointsManipulationRequestBus::MoveJointToPosition
        AZ::Outcome<void, AZStd::string> MoveJointToPosition(const AZStd::string& jointName, JointPosition position) override;
        //! @see ROS2::JointsManipulationRequestBus::Stop
        void Stop() override;

    private:
        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void MoveToSetPositions(float deltaTime);
        void PositionCommandCallback(std_msgs::msg::Float64MultiArray command);

        AZStd::string GetManipulatorNamespace() const;

        AZ::Outcome<JointPosition, AZStd::string> GetJointPosition(const JointInfo& jointInfo);
        AZ::Outcome<JointVelocity, AZStd::string> GetJointVelocity(const JointInfo& jointInfo);
        AZ::Outcome<JointEffort, AZStd::string> GetJointEffort(const JointInfo& jointInfo);

        AZStd::unique_ptr<JointStatePublisher> m_jointStatePublisher;
        PublisherConfiguration m_jointStatePublisherConfiguration;
        ManipulationJoints m_manipulationJoints; //!< Map of JointInfo where the key is a joint name (with namespace included)
        AZStd::vector<AZStd::pair<AZStd::string, JointPosition>>
            m_initialPositions; //!< Initial positions where the first value of each pair is joint name (without namespace included)
        //TODO std ????
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>> m_jointPositionSubscriber;
        AZStd::string m_positionCommandTopic;
    };
} // namespace ROS2
