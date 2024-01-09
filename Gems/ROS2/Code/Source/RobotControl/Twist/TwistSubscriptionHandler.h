/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <ROS2/Communication/SubscriptionHandler.h>
#include <geometry_msgs/msg/twist.hpp>

namespace ROS2
{
    class TwistSubscriptionHandler : public SubscriptionHandler<geometry_msgs::msg::Twist>
    {
    private:
        void ExecuteUponMessage(const geometry_msgs::msg::Twist& message) override;
    };
} // namespace ROS2
