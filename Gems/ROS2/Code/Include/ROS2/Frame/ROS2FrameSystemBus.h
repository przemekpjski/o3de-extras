/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/Policies.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>

namespace ROS2
{
    class ROS2FrameSystemRequests
    {
    public:
        AZ_RTTI(ROS2FrameSystemRequests, "{24fe4584-0499-4a37-bc1a-00ca04bd22f5}");

        //! Registers the ROS2FrameEditorComponent into the system. All ROS2FrameEditorComponents should
        //! register using this function during activation
        //! @param frameEntityId entityId containing the frame to register.
        virtual void RegisterFrame(AZ::EntityId frameEntityId) = 0;

        //! Unregister the ROS2FrameEditorComponent into the system. All ROS2FrameEditorComponents should
        //! unregister using this function during deactivation.
        //! @param frameEntityId entityId containing the frame to unregister.
        virtual void UnregisterFrame(AZ::EntityId frameEntityId) = 0;

        //! Move the frame in the entity tree.
        //! Moves the frame entity and updates all namespaces.
        //! @param frameEntityId entityId of the frame to move.
        //! @param newParent entityId of the new parent of the moved frame (does not need to be a entity
        //! containing a frame component).
        virtual void MoveFrame(AZ::EntityId frameEntityId, AZ::EntityId newParent) = 0;

        //! Notify the system entity about frames configuration change.
        //! @param frameEntityId entityId of the frame components entity that has changed its configuration.
        virtual void NotifyChange(AZ::EntityId frameEntityId) = 0;

        //! Check if the frame is the highest frame in the entity tree.
        //! @param frameEntityId entityId of the frame to check.
        //! @return boolean value of the check. True for top level.
        virtual bool IsTopLevel(AZ::EntityId frameEntityId) const = 0;

        //! Find the parent frame of the entity.
        //! @param frameEntityId entityId of the frame to check.
        //! @return entityId of the parent frame or an invalid entityId if the frame is top level.
        virtual AZ::EntityId GetParentEntityId(AZ::EntityId frameEntityId) const = 0;
    };

    class ROS2FrameSystemBusTraits : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusHandlerPolicy AddressPolicy = AZ::EBusHandlerPolicy::Single;
    };

    using ROS2FrameSystemInterface = AZ::Interface<ROS2FrameSystemRequests>;
    using ROS2FrameSystemBus = AZ::EBus<ROS2FrameSystemRequests>;
} // namespace ROS2
