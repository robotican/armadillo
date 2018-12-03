/*******************************************************************************
* Copyright (c) 2018, RoboTICan, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/* Author: Elchay Rauper*/


#ifndef ROBOTICAN_COMMON_GAZEBOGRASPGRIPPER_H_H
#define ROBOTICAN_COMMON_GAZEBOGRASPGRIPPER_H_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <stdio.h>

namespace gazebo {

/**
 * \brief Helper class for GazeboGraspFix which holds information for one arm.
 * Attaches /detaches objects to the palm of this arm.
 *
 * \author Jennifer Buehler
 */
    class GazeboGraspGripper {
    public:
        GazeboGraspGripper();
        GazeboGraspGripper(const GazeboGraspGripper& o);
        virtual ~GazeboGraspGripper();

        /**
         *
         * \param disableCollisionsOnAttach when an object is attached, collisions with it will be disabled. This is useful
         *      if the robot then still keeps wobbling.
         */
        bool Init(physics::ModelPtr& _model,
                  const std::string& _gripperName,
                  const std::string& palmLinkName,
                  const std::vector<std::string>& fingerLinkNames,
                  bool _disableCollisionsOnAttach,
                  std::map<std::string, physics::CollisionPtr>& _collisions);

        const std::string& getGripperName() const;

        /**
         * Has the link name (URDF)
         */
        bool hasLink(const std::string& linkName) const;

        /**
         * Has the collision link name (Gazebo collision element name)
         */
        bool hasCollisionLink(const std::string& linkName) const;

        bool isObjectAttached() const;

        const std::string& attachedObject() const;

        /**
         * \param gripContacts contact forces on the object sorted by the link name colliding.
         */
        bool HandleAttach(const std::string& objName);
        void HandleDetach(const std::string& objName);

    private:

        physics::ModelPtr model;

        // name of the gripper
        std::string gripperName;

        // names of the gripper links
        std::vector<std::string> linkNames;
        // names and Collision objects of the collision links in Gazebo (scoped names)
        // Not necessarily equal names and size to linkNames.
        std::map<std::string, physics::CollisionPtr> collisionElems;

        physics::JointPtr fixedJoint;

        physics::LinkPtr palmLink;

        // when an object is attached, collisions with it may be disabled, in case the
        // robot still keeps wobbling.
        bool disableCollisionsOnAttach;

        // flag holding whether an object is attached. Object name in \e attachedObjName
        bool attached;
        // name of the object currently attached.
        std::string attachedObjName;
    };

}

#endif //ROBOTICAN_COMMON_GAZEBOGRASPGRIPPER_H_H
