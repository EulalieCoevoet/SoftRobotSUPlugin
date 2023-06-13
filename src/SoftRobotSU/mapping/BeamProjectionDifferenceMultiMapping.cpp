/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#define SOFTROBOTSU_MAPPING_BEAMPROJECTIONDIFFERENCEMULTIMAPPING_CPP

#include <SoftRobotSU/mapping/BeamProjectionDifferenceMultiMapping.inl>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/ObjectFactory.h>

namespace softrobotsu::mapping
{

using namespace sofa::defaulttype;

// Register in the Factory
int BeamProjectionDifferenceMultiMappingClass = sofa::core::RegisterObject("Computes the difference between given points of a model and their projection on a beam.")
//        .add< BeamProjectionDifferenceMultiMapping< Vec3Types, Rigid3Types, Vec3Types > >()
        .add< BeamProjectionDifferenceMultiMapping< Rigid3Types, Rigid3Types, Rigid3Types > >();

} // namespace
