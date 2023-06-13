/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
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
#define SOFTROBOSU_MAPPING_ROTATIONANDBARYCENTRICMAPPING_CPP

#include <sofa/core/ObjectFactory.h>

#include <SoftRobotSU/mapping/RotationAndBarycentricMapping.inl>

namespace softrobotsu::mapping
{

using namespace sofa::defaulttype;

int RotationAndBarycentricMappingClass = sofa::core::RegisterObject(
                                                            "Applies a rigid rotation (input2 Vec1) on the rest position of the output (around the given axis) and "
                                                            "applies a barycentric mapping between input1 and output.")
    .add< RotationAndBarycentricMapping< Vec3Types, Vec1Types, Vec3Types > >(true)
;

template class RotationAndBarycentricMapping< Vec3Types, Vec1Types, Vec3Types >;

}
