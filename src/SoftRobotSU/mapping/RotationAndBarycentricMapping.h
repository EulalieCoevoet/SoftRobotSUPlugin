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

#pragma once

#include <sofa/core/Multi2Mapping.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/core/topology/BaseMeshTopology.h>

#include <SofaMiscMapping/config.h>
#include <sofa/component/mapping/linear/BarycentricMappers/TopologyBarycentricMapper.h>

#include <SoftRobotSU/config.h>

namespace softrobotsu::mapping
{

using sofa::defaulttype::Vec3Types;
using sofa::defaulttype::Rigid3Types;
using sofa::defaulttype::Vec1Types;
using sofa::type::Quat;

using sofa::core::MechanicalParams;
using sofa::core::ConstraintParams;
using sofa::core::MultiVecDerivId;
using sofa::core::ConstMultiVecDerivId;
using sofa::core::topology::Topology;
using sofa::core::Multi2Mapping;
using sofa::core::topology::BaseMeshTopology;

using sofa::type::vector;

/**
 * Applies a rigid rotation (input2 Vec1) on the rest position of the output (around the given axis) and applies a barycentric mapping between input1 and output.
 */
template <class TIn1, class TIn2, class TOut>
class RotationAndBarycentricMapping : public Multi2Mapping<TIn1, TIn2, TOut>
{

public:

    SOFA_CLASS(SOFA_TEMPLATE3(RotationAndBarycentricMapping, TIn1, TIn2, TOut),
               SOFA_TEMPLATE3(Multi2Mapping, TIn1, TIn2, TOut));

    typedef TIn1 In1; /// Input FEM Model Type
    typedef TIn2 In2; /// Input Rotation Model Type
    typedef TOut Out; /// Output Collision Model Type

    typedef typename TIn1::Real Real;
    typedef sofa::type::Vec<3,Real> Vec3;

    typedef Multi2Mapping<In1, In2, Out> Inherit;
    typedef sofa::component::mapping::linear::TopologyBarycentricMapper<In1, Out> Mapper; /// Barycentric mapping utils

protected:

    RotationAndBarycentricMapping();
    virtual ~RotationAndBarycentricMapping(){}

    sofa::Data<vector<Vec3>> d_axis;
    sofa::Data<vector<Vec3>> d_center;

    typename Out::VecDeriv m_outRotationVelocities;

public:

    using Inherit::fromModels1;
    using Inherit::fromModels2;
    using Inherit::toModels;
    using Inherit::d_componentState;

    sofa::SingleLink< RotationAndBarycentricMapping<In1,In2,Out>, Mapper, sofa::BaseLink::FLAG_STRONGLINK > m_mapper;
    sofa::SingleLink< RotationAndBarycentricMapping<In1,In2,Out>, BaseMeshTopology, sofa::BaseLink::FLAG_STRONGLINK > d_topologyFEM;
    sofa::SingleLink< RotationAndBarycentricMapping<In1,In2,Out>, BaseMeshTopology, sofa::BaseLink::FLAG_STRONGLINK > d_topologyCollision;

    void init() override;
    void reinit() override;

    void apply(
                const MechanicalParams* mparams, const vector<sofa::Data<typename Out::VecCoord>*>& dataVecOutPos,
                const vector<const sofa::Data<typename In1::VecCoord>*>& dataVecIn1Pos ,
                const vector<const sofa::Data<typename In2::VecCoord>*>& dataVecIn2Pos) override;
    void applyJ(
                const MechanicalParams* mparams, const vector<sofa::Data<typename Out::VecDeriv>*>& dataVecOutVel,
                const vector<const sofa::Data<typename In1::VecDeriv>*>& dataVecIn1Vel,
                const vector<const sofa::Data<typename In2::VecDeriv>*>& dataVecIn2Vel) override;
    void applyJT(
                const MechanicalParams* mparams, const vector<sofa::Data<typename In1::VecDeriv>*>& dataVecOut1Force,
                const vector< sofa::Data<typename In2::VecDeriv>*>& dataVecOut2Force,
                const vector<const sofa::Data<typename Out::VecDeriv>*>& dataVecInForce) override;
    void applyJT(const ConstraintParams* cparams, const vector<sofa::Data<typename In1::MatrixDeriv>*>& dataMatOut1 ,
                const vector< sofa::Data<typename In2::MatrixDeriv>*>&  dataMatOut2 ,
                const vector<const sofa::Data<typename Out::MatrixDeriv>*>& dataMatIn) override;
    void applyDJT(
                const MechanicalParams* mparams,
                MultiVecDerivId inForce,
                ConstMultiVecDerivId outForce) override;

    void handleTopologyChange(Topology* t) override;

private:

    void createMapperFromTopology();
    void populateTopologies();
    void applyRotation(const Quat<SReal>& q, const Vec3& center, typename Out::VecCoord &out);
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTSU_MAPPING_ROTATIONANDBARYCENTRICMAPPING_CPP)
extern template class SOFA_SOFTROBOTSU_API RotationAndBarycentricMapping< Vec3Types, Vec1Types, Vec3Types >;
#endif

}

