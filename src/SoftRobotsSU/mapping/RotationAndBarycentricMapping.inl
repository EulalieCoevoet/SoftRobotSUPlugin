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

#include <SoftRobotsSU/mapping/RotationAndBarycentricMapping.h>

#include <sofa/component/topology/container/grid/RegularGridTopology.h>
#include <SofaBaseTopology/SparseGridTopology.h>
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <SofaBaseTopology/PointSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <SofaBaseTopology/QuadSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/HexahedronSetTopologyContainer.h>

#include<SofaBaseMechanics/BarycentricMappers/BarycentricMapperMeshTopology.h>
#include<SofaBaseMechanics/BarycentricMappers/BarycentricMapperRegularGridTopology.h>
#include<SofaBaseMechanics/BarycentricMappers/BarycentricMapperSparseGridTopology.h>
#include<SofaBaseMechanics/BarycentricMappers/BarycentricMapperEdgeSetTopology.h>
#include<SofaBaseMechanics/BarycentricMappers/BarycentricMapperTriangleSetTopology.h>
#include<SofaBaseMechanics/BarycentricMappers/BarycentricMapperQuadSetTopology.h>
#include<SofaBaseMechanics/BarycentricMappers/BarycentricMapperTetrahedronSetTopology.h>
#include<SofaBaseMechanics/BarycentricMappers/BarycentricMapperHexahedronSetTopology.h>

namespace sofa::component::mapping {

using sofa::component::topology::container::grid::RegularGridTopology;
using sofa::core::objectmodel::ComponentState;
using topology::HexahedronSetTopologyContainer;
using topology::TetrahedronSetTopologyContainer;
using topology::QuadSetTopologyContainer;
using topology::TriangleSetTopologyContainer;
using topology::EdgeSetTopologyContainer;
using topology::PointSetTopologyContainer;
using helper::WriteAccessor;
using helper::ReadAccessor;

using sofa::core::objectmodel::New;

template <typename T, typename V>
static bool is_a(const V * topology) {
    return dynamic_cast<const T *>(topology) != nullptr;
}

template <class In1, class In2, class Out>
RotationAndBarycentricMapping<In1, In2, Out>::RotationAndBarycentricMapping()
    : Inherit()
    , d_axis(initData(&d_axis, "axis", "Axis of rotation"))
    , d_center(initData(&d_center, "center", "Center of rotation"))
    , d_topologyFEM(initLink("topologyFEM", ""))
    , d_topologyCollision(initLink("topologyCollision", ""))
{
    WriteAccessor<Data<vector<Vec3>>> axis = d_axis;
    WriteAccessor<Data<vector<Vec3>>> center = d_center;
    axis.resize(1);
    axis[0] = Vec3(1,0,0);
    center.resize(1);
    center[0] = Vec3(0,0,0);
}

template <class In1, class In2, class Out>
void RotationAndBarycentricMapping<In1, In2, Out>::init()
{
    d_componentState.setValue(ComponentState::Invalid);
    Inherit::init();

    if(fromModels1.size()==0||fromModels2.size()==0||toModels.size()==0){
        msg_error() << "Could not find the mechanical states.";
        return;
    }

    populateTopologies();

    if (m_mapper == nullptr) // try to create a mapper according to the topology of the In models
        createMapperFromTopology();

    if (m_mapper == nullptr){
        msg_error() << "No compatible input topology for the FEM found. Make sure the input topology ('" << d_topologyFEM.getPath()
                    << "') is a class derived from BaseMeshTopology.";
        return;
    }

    m_mapper->init(((const core::State<Out> *)toModels[0])->read(core::ConstVecCoordId::restPosition())->getValue(),
                   ((const core::State<In1> *)fromModels1[0])->read(core::ConstVecCoordId::restPosition())->getValue());

    d_componentState.setValue(ComponentState::Valid) ;
}

template <class In1, class In2, class Out>
void RotationAndBarycentricMapping<In1, In2, Out>::reinit()
{
    if(m_mapper){
        m_mapper->clear();
        m_mapper->init(((const core::State<Out> *)toModels[0])->read(core::ConstVecCoordId::restPosition())->getValue(),
                       ((const core::State<In1> *)fromModels1[0])->read(core::ConstVecCoordId::restPosition())->getValue());
    }
}

template <class In1, class In2, class Out>
void RotationAndBarycentricMapping<In1, In2, Out>::handleTopologyChange (Topology* t)
{
    // Foward topological modifications to the mapper
    if (m_mapper.get()){
        m_mapper->processTopologicalChanges(
                                            ((const core::State<Out> *)this->toModels[0])->read(core::ConstVecCoordId::restPosition())->getValue(),
                                            ((const core::State<In1> *)this->fromModels1[0])->read(core::ConstVecCoordId::restPosition())->getValue(),
                                            t);
    }
}

template <class In1, class In2, class Out>
void RotationAndBarycentricMapping<In1, In2, Out>::populateTopologies()
{
    if (!d_topologyFEM.get()) {
        BaseMeshTopology * topologyFEM = nullptr;
        fromModels1[0]->getContext()->get(topologyFEM);
        if (topologyFEM)
            d_topologyFEM.set(topologyFEM);
        else
            msg_error() << "No input topology found. Consider setting the 'topologyFEM' attribute. ";
    }

    if (!d_topologyCollision.get()) {
        BaseMeshTopology * topologyCollision = nullptr;
        toModels[0]->getContext()->get(topologyCollision);
        if (topologyCollision)
            d_topologyCollision.set(topologyCollision);
        else
            msg_error() << "No input topology found. Consider setting the 'topologyCollision' attribute. ";
    }
}

template <class In1, class In2, class Out>
void RotationAndBarycentricMapping<In1, In2, Out>::createMapperFromTopology()
{
    using sofa::core::behavior::BaseMechanicalState;
    using sofa::core::topology::TopologyContainer;
    using sofa::component::topology::SparseGridTopology;

    using RegularGridMapper = BarycentricMapperRegularGridTopology< In1, Out >;
    using SparseGridMapper = BarycentricMapperSparseGridTopology< In1, Out >;
    using MeshMapper = BarycentricMapperMeshTopology< In1, Out >;
    using HexahedronSetMapper = BarycentricMapperHexahedronSetTopology<In1, Out>;
    using TetrahedronSetMapper = BarycentricMapperTetrahedronSetTopology<In1, Out>;
    using QuadSetMapper = BarycentricMapperQuadSetTopology<In1, Out>;
    using TriangleSetMapper = BarycentricMapperTriangleSetTopology<In1, Out>;
    using EdgeSetMapper = BarycentricMapperEdgeSetTopology<In1, Out>;

    auto topologyFEM = d_topologyFEM.get();
    auto topologyCollision = dynamic_cast<PointSetTopologyContainer*> (d_topologyCollision.get());

    if (!topologyFEM) // Topology collision container could be null, as most of the mappers do not need it.
        return;

    m_mapper = nullptr;
    RegularGridTopology* rgt = nullptr;
    SparseGridTopology* sgt = nullptr;

    if (is_a<RegularGridTopology>(topologyFEM) ) {
        rgt = dynamic_cast<RegularGridTopology*>(topologyFEM);
        if (rgt->hasVolume()) {
            msg_info() << "Creating RegularGridMapper";
            m_mapper = New<RegularGridMapper>(rgt, topologyCollision);
        }
        goto end;
    }

    if (is_a<SparseGridTopology>(topologyFEM) ) {
        sgt = dynamic_cast<SparseGridTopology*>(topologyFEM);
        if (sgt->hasVolume()) {
            msg_info() << "Creating SparseGridMapper";
            m_mapper = New<SparseGridMapper>(sgt, topologyCollision);
        }
        goto end;
    }

    if (is_a<HexahedronSetTopologyContainer>(topologyFEM)) {
        msg_info() << "Creating HexahedronSetMapper";
        m_mapper = New<HexahedronSetMapper>(
            dynamic_cast<HexahedronSetTopologyContainer*>(topologyFEM), topologyCollision);
        goto end;
    }

    if (is_a<TetrahedronSetTopologyContainer>(topologyFEM)) {
        msg_info() << "Creating TetrahedronSetMapper";
        m_mapper = sofa::core::objectmodel::New<TetrahedronSetMapper >(
            dynamic_cast<TetrahedronSetTopologyContainer*>(topologyFEM), topologyCollision);
        goto end;
    }

    if (is_a<QuadSetTopologyContainer>(topologyFEM)) {
        msg_info() << "Creating QuadSetMapper";
        m_mapper = sofa::core::objectmodel::New<QuadSetMapper >(
            dynamic_cast<QuadSetTopologyContainer*>(topologyFEM), topologyCollision);
        goto end;
    }

    if (is_a<TriangleSetTopologyContainer>(topologyFEM)) {
        msg_info() << "Creating TriangleSetMapper";
        m_mapper = sofa::core::objectmodel::New<TriangleSetMapper >(
            dynamic_cast<TriangleSetTopologyContainer*>(topologyFEM), topologyCollision);
        goto end;
    }

    if (is_a<EdgeSetTopologyContainer>(topologyFEM)) {
        msg_info() << "Creating EdgeSetMapper";
        m_mapper = sofa::core::objectmodel::New<EdgeSetMapper >(
            dynamic_cast<EdgeSetTopologyContainer*>(topologyFEM), topologyCollision);
        goto end;
    }

    m_mapper = sofa::core::objectmodel::New<MeshMapper>(topologyFEM, topologyCollision);

end:
    if (m_mapper) {
        this->addSlave(m_mapper.get());
    }
}


template <class In1, class In2, class Out>
void RotationAndBarycentricMapping<In1, In2, Out>::apply(
                                                        const MechanicalParams* mparams, const vector<Data<typename Out::VecCoord>*>& dataVecOutPos,
                                                        const vector<const Data<typename In1::VecCoord>*>& dataVecIn1Pos,
                                                        const vector<const Data<typename In2::VecCoord>*>& dataVecIn2Pos)
{
    SOFA_UNUSED(mparams);

    if(m_mapper == nullptr)
        return;

    const Data<typename In1::VecCoord>* in1 = dataVecIn1Pos[0];
    const Data<typename In2::VecCoord>* in2 = dataVecIn2Pos[0];
    WriteAccessor<Data<typename Out::VecCoord>> out = *dataVecOutPos[0];
    WriteAccessor<Data<typename Out::VecCoord>> outRest = *((core::State<Out> *)toModels[0])->write(core::VecCoordId::restPosition());

    // Store position before rotation, for velocities
    m_mapper->resize(toModels[0]);
    m_mapper->apply(out.wref(), in1->getValue());
    m_outRotationVelocities.resize(out.size());
    for(size_t i=0; i<out.size(); i++)
        m_outRotationVelocities[i] = -out[i]; // -x_i

    // Apply rotation on rest position
    // One rotation for now
    Real theta = in2->getValue()[0][0];
    Vec3 center = d_center.getValue()[0];
    Vec3 axis = d_axis.getValue()[0];
    axis.normalize();

    Quat q = Quat(axis[0]*(Real)sin(theta/2.),
                  axis[1]*(Real)sin(theta/2.),
                  axis[2]*(Real)sin(theta/2.),
                  cos(theta/2.));

    applyRotation(q, center, outRest.wref());

    // Reinit mapping
    reinit();

    // Apply FEM deformation
    m_mapper->apply(out.wref(), in1->getValue());

    // Compute velocity directions from rotation
    for(size_t i=0; i<out.size(); i++)
        m_outRotationVelocities[i] += out[i]; // +x_i+1
}

template <class In1, class In2, class Out>
void RotationAndBarycentricMapping<In1, In2, Out>::applyJ(
                                                        const MechanicalParams* mparams, const vector<Data<typename Out::VecDeriv>*>& dataVecOutVel,
                                                        const vector<const Data<typename In1::VecDeriv>*>& dataVecIn1Vel,
                                                        const vector<const Data<typename In2::VecDeriv>*>& dataVecIn2Vel)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(dataVecIn2Vel);

    if(m_mapper == nullptr)
        return;

    const Data<typename In1::VecDeriv>* in1Vel = dataVecIn1Vel[0];
    const typename In2::VecCoord& in2Pos = ((core::State<In2> *)fromModels2[0])->read(core::ConstVecCoordId::position())->getValue();
    WriteAccessor<Data<typename Out::VecDeriv>> outVel = *dataVecOutVel[0];
    const typename Out::VecCoord& outRestPos = ((core::State<Out> *)toModels[0])->read(core::ConstVecCoordId::restPosition())->getValue();

    // Apply FEM velocities
    m_mapper->applyJ(outVel.wref(), in1Vel->getValue());

    // Compute angular velocity
    Real theta = in2Pos[0][0]; // One rotation for now
    Vec3 center = d_center.getValue()[0];
    Vec3 axis = d_axis.getValue()[0]; axis.normalize();
    Real dt = this->getContext()->getDt();
    Vec3 omega = theta/dt*axis;

    // Compute rigid angular velocities on rest position and apply amplitude to deformed model
    for(size_t i=0; i<outVel.size(); i++){
        m_outRotationVelocities[i].normalize();
        m_outRotationVelocities[i] *= cross(omega, outRestPos[i]-center).norm();
    }

    // Add rigid angular velocities
    for(size_t i=0; i<outVel.size(); i++)
        outVel[i] += m_outRotationVelocities[i];
}

template <class In1, class In2, class Out>
void RotationAndBarycentricMapping<In1, In2, Out>::applyJT(
                                                        const MechanicalParams* mparams, const vector<Data<typename In1::VecDeriv>*>& dataVecOut1Force,
                                                        const vector< Data<typename In2::VecDeriv>*>& dataVecOut2Force,
                                                        const vector<const Data<typename Out::VecDeriv>*>& dataVecInForce)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(dataVecOut2Force);

    if(m_mapper == nullptr)
        return;

    const Data<typename Out::VecDeriv>* in = dataVecInForce[0];
    WriteAccessor<Data<typename In1::VecDeriv>> out1 = *dataVecOut1Force[0];
    m_mapper->applyJT(out1.wref(), in->getValue());
}

template <class In1, class In2, class Out>
void RotationAndBarycentricMapping<In1, In2, Out>::applyJT(const ConstraintParams* cparams, const vector<Data<typename In1::MatrixDeriv>*>& dataMatOut1 ,
                                                        const vector< Data<typename In2::MatrixDeriv>*>&  dataMatOut2 ,
                                                        const vector<const Data<typename Out::MatrixDeriv>*>& dataMatIn)
{
    SOFA_UNUSED(cparams);
    SOFA_UNUSED(dataMatOut2);

    if(m_mapper == nullptr)
        return;

    const Data<typename Out::MatrixDeriv>* in = dataMatIn[0];
    WriteAccessor<Data<typename In1::MatrixDeriv>> out1 = *dataMatOut1[0];
    m_mapper->applyJT(out1.wref(), in->getValue());
}


template <class In1, class In2, class Out>
void RotationAndBarycentricMapping<In1, In2, Out>::applyDJT(
                                                        const MechanicalParams* mparams,
                                                        MultiVecDerivId inForce,
                                                        ConstMultiVecDerivId outForce)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(inForce);
    SOFA_UNUSED(outForce);
}

template <class In1, class In2, class Out>
void RotationAndBarycentricMapping<In1, In2, Out>::applyRotation(const Quat<SReal> &q, const Vec3 &center, typename Out::VecCoord& out)
{
    for (unsigned int i = 0; i < out.size(); i++)
    {
        Vec3 pos;
        Out::get(pos[0], pos[1], pos[2], out[i]);
        pos -= center;
        Vec3 newposition = q.rotate(pos);
        newposition += center;
        Out::set(out[i], newposition[0], newposition[1], newposition[2]);
    }
}

}
