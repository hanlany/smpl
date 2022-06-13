#include <smpl_urdf_robot_model/urdf_robot_model.h>

// project includes
#include <smpl_urdf_robot_model/robot_state_bounds.h>
#include <smpl_urdf_robot_model/robot_model.h>
#include <iostream>
namespace smpl {
namespace urdf {

bool Init(
    URDFRobotModel* model,
    const RobotModel* robot_model,
    const std::vector<std::string>* planning_joint_names,
    int num_threads)
{
    URDFRobotModel urdf_model;
    std::vector<std::string> planning_variables;

    for (auto& joint_name : *planning_joint_names) {
        auto* joint = GetJoint(robot_model, &joint_name);
        if (joint == NULL) {
            return false;
        }
        for (auto& variable : Variables(joint)) {
            planning_variables.push_back(*GetName(&variable));

            // cache variable limits
            auto* limits = GetVariableLimits(&variable);
            URDFRobotModel::VariableProperties props;
            props.bounded = limits->has_position_limits;

            if (((joint->type == JointType::Revolute) & (!limits->has_position_limits)) |
                ((joint->type == JointType::Planar) & (&variable == joint->vfirst + 2)))
            {
                props.continuous = true;
            } else {
                props.continuous = false;
            }

            props.min_position = limits->min_position;
            props.max_position = limits->max_position;
            props.vel_limit = limits->max_velocity;
            props.acc_limit = limits->max_effort; // TODO: hmm...
            model->vprops.push_back(props);

            // initialize mapping from planning group variable to state variable
            auto index = GetVariableIndex(robot_model, &variable);
            model->planning_to_state_variable.push_back(index);
        }
    }

    model->setPlanningJoints(planning_variables);
    model->robot_model = robot_model;

    model->robot_state_vec.resize(num_threads);
    for (int tidx = 0; tidx < num_threads; ++tidx)
        if (!InitRobotState(&model->robot_state_vec[tidx], robot_model)) 
            return false;

    return true;
}

bool SetPlanningLink(URDFRobotModel* urdf_model, const char* link_name)
{
    auto* link = GetLink(urdf_model->robot_model, link_name);
    if (link == NULL) return false;
    return SetPlanningLink(urdf_model, link);
}

bool SetPlanningLink(URDFRobotModel* urdf_model, const std::string* link_name)
{
    return SetPlanningLink(urdf_model, link_name->c_str());
}

bool SetPlanningLink(URDFRobotModel* urdf_model, const Link* link)
{
    urdf_model->planning_link = link;
    return true;
}

void SetReferenceState(URDFRobotModel* model, const double* positions)
{
    for (int tidx=0; tidx < model->robot_state_vec.size(); tidx++)
        SetVariablePositions(&model->robot_state_vec[tidx], positions);
}

static
void UpdateState(URDFRobotModel* model, const smpl::RobotState* state, int tidx)
{
    for (auto i = 0; i < model->jointVariableCount(); ++i) {
        SetVariablePosition(
                &model->robot_state_vec[tidx],
                model->planning_to_state_variable[i],
                (*state)[i]);
    }
}

auto URDFRobotModel::computeFK(const smpl::RobotState& state, int tidx)
    -> Eigen::Affine3d
{
    UpdateState(this, &state, tidx);
    UpdateLinkTransform(&this->robot_state_vec[tidx], this->planning_link);
    return *GetLinkTransform(&this->robot_state_vec[tidx], this->planning_link);
}

auto URDFRobotModel::computeFK(const smpl::RobotState& state)
    -> Eigen::Affine3d
{
    return computeFK(state, 0);
}

double URDFRobotModel::minPosLimit(int jidx) const
{
    return this->vprops[jidx].min_position;
}

double URDFRobotModel::maxPosLimit(int jidx) const
{
    return this->vprops[jidx].max_position;
}

bool URDFRobotModel::hasPosLimit(int jidx) const
{
    return this->vprops[jidx].bounded;
}

bool URDFRobotModel::isContinuous(int jidx) const
{
    return this->vprops[jidx].continuous;
}

double URDFRobotModel::velLimit(int jidx) const
{
    return this->vprops[jidx].vel_limit;
}

double URDFRobotModel::accLimit(int jidx) const
{
    return this->vprops[jidx].acc_limit;
}

bool URDFRobotModel::checkJointLimits(
    const smpl::RobotState& state,
    int tidx,
    bool verbose)
{
    UpdateState(this, &state, tidx);
    for (auto i = 0; i < this->jointVariableCount(); ++i) {
        auto* var = GetVariable(this->robot_model, this->planning_to_state_variable[i]);
        if (!SatisfiesBounds(&this->robot_state_vec[tidx], var)) {
            return false;
        }
    }
    return true;
}

auto URDFRobotModel::getExtension(size_t class_code) -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<smpl::RobotModel>()) return this;
    if (class_code == smpl::GetClassCode<smpl::ForwardKinematicsInterface>()) return this;
    return NULL;
}

} // namespace urdf
} // namespace smpl

