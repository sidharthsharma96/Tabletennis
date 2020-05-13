#include <ros/ros.h>
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "table_tennis/SolveIKPose.h"
#include "table_tennis/SolveIKVelocity.h"
#include "table_tennis/SolveFKPose.h"

namespace table_tennis
{
    class IKSolver
    {
    public:
        IKSolver(ros::NodeHandle *node, KDL::Tree tree)
        {
            this->node = node;

            KDL::Chain chain;
            tree.getChain("iiwa_link_0", "tool_link_ee", chain);
            this->chain = chain;

            ROS_INFO("Initializing IKPoseSolver service...");
            this->ikPoseService = node->advertiseService("/table_tennis/ik/position", &IKSolver::solveIKPose, this);

            ROS_INFO("Initializing IKVelocitySolver service...");
            this->ikVelocityService = node->advertiseService("/table_tennis/ik/velocity", &IKSolver::solveIKVel, this);

            ROS_INFO("Initializing FKPoseSolver service...");
            this->fkPoseService = node->advertiseService("/table_tennis/fk/pose", &IKSolver::solveFKPose, this);

            ROS_INFO("Ready.");
        }

        ~IKSolver()
        {
            this->node->shutdown();
        }

        bool solveIKPose(table_tennis::SolveIKPoseRequest &req,
                         table_tennis::SolveIKPoseResponse &res)
        {
            ROS_INFO("Solving IK Pose...");

            ROS_INFO("Initializing solver...");
            KDL::ChainIkSolverPos_LMA posSolver(this->chain);
            std::vector<double> initialstate = req.initialState;

            KDL::JntArray jntArray = KDL::JntArray(7);
            jntArray.data = Eigen::Map<const Eigen::VectorXd>(initialstate.data(), initialstate.size());

            ROS_INFO("Parsing rotation...");
            KDL::Vector rotx = KDL::Vector(req.rotx[0], req.rotx[1], req.rotx[2]);
            KDL::Vector roty = KDL::Vector(req.roty[0], req.roty[1], req.roty[2]);
            KDL::Vector rotz = KDL::Vector(req.rotz[0], req.rotz[1], req.rotz[2]);

            ROS_INFO("Parsing posiion...");
            KDL::Vector targetpos = KDL::Vector(req.x, req.y, req.z);
            KDL::Rotation targetrot = KDL::Rotation(rotx, roty, rotz);

            ROS_INFO("Generating goal...");
            KDL::Frame targetFrame = KDL::Frame(targetrot, targetpos);

            KDL::JntArray jntSol = KDL::JntArray(7);
            ROS_INFO("[%f, %f, %f]; [[%f, %f, %f], [%f, %f, %f], [%f, %f, %f]]",
                        targetpos.data[0], targetpos.data[1], targetpos.data[2],
                        rotx.data[0], roty.data[0], rotz.data[0],
                        rotx.data[1], roty.data[1], rotz.data[1],
                        rotx.data[2], roty.data[2], rotz.data[2]);

            ROS_INFO("Running solver...");
            int result = posSolver.CartToJnt(jntArray, targetFrame, jntSol);
            ROS_INFO("Result: %d", result);

            for (int i = 0; i < jntSol.rows() * jntSol.columns(); i++)
            {
                ROS_INFO("%f", jntSol.data[i]);
            }

            std::vector<double> sol(jntSol.data.data(), jntSol.data.data() + jntSol.data.rows() * jntSol.data.cols());

            res.solution = sol;
            res.error = result;

            return true;
        }

        bool solveIKVel(table_tennis::SolveIKVelocityRequest &req,
                        table_tennis::SolveIKVelocityResponse &res)
        {
            ROS_INFO("Solving IK Velocity...");
            KDL::ChainIkSolverVel_pinv velSolver(this->chain);
            std::vector<double> initialstate = req.initialState;

            KDL::JntArray jntArray = KDL::JntArray(7);
            jntArray.data = Eigen::Map<const Eigen::VectorXd>(initialstate.data(), initialstate.size());

            KDL::Vector linVel = KDL::Vector(req.linear[0], req.linear[1], req.linear[2]);
            KDL::Vector rotVel = KDL::Vector(req.angular[0], req.angular[1], req.angular[2]);
            KDL::Twist targetTwist = KDL::Twist(linVel, rotVel);

            KDL::JntArray jntSol = KDL::JntArray(7);

            int result = velSolver.CartToJnt(jntArray, targetTwist, jntSol);
            ;
            ROS_INFO("Result: %d", result);

            for (int i = 0; i < jntSol.rows() * jntSol.columns(); i++)
            {
                ROS_INFO("%f", jntSol.data[i]);
            }

            std::vector<double> sol(jntSol.data.data(), jntSol.data.data() + jntSol.data.rows() * jntSol.data.cols());

            res.solution = sol;
            res.error = result;

            return true;
        }

        bool solveFKPose(table_tennis::SolveFKPoseRequest &req,
                         table_tennis::SolveFKPoseResponse &res)
        {
            ROS_INFO("FK Pose request received.");
            KDL::ChainFkSolverPos_recursive posSolver(this->chain);

            ROS_INFO("Joint positions:");
            std::vector<double> jointPositions = req.jointPositions;
            for (int i = 0; i < jointPositions.size(); ++i)
            {
                ROS_INFO("  %f", jointPositions[i]);
            }

            KDL::JntArray jntArray = KDL::JntArray(7);
            jntArray.data = Eigen::Map<const Eigen::VectorXd>(jointPositions.data(), jointPositions.size());

            KDL::Frame frame = KDL::Frame();

            int result = posSolver.JntToCart(jntArray, frame);
            res.error = result;

            std::vector<double> position = std::vector<double> {
                frame.p.data[0],
                frame.p.data[1],
                frame.p.data[2]
            };

            std::vector<double> rotx = std::vector<double> {
                frame.M.UnitX().data[0],
                frame.M.UnitX().data[1],
                frame.M.UnitX().data[2]
            };
            
            std::vector<double> roty = std::vector<double> {
                frame.M.UnitY().data[0],
                frame.M.UnitY().data[1],
                frame.M.UnitY().data[2]
            };

            std::vector<double> rotz = std::vector<double> {
                frame.M.UnitZ().data[0],
                frame.M.UnitZ().data[1],
                frame.M.UnitZ().data[2]
            };

            res.position = position;
            res.rotx = rotx;
            res.roty = roty;
            res.rotz = rotz;

            ROS_INFO("Solved position: {%f, %f, %f}", position[0], position[1], position[2]);
            ROS_INFO("Solved rotx: {%f, %f, %f}", rotx[0], rotx[1], rotx[2]);
            ROS_INFO("Solved roty: {%f, %f, %f}", roty[0], roty[1], roty[2]);
            ROS_INFO("Solved rotz: {%f, %f, %f}", rotz[0], rotz[1], rotz[2]);

            return true;
        }

    private:
        ros::NodeHandle *node;
        ros::ServiceServer ikPoseService;
        ros::ServiceServer ikVelocityService;
        ros::ServiceServer fkPoseService;

        KDL::Chain chain;
    };
} // namespace table_tennis

int main(int argc, char **argv)
{
    ROS_INFO("Starting Inverse Kinematics solver...");
    ROS_INFO("Building robot tree from param server...");
    ros::init(argc, argv, "ik_solver");
    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param("/robot_description", robot_desc_string, std::string());
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_desc_string, tree))
    {
        ROS_INFO("Failed to construct kdl tree");
        return 0;
    }
    else
    {
        ROS_INFO("Build chain of %d segments", tree.getNrOfSegments());
    }

    KDL::SegmentMap::const_iterator it;
    for (it = tree.getSegments().begin(); it != tree.getSegments().end(); it++)
    {
        ROS_INFO("Segment %s", it->second.segment.getName().c_str());
    }

    table_tennis::IKSolver solver(&node, tree);

    ros::spin();

    return 0;
}