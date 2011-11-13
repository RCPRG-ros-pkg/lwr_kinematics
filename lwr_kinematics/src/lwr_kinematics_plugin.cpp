#include "ros/ros.h"
#include <kinematics_base/kinematics_base.h>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <map>
#include <iostream>
#define IKFAST_NO_MAIN
#ifdef IKFAST_REAL
typedef IKFAST_REAL IKReal;
#else
typedef double IKReal;
#endif


namespace lwr_arm_kinematics
{

class ik_solver_base {
public:
    virtual int solve(KDL::Frame &pose_frame, const std::vector<double> &ik_seed_state) = 0;
    virtual void getSolution(int i, std::vector<double> &solution) = 0;
    virtual void getClosestSolution(const std::vector<double> &ik_seed_state, std::vector<double> &solution) = 0;
};

template <class T> class ikfast_solver: public ik_solver_base {
public:
    typedef bool (*ik_type)(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<T>& vsolutions);
    ikfast_solver(ik_type ik,int numJoints):ik(ik),numJoints(numJoints) {}
    virtual int solve(KDL::Frame &pose_frame, const std::vector<double> &vfree) {

        solutions.clear();
        ik(pose_frame.p.data, pose_frame.M.data, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

        return solutions.size();
    }
    virtual void getSolution(int i, std::vector<double> &solution) {
        solution.clear();
        std::vector<IKReal> vsolfree(solutions[i].GetFree().size());
        solution.clear();
        solution.resize(numJoints);
        solutions[i].GetSolution(&solution[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        std::cout << "solution " << i << ":" ;
        for(int j=0; j<numJoints; ++j)
            std::cout << " " << solution[j];
        std::cout << std::endl;

        //ROS_ERROR("%f %d",solution[2],vsolfree.size());
    }

    virtual void getClosestSolution(const std::vector<double> &ik_seed_state, std::vector<double> &solution) {
        double mindist = 0;
        int minindex = -1;
        std::vector<double> sol;
        for(size_t i=0; i<solutions.size(); ++i) {
            getSolution(i,sol);
            double dist = 0;
            for(int j=0; j< numJoints; ++j) {
                double diff = ik_seed_state[j] - sol[j];
                dist += diff*diff;
            }
            if(minindex == -1 || dist<mindist) {
                minindex = i;
                mindist = dist;
            }
        }
        if(minindex >= 0) getSolution(minindex,solution);
    }



private:
    ik_type ik;
    std::vector<T> solutions;
    int numJoints;
};


namespace lwr {
#include "ikfast_lwr.cpp"
}

class IKFastKinematicsPlugin : public kinematics::KinematicsBase
{
    std::vector<std::string> joints;
    std::vector<double> jointMin;
    std::vector<double> jointMax;
    std::vector<std::string> links;
    std::map<std::string,size_t> link2index;
    std::string tip_name, root_name;
    KDL::Chain kdl_chain;
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver;
    ik_solver_base* ik_solver;
    size_t numJoints;
    std::vector<int> freeParams;

public:

    IKFastKinematicsPlugin():ik_solver(0) {}

    ~IKFastKinematicsPlugin() {
        if(ik_solver) delete ik_solver;
    }

    void fillFreeParams(int count, int *array) {
        freeParams.clear();
        for(int i=0; i<count; ++i) freeParams.push_back(array[i]);
    }

    bool initialize(std::string name) {

        ros::NodeHandle private_handle("~/"+name);

        fillFreeParams(lwr::getNumFreeParameters(),lwr::getFreeParameters());
        numJoints = lwr::getNumJoints();
        ik_solver = new ikfast_solver<lwr::IKSolution>(lwr::ik, numJoints);

        if(freeParams.size()>1) {
            ROS_FATAL("Only one free joint paramter supported!");
            return false;
        }


        urdf::Model robot_model;
        std::string xml_string;

        std::string urdf_xml,full_urdf_xml;
        private_handle.param("urdf_xml",urdf_xml,std::string("robot_description"));
        private_handle.searchParam(urdf_xml,full_urdf_xml);


    		TiXmlDocument xml;
    		ROS_DEBUG("Reading xml file from parameter server\n");
    		std::string result;
    		if (private_handle.getParam(full_urdf_xml, result))
      		xml.Parse(result.c_str());
    		else
    		{
      		ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
      		return false;
    		}
    		xml_string = result;
    		TiXmlElement *root_element = xml.RootElement();
    		TiXmlElement *root = xml.FirstChildElement("robot");
    		if (!root || !root_element)
    		{
      		ROS_FATAL("Could not parse the xml from %s\n", urdf_xml.c_str());
      		return false;
    		}
    		robot_model.initXml(root);
    		if (!private_handle.getParam("root_name", root_name)){
      		ROS_FATAL("PR2IK: No root name found on parameter server");
      		return false;
    		}
    		if (!private_handle.getParam("tip_name", tip_name)){
      		ROS_FATAL("PR2IK: No tip name found on parameter server");
      		return false;
    		}

        KDL::Tree tree;
        if (!kdl_parser::treeFromString(xml_string, tree))
        {
            ROS_ERROR("Could not initialize tree object");
            return false;
        }

        if (!tree.getChain(root_name, tip_name, kdl_chain))
        {
            ROS_ERROR("Could not initialize kdl_chain object");
            return false;
        }

        int i=0; // segment number
        while(i < (int)kdl_chain.getNrOfSegments())
        {
            link2index.insert(std::make_pair(kdl_chain.getSegment(i).getName(),i));
            links.push_back(kdl_chain.getSegment(i).getName());

            joints.push_back(kdl_chain.getSegment(i).getJoint().getName());
            ROS_INFO("Joint %s",joints.back().c_str());

            i++;
        }
        if(joints.size() != numJoints) {
            ROS_ERROR("Joints count mismatch");
            return false;
        }

				int num_joints = 0;
  			boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
  			while(link && num_joints < 7)
  			{
    			boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);
    			if(!joint)
    			{
      			ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
      			return false;
    			}
    			if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    			{
      			if(joint->type != urdf::Joint::CONTINUOUS)
      			{
        			jointMin.push_back(joint->safety->soft_lower_limit);
        			jointMax.push_back(joint->safety->soft_upper_limit);
      			}
      			else
      			{
        			jointMin.push_back(-M_PI);
        			jointMax.push_back(M_PI);
      			}
      			num_joints++;
    			}
    			link = robot_model.getLink(link->getParent()->name);
  			} 

				// We expect order from root to tip, so reverse the order
  			std::reverse(jointMin.begin(),jointMin.end());
  			std::reverse(jointMax.begin(),jointMax.end());

        jnt_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));


        return true;
    }
bool getCount(int &count, 
                              const int &max_count, 
                              const int &min_count)
{
  if(count > 0)
  {
    if(-count >= min_count)
    {   
      count = -count;
      return true;
    }
    else if(count+1 <= max_count)
    {
      count = count+1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(1-count <= max_count)
    {
      count = 1-count;
      return true;
    }
    else if(count-1 >= min_count)
    {
      count = count -1;
      return true;
    }
    else
      return false;
  }
}

		bool checkJoints(const std::vector<double> &sol)
		{
			for(size_t i = 0; i<sol.size(); i++)
			{
				if((sol[i] < jointMin[i]) || (sol[i] > jointMax[i]))
					return false;
			}
			return true;
		}

    bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                       const std::vector<double> &ik_seed_state,
                       std::vector<double> &solution,
                       int &error_code) {
        //ROS_ERROR("getPositionIK");

        std::vector<double> vfree(freeParams.size());
        for(std::size_t i = 0; i < freeParams.size(); ++i) {
            int p = freeParams[i];
// ROS_ERROR("%u is %f",p,ik_seed_state[p]);
            vfree[i] = ik_seed_state[p];
        }

        KDL::Frame frame;
        tf::PoseMsgToKDL(ik_pose,frame);

        int numsol = ik_solver->solve(frame,vfree);


        if(numsol) {
            ik_solver->getClosestSolution(ik_seed_state,solution);
            //ROS_ERROR("Solved!");
            error_code = kinematics::SUCCESS;
            return true;
        }

        error_code = kinematics::NO_IK_SOLUTION;
        return false;
    }

    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          const double &timeout,
                          std::vector<double> &solution, int &error_code) {

        //ROS_ERROR("searchPositionIK1");
        if(freeParams.size()==0) {
            return getPositionIK(ik_pose, ik_seed_state,solution, error_code);
        }
        bool solved = false;
        std::vector<double> sol;
        double mindist = 9999999;
        KDL::Frame frame;
        tf::PoseMsgToKDL(ik_pose,frame);

        std::vector<double> vfree(freeParams.size());

        ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
        const double increment = 0.02; //TODO: make it a plugin parameter

        vfree[0] = -2.9; //jointMin[freeParams[0]];

        while (vfree[0] < 2.9 /*jointMax[freeParams[0]]*/) {

            int numsol = ik_solver->solve(frame,vfree);

            if(numsol > 0) {
                double dist = 0;
                ik_solver->getClosestSolution(ik_seed_state,sol);
                for(size_t i = 0; i<sol.size(); i++)
                    dist += (sol[i] - ik_seed_state[i]) * (sol[i] - ik_seed_state[i]);
                if(dist < mindist)
                {
                    solution = sol;
                    mindist = dist;
                }
                solved = true;
            }

            vfree[0] += increment;
						if(ros::Time::now() > maxTime)
							break;
        }

        if(solved)
        {
            //ROS_ERROR("Solved!");
            error_code = kinematics::SUCCESS;
            return true;
        } else {
            error_code = kinematics::NO_IK_SOLUTION;
            return false;
        }
    }

    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          const double &timeout,
                          std::vector<double> &solution,
                          const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                          const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
                          int &error_code) {
				KDL::Frame frame;
				int count = 0;
				int numsol;
				std::vector<double> sol;

        if(!desired_pose_callback.empty())
            desired_pose_callback(ik_pose,ik_seed_state,error_code);

        if(error_code < 0)
        {
            ROS_ERROR("Could not find inverse kinematics for desired end-effector pose since the pose may be in collision");
            return false;
        }

        
				
        tf::PoseMsgToKDL(ik_pose,frame);

        std::vector<double> vfree(freeParams.size());

        ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
        const double search_discretization_angle_ = 0.02; //TODO: make it a plugin parameter

   			double initial_guess = vfree[0] = ik_seed_state[freeParams[0]];

				int num_positive_increments = (int)((jointMax[freeParams[0]]-initial_guess)/search_discretization_angle_);
  			int num_negative_increments = (int)((initial_guess + jointMin[freeParams[0]])/search_discretization_angle_);
  			ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,jointMax[freeParams[0]],jointMin[freeParams[0]],num_positive_increments,num_negative_increments);

        while (ros::Time::now() < maxTime) {
            if(numsol = ik_solver->solve(frame,vfree) > 0) {
							if(solution_callback.empty()) {
                
                ik_solver->getClosestSolution(ik_seed_state,solution);
								error_code = kinematics::SUCCESS;
								return true;
							} else {
								for (int s = 0; s < numsol; ++s) {
                  ik_solver->getSolution(s,sol);
									if(checkJoints(sol))
									{
                  	solution_callback(ik_pose,sol,error_code);
									
										if( error_code == kinematics::SUCCESS) {
                  	  	solution = sol;
    										return true;
                		}
									}
								}
              }
        		}

    				if(!getCount(count,num_positive_increments,-num_negative_increments)) {
      				error_code = kinematics::NO_IK_SOLUTION;
      				return false;
    				}

    				vfree[0] = initial_guess + search_discretization_angle_ * count;
    				ROS_DEBUG("Redundancy search, index:%d, free angle value: %f",count,vfree[0]);
        }

        ROS_DEBUG("IK Timed out in %f seconds",timeout);
    		error_code = kinematics::TIMED_OUT;

				return false;
    }

    bool getPositionFK(const std::vector<std::string> &link_names,
                       const std::vector<double> &joint_angles,
                       std::vector<geometry_msgs::Pose> &poses) {
        KDL::Frame p_out;
        KDL::JntArray jnt_pos_in;
        geometry_msgs::PoseStamped pose;
        tf::Stamped<tf::Pose> tf_pose;

        jnt_pos_in.resize(joints.size());
        for(size_t i=0; i < joints.size(); i++)
        {
            jnt_pos_in(i) = joint_angles[i];
        }

        poses.resize(link_names.size());

        bool valid = true;
        for(unsigned int i=0; i < poses.size(); i++)
        {
            if(jnt_to_pose_solver->JntToCart(jnt_pos_in,p_out,link2index[link_names[i]]) >=0)
            {
                tf::PoseKDLToMsg(p_out,poses[i]);
            }
            else
            {
                ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
                valid = false;
            }
        }
        //ROS_ERROR("getPositionFK %s",valid?"succeded":"failed");

        return valid;
    }
    std::string getBaseFrame() {
        //ROS_ERROR("getBaseFrame");
        return root_name;
    }
    std::string getToolFrame() {
        //ROS_ERROR("getToolFrame");
        return tip_name;
    }
    std::vector<std::string> getJointNames() {
        //ROS_ERROR("getJointNames");
        return joints;
    }
    std::vector<std::string> getLinkNames() {
        //ROS_ERROR("getLinkNames");
        return links;
    }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(lwr_arm_kinematics,IKFastKinematicsPlugin, lwr_arm_kinematics::IKFastKinematicsPlugin, kinematics::KinematicsBase)
