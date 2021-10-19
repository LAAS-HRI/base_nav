#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <actionlib/server/simple_action_server.h>
#include <base_nav/DockAction.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

class Docking{
public:
    struct Params{
        double goalTolerance_dist;
        double goalTolerance_ang;

        double maxSpeed;
        double p_ang;
        double p_dist;

        double i_ang;
        double i_dist;
        double maxIntegral;
        ros::Duration controlPeriod;
        std::string tableFrame;
    };

    Docking(ros::NodeHandle& nh): nh_(nh), dockActionServer_(nh, "dock", boost::bind(&Docking::onDock, this, _1), false){
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
        cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 100, true);
        dockActionServer_.start();
    }

    void onDock(const base_nav::DockGoalConstPtr& goal){
        ros::Time timeZero(0.0);
        Params params;
        getParams(params);
        
        base_nav::DockResult asResult;
        if (!tfBuffer_.canTransform("map","base_footprint", timeZero))
        {
            ROS_WARN_STREAM_NAMED("Docking", "Can't transform base_footprint frame in map frame. Aborting action.");
            asResult.error_code = asResult.NO_MAP_TO_FOOTPRINT_TRANSFORM;
            dockActionServer_.setAborted(asResult);
            return;
        }

        tf2::Transform poseTf; 

        poseTf.setOrigin( tf2::Vector3(goal->targetPose.pose.position.x, goal->targetPose.pose.position.y, goal->targetPose.pose.position.z) );
        poseTf.setRotation( tf2::Quaternion(goal->targetPose.pose.orientation.x, goal->targetPose.pose.orientation.y, goal->targetPose.pose.orientation.z, goal->targetPose.pose.orientation.w) );
      
        //============================================================
        //  Handle X/Y position correction
        //============================================================

        geometry_msgs::TransformStamped map2footprintMsg = tfBuffer_.lookupTransform("map","base_footprint", timeZero);
        tf2::Transform map2footprint;
        tf2::fromMsg(map2footprintMsg.transform, map2footprint);

        geometry_msgs::TransformStamped pose2mapMsg = tfBuffer_.lookupTransform("map", goal->targetPose.header.frame_id, timeZero);
        tf2::Transform pose2map;
        tf2::fromMsg(pose2mapMsg.transform, pose2map);

        poseTf = pose2map * poseTf;

	double error_dist = tf2::tf2Distance2(poseTf.getOrigin(), map2footprint.getOrigin());

        double error_ang;
        double cmd_dist = 0.0, integral_dist;
        double cmd_ang = 0.0, integral_ang;

        tf2::Quaternion angle;
        tf2::Matrix3x3 m;
        double errorRoll, errorPitch, errorYaw;

        ros::Time lastControl = ros::Time::now();
        ros::Time now = lastControl;
        ros::Time actionStart = ros::Time::now();
        geometry_msgs::Twist cmdVel;
        integral_dist = 0.0;
        tf2::Transform footprint2pose;
        base_nav::DockFeedback feedback;
        feedback.action_start = actionStart;
        feedback.distance_to_goal = std::sqrt(error_dist);
        dockActionServer_.publishFeedback(feedback);


        while ((std::abs(error_dist) > params.goalTolerance_dist) || (std::abs(error_ang) > params.goalTolerance_ang) && ros::ok())
        {
            map2footprintMsg = tfBuffer_.lookupTransform("map","base_footprint", ros::Time(0.0));
            tf2::fromMsg(map2footprintMsg.transform, map2footprint);

            footprint2pose = map2footprint.inverse() * poseTf;

            angle = footprint2pose.getRotation();

            m = (tf2::Matrix3x3) angle;
            m.getRPY(errorRoll, errorPitch, errorYaw);

            error_ang = errorYaw;

            error_dist = std::pow(footprint2pose.getOrigin().getX(), 2) + std::pow(footprint2pose.getOrigin().getY(), 2);

            feedback.distance_to_goal = std::sqrt(error_dist);

            dockActionServer_.publishFeedback(feedback);

            getParams(params);
            now = ros::Time::now();

            if (dockActionServer_.isPreemptRequested())
            {
                cmdVel.linear.x = 0.0;
                cmdVel.linear.y = 0.0;
                cmdVelPub_.publish(cmdVel);
                asResult.error_code = asResult.PREEMPTED;
                asResult.action_end = ros::Time::now();
                dockActionServer_.setPreempted(asResult);
                return;
            }

            integral_dist = std::max(-params.maxIntegral, std::min(params.maxIntegral, integral_dist + error_dist * (now - lastControl).toSec()));

            cmd_dist = std::max(-params.maxSpeed, std::min(params.maxSpeed, params.p_dist * error_dist + params.i_dist * integral_dist));


            integral_ang = std::max(-params.maxIntegral, std::min(params.maxIntegral, integral_ang + error_ang * (now - lastControl).toSec()));

            cmd_ang = std::max(-params.maxSpeed, std::min(params.maxSpeed, params.p_ang * error_ang + params.i_ang * integral_ang));

            double angle = std::atan2(footprint2pose.getOrigin().getY(), footprint2pose.getOrigin().getX());

            cmdVel.linear.x = cmd_dist * std::cos(angle);
            cmdVel.linear.y = cmd_dist * std::sin(angle);
            cmdVel.angular.z = cmd_ang ;

            cmdVelPub_.publish(cmdVel);

            lastControl = now;
            params.controlPeriod.sleep();
        }
        cmdVel.linear.x = 0.0;
        cmdVel.linear.y = 0.0;
        cmdVel.angular.z = 0.0;
        cmdVelPub_.publish(cmdVel);

        asResult.error_code = asResult.SUCCESS;
        dockActionServer_.setSucceeded(asResult);
    }

    void getParams(Params& params)
    {
        double controlPeriod;
        nh_.param("goal_tolerance_dist", params.goalTolerance_dist, 0.05);
        nh_.param("goal_tolerance_ang", params.goalTolerance_ang, 0.02);
        params.goalTolerance_dist *= params.goalTolerance_dist;
        nh_.param("max_speed", params.maxSpeed, 0.2);
        nh_.param("P_dist", params.p_dist, 0.1);
        nh_.param("I_dist", params.i_dist, 0.05);
        nh_.param("P_ang", params.p_ang, 0.1);
        nh_.param("I_ang", params.i_ang, 0.05);
        nh_.param("max_integral", params.maxIntegral, 5.0);
        nh_.param("control_period", controlPeriod, 0.1);
        params.controlPeriod.fromSec(controlPeriod);
    }



protected:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    actionlib::SimpleActionServer<base_nav::DockAction> dockActionServer_;
    ros::Publisher cmdVelPub_;

};

int main (int argc, char** argv){
    ros::init(argc, argv, "docking_node");
    ros::NodeHandle nh("~");
    Docking docking(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();
}
