#include "../Eigen/Core"
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>

#ifndef TESTUDOG_H
#define TESTUDOG_H

namespace robot {
    class Testudog{
        public:
            // default constructor
            Testudog() :
                body_front{0.102},
                body_back{0.252},
                body_width{0.150},
                hip_length{0.0623},
                upperleg_length{0.118},
                lowerleg_length{0.118},
                x_global{0},
                y_global{0},
                z_global{0.2},
                roll{0},
                pitch{0},   
                yaw{0},
                count{0},
                imu_roll{0},
                imu_pitch{0}{
                //FL
                legpos_global(0,0) = x_global + body_width/2 + hip_length,
                legpos_global(1,0) = y_global - body_front;
                legpos_global(2,0) = 0;
                //FR
                legpos_global(0,1) = x_global - body_width/2 - hip_length;
                legpos_global(1,1) = y_global - body_front;
                legpos_global(2,1) = 0;
                //BL
                legpos_global(0,2) = x_global + body_width/2 + hip_length;
                legpos_global(1,2) = y_global + body_back;
                legpos_global(2,2) = 0;
                //BR
                legpos_global(0,3) = x_global - body_width/2 - hip_length;
                legpos_global(1,3) = y_global + body_back;
                legpos_global(2,3) = 0;
                //joint angles
                joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
                }

            Eigen::Matrix<double, 3, 4> get_joint_angle();

            Eigen::Matrix<double, 3, 4> global2local_legpos(Eigen::Matrix<double, 3, 4> leg_pos_global, 
                double x_global, double y_global, double z_global, double roll, double pitch, double yaw);

            Eigen::Matrix4d euler2transf(double x, double y, double z, double roll, double pitch, double yaw);

            Eigen::Matrix4d invtransf(Eigen::Matrix4d transf);

            Eigen::Matrix<double, 3, 4> inv_kine(Eigen::Matrix<double, 3, 4> leg_pos_local);

            void push_up(double speed = 5.0);

            void drunk(double speed = 5.0);

            void twist_yaw(double speed = 5.0);

            void twist_roll(double speed = 5.0);

            void twist_pitch(double speed = 5.0);

            void trot(double step_size = 0.2, double step_height = 0.03, double step_time = 0.5);

            void crawl(double step_size = 0.2, double step_height = 0.1, double step_time = 1.5);

            void get_imu(const sensor_msgs::Imu::ConstPtr& msg);

            void balance();
            
            void circle(double speed = 5.0);
            
            void test();

            Eigen::Array2d orientation_control();


        private:
            // units in m
            double body_front;
            double body_back;
            double body_width;
            double hip_length;
            double upperleg_length;
            double lowerleg_length;
            double x_global;
            double y_global;
            double z_global;
            double roll;
            double pitch;
            double yaw;
            int count;
            Eigen::Matrix<double, 3, 4> legpos_global;
            Eigen::Matrix<double, 3, 4> joint_angle;
            double imu_roll;
            double imu_pitch;
    };
}
#endif
