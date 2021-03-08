#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include<iostream>
#include<fstream>

#define pi 3.1415926

using namespace std;

int evader_num=1, pursuer_num=1;

double roll = {0, 0}, pitch = {0, 0}, yaw = {0, 0};
float linear_x = {0, 0}, linear_y = {0, 0}, angular_z = {0, 0};
float pos_x = {0, 0}, pos_y = {0, 0};

float V_em = [0.9], V_pm = [1.0];
float W_em = [1.0], W_pm = [0.8];

// subscribe two robots info
void DataCallback1(const nav_msgs::Odometry &msg){
   tf::Quaternion quat;
   tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
   tf::Matrix3x3(quat).getRPY(roll[0], pitch[0], yaw[0]);

   pos_x[0] = msg.pose.pose.position.x;
   pos_y[0] = msg.pose.pose.position.y;

   linear_x[0] = msg.twist.twist.linear.x;
   linear_y[0] = msg.twist.twist.linear.y;
   angular_z[0] = msg.twist.twist.angular.z;
   
   ROS_INFO("Listener Evader: %f %f %f %f %f %f", pos_x[0], pos_y[0], yaw[0], linear_x[0], linear_y[0], angular_z[0]);
}

void DataCallback2(const nav_msgs::Odometry &msg){
   tf::Quaternion quat;
   tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
   tf::Matrix3x3(quat).getRPY(roll[1], pitch[1], yaw[1]);

   pos_x[1] = msg.pose.pose.position.x;
   pos_y[1] = msg.pose.pose.position.y;

   linear_x[1]= msg.twist.twist.linear.x;
   linear_y[1] = msg.twist.twist.linear.y;
   angular_z[1] = msg.twist.twist.angular.z;
   
   ROS_INFO("Listener Pursuer: %f %f %f %f %f %f", pos_x[1], pos_y[1], yaw[1], linear_x[1], linear_y[1], angular_z[1]);
}``

float dis(float e, float p)
{
   float temp1 = pow(e[0]-p[0], 2);
   float temp2 = pow(e[1]-p[1], 2);
   float temp3 = sqrt(temp1+temp2);
   return temp3;
}

float get_Vex(int index, float e[][2], float p[][2], int delta_)
{
   bool flag =  false;
   float v[2] = {e[index][0], e[index][1]};
   float temp2 = 0, temp3 = 0, temp4 = 0;
   for(int i=0; i<evader_num; i++);
   {
      float v1[2] = {e[i][0], e[i][1]};
      float temp1 = 0;
      for(int j=0; j<pursuer_num; j++)
      {
         float v2[2] = {p[j][0], p[j][1]};
         temp1 = temp1 + pow(dis(v1, v2), ((-1)*delta_));
         if(!flag)
         {
            temp3 = temp3 + (v[0] - v2[0]) * pow(dis(v, v2), ((-1)*delta_-2));
            temp4 = temp4 + pow(dis(v, v2), ((-1)*delta_));
            flag = true;
         }
      temp2 = temp2 + pursuer_num / temp1;
    }
    return pursuer_num * pow(temp2, 1.0/delta_ -1) * (temp3 / pow(temp4, 2)); 
}

float get_Vey(int index, float e[][2], float p[][2], int delta_)
{
   bool flag =  false;
   float v[2] = {e[index][0], e[index][1]};
   float temp2 = 0, temp3 = 0, temp4 = 0;
   for(int i=0; i<evader_num; i++);
   {
      float v1[2] = {e[i][0], e[i][1]};
      float temp1 = 0;
      for(int j=0; j<pursuer_num; j++)
      {
         float v2[2] = {p[j][0], p[j][1]};
         temp1 = temp1 + pow(dis(v1, v2), ((-1)*delta_));
         if(!flag)
         {
            temp3 = temp3 + (v[1] - v2[1]) * pow(dis(v, v2), ((-1)*delta_-2));
            temp4 = temp4 + pow(dis(v, v2), ((-1)*delta_));
            flag = true;
         }
      temp2 = temp2 + pursuer_num / temp1;
    }
    return pursuer_num * pow(temp2, 1.0/delta_ -1) * (temp3 / pow(temp4, 2)); 
}

float get_Vpx(int index, float e[][2], float p[][2], int delta_)
{
   float v[2] = {p[index][0], p[index][1]};
   float temp2 = 0, temp3 = 0, temp4 = 0;
   for(int i=0; i<evader_num; i++)
   {
      v1[2] = {e[i][0], e[i][1]};
      float temp1 = 0;
      for(int j=0; j<pursuer_num; j++)
      {
         float v2[2] = {p[j][0], p[j][1]};
         temp1 = temp1 + pow(dis(v1, v2), ((-1)*delta_));
      }
      temp2 = temp2 + pursuer_num / temp1;
      temp3 = (v[0]-v1[0]) * pow(dis(v1, v), (-1)*delta_-2);
      temp4 = temp4 + temp3 / pow(temp1, 2);
    return pursuer_num * pow(temp2, 1.0/delta_-1) * temp4;
}

float get_Vpy(int index, float e[][2], float p[][2], int delta_)
{
   float v[2] = {p[index][0], p[index][1]};
   float temp2 = 0, temp3 = 0, temp4 = 0;
   for(int i=0; i<evader_num; i++)
   {
      v1[2] = {e[i][0], e[i][1]};
      float temp1 = 0;
      for(int j=0; j<pursuer_num; j++)
      {
         float v2[2] = {p[j][0], p[j][1]};
         temp1 = temp1 + pow(dis(v1, v2), ((-1)*delta_));
      }
      temp2 = temp2 + pursuer_num / temp1;
      temp3 = (v[1]-v1[1]) * pow(dis(v1, v), (-1)*delta_-2);
      temp4 = temp4 + temp3 / pow(temp1, 2);
    return pursuer_num * pow(temp2, 1.0/delta_-1) * temp4;
}

int main(int argc, char** argv){
   ros::init(argc,argv,"velcontrol");  
   ros::NodeHandle n;
   ros::Subscriber sub_1 = n.subscribe("/husky_alpha/husky_velocity_controller/odom", 10, DataCallback1);
   ros::Subscriber sub_2 = n.subscribe("/husky_beta/husky_velocity_controller/odom", 10, DataCallback2);
   ros::Publisher velpub_1 = n.advertise<geometry_msgs::Twist>("/husky_beta/husky_velocity_controller/cmd_vel", 10);
   ros::Publisher velpub_2 = n.advertise<geometry_msgs::Twist>("/husky_beta/husky_velocity_controller/cmd_vel", 10);
   ros::Rate loop_rate(50);
   geometry_msgs::Vector3 linear;
   geometry_msgs::Vector3 angular;
   float evader[evader_num][2], V_ex[evader_num], V_ey[evader_num], theta_er[evader_num], V_e[evader_num], W_e[evader_num];
   float pursuer[pursuer_num][2], V_px[pursuer_num], V_py[pursuer_num], theta_pr[pursuer_num], V_p[pursuer_num], W_p[pursuer_num];
   while(ros::ok())
   {
      geometry_msgs::Twist msg[evader_num+pursuer_num];
      for(int i=0; i<evader_num; i++)
      {
         evader[i] = {pos_x[i], pos_y[i]};
      }
      for(int j=0; j<pursuer_num; j++)
      {
         pursuer[j] = {pos_x[i+evader_num], pos_y[i+evader_num]};
      }
      for(int i=0; i<evader_num; i++)
      {
         V_ex[i] = get_Vex(i, evader, pursuer, delta);
         V_ey[j] = get_Vey(i, evader, pursuer, delta);
         theta_er[i] = 1.0/2.0 * pi - atan(V_ex[i] / V_ey[i]);
         V_e[i] = V_em[i] * sign(V_ex[i]*cos(yaw[i]) + V_ey[i]*sin(yaw[i]));
         W_e[i] = (-1) * (W_em[i]) * sign(yaw - theta_er[i]);
         msg[i].linear.x = V_e[i] * cos(yaw[i]);
         msg[i].linear.y = V_e[i] * sin(yaw[i]);
         msg[i].linear.z = 0;
         msg[i].angular.x = 0;
         msg[i].angular.y = 0;
         msg[i].angular.z = W_e[i];
      }
      for(int j=0; j<pursuer_num; j++)
      {
         V_px[j] = get_Vpx(i, evader, pursuer, delta);
         V_py[j] = get_Vpy(i, evader, pursuer, delta);
         theta_pr[j] = 1.0/2.0 * pi - atan(V_px[j] / V_py[j]);
         V_p[j] = (-1) * V_pm[j] * sign(V_px[j]*cos(yaw[evader_num+j]) + V_py[j]*sin(yaw[evader_num+j]));
         W_P[j] = (-1) * W_pm[j] * sign(yaw[evader_num+j] - theta_pr[j]);
         msg[evader_num+j].linear.x = V_p[j] * cos(yaw[evader_num+j]);
         msg[evader_num+j].linear.y = V_p[j] * sin(yaw[evader_num+j]);
         msg[evader_num+j].linear.z = 0;
         msg[evader_num+j].angular.x = 0;
         msg[evader_num+j].angular.y = 0;
         msg[evader_num+j].angular.z = W_p[j];
      }
      velpub_1.publish(msg[0]);
      velpub_2.publish(msg[1]);
      ROS_INFO("Publish velocity of robots, evader: %f, %f, %f, pursuer: %f, %f, %f", msg[0].linear.x, msg[0].linear.y, msg[0].angular.z, msg[1].linear.x, msg[1].linear.y, msg[1].angular.z);
      loop_rate.sleep();
   }

      

         
      
   
