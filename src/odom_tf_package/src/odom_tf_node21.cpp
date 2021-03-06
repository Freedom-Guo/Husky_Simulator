#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <fstream>
#include <math.h>

#define pi 3.1415926

using namespace std;

int evader_num=1, pursuer_num=2;
double roll[3] = {0}, pitch[3] = {0}, yaw[3] = {0};
float linear_x[3] = {0}, linear_y[3] = {0}, angular_z[3] = {0};
float pos_x[3] = {0, -7, -5}, pos_y[3] = {0, -5, 5};

float V_em[1] = {0.6}, V_pm[2] = {1.0, 1.0};
float W_em[1] = {1.0}, W_pm[2] = {0.8, 0.8};
int delta = 3;

int sign(float x)
{
   if(x>0)
      return 1;
   else if(x==0)
      return 0;
   else
      return -1;
}

float dis(float e[], float p[])
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
   for(int i=0; i<evader_num; i++)
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
   }
   return pursuer_num * pow(temp2, 1.0/delta_ -1) * (temp3 / pow(temp4, 2)); 
}

float get_Vey(int index, float e[][2], float p[][2], int delta_)
{
   bool flag =  false;
   float v[2] = {e[index][0], e[index][1]};
   float temp2 = 0, temp3 = 0, temp4 = 0;
   for(int i=0; i<evader_num; i++)
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
    }
    return pursuer_num * pow(temp2, 1.0/delta_ -1) * (temp3 / pow(temp4, 2)); 
}

float get_Vpx(int index, float e[][2], float p[][2], int delta_)
{
   float v[2] = {p[index][0], p[index][1]};
   float temp2 = 0, temp3 = 0, temp4 = 0;
   for(int i=0; i<evader_num; i++)
   {
      float v1[2] = {e[i][0], e[i][1]};
      float temp1 = 0;
      for(int j=0; j<pursuer_num; j++)
      {
         float v2[2] = {p[j][0], p[j][1]};
         temp1 = temp1 + pow(dis(v1, v2), ((-1)*delta_));
      }
      temp2 = temp2 + pursuer_num / temp1;
      temp3 = (v[0]-v1[0]) * pow(dis(v1, v), (-1)*delta_-2);
      temp4 = temp4 + temp3 / pow(temp1, 2);
    }
    return pursuer_num * pow(temp2, 1.0/delta_-1) * temp4;
}

float get_Vpy(int index, float e[][2], float p[][2], int delta_)
{
   float v[2] = {p[index][0], p[index][1]};
   float temp2 = 0, temp3 = 0, temp4 = 0;
   for(int i=0; i<evader_num; i++)
   {
      float v1[2] = {e[i][0], e[i][1]};
      float temp1 = 0;
      for(int j=0; j<pursuer_num; j++)
      {
         float v2[2] = {p[j][0], p[j][1]};
         temp1 = temp1 + pow(dis(v1, v2), ((-1)*delta_));
      }
      temp2 = temp2 + pursuer_num / temp1;
      temp3 = (v[1]-v1[1]) * pow(dis(v1, v), (-1)*delta_-2);
      temp4 = temp4 + temp3 / pow(temp1, 2);
    }
    return pursuer_num * pow(temp2, 1.0/delta_-1) * temp4;
}

class Subscribe_Publish
{
public:
   Subscribe_Publish(string robot_name, int index)
   {
       sub = n.subscribe("/gazebo/model_states", 10, &Subscribe_Publish::DataCallback, this);
       velpub = n.advertise<geometry_msgs::Twist>(robot_name+"/husky_velocity_controller/cmd_vel", 10);
       index_ = index;
       this->robot_name = robot_name;
       if(index_==2)
          outFile.open("test.csv", ios::out);
   }

   void DataCallback(const gazebo_msgs::ModelStatesConstPtr &msg){
       std::vector<std::string> model_names = msg->name;
       int index;
       for(size_t i = 0; i < model_names.size(); i++)
       {
          if(model_names[i] == robot_name)
          {
             index = i;
             break;
          }
       }

       tf::Quaternion quat;
       tf::quaternionMsgToTF(msg->pose[index].orientation, quat);
       tf::Matrix3x3(quat).getRPY(roll[index_], pitch[index_], yaw[index_]);
       
       pos_x[index_] = msg->pose[index].position.x;
       pos_y[index_] = msg->pose[index].position.y;
       linear_x[index_] = msg->twist[index].linear.x;
       linear_y[index_] = msg->twist[index].linear.y;
       angular_z[index_] = msg->twist[index].angular.z;

       ROS_INFO("Listener No.%d : %f %f %f %f %f %f", index_, pos_x[index_], pos_y[index_], yaw[index_], linear_x[index_], linear_y[index_], angular_z[index_]);

       float evader[evader_num][2], V_ex[evader_num], V_ey[evader_num], theta_er[evader_num], V_e[evader_num], W_e[evader_num];
       float pursuer[pursuer_num][2], V_px[pursuer_num], V_py[pursuer_num], theta_pr[pursuer_num], V_p[pursuer_num], W_p[pursuer_num];
       
       geometry_msgs::Twist velmsg;
       for(int i=0; i<evader_num; i++)
       {
          evader[i][0] = pos_x[i];
          evader[i][1] = pos_y[i];
       }
       for(int j=0; j<pursuer_num; j++)
       {
          pursuer[j][0] = pos_x[j+evader_num];
          pursuer[j][1] = pos_y[j+evader_num];
       }
       if(index_ == 0)
       { 
          V_ex[index_] = get_Vex(index_, evader, pursuer, delta);
          V_ey[index_] = get_Vey(index_, evader, pursuer, delta);
          theta_er[index_] = 1.0/2.0 * pi - atan(V_ex[index_] / V_ey[index_]);
          // V_e[index_] = V_em[index_] * tanh((V_ex[index_]*cos(yaw[index_]) + V_ey[index_]*sin(yaw[index_]))/0.1);
          V_e[index_] = V_em[index_] * sign((V_ex[index_]*cos(yaw[index_]) + V_ey[index_]*sin(yaw[index_]))/1);
          W_e[index_] = (-1) * (W_em[index_]) * tanh((yaw[index_] - theta_er[index_])/0.1);
          velmsg.linear.x = V_e[index_];
          velmsg.linear.y = 0;
          velmsg.linear.z = 0;
          velmsg.angular.x = 0;
          velmsg.angular.y = 0;
          velmsg.angular.z = W_e[index_];
       }
       else
       {
          V_px[index_-evader_num] = get_Vpx(index_-evader_num, evader, pursuer, delta);
          V_py[index_-evader_num] = get_Vpy(index_-evader_num, evader, pursuer, delta);
          theta_pr[index_-evader_num] = 1.0/2.0 * pi - atan(V_px[index_-evader_num] / V_py[index_-evader_num]);
          // V_p[index_-evader_num] = (-1) * V_pm[index_-evader_num] * tanh((V_px[index_-evader_num]*cos(yaw[index_]) + V_py[index_-evader_num]*sin(yaw[index_]))/0.1);
          V_p[index_-evader_num] = (-1) * V_pm[index_-evader_num] * sign((V_px[index_-evader_num]*cos(yaw[index_]) + V_py[index_-evader_num]*sin(yaw[index_]))/1);
          W_p[index_-evader_num] = (-1) * W_pm[index_-evader_num] * tanh((yaw[index_] - theta_pr[index_-evader_num])/0.1);
          velmsg.linear.x = V_p[index_-evader_num];
          velmsg.linear.y = 0;
          velmsg.linear.z = 0;
          velmsg.angular.x = 0;
          velmsg.angular.y = 0;
          velmsg.angular.z = W_p[index_-evader_num];
       }
       velpub.publish(velmsg);
       if(index_ == 2)
          outFile << velmsg.linear.y<<','<<velmsg.angular.z<<','<<endl;
       ROS_INFO("Publish velocity of robots, No: %d, %f, %f, %f, %f", index_, theta_pr[index_-evader_num], velmsg.linear.x, velmsg.linear.y, velmsg.angular.z);

   }
private:
   ros::NodeHandle n;
   ros::Subscriber sub;
   ros::Publisher velpub;
   string robot_name;
   ofstream outFile;
   int index_;
};

int main(int argc, char** argv){
   ros::init(argc,argv,"velcontrol");  
   Subscribe_Publish evader_0("husky_alpha", 0), pursue_0("husky_gamma", 1), pursuer_1("husky_delta", 2);
   ros::spin();
}

      

         
      
   
