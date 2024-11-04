#include <ros/ros.h>
#include <bits/stdc++.h>
#include <geometry_msgs/PoseStamped.h>
#include <race/path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <race/pid_input.h>


double car_coordinate_x=0;
double car_coordinate_y=0;
double roll,pitch,yaw;
double forward_projection=0.2;
double i=0;
std::vector<double> final_path;
std::vector<double> path;
int s=0;


void call(const geometry_msgs::PoseStamped::ConstPtr& msg, ros::Publisher& err_pub,ros::Publisher& marker_pub){
    std::cout<<"call"<<std::endl;
    car_coordinate_x=msg->pose.position.x;
    car_coordinate_y=msg->pose.position.y;
    double q_x = msg->pose.orientation.x;
    double q_y = msg->pose.orientation.y;
    double q_z = msg->pose.orientation.z;
    double q_w = msg->pose.orientation.w;
    tf2::Quaternion quat(q_x, q_y, q_z, q_w);
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);
    //std::cout<<"yaw="<<yaw * (180.0 / 3.14)<<std::endl;
    
    if(final_path.size()==0 && path.size()==0){
        return;
    }

    if(final_path.size()==0){
        i=0;
        visualization_msgs::Marker nodes_marker;
        nodes_marker.header.frame_id = "map"; // Set to your frame
        nodes_marker.header.stamp = ros::Time::now();
        nodes_marker.ns = "rrt_tree_nodes";
        nodes_marker.id = 1;
        nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST; 
        nodes_marker.action = visualization_msgs::Marker::ADD;
        nodes_marker.lifetime = ros::Duration();
        nodes_marker.scale.x = 0.2; 
        nodes_marker.scale.y = 0.2;
        nodes_marker.scale.z = 0.2;
        nodes_marker.color.r = 0.0; 
        nodes_marker.color.g = 0.5;
        nodes_marker.color.b = 1.0;
        nodes_marker.color.a = 1.0;

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "map"; 
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "waypoints";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.lifetime = ros::Duration();

        line_strip.scale.x = 0.06; 
        line_strip.color.r = 1.0;
        line_strip.color.g = 0.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;
        final_path=path;

        for(int j=0;j<final_path.size();j=j+2){
            
            geometry_msgs::Point node_point;
            node_point.x = final_path[j];
            node_point.y = final_path[j+1];
            node_point.z = 0; // Assuming 2D tree
            nodes_marker.points.push_back(node_point);

            geometry_msgs::Point p;
            p.x = final_path[j];
            p.y = final_path[j+1];
            p.z = 0; 
            line_strip.points.push_back(p);
        }
        marker_pub.publish(line_strip);
        marker_pub.publish(nodes_marker);

        //std::reverse(final_path.begin(),final_path.end());
        //std::cout<<"changed"<<std::endl;
        

    }
    
 
    if((final_path.size()-i)<4){
        final_path.clear();
        //std::cout<<"ended"<<std::endl;
        return;
    }

    double start_x=final_path[i];
    double start_y=final_path[i+1];
    double goal_x=final_path[i+2];
    double goal_y=final_path[i+3];

    double num=goal_y-start_y;
    double den=goal_x-start_x;

    double slope= abs(num/den);    
    double dist_points=abs(sqrt(pow(num, 2) + pow(den, 2)));

    if(dist_points<0.6){
        //std::cout<<"skipped"<<std::endl;
        i=i+2;
        return;
    }
    double ratio = den / dist_points;
    double ideal = std::acos(ratio);
    //std::cout<<"ideal"<<(ideal*180)/3.14159<<"  yaw"<<(yaw*180)/3.14159;
    //std::cout<<"ideal"<<(ideal*180)/3.14<<"yaw"<<(yaw*180)/3.14<<std::endl;
    if(num<0){
        ideal=ideal*(-1);
    } 
    //std::cout<<"ideal"<<(ideal*180)/3.14<<"yaw"<<(yaw*180)/3.14<<std::endl;
    // std::cout<<"ideal"<<(ideal*180)/3.14159<<"  yaw"<<(yaw*180)/3.14159;

    double A=start_y-goal_y;
    double B=goal_x-start_y;
    double C=(start_x*goal_y)-(goal_x*start_y);

    double err=(abs((A*car_coordinate_x)+(B*car_coordinate_y+C))/sqrt(pow(A, 2) + pow(B, 2)));

    double d=((car_coordinate_x-start_x)*(goal_y-start_y))-((car_coordinate_y-start_y)*(goal_x-start_x));

    double Alpha=yaw-ideal;
    double error;
    if(d<0){
        //std::cout<<"left"<<std::endl;
        error=forward_projection*std::sin(Alpha)+err;
    }
    else{
        //std::cout<<"right"<<std::endl;
        error=forward_projection*std::sin(Alpha)-err;
    }

    race::pid_input pid_msg;
    pid_msg.pid_vel = 1.0;  // Set velocity to constant 1
    pid_msg.pid_error = error;  // Set the computed error
    err_pub.publish(pid_msg);  // Publish the message

    double gap = abs(sqrt(pow(car_coordinate_x - goal_x, 2) + pow(car_coordinate_y - goal_y, 2)));
    //std::cout<<gap<<std::endl;
    if (gap < 0.2) {

        i=i+2;
    }

}


void callback(const race::path::ConstPtr& msg){
    s=msg->path.size();
    int length=s/2;
    int count=0;
    std::cout<<"callback"<<std::endl;

    if(s>=4){
        path.clear();
        for(int j=s-1;j>=0;j=j-2){
            path.push_back(msg->path[j]);
            path.push_back(msg->path[j-1]);
        }
    }

    // if(final_path.size()==0){
    //     i=0;
    //     visualization_msgs::Marker nodes_marker;
    //     nodes_marker.header.frame_id = "map"; // Set to your frame
    //     nodes_marker.header.stamp = ros::Time::now();
    //     nodes_marker.ns = "rrt_tree_nodes";
    //     nodes_marker.id = 1;
    //     nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST; 
    //     nodes_marker.action = visualization_msgs::Marker::ADD;
    //     nodes_marker.lifetime = ros::Duration();
    //     nodes_marker.scale.x = 0.2; 
    //     nodes_marker.scale.y = 0.2;
    //     nodes_marker.scale.z = 0.2;
    //     nodes_marker.color.r = 0.0; 
    //     nodes_marker.color.g = 0.5;
    //     nodes_marker.color.b = 1.0;
    //     nodes_marker.color.a = 1.0;

    //     visualization_msgs::Marker line_strip;
    //     line_strip.header.frame_id = "map"; 
    //     line_strip.header.stamp = ros::Time::now();
    //     line_strip.ns = "waypoints";
    //     line_strip.id = 0;
    //     line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    //     line_strip.action = visualization_msgs::Marker::ADD;
    //     line_strip.lifetime = ros::Duration();

    //     line_strip.scale.x = 0.06; 
    //     line_strip.color.r = 1.0;
    //     line_strip.color.g = 0.0;
    //     line_strip.color.b = 0.0;
    //     line_strip.color.a = 1.0;

    //     for(int j=s-1;j>=0;j=j-2){
    //         final_path.push_back(msg->path[j]);
    //         final_path.push_back(msg->path[j-1]);

    //         geometry_msgs::Point node_point;
    //         node_point.x = msg->path[j];
    //         node_point.y = msg->path[j-1];
    //         node_point.z = 0; // Assuming 2D tree
    //         nodes_marker.points.push_back(node_point);

    //         geometry_msgs::Point p;
    //         p.x = msg->path[j];
    //         p.y = msg->path[j-1];
    //         p.z = 0; 
    //         line_strip.points.push_back(p);
    //     }
    //     marker_pub.publish(line_strip);
    //     marker_pub.publish(nodes_marker);

    //     //std::reverse(final_path.begin(),final_path.end());
    //     //std::cout<<"changed"<<std::endl;

    // }
    // else if(final_path.size()==0 && s<4){
    //     return;
    // }
    

    // if((final_path.size()-i)<4){
    //     final_path.clear();
    //     //std::cout<<"ended"<<std::endl;
    //     return;
    // }
    // //std::cout<<"i="<<i<<std::endl;
    // double start_x=final_path[i];
    // double start_y=final_path[i+1];
    // double goal_x=final_path[i+2];
    // double goal_y=final_path[i+3];

    // double num=goal_y-start_y;
    // double den=goal_x-start_x;

    // double slope= abs(num/den);    
    // double dist_points=abs(sqrt(pow(num, 2) + pow(den, 2)));

    // if(dist_points<0.6){
    //     //std::cout<<"skipped"<<std::endl;
    //     i=i+2;
    //     return;
    // }
    // double ratio = den / dist_points;
    // double ideal = std::acos(ratio);
    // //std::cout<<"ideal"<<(ideal*180)/3.14159<<"  yaw"<<(yaw*180)/3.14159;
    // //std::cout<<"ideal"<<(ideal*180)/3.14<<"yaw"<<(yaw*180)/3.14<<std::endl;
    // if(num<0){
    //     ideal=ideal*(-1);
    // } 
    // //std::cout<<"ideal"<<(ideal*180)/3.14<<"yaw"<<(yaw*180)/3.14<<std::endl;
    // // std::cout<<"ideal"<<(ideal*180)/3.14159<<"  yaw"<<(yaw*180)/3.14159;

    // double A=start_y-goal_y;
    // double B=goal_x-start_y;
    // double C=(start_x*goal_y)-(goal_x*start_y);

    // double err=(abs((A*car_coordinate_x)+(B*car_coordinate_y+C))/sqrt(pow(A, 2) + pow(B, 2)));

    // double d=((car_coordinate_x-start_x)*(goal_y-start_y))-((car_coordinate_y-start_y)*(goal_x-start_x));

    // double Alpha=yaw-ideal;
    // double error;
    // if(d<0){
    //     //std::cout<<"left"<<std::endl;
    //     error=forward_projection*std::sin(Alpha)+err;
    // }
    // else{
    //     //std::cout<<"right"<<std::endl;
    //     error=forward_projection*std::sin(Alpha)-err;
    // }

    // race::pid_input pid_msg;
    // pid_msg.pid_vel = 1.0;  // Set velocity to constant 1
    // pid_msg.pid_error = error;  // Set the computed error
    // err_pub.publish(pid_msg);  // Publish the message

    // double gap = abs(sqrt(pow(car_coordinate_x - goal_x, 2) + pow(car_coordinate_y - goal_y, 2)));
    // //std::cout<<gap<<std::endl;
    // if (gap < 0.2) {

    //     i=i+2;
    // }
            

    


}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "planning");
  ros::NodeHandle nh;
  ros::Publisher err_pub = nh.advertise<race::pid_input>("/err", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber subs = nh.subscribe<geometry_msgs::PoseStamped>("gt_pose", 1,
        [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            call(msg, err_pub, marker_pub);  // Now 'call' can use err_pub and marker_pub
        }
    );
  ros::Subscriber sub = nh.subscribe<race::path>("rrt_to_planning", 1, callback);
        // [&](const race::path::ConstPtr& msg) {
        //     callback(msg, err_pub,marker_pub);
        // });

    // ros::Rate loop_rate(1000);
    // while (ros::ok()) {
    //     // The global variable will be updated by the callback when data is received
    //     // You can use or process the global variable in this loop

    //     std::cout<<"hello"<<std::endl;

    //     // Handle any pending callbacks (e.g., updating the global variable)
    //     ros::spinOnce();
    //     loop_rate.sleep(); 
    // }


  //return 0;
  ros::spin();
}