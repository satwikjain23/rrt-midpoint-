#include <ros/ros.h>
#include <race/perception.h>
#include <geometry_msgs/PoseStamped.h>
#include <bits/stdc++.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <race/path.h>
#include <iomanip>
#include <ctime> 


double car_coordinate_x=0;
double car_coordinate_y=0;
double length=0.5;
double roll,pitch,yaw;
std::vector<std::vector<double>> leftcones;
std::vector<std::vector<double>> rightcones;
double radius = 40;
double stepSize = 20;

class point{
  public:
    double locationX, locationY;
    point* parent;
    std::vector<point*> children;
    double cost;
    point(double a,double b){
      locationX=a;
      locationY=b;
      parent= NULL;

    }
};

class RRTSTAR{
  public:
    point* start;
    point* final;
    double radius;
    double path_distance;
    int numWaypoints;
    std::vector<point*> waypoints;
    ros::Publisher marker_pub;
    

    RRTSTAR(ros::Publisher& pub) : marker_pub(pub){
        start= new point(car_coordinate_x,car_coordinate_y);
        if (!leftcones.empty() && !rightcones.empty()) {
            final = new point((leftcones[1][0] + rightcones[1][0]) / 2, (leftcones[1][1] + rightcones[1][1]) / 2);
        }
        radius=5;
        path_distance=0;
        numWaypoints=0;


    }

    void publishPath(ros::Publisher& pub, const race::path& path_msg) {
      pub.publish(path_msg);  
    }
    

    std::vector<int> randompoint(){

      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<int> dis(-300, 300);
      int random_number1 = dis(gen);
      int random_number2 = dis(gen);
      std::vector<int> a(2,1);
      a[0]=random_number1;
      a[1]=random_number2;
      return a;

    }

    point* nearestNode(point* root, std::vector<double> po) {
        double minDist = 99999999.2;
        point* nearestNode = nullptr;
        std::stack<point*> s;
        s.push(root);
        while (!s.empty()) {
            point* currentNode = s.top();
            s.pop();
            if (currentNode) {
                double dist = distanceNaP(currentNode, po);
                if (dist < minDist) {
                    minDist = dist;
                    nearestNode = currentNode;
                }
                for (point* child : currentNode->children) {

                    s.push(child);
                }
            }
        }
        return nearestNode;
    }

    double distanceNaP(point* node, std::vector<double> p) {
      return sqrt(pow(node->locationX - p[0], 2) + pow(node->locationY - p[1], 2));
    }

    double distanceNaN(point* node, point* node2) {
      return sqrt(pow(node->locationX - node2->locationX, 2) + pow(node->locationY - node2->locationY, 2));
    }

     std::vector<double> steerToPoint(point* start, std::vector<double> end) {
        std::vector<double> u_hat = unitVector(start, end);
        std::vector<double> newPoint(2,-1);
        newPoint[0]=start->locationX + length * u_hat[0];
        newPoint[1]=start->locationY + length * u_hat[1];
        return newPoint;
    }

    std::vector<double> unitVector(point* start, std::vector<double> end) {
        std::vector<double> v(2,-1);
        v[0]=end[0] - start->locationX;
        v[1]= end[1] - start->locationY;
        double angleRadians = atan2(v[1], v[0]);
        double d= sqrt(pow(v[0], 2) + pow(v[1], 2));
        v[0]=v[0]/d;
        v[1]=v[1]/d;
        double angle=angleRadians * (180.0 / 3.14);
        double yawdegree= yaw * (180.0 / 3.14);
        if(abs(yawdegree-angle)>90){
          v[0]*=-1.0;
          v[1]*=-1.0;
        }
        
        return v;
    }

    bool Obstacle(point* nearest,std::vector<double> newPoint){
      
      double a1=newPoint[1]-nearest->locationY;
      double b1=nearest->locationX-newPoint[0];
      double c1=nearest->locationY*newPoint[0]-newPoint[1]*nearest->locationX;
      for(int i=0;i<leftcones.size()-1;i++){
        double distance= sqrt(pow(newPoint[0]-leftcones[i][0], 2) + pow(newPoint[1]-leftcones[i][1], 2));
        double a2=leftcones[i+1][1]-leftcones[i][1];
        double b2=leftcones[i][0]-leftcones[i+1][0];
        double c2=leftcones[i+1][0]*leftcones[i][1]-leftcones[i][0]*leftcones[i+1][1];
        double x=(b1*c2-b2*c1)/(a1*b2-a2*b1);
        double y=(a2*c1-a1*c2)/(a1*b2-a2*b1);
        double d1= sqrt(pow(x-nearest->locationX, 2) + pow(y-nearest->locationY, 2));
        double d2= sqrt(pow(x-newPoint[0], 2) + pow(y-newPoint[1], 2));
        double d= sqrt(pow(newPoint[0]-nearest->locationX, 2) + pow(newPoint[1]-nearest->locationY, 2));
        //std::cout<<" diff="<<i<<"  "<<d-(d1+d2)<<std::endl;
        if(((abs(d-(d1+d2)))<0.1) || distance<2){
          return true;
        }

      }
      for(int i=0;i<rightcones.size()-1;i++){
        double distance= sqrt(pow(newPoint[0]-rightcones[i][0], 2) + pow(newPoint[1]-rightcones[i][1], 2));
        double a2=rightcones[i+1][1]-rightcones[i][1];
        double b2=rightcones[i][0]-rightcones[i+1][0];
        double c2=rightcones[i+1][0]*rightcones[i][1]-rightcones[i][0]*rightcones[i+1][1];
        double x=(b1*c2-b2*c1)/(a1*b2-a2*b1);
        double y=(a2*c1-a1*c2)/(a1*b2-a2*b1);
        double d1= sqrt(pow(x-nearest->locationX, 2) + pow(y-nearest->locationY, 2));
        double d2= sqrt(pow(x-newPoint[0], 2) + pow(y-newPoint[1], 2));
        double d= sqrt(pow(newPoint[0]-nearest->locationX, 2) + pow(newPoint[1]-nearest->locationY, 2));
        //std::cout<<" diff="<<i<<"  "<<d-(d1+d2)<<std::endl;
        if((abs(d-(d1+d2)))<0.1 || distance<2){
          return true;
        }

      }
      /*
      double a2=leftcones[1][1]-rightcones[1][1];
      double b2=rightcones[1][0]-leftcones[1][0];
      double c2=leftcones[1][0]*rightcones[1][1]-rightcones[1][0]*leftcones[1][1];
      double x=(b1*c2-b2*c1)/(a1*b2-a2*b1);
      double y=(a2*c1-a1*c2)/(a1*b2-a2*b1);
      double d1= sqrt(pow(x-nearest->locationX, 2) + pow(y-nearest->locationY, 2));
      double d2= sqrt(pow(x-newPoint[0], 2) + pow(y-newPoint[1], 2));
      double d= sqrt(pow(newPoint[0]-nearest->locationX, 2) + pow(newPoint[1]-nearest->locationY, 2));
      if((abs(d-(d1+d2)))<0.01 ){
          return true;
      }
      */
      return false;
    }

    point* addNode(point* parent, double x, double y,double cost) {
        point* newNode = new point(x, y);
        newNode->parent = parent;
        parent->children.push_back(newNode);
        newNode->cost=cost;
        return newNode;
    }

    bool GoalReached(point* newNode){
      double A1=leftcones[1][1]-rightcones[1][1];
	    double B1=rightcones[1][0]-leftcones[1][0];
	    double C1=(leftcones[1][0]*rightcones[1][1])-(rightcones[1][0]*leftcones[1][1]);
	    double d=(abs((A1*newNode->locationX)+(B1*newNode->locationY+C1))/sqrt(pow(A1,2)+pow(B1,2)));
      if(d<2){
        return true;
      }
      return false;

      // double mid_x=(leftcones[1][1]+rightcones[1][1])/2;
      // double mid_y=(rightcones[1][0]+leftcones[1][0])/2;
      // double d=sqrt(pow(mid_x-newNode->locationX, 2) + pow(mid_y-newNode->locationY, 2));
      // if(d<3){
      //   return true;
      // }
      // return false;
    }

    void extractPath(point* newNode, race::path& path_msg){
      point* node= newNode;
      while (node->parent) {
            path_msg.path.push_back(node->locationY);
            path_msg.path.push_back(node->locationX);
            waypoints.push_back(node);
            node = node->parent;
      }
      waypoints.push_back(start);
      path_msg.path.push_back(start->locationY);
      path_msg.path.push_back(start->locationX);
      reverse(waypoints.begin(), waypoints.end());
      //std::cout<<waypoints.size()<<std::endl;

    }

    void updateParent(point* newNode,point* newParent, double newCost){
      newNode->parent->children.pop_back();
      newNode->parent=newParent;
      newNode->cost=newCost;
      newParent->children.push_back(newNode);
    }

    void rewire(point* newNode, std::vector<point*> nearnodes){
      point* newParent=newNode->parent;
      double newCost=newNode->cost;
      std::vector<double> temp{newNode->locationX,newNode->locationY};
      for(auto i:nearnodes){
        if(!Obstacle(i,temp)){
          double c=i->cost+distanceNaN(newNode,i);
          if(c<newCost){
            newParent=i;
            newCost=c;
          }
        }
      }
      updateParent(newNode,newParent,newCost);
    }

    void visualize(){
      visualization_msgs::Marker line_strip;
      line_strip.header.frame_id = "map"; // Change to your frame
      line_strip.header.stamp = ros::Time::now();
      line_strip.ns = "waypoints";
      line_strip.id = 0;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      line_strip.action = visualization_msgs::Marker::ADD;
      line_strip.lifetime = ros::Duration();

      // Set the scale of the line
      line_strip.scale.x = 0.06; // Line width

      // Set the color (e.g., red)
      line_strip.color.r = 1.0;
      line_strip.color.g = 0.0;
      line_strip.color.b = 1.0;
      line_strip.color.a = 1.0;

      // Add the waypoints to the marker
      for(auto wp : waypoints){
          geometry_msgs::Point p;
          p.x = wp->locationX;
          p.y = wp->locationY;
          p.z = 0; // Assuming a 2D plane
          line_strip.points.push_back(p);
      }

      // Publish the marker
      marker_pub.publish(line_strip);

    }

    void visualizeTree(point* root) {
    // Marker for the nodes (points)
      visualization_msgs::Marker nodes_marker;
      nodes_marker.header.frame_id = "map"; // Set to your frame
      nodes_marker.header.stamp = ros::Time::now();
      nodes_marker.ns = "rrt_tree_nodes";
      nodes_marker.id = 1;
      nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST; // Display all nodes as spheres
      nodes_marker.action = visualization_msgs::Marker::ADD;
      nodes_marker.lifetime = ros::Duration();
      nodes_marker.scale.x = 0.2; // Sphere size
      nodes_marker.scale.y = 0.2;
      nodes_marker.scale.z = 0.2;
      nodes_marker.color.r = 0.0; // Set color for the nodes (light blue)
      nodes_marker.color.g = 0.5;
      nodes_marker.color.b = 1.0;
      nodes_marker.color.a = 1.0;

      // Marker for the edges (connections between nodes)
      visualization_msgs::Marker edges_marker;
      edges_marker.header.frame_id = "map"; // Set to your frame
      edges_marker.header.stamp = ros::Time::now();
      edges_marker.ns = "rrt_tree_edges";
      edges_marker.id = 2;
      edges_marker.type = visualization_msgs::Marker::LINE_LIST; // Display all edges as lines
      edges_marker.action = visualization_msgs::Marker::ADD;
      edges_marker.lifetime = ros::Duration();
      edges_marker.scale.x = 0.05; // Line width
      edges_marker.color.r = 1.0; // Set color for the edges (pink)
      edges_marker.color.g = 0.0;
      edges_marker.color.b = 0.0;
      edges_marker.color.a = 1.0;

      // Traverse the tree and add nodes and edges
      std::stack<point*> stack;
      stack.push(root);
      int count=0;
      while (!stack.empty()) {
          count++;
          point* current = stack.top();
          stack.pop();

          // Add the current node to the nodes marker
          geometry_msgs::Point node_point;
          node_point.x = current->locationX;
          node_point.y = current->locationY;
          node_point.z = 0; // Assuming 2D tree
          nodes_marker.points.push_back(node_point);

          // Add edges (connections) to the edges marker
          if (current->parent) {
              // Create two points: one at current and one at parent
              geometry_msgs::Point parent_point;
              parent_point.x = current->parent->locationX;
              parent_point.y = current->parent->locationY;
              parent_point.z = 0; // Assuming 2D tree

              // Add both the current and parent points to the LINE_LIST
              edges_marker.points.push_back(node_point); // Current node
              edges_marker.points.push_back(parent_point); // Parent node
          }

          // Push all children to the stack for further exploration
          for (point* child : current->children) {
              stack.push(child);
          }
      }
      //std::cout<<count<<std::endl;

      // Publish both markers (nodes and edges)
      marker_pub.publish(nodes_marker);
      marker_pub.publish(edges_marker);
    }

};


void callback(const race::perception::ConstPtr& msg, ros::Publisher& marker_pub, ros::Publisher& path_pub) {
    try {
        RRTSTAR rrt(marker_pub);
        int totalNodes = 0;
        int totalIterations = 0;
        leftcones.clear();
        rightcones.clear();
        int s = msg->oned_list.size();
        for (int i = 0; i < s; i = i + 4) {
            std::vector<double> temp(2, 0);
            temp[0] = msg->oned_list[i];
            temp[1] = msg->oned_list[i + 1];
            if (temp[0] < 0) {
                leftcones.push_back(temp);
            } else {
                rightcones.push_back(temp);
            }
        }
        std::reverse(leftcones.begin(), leftcones.end());
        if(leftcones.size()<=1){
          return;
        }
        if(rightcones.size()<=1){
          return;
        }
        std::vector<point*> nodes;
        int n = 100;
        point* f = NULL;
        for (int i = 0; i < n; ++i) {
            totalIterations++;
            std::vector<int> rand = rrt.randompoint();
            std::vector<double> r(2, -1.0);
            r[0] = double(rand[0]);
            r[1] = double(rand[1]);

            point* nearest = rrt.nearestNode(rrt.start, r);
            std::vector<double> newPoint = rrt.steerToPoint(nearest, r);

            if (!rrt.Obstacle(nearest, newPoint)) {
                std::vector<point*> nearnodes;
                if (nearest->cost + length < 10) {
                    point* newNode = rrt.addNode(nearest, newPoint[0], newPoint[1], nearest->cost + length);
                    for (auto i : nodes) {
                        double dist = rrt.distanceNaN(i, newNode);
                        if (dist < 5) {
                            nearnodes.push_back(i);
                        }
                    }
                    nodes.push_back(newNode);
                    rrt.rewire(newNode, nearnodes);

                    if (rrt.GoalReached(newNode)) {
                        f = newNode;
                        // if(i>100){
                        //   std::cout<<i<<std::endl;
                        // }
                        // break;
                    }
                }
            }
        }

        if (f != nullptr) {
            race::path path_msg;
            rrt.extractPath(f, path_msg);
            rrt.publishPath(path_pub, path_msg);
            //rrt.visualizeTree(rrt.start);
            std::cout<<"ok"<<std::endl;
            //rrt.visualize();
        } else {
            //rrt.visualizeTree(rrt.start);
            ROS_ERROR("Goal was not reached, no path to extract!");
        }

    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception caught in callback: " << e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception caught in callback!");
    }
}


// void callback (const race::perception::ConstPtr& msg, ros::Publisher& marker_pub,ros::Publisher& path_pub){
    
  
//     RRTSTAR rrt(marker_pub);
//     int totalNodes = 0;
//     int totalIterations = 0;
//     leftcones.clear();
//     rightcones.clear();
//     int s=msg->oned_list.size();
//     for(int i=0;i<s;i=i+4){
//       std::vector<double> temp(2,0);
//       temp[0]=msg->oned_list[i];
//       temp[1]=msg->oned_list[i+1];
//       if(temp[0]<0){
//         leftcones.push_back(temp);

//       }
//       else{
//         rightcones.push_back(temp);
//       }
//     }
//     std::reverse(leftcones.begin(),leftcones.end());
//     // for(auto i: leftcones){
//     //   std::cout<<i[0]<<"-"<<i[1]<<std::endl;
//     // }
//     // std::cout<<"-----------------"<<std::endl;
//     // for(auto i: rightcones){
//     //   std::cout<<i[0]<<"-"<<i[1]<<std::endl;
//     // }
//     // std::cout<<"+++++++++++++++++"<<std::endl;

//       // ros::Time current_time = ros::Time::now();

//       // int sec = current_time.sec;      
//       // int nsec = current_time.nsec;    
//       // int microsec = nsec / 1000;

//       // std::time_t current_sec_time = sec;
//       // std::tm* local_time = std::localtime(&current_sec_time);

//       // int minutes = local_time->tm_min;  
//       // int seconds = local_time->tm_sec; 

//       // std::cout << "Minutes: " << minutes << std::endl;
//       // std::cout << "Seconds: " << seconds << std::endl;
//       // std::cout << "Microseconds: " << microsec << std::endl;

//       std::vector<point*> nodes;
//       int n=100;
//       point* f=NULL;
//       for(int i=0;i<n;){
//         totalIterations++;
//         i=i+1;
//         std::vector<int> rand= rrt.randompoint();
//         //std::cout<<"rand="<<rand[0]<<" "<<rand[1]<<std::endl;
//         std::vector<double> r(2,-1.0);
//         r[0]=double(rand[0]);
//         r[1]=double(rand[1]);
        
//         //std::cout<<"r="<<r[0]<<" "<<r[1]<<std::endl;
//         point* nearest = rrt.nearestNode(rrt.start, r);
//         //std::cout<<"nearest="<<nearest->locationX<<" "<<nearest->locationY<<std::endl;
//         std::vector<double> newPoint = rrt.steerToPoint(nearest, r);
//         //std::cout<<"newpoint="<<newPoint[0]<<" "<<newPoint[1]<<std::endl;
//         bool a=rrt.Obstacle(nearest,newPoint);
//         //std::cout<<a<<std::endl;
//         if(!rrt.Obstacle(nearest,newPoint)){
//           std::vector<point*> nearnodes;
//           if(nearest->cost+length<10){
//             point* newNode = rrt.addNode(nearest, newPoint[0], newPoint[1],nearest->cost+length);
//             for(auto i:nodes){
//               double dist=rrt.distanceNaN(i,newNode);
//               if(dist<5){
//                 nearnodes.push_back(i);
//               }
//             }
//             //std::cout<<"hell"<<nodes.size()<<std::endl;
//             //std::cout<<"distcheckok"<<std::endl;
//             nodes.push_back(newNode);
//             rrt.rewire(newNode,nearnodes);
//             //std::cout<<"rewire ok"<<std::endl;
//             if(rrt.GoalReached(newNode)){
//               //std::cout<<"goalreached"<<std::endl;
//               f=newNode;
//               // race::path path_msg;
//               // rrt.extractPath(f, path_msg);
//               // rrt.publishPath(path_pub, path_msg);
//               // rrt.visualize();
//               // rrt.visualizeTree(rrt.start);
//               // break;
//             }
//           }

//         }
//         // else{
//         //   i--;
//         // }
        
        

//         //std::cout<<"-----------------"<<std::endl;
        
//       }
//       std::cout<<"======================================================="<<std::endl;
//       if (f != nullptr) {
//         race::path path_msg;
//         rrt.extractPath(f, path_msg);
//         rrt.publishPath(path_pub, path_msg);
//         //rrt.visualize();
//         //rrt.visualizeTree(rrt.start);
//       } else {
//         //rrt.visualizeTree(rrt.start);
//         ROS_ERROR("Goal was not reached, no path to extract!");
//       }

   
    
    
// }



void call(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    try {
        car_coordinate_x = msg->pose.position.x;
        car_coordinate_y = msg->pose.position.y;
        double q_x = msg->pose.orientation.x;
        double q_y = msg->pose.orientation.y;
        double q_z = msg->pose.orientation.z;
        double q_w = msg->pose.orientation.w;
        tf2::Quaternion quat(q_x, q_y, q_z, q_w);
        tf2::Matrix3x3 m(quat);
        m.getRPY(roll, pitch, yaw);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception caught in call: " << e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception caught in call!");
    }
}


// void call(const geometry_msgs::PoseStamped::ConstPtr& msg){
//     car_coordinate_x=msg->pose.position.x;
//     car_coordinate_y=msg->pose.position.y;
//     double q_x = msg->pose.orientation.x;
//     double q_y = msg->pose.orientation.y;
//     double q_z = msg->pose.orientation.z;
//     double q_w = msg->pose.orientation.w;
//     tf2::Quaternion quat(q_x, q_y, q_z, q_w);
//     tf2::Matrix3x3 m(quat);
//     m.getRPY(roll, pitch, yaw);
//     //std::cout<<"yaw="<<yaw * (180.0 / 3.14)<<std::endl;

// }


int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "rrt");
        ros::NodeHandle nh;
        ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        ros::Publisher path_pub = nh.advertise<race::path>("rrt_to_planning", 1);

        ros::Subscriber sub = nh.subscribe<race::perception>("perception_to_slam", 1,
            [&](const race::perception::ConstPtr& msg) {
                callback(msg, marker_pub, path_pub);
            });
        ros::Subscriber subs = nh.subscribe("gt_pose", 1, call);

        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception caught in main: " << e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception caught in main!");
    }

    return 0;
}


// int main (int argc, char** argv)
// {
//   ros::init (argc, argv, "rrt");
//   ros::NodeHandle nh;
//   ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
//   ros::Publisher path_pub = nh.advertise<race::path>("rrt_to_planning", 1);
//   //ros::Subscriber sub = nh.subscribe ("perception_to_slam", 1000, callback);
//   ros::Subscriber sub = nh.subscribe<race::perception>("perception_to_slam", 1, 
//       [&](const race::perception::ConstPtr& msg) {
//           callback(msg, marker_pub,path_pub);
//       });
//   ros::Subscriber subs = nh.subscribe ("gt_pose", 1, call);

//   ros::spin ();
// }