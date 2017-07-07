/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/

/*********************************************************************
*
*   Versión de Antonio Miguel Morillo Chica para la asignatura de
*   Técnicas de Sistemas Inteligentes de la UGR. Grado de Ingeniería
*   Informática (2016-2017)
*
* ********************************************************************/
#include <string>
#include <sstream>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <visualization_msgs/Marker.h>
#include <pluginlib/class_list_macros.h>
#include "../include/my_astar_planner/myAstarPlanner.h"

#define CALCULO_H 5
#define PESO_H 0

PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)
namespace myastar_planner {
  /**
   * Controla si en un contener de tipo set existe un nodo determinado
   * @param  list1  Set sobre el que se comprueba
   * @param  cellID ID del nodo que se busca
   * @return        true si existe, false en caso contrario
   */
  bool isContainsSet(std::set<myastar_planner::coupleOfCells, myastar_planner::CompareSet> & list1, int cellID);
  /**
   * Controla si en un contener de tipo PQ existe un nodo determinado
   * @param  list1  Lista de prioridad sobre la que se comprueba
   * @param  cellID ID del nodo que se busca
   * @return        true si existe, false en caso contrario
   */
  bool isContainsPQ(std::priority_queue<myastar_planner::coupleOfCells, std::vector<myastar_planner::coupleOfCells>, myastar_planner::CompareQueue> & list1, int cellID);
  set<coupleOfCells>::iterator getPositionInSet(set<coupleOfCells,CompareSet> &list1, unsigned int cellID){
    for (set<coupleOfCells>::iterator it = list1.begin(); it != list1.end() ; ++it)
      if (it->index == cellID)
        return it;
  }

  MyastarPlanner::MyastarPlanner() : costmap_ros_(NULL), initialized_(false){}

  MyastarPlanner::MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  void MyastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);

      //vamos a asumir estos parámetros, que no es necesario enviar desde el launch.
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      //world_model_ = new base_local_planner::CostmapModel(*costmap_);

      //el plan se va a publicar en el topic "planTotal"
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("planTotal",1);
      //los puntos del espacio de búsqueda se visualizan en "visualization_marker"
      marker_Closed_publisher = private_nh.advertise<visualization_msgs::Marker>("closed_list", 1000);
      marker_Open_publisher = private_nh.advertise<visualization_msgs::Marker>("open_list", 1000);
      marker_Goals_publisher = private_nh.advertise<visualization_msgs::Marker>("goals_markers", 1000);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  double MyastarPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    // if we have no footprint... do nothing
    if(footprint.size() < 4)
      return -1.0;
    else{
      return 1;
    }
  }

  bool MyastarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    struct timeval t_ini, t_fin;
    double secs;

    priority_queue<coupleOfCells, std::vector<coupleOfCells>, CompareQueue> openList; //!< the open list: it contains all the expanded cells (current cells)
    set<coupleOfCells,CompareSet> closedList; //!< the closed list: contains the explored cells

    //******************************************************************
    //    Inicio de gestion de ROS
    //******************************************************************

    if(!initialized_){
      ROS_ERROR("The astar planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("MyastarPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    // obtenemos el costmap global  que está publicado por move_base.
    costmap_ = costmap_ros_->getCostmap();

    // Obligamos a que el marco de coordenadas del goal enviado y
    // del costmap sea el mismo. esto es importante para evitar
    // errores de transformaciones de coordenadas.
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
      costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;
    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    // obtenemos la orientación start y goal en start_yaw y goal_yaw.
    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

    //******************************************************************
    //    Fin gestión de ROS
    //******************************************************************

    plan.clear();

    //pasamos el goal y start a estructura coupleOfCells
    coupleOfCells cpstart, cpgoal;
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    unsigned int mgoal_x, mgoal_y;
    costmap_->worldToMap(goal_x,goal_y,mgoal_x, mgoal_y);
    cpgoal.index = MyastarPlanner::costmap_->getIndex(mgoal_x, mgoal_y);
    cpgoal.parent=0;
    cpgoal.gCost=0;
    cpgoal.hCost=0;
    cpgoal.fCost=0;

    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    unsigned int mstart_x, mstart_y;

    costmap_->worldToMap(start_x,start_y, mstart_x, mstart_y);
    cpstart.index = MyastarPlanner::costmap_->getIndex(mstart_x, mstart_y);
    cpstart.parent = cpstart.index;
    cpstart.gCost = 0;
    cpstart.hCost = MyastarPlanner::calculateHCost(cpstart.index,cpgoal.index);

    //insertamos la casilla inicial en abiertos
    openList.push(cpstart);
    ROS_INFO("Inserto en Abiertos: %d", cpstart.index );
    ROS_INFO("Index del goal: %d", cpgoal.index );

    //***********************************************************************
    //     Visualización abiertos y cerrados
    //***********************************************************************

    inicializaMarkersPoints(markers_OpenList,"openList", 0,0.0f,1.0f,0.0f);
    inicializaMarkersPoints(markers_ClosedList,"closedList", 1,1.0f,0.0f,0.0f);
    inicializaMarkersLine_List(markers_Goals, "goals", 2, 0.0f, 0.0f,1.0f);
    limpiaMarkers(marker_Open_publisher, markers_ClosedList);
    limpiaMarkers(marker_Closed_publisher, markers_OpenList);

    //***********************************************************************
    //     Fin visualización
    //***********************************************************************

    //visualizamos start.
    visualizaCelda(marker_Open_publisher, markers_OpenList, cpstart.index);

    unsigned int explorados = 0;
    unsigned int currentIndex = cpstart.index;

    bool solucion  = false;

    gettimeofday(&t_ini, NULL);
    while (!openList.empty() && currentIndex != cpgoal.index && !solucion){
      // 1) Cojo el mejor que es el primero de la priority
      coupleOfCells COfCells = openList.top();
      currentIndex = COfCells.index;

      // 2) Lo inserto en cerrados
      closedList.insert(COfCells);

      visualizaCelda(marker_Closed_publisher, markers_ClosedList, COfCells.index);

      // 2.1) Plan encontrado
      if(currentIndex == cpgoal.index){

        // el plan lo construimos partiendo del goal, del parent del goal y saltando en cerrados "de parent en parent"
        // vamos insertando al final los waypoints (los nodos de cerrados), por tanto, cuando finaliza el bucle hay que darle la vuelta al plan
        ROS_INFO("Se han explorado %u nodos y cerrados tiene %u nodos", explorados, (unsigned int)closedList.size());

        //convertimos goal a poseStamped nueva
        geometry_msgs::PoseStamped pose;
        pose.header.stamp =  ros::Time::now();
        pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el de la entrada
        pose.pose.position.x = goal_x;   pose.pose.orientation.x = 0.0;
        pose.pose.position.y = goal_y;   pose.pose.orientation.y = 0.0;
        pose.pose.position.z = 0.0;      pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        //lo añadimos al plan
        plan.push_back(pose);

        coupleOfCells currentCouple = cpgoal;
        unsigned int currentParent = COfCells.parent;
        ROS_INFO("Inserta en Plan GOAL: %f, %f PADRE: %u", pose.pose.position.x, pose.pose.position.y, currentParent);
        gettimeofday(&t_fin, NULL);
        while (currentParent != cpstart.index){

          std::set<coupleOfCells>::iterator it = getPositionInSet(closedList,currentParent);

          currentCouple.index=currentParent;
          currentCouple.parent=(*it).parent;
          currentCouple.gCost=(*it).gCost;
          currentCouple.hCost=(*it).hCost;
          currentCouple.fCost=(*it).fCost;

          //creamos una PoseStamped con la informaciuón de currentCouple.index
          //primero hay que convertir el currentCouple.index a world coordinates
          unsigned int mpose_x, mpose_y;
          double wpose_x, wpose_y;

          costmap_->indexToCells((*it).index, mpose_x, mpose_y);
          costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);

          ROS_INFO("Las coordenadas de El PADRE de %u son (%u, %u) -> (%f, %f). Y su PADRE es %u.", currentParent, mpose_x,mpose_y,wpose_x, wpose_y, (*it).parent);
          //ros::Duration(1).sleep();
          //después creamos la pose
          geometry_msgs::PoseStamped pose;
          pose.header.stamp =  ros::Time::now();
          pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el de la entrada
          pose.pose.position.x = wpose_x;
          pose.pose.position.y = wpose_y;
          pose.pose.position.z = 0.0;
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;
          //insertamos la pose en el plan
          plan.push_back(pose);
          ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);
          //hacemos que currentParent sea el parent de currentCouple
          currentParent = (*it).parent;
        }
        solucion = true;
        ROS_INFO("Se han explorado %u nodos y cerrados tiene %u nodos", explorados, (unsigned int)closedList.size());
        ROS_INFO("Nodos del plan %u", (unsigned int) plan.size());
        secs = timeval_diff(&t_fin, &t_ini);
        ROS_INFO("Tiempo dusrante la expansión %.6g seg", secs);
        std::reverse(plan.begin(),plan.end());

        //lo publica en el topic "planTotal"
        publishPlan(plan);

        return solucion ;
      }
      else if (COfCells.fCost > (calculateHCost(cpstart.index,cpgoal.index)*3)) {
        // El mejor de abiertos es demasiado malo.
        ROS_INFO("Nodo muy malo");
        // Paro de iterar porque los demás hijos serán malos tambien.
        return false;

      }

      // Como no he acabado, elimino el mejor de abiertos ya que está en cerrados
      openList.pop();

      // Busco las 8 adyacentes
      vector <unsigned int> neighborCells=findFreeNeighborCell(currentIndex);
      ROS_INFO("Ha encontrado %u vecinos", (unsigned int)neighborCells.size());

      // Guardamos los que NO están en CERRADOS
      vector <unsigned int> neighborNotInClosedList;
      for(uint i=0; i<neighborCells.size(); i++)
        if(!isContainsSet(closedList,neighborCells[i]))
          neighborNotInClosedList.push_back(neighborCells[i]);
      ROS_INFO("Ha encontrado %u vecinos que no están en cerrados", (unsigned int)neighborNotInClosedList.size());

      //Buscamos los que ya existen en ABIERTOS
      vector <unsigned int> neighborsInOpenList;
      vector <unsigned int> neighborsNotInOpenList;
      for(uint i=0; i<neighborNotInClosedList.size(); i++){
        if(isContainsPQ(openList,neighborNotInClosedList[i]))
          neighborsInOpenList.push_back(neighborNotInClosedList[i]);
        else
          neighborsNotInOpenList.push_back(neighborNotInClosedList[i]);
      }

      //add the neighbors that are not in the open list to the open list and mark the current cell as their parent
      addNeighborCellsToOpenList(openList, neighborsNotInOpenList, currentIndex, cpstart.gCost, cpgoal.index, cpstart.index);
      explorados++;

      //PINTO ABIERTOS
      //Anyadir neighborCells a points. pushback()
      visualizaLista(marker_Open_publisher, markers_OpenList, neighborsNotInOpenList);
      visualizaCelda(marker_Closed_publisher,markers_ClosedList, COfCells.index);

      //Para los nodos que ya están en abiertos, comprobar en cerrados su coste y actualizarlo si fuera necesario
      // Podemos asumir que en cerrados ya están los mejores nodos posibles.

    }
    if(openList.empty()){
      ROS_INFO("Failure to find a path !");
      return false;
      // exit(1);
    }
    return solucion;
  };

  double MyastarPlanner::timeval_diff(struct timeval *a, struct timeval *b)
  {
    return
      (double)(a->tv_sec + (double)a->tv_usec/1000000) -
      (double)(b->tv_sec + (double)b->tv_usec/1000000);
  }

  double MyastarPlanner::calculateHCost(unsigned int start, unsigned int goal) {
    unsigned int mstart_x, mstart_y,
                 mgoal_x, mgoal_y;

    double wstart_x, wstart_y,
           wgoal_x, wgoal_y;

    //trasformamos el indice de celdas a coordenadas del mundo.
    //ver http://docs.ros.org/indigo/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

    costmap_->indexToCells(start, mstart_x, mstart_y);
    costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
    costmap_->indexToCells(goal, mgoal_x, mgoal_y);
    costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);

    double distancia;
    double dx, dy, dx2, dy2, D, DD;


    switch (CALCULO_H) {
      // Default cuando nos dieron la practica
      case 1:
      dx = std::abs(wstart_x - wgoal_x);
      dy = std::abs(wstart_y - wgoal_y);
      D = 1;

      distancia = D * sqrt(dx * dx + dy * dy);
      break;
      // Distancia euclidia con sobreestimación
      case 2:
        dx = std::abs(wstart_x - wgoal_x);
        dy = std::abs(wstart_y - wgoal_y);
        D = 1;
        distancia = D * (dx * dx + dy * dy);
      break;
      // Distancia octil
      case 3:
        dx = std::abs(wstart_x - wgoal_x);
        dy = std::abs(wstart_y - wgoal_y);
        D = sqrt(2); DD = 1;
        distancia = D * (dx + dy) + (D - 2 * DD) * std::min(dx, dy);
      break;
      // Distancia chebyshev
      case 4:
        dx = std::abs(wstart_x - wgoal_x);
        dy = std::abs(wstart_y - wgoal_y);
        D = 1; DD = 1;
        distancia = D * (dx + dy) + (D - 2 * DD) * std::min(dx, dy);
      break;
      // PArecido a la octil
      case 5:
        dx = std::abs(wstart_x - wgoal_x);
        dy = std::abs(wstart_y - wgoal_y);
        D = sqrt(2); DD = 1;
        distancia = D * std::min(dx, dy) + DD * (std::max(dx,dy) - std::min(dx,dy));
      break;
    }


    return distancia;
   }

  void MyastarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
      ROS_ERROR( "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    } //create a message for the plan

    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty()) {
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    } // Extract the plan in world co-ordinates, we assume the path is all in the same frame

    for (unsigned int i = 0; i < path.size(); i++) {
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }

  bool MyastarPlanner::compareFCost(coupleOfCells const &c1, coupleOfCells const &c2){
     return c1.fCost < c2.fCost;
  }

  vector <unsigned int> MyastarPlanner::findFreeNeighborCell (unsigned int CellID){
    double  wx, wy;
    unsigned int mx, my;
    costmap_->indexToCells(CellID,mx,my);
    ROS_INFO("Viendo vecinos de index: %u, Map coords: (%u,%u)", CellID, mx,my);

    vector <unsigned int>  freeNeighborCells;
    for (int x=-1;x<=1;x++)
      for (int y=-1; y<=1;y++){
        //check whether the index is valid
        //ROS_INFO("A ver: X = %u, Size_X = %u, Y = %u Size_Y = %u",mx+x, (unsigned int)costmap_->getSizeInCellsX(),my+y, (unsigned int)costmap_->getSizeInCellsY());
        if ((mx+x>=0)&&(mx+x < costmap_->getSizeInCellsX())&&(my+y >=0 )&&(my+y < costmap_->getSizeInCellsY())){
          costmap_->mapToWorld( (unsigned int) mx+x, (unsigned int) my+y, wx, wy);
          //ROS_INFO("Comprobando casilla con Map coords(%u,%u), World coords (%f,%f)", mx+x, my+y ,wx,wy);
          if(costmap_->getCost(mx+x,my+y) < 127   && (!(x==0 && y==0))){
            unsigned int index = costmap_->getIndex(mx+x,my+y);
            freeNeighborCells.push_back(index);
          }
        }
      }


    return  freeNeighborCells;
  }

  bool isContainsSet(set<myastar_planner::coupleOfCells, myastar_planner::CompareSet> & list1, int cellID){

    coupleOfCells current; current.index = cellID;
    std::set<coupleOfCells>::iterator it = list1.find(current);

    for (set<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
      if (it->index == cellID)
        return true;
    }
      return false;
  }

  bool isContainsPQ(std::priority_queue<coupleOfCells, std::vector<coupleOfCells>, CompareQueue>& list1, int cellID){

    std::priority_queue<coupleOfCells, std::vector<coupleOfCells>, CompareQueue> aux = list1;
    coupleOfCells element;

    while (!aux.empty()){
      element = aux.top();
      if (element.index == cellID)
        return true;
      else
        aux.pop();
    }

    return false;
  }

  double MyastarPlanner::getMoveCost(unsigned int here, unsigned int there) {
    //calculo el coste de moverme entre celdas adyacentes como la distancia euclídea.
    return calculateHCost(here,there);
  }

  void MyastarPlanner::addNeighborCellsToOpenList(std::priority_queue<coupleOfCells, std::vector<coupleOfCells>, CompareQueue> & OPL, std::vector <unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell, unsigned int startCell){
    double w;
    unsigned int x, y;

    for(uint i=0; i< neighborCells.size(); i++) {
      costmap_->indexToCells(neighborCells[i], x, y);
      coupleOfCells CP;
      CP.index=neighborCells[i]; //insert the neighbor cell
      CP.parent=parent; //insert the parent cell
      CP.gCost = gCostParent + getMoveCost(CP.index, parent);
      CP.hCost = calculateHCost(CP.index, goalCell);
      w = calculateHCost(CP.index,goalCell)/calculateHCost(startCell, goalCell);
      if (footprintCost(x,y,0) < 0) CP.fCost = 1000000000;
      else CP.fCost = CP.gCost + CP.hCost;
      OPL.push(CP);
    }
  }

  /******************* VISUALIZAR ESPACIO DE BUSQUEDA ***************************/

  void MyastarPlanner::inicializaMarkersPoints(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
    marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
    marker.header.stamp =  ros::Time::now();
    marker.ns = ns;

    marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
    marker.pose.orientation.w = 0.0;

    marker.id = id;
    marker.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    marker.scale.x = costmap_->getResolution();
    marker.scale.y = costmap_->getResolution();

    // Points are green
    marker.color.g = g;
    marker.color.r = r;
    marker.color.b = b;
    marker.color.a = 1.0;
  }

  void MyastarPlanner::inicializaMarkersLine_List(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
    marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
    marker.header.stamp =  ros::Time::now();
    marker.ns = ns;

    marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
    marker.pose.orientation.w = 0.0;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;

    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;

    //Line lists also have some special handling for scale: only scale.x is used and it controls the width of the line segments.
    marker.scale.x = marker.scale.y = 0.5;
    // marker.scale.y = costmap_->getResolution();
    // Points are green
    marker.color.g = g;
    marker.color.r = r;
    marker.color.b = b;
    marker.color.a = 1.0;
  }

  void MyastarPlanner::visualizaCoords(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y) {
    //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0; //¿?

    marker.points.push_back(p); //anyado el punto inicial
    where.publish(marker); //lo publico
    //points.points.pop_back(); //quito el punto de la lista de puntos, lo borro con DELETE cuando lo saque de abiertos.
  }

  void MyastarPlanner::visualizaCoordsLineUp(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y, double z) {
    //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    where.publish(marker); //lo publico
    //points.points.pop_back(); //quito el punto de la lista de puntos, lo borro con DELETE cuando lo saque de abiertos.
  }

  void MyastarPlanner::visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, unsigned int index) {
    unsigned int mpose_x, mpose_y;
    double wpose_x, wpose_y;
    costmap_->indexToCells(index, mpose_x, mpose_y);
    costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
    visualizaCoords(where, marker, wpose_x, wpose_y);
  }

  void MyastarPlanner::visualizaLista(ros::Publisher where, visualization_msgs::Marker &marker, vector<unsigned int> lista) {
    for(vector<unsigned int>::iterator i = lista.begin(); i != lista.end(); ++i){
      unsigned int mpose_x, mpose_y;
      double wpose_x, wpose_y;
      costmap_->indexToCells(*i, mpose_x, mpose_y);
      costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
      //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
      geometry_msgs::Point p;
      p.x = wpose_x;
      p.y = wpose_y;
      p.z = 0; //¿?

      marker.points.push_back(p);
    }
    where.publish(marker);
    //quitar neighborCells de points .popback
  }

  void MyastarPlanner::limpiaMarkers(ros::Publisher where, visualization_msgs::Marker &marker) {
      if (!marker.points.empty()){
          marker.action = visualization_msgs::Marker::DELETE;
          where.publish(marker);
          marker.action = visualization_msgs::Marker::ADD;
      }
      marker.points.clear();
  }

}
