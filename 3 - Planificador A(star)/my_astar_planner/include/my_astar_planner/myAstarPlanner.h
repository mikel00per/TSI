/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  Copyright (c) 2015, Juan Fdez-Olivares
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
* Author: Juan Fdez-Olivares, Eitan Marder-Eppstein
*********************************************************************/
#ifndef MYASTAR_PLANNER_H_
#define MYASTAR_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> //??
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <visualization_msgs/Marker.h>
#include <vector>
#include <queue>
#include <list>
#include <set>

/**
* @class my_astar_planner
* @brief Clase que implementa una interfaz para crear planes globales
*        con ROS, costmap y con nav_core.
*/
namespace myastar_planner{
  using namespace std;
  /**
  * @struct coupleOfCells
  * @brief A struct that represents a node, that is, a couple of current and parent cells
  */
  struct coupleOfCells {
    unsigned int index;
    unsigned int parent;
    double gCost;
    double hCost;
    double fCost;
  };
  /**
   * Compare para el Set.
   * @return devuelve si el objeto a es mejor que el objeto b
   */
  struct CompareSet{
    bool operator()(const coupleOfCells &a, const coupleOfCells &b){
      if (a.index < b.index) {
        return true;
      }else
        return false;
    }
  };
  /**
   * Compare para la PQ.
   * @return devuelve si el objeto a es mejor que el objeto b
   */
  struct CompareQueue{
    bool operator()(const coupleOfCells &a, const coupleOfCells &b){
      return a.fCost > b.fCost;
    }
  };

  class MyastarPlanner : public nav_core::BaseGlobalPlanner { //implementa la interfaz que provee nav_core::BaseGlobalPlanner
    public:
      /**
       * Constructor sin parámetros
       */
      MyastarPlanner();
      /**
       * Constructor con parámetros, llama a inicializar.
       * @param name        nombre del nodo
       * @param costmap_ros puntero al costmap a usar
       */
      MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      /**
       * Inicializa el mapa de costes y demás
       * @param name        nombre del nodo
       * @param costmap_ros puntero al costmap a usar
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * Construye un plan desde la posición actual a la posición final
       * @param  start Posición del robot
       * @param  goal  Posificón final del robot
       * @param  plan  lista de acciones que se deven ejecutar para llegar al objetivo
       * @return       true si existe un plan, false en caso contrario.
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      double timeval_diff(struct timeval *a, struct timeval *b);

    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      double step_size_, min_dist_from_robot_;
      costmap_2d::Costmap2D* costmap_;
      ros::Publisher plan_pub_;

      /**
       * Comprueba la legalidad o ilegalizad de que la huella del robot y orientación sean correctos.
       * @param  x_i     x de la casilla
       * @param  y_i     y de la casilla
       * @param  theta_i angulo con el que se pretende llegar
       * @return         -1 si no es posible encajar al robot, 1 en caso contrario
       */
      double footprintCost(double x_i, double y_i, double theta_i);
      /**
       * Compara el coste que tienen dos nodos.
       * @param  c1 Nodo uno
       * @param  c2 Nodo dos
       * @return    true si es el coste de c1 es menor que el de c2
       */
      static bool compareFCost(coupleOfCells const &c1, coupleOfCells const &c2);
      /**
       * Añade los hijos colindates a la lista de abiertos..
       * @param ID de la delda del robot.
       * @return vector de ID de los hijos colindantes
       */
      void addNeighborCellsToOpenList(std::priority_queue<coupleOfCells, std::vector<coupleOfCells>, CompareQueue>& OPL, std::vector <unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell, unsigned int startCell);
      /**
       * Calcula el coste de moverse de una casilla a otra
       * @param  here  Inicio
       * @param  there Fin
       * @return       distania entre here y there
       */
      vector <unsigned int> findFreeNeighborCell (unsigned int CellID);
      /**
       * Añade hijos a la lista de abiertos, controlando que el footprintCost no
       * sea malo y poniendo los costos a los hijos.
       * @param OPL           Lista sobre la que añadir
       * @param neighborCells Lista de hijos colindantes libres
       * @param parent        ID del PADRE
       * @param gCostParent   Coste hasta llegar al PADRE
       * @param goalCell      ID goalCell
       * @param startCell     ID startCell
       */
      double getMoveCost(unsigned int here, unsigned int there);
      /**
       * Calculo de la distancia euclidia entre dos nodos.
       * @param  start Nodo uno
       * @param  goal  Nodo dos
       * @return       coste de ir desde el nodo uno al nodo dos en linea recta
       */
      double calculateHCost(unsigned int start, unsigned int goal);
      /**
       * Publica un plan en el planificador global.
       * @param path lista de poses que se han de realizar.
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

//************************************************************************
//    VISUALIZAR ESPACIO DE BUSQUEDA
//************************************************************************
      //para publicar puntos de abiertos y cerrados
      ros::Publisher marker_Open_publisher;
      ros::Publisher marker_Closed_publisher;
      //para publicar las marcas de los goals
      ros::Publisher marker_Goals_publisher;
      //necesario para guardar la lista de puntos de abiertos y cerrados a visualizar como markers en rviz.
      visualization_msgs::Marker markers_OpenList;
      visualization_msgs::Marker markers_ClosedList;
      //necesario para guardar las line_list que marcan los puntos objetivo recomendados.
      visualization_msgs::Marker markers_Goals;
      void inicializaMarkersPoints(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b);
      void inicializaMarkersLine_List(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b);
      void visualizaCoords(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y);
      void visualizaCoordsLineUp(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y, double z);
      void visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, unsigned int cell);
      void visualizaLista(ros::Publisher where, visualization_msgs::Marker &marker, vector<unsigned int> lista);
      void limpiaMarkers(ros::Publisher where, visualization_msgs::Marker &marker);
//************************************************************************
//    FIN ESPACIO DE BUSQUEDA
//************************************************************************

      bool initialized_;
  };
};
#endif
