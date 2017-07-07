#ifndef FRONTIEREXPLORER_H
#define FRONTIEREXPLORER_H


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>


#include "costmap_2d/costmap_2d_ros.h"
#include <costmap_2d/costmap_2d.h>

#include <visualization_msgs/Marker.h> //para dibujar en rviz

//Tipo de los nodos que forman parte de una frontera

struct nodeOfFrontier {
	double x; //coordenada x de un punto de la frontera
	double y; //coordenada y de un punto de la frontera
};

//La frontera es un vector de nodos de la forma (x,y).
typedef std::vector<nodeOfFrontier> TipoFrontera;

// Para enviar un nodo
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class FrontierExplorer {
	public:
		bool finGirar;
		bool mapIsEmpty; //indica si el mapa está vacío, es decir,  no tiene elementos != -1
		//Guarda la posicion del robot capturada en la callback de odometría
		nodeOfFrontier nodoPosicionRobot;
		//Posición del objetivo del robot (calculada en la función selectNode.
		nodeOfFrontier nodoObjetivo;
		//Lista de las posiciones de los puntos de la frontera detectada. Calculada en la función labelFrontierNodes
		TipoFrontera frontera;
		//guarda el último mensaje de odometría recibido desde la callback de odometría
		nav_msgs::Odometry odometria;
		//Guarda el último mensaje de mapa recibido desde gmapping
		nav_msgs::OccupancyGrid cmGlobal;
		// Matriz que alamacena el mapa, todas las operaciones sobre el mapa van referidas a esta matriz
		std::vector<std::vector<int> > theGlobalCm;
		//Posición del objetivo
		nodeOfFrontier posGoal;
		//Angulo (en radianes) de orientación del robot
		double yaw;
		//velocidad angular para enviar al robot
		double v_angular;
		//velocidad lineal para enviar al robot
		double v_lineal;

		const static double V_ANGULAR_CTE = 2*M_PI/8;  //Valor de la velocidad angular constante.

		//constructor.
		FrontierExplorer();

		//Envia el robot al global
		void send(nodeOfFrontier node) ;

		//Método para que el robot gire 360º
		void gira360();
		//Devuelve true si algún vecino de la celda (x,y), en coordenadas cartesianas, es desconocido
		bool someNeighbourIsUnknown(int cell_x, int cell_y);
		//Devuelve true si algún vecino de la celda (x,y), en coordenadas cartesianas, es un obstáculot
		bool someNeighbourIsObstacle(int cell_x, int cell_y);
		//Método que detecta nodos (celdas) frontera y las almacena en la lista frontera
		void labelFrontierNodes();
		//Método que selecciona un nodo de la frontera para establecer el objetivo al que debe navegar el robot.
		void selectNode(nodeOfFrontier &selectedNode);
		//Borra nodos de la frontera que se encuentren a una distancia (programada en la función) del punto real del mapa (x,y)
		void eraseFrontier(double x, double y);
		// Imprime el mapa en un fichero.
		void printMapToFile();
		//Método que modifica el mapa rellenando obstáculos alrededor de la celda (x,y), en coordenadas cartesianas
		void rellenaObstaculos(int cell_x, int cell_y);

		/*************** VISUALIZACIÓN ***************************************/

		ros::Publisher marker_pub;  //Publicador de  celdas de la frontera en rviz
		ros::Publisher marker_pubobjetivo; //Publicador de objetivo en rviz

		visualization_msgs::Marker markers_frontera; //Contiene la lista de celdas a pintar en rviz.
		visualization_msgs::Marker marker_objetivo; //Contiene el objetivo a dibujar en rviz.

		void inicializaMarkers(std::string ns, int id, float r, float g, float b); //inicializa la lista de celdas frontera a pintar en rviz
		void inicializaMarkerSphere(std::string ns, int id, float r, float g, float b); //inicializa el objetivo a dibujar en rviz.

		void visualizaObjetivo(double x, double y); //Visualiza el objetivo (como una esfera azul) en la posición real del mapa x,y

		void limpiaMarkerObjetivo();//Quita el objetivo de la visualización.

		void visualizaLista(TipoFrontera lista); //Visualiza la lista de celdas de la frontera (en verde)
		void limpiaMarkers();// Elimina de la visualización las celdas actualmente mostradas.


		TipoFrontera getListaFronteras() {return frontera;};

		/************** VISUALIZACIÓN **********************************************/

	protected:
	private:
		ros::NodeHandle node;       //Manejador de nodo.
		ros::Publisher commandPub;  //Publicador de velocidades

		ros::Subscriber laserSub;   //Suscriptor del scan laser
		ros::Subscriber odomSub;    //Suscriptor de la odometría.b
		ros::Subscriber mapSub; //Suscriptor del mapa global

		//void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);
		void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg); //Callback para obtener la pose estimada publicada en el topic /odom
		void getmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);//Callback para obtener el mapa publicado por gmapping en el topic /map
};

#endif // FRONTIEREXPLORER_H
