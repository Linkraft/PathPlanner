#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
#include "../Framework/TileSystem/TileMap.h"
#include <queue>
#include <vector>
#include <unordered_map>
using namespace std;

namespace ufl_cap4053 {
	namespace searches {
		struct MapNode {

			// Constructor(s) & Destructor
			MapNode(Tile* vertex) {
				row = vertex->getRow();
				col = vertex->getColumn();
				x = vertex->getXCoordinate();
				y = vertex->getYCoordinate();
				givenCost = 0;
				heuristicCost = 0;
				finalCost = 0;
				terrainWeight = vertex->getWeight();
				visited = false;
				this->vertex = vertex;
				parent = nullptr;
			}

			~MapNode() {
				reset();
				vertex = nullptr;
				while (!edges.empty()) edges.pop_back();
				edges.clear();
			}
			
			void addEdge(MapNode* edge) {
				if (edge->terrainWeight > 0) 
					edges.push_back(edge);
			}

			void reset() {
				parent = nullptr;
				visited = false;
			}

			double getHeuristicCost(MapNode* goal) {
				// Calculate straight line distance
				double goalX = goal->vertex->getXCoordinate();
				double goalY = goal->vertex->getYCoordinate();
				return sqrt(pow(goalX - x, 2) + pow(goalY - y, 2));
			}

			// Class Variables
			int row;
			int col;
			double x;
			double y;
			double givenCost;
			double heuristicCost;
			double finalCost;
			int terrainWeight;
			bool visited;
			Tile* vertex;
			MapNode* parent;
			vector<MapNode *> edges;
		};

		class PathSearch {
		public:
			DLLEXPORT PathSearch(); // EX: DLLEXPORT required for public methods - see platform.h
			DLLEXPORT ~PathSearch();
			DLLEXPORT void load(TileMap* _tileMap);
			DLLEXPORT void initialize(int startRow, int startCol, int goalRow, int goalCol);
			DLLEXPORT void update(long timeslice);
			DLLEXPORT void shutdown();
			DLLEXPORT void unload();
			DLLEXPORT bool isDone() const;
			DLLEXPORT std::vector<Tile const*> const getSolution() const;

		private:
			TileMap* tileMap;
			int numColumns;
			int numRows;
			double tileRadius;
			double heuristicWeight;
			unordered_map<Tile const*, MapNode*> nodeMap;
			priority_queue<pair<double, MapNode*>, vector<pair<double, MapNode*>>, greater<pair<double, MapNode*>>> open;
			MapNode* start; 
			MapNode* goal;
			bool done;
		};
	}
}  // close namespace ufl_cap4053::searches
