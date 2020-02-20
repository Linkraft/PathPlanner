#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
#include "../Framework/TileSystem/TileMap.h"
#include <queue>
#include <vector>
#include <iostream>
#include <unordered_map>
using namespace std;

namespace ufl_cap4053 {
	namespace searches {
		class MapNode {
		public:
			// Constructor(s) & Destructor
			MapNode(Tile* vertex) {
				this->vertex = vertex;
				parent = nullptr;
			}

			MapNode(Tile* vertex, MapNode* parent) {
				this->vertex = vertex;
				this->parent = parent;
			}

			~MapNode() {
				parent = nullptr;
				edges.clear();
			}

			// Getters & Setters
			Tile* getVertex() { return vertex; }

			unordered_map<MapNode*, int> getEdges() { return edges; }
			
			void setEdges(unordered_map<MapNode*, int> edges) {
				this->edges = edges;
			}

			void setParent(MapNode* parent) { this->parent = parent; }

			// Public Functions
			void addEdge(MapNode* edge) {
				int cost = calculateCost(this, edge);
				edges.emplace(edge, cost);
			}

		private:
			// Class Variables
			int x, y;
			unordered_map<MapNode*, int> edges;
			Tile* vertex;
			MapNode* parent;

			// Helper Functions
			int calculateCost(MapNode* node, MapNode* edge) {
				int nodeX = node->getVertex()->getRow();
				int nodeY = node->getVertex()->getColumn();
				int edgeX = edge->getVertex()->getRow();
				int edgeY = edge->getVertex()->getColumn();
				return (int)edge->getVertex()->getWeight();
			}
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
			unordered_map<Tile const*, MapNode*> nodeMap;
			priority_queue<MapNode*> open;
			unordered_map<Tile const*, MapNode*> visited;
			MapNode* start;
			MapNode* goal;
			bool done;
		};

	}
}  // close namespace ufl_cap4053::searches
