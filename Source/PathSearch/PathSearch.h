#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
#include "../Framework/TileSystem/TileMap.h"
#include <queue>
#include <vector>
#include <iostream>
#include <unordered_map>
using namespace std;

namespace ufl_cap4053 {
	namespace searches {
		struct MapNode {
			// Constructor(s) & Destructor
			MapNode(Tile* vertex) {
				this->vertex = vertex;
				parent = nullptr;
			}

			~MapNode() {
				parent = nullptr;
				edges.clear();
			}
			
			void addEdge(MapNode* edge) {
				if (edge->vertex->getWeight() > 0) edges.push_back(edge);
			}

			// Class Variables
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
			unordered_map<Tile const*, MapNode*> nodeMap;
			queue<MapNode*> open; // Change to priority queue eventually (fixed BFS)
			unordered_map<Tile const*, MapNode*> visited;
			MapNode* start;
			MapNode* goal;
			bool done;
		};

	}
}  // close namespace ufl_cap4053::searches
