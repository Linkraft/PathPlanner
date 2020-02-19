#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
#include "../Framework/TileSystem/TileMap.h"
#include <vector>
#include <unordered_map>
using namespace std;

namespace ufl_cap4053 {
	namespace searches {
		class MapNode {
		public:
			// Constructor(s) & Destructor
			MapNode(Tile* tile) {
				x = tile->getColumn();
				y = tile->getRow();
				parent = nullptr;
			}

			~MapNode() {
				parent = nullptr;
				edges.clear();
			}

			// Getters & Setters
			int getX() { return x; }

			int getY() { return y; }
			
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
			MapNode* parent;

			// Helper Functions
			int calculateCost(MapNode* node, MapNode* edge) {
				int nodeX = node->getX();
				int nodeY = node->getY();
				int edgeX = edge->getX();
				int edgeY = edge->getY();
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
			DLLEXPORT bool areAdjacent(const Tile *lhs, const Tile* rhs);
			unordered_map<Tile const*, MapNode*> tileMap;
			int boundaryX, boundaryY;
		};

	}
}  // close namespace ufl_cap4053::searches
