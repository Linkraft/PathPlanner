#pragma once
#include <chrono>
#include "PathSearch.h"
using namespace std::chrono;

namespace ufl_cap4053 {
	namespace searches {

		// The constructor; takes no arguments.
		PathSearch::PathSearch() {
			done = false;
		}

		// The destructor should perform any final cleanup required before deletion of the object.
		PathSearch::~PathSearch() {
			shutdown();
			unload();
		}
		
		// Called after the tile map is loaded. This is usually where the search graph is generated.
		void PathSearch::load(TileMap* _tileMap) {
			tileMap = _tileMap;
			// First, add the tile locations into the search map
			for (int col = 0; col < tileMap->getColumnCount(); col++) {
				for (int row = 0; row < tileMap->getRowCount(); row++) {
					MapNode* node = new MapNode(tileMap->getTile(row, col));
					nodeMap.emplace(tileMap->getTile(row, col), node);
				}
			}
			// Then calculate the edges for all the search map nodes
			for (int col = 0; col < tileMap->getColumnCount(); col++) {
				for (int row = 0; row < tileMap->getRowCount(); row++) {
					MapNode* node = nodeMap.at(tileMap->getTile(row, col));

					// Create booleans to check position of node against boundaries
					bool topRow = row - 1 < 0;
					bool bottomRow = row + 1 >= tileMap->getRowCount();
					bool leftmostColumn = col - 1 < 0;
					bool rightmostColumn = col + 1 >= tileMap->getColumnCount();

					// Even row nodes (cyan) have a unique criteria for calculating edges
					if (row % 2 == 0) {
						if (!topRow)					   // North
							if (tileMap->getTile(row - 1, col)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row - 1, col)));
						if (!topRow && !leftmostColumn)    // Northwest
							if (tileMap->getTile(row - 1, col- 1)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row - 1, col - 1)));
						if (!leftmostColumn)			   // West
							if (tileMap->getTile(row, col - 1)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row, col - 1)));
						if (!leftmostColumn && !bottomRow) // Southwest
							if (tileMap->getTile(row + 1, col - 1)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row + 1, col - 1)));
						if (!bottomRow)					   // South
							if (tileMap->getTile(row + 1, col)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row + 1, col)));
						if (!rightmostColumn)			   // East
							if (tileMap->getTile(row, col + 1)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row, col + 1)));
					} 
					// ..as do odd row nodes (magenta)
					else {
						if (!topRow)						// North
							if (tileMap->getTile(row - 1, col)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row - 1, col)));
						if (!topRow && !rightmostColumn)	// Northeast
							if (tileMap->getTile(row - 1, col + 1)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row - 1, col + 1)));
						if (!rightmostColumn)				// East
							if (tileMap->getTile(row, col + 1)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row, col + 1)));
						if (!rightmostColumn && !bottomRow) // Southeast
							if (tileMap->getTile(row + 1, col + 1)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row + 1, col + 1)));
						if (!bottomRow)						// South
							if (tileMap->getTile(row + 1, col)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row + 1, col)));
						if (!leftmostColumn)				// West
							if (tileMap->getTile(row, col- 1)->getWeight() > 0) node->addEdge(nodeMap.at(tileMap->getTile(row, col - 1)));
					}
					//cout << "Edge count for (" << node->getVertex()->getColumn() << ", " << node->getVertex()->getRow() << "): " << node->getEdges().size() << endl;
					//for (std::pair<MapNode*, int> edge : node->getEdges()) cout << "   Edge: (" << edge.first->getVertex()->getColumn() << ", " << edge.first->getVertex()->getRow() << ")" << endl;
				}
			}
		}

		// Called before any update of the path planner; should prepare for search to be performed between the tiles at
		// the coordinates indicated.
		void PathSearch::initialize(int startRow, int startCol, int goalRow, int goalCol) {
			start = nodeMap.at(tileMap->getTile(startRow, startCol));
			goal = nodeMap.at(tileMap->getTile(goalRow, goalCol));
			open.push(start);
			visited.emplace(tileMap->getTile(goalRow, goalCol), start);
		}

		// Called to allow the path planner to execute for the specified timeslice (in milliseconds). Within this method 
		// the search should be performed until the time expires or the solution is found. If timeslice is zero (0), this 
		// method should only do a single iteration of the algorithm. Otherwise the update should only iterate for the 
		// indicated number of milliseconds.
		void PathSearch::update(long _timeslice) { // c_time
			milliseconds timeslice = milliseconds((long long)_timeslice);
			high_resolution_clock::time_point start = high_resolution_clock::now();
			duration<double, std::milli> elapsedTime = milliseconds(0);

			cout << "Timeslice: " << timeslice.count() << endl;
			while (open.size() != 0) {
				MapNode* current = open.top();
				open.pop();
				current->getVertex()->setFill(0);
				cout << "Current coordinates: (" << current->getVertex()->getColumn() << ", " << current->getVertex()->getRow() << ")" << endl;
				//cout << "Number of edges: " << current->getEdges().size() << endl;

				if (current->getVertex() == goal->getVertex()) {
					cout << "-=-=-=-=-=-=-=-=-=- GOAL FOUND! -=-=-=-=-=-=-=-=-=-" << endl;
					done = true;
					return;
				}

				unordered_map<MapNode*, int> edges = current->getEdges();

				for (auto edge = edges.begin(); edge != edges.end(); ++edge) {
					MapNode* successor = edge->first;
					//cout << "   Edge of current: (" << successor->getVertex()->getColumn() << ", " << successor->getVertex()->getRow() << ")" << endl;
					if (visited.find(successor->getVertex()) == visited.end()) {
						MapNode* newNode = new MapNode(successor->getVertex(), current);
						newNode->setEdges(successor->getEdges());
						visited[successor->getVertex()] = newNode;
						open.push(newNode);
						//cout << "Open queue size: " << open.size() << endl;
					}
				}

				elapsedTime = high_resolution_clock::now() - start;
				//cout << "Elapsed Time: " << elapsedTime.count() << endl;
				if (elapsedTime.count() < timeslice.count()) {
					cout << "-=-=-=-=-=-=-=-=-=- TIME'S UP! -=-=-=-=-=-=-=-=-=-" << endl;
					return;
				}
			}
		}

		// Called when the current search data is no longer needed. It should clean up any memory allocated for this 
		// search. Note that this is not the same as the destructor, as the search object may be reset to perform another 
		// search on the same map.
		void PathSearch::shutdown() {
			while (!open.empty()) open.pop();
			visited.clear();
		}

		// Called when the tile map is unloaded.It should clean up any memory allocated for this tile map.Note that
		// this is not the same as the destructor, as the search object may be reinitialized with a new map.
		void PathSearch::unload() {
			nodeMap.clear();
		}

		// Returns true if the update function has finished because it found a solution, and false otherwise
		bool PathSearch::isDone() const {
			return done;
		}

		// Return a vector containing the solution path as an ordered series of Tile pointers from finish to start.
		vector<Tile const*> const PathSearch::getSolution() const {
			vector<Tile const*> finalPath;
			
			return finalPath;
		}

	}
}