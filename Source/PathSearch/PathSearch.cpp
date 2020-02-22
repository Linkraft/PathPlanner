#pragma once
#include <chrono>
#include "PathSearch.h"
using namespace std::chrono;

namespace ufl_cap4053 {
	namespace searches {
		// The constructor; takes no arguments.
		PathSearch::PathSearch() {
			numColumns = 0;
			numRows = 0;
			tileRadius = 0;
			heuristicWeight = 1.2;
			tileMap = nullptr;
			start = nullptr;
			goal = nullptr;
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
			numColumns = tileMap->getColumnCount();
			numRows = tileMap->getRowCount();
			tileRadius = tileMap->getTileRadius();

			// First, add the tile locations into the search map
			for (int col = 0; col < numColumns; col++) {
				for (int row = 0; row < numRows; row++) {
					MapNode* node = new MapNode(tileMap->getTile(row, col));
					nodeMap.emplace(tileMap->getTile(row, col), node);
				}
			}

			// Then calculate the edges for all the search map nodes
			for (int col = 0; col < numColumns; col++) {
				for (int row = 0; row < numRows; row++) {
					MapNode* node = nodeMap.at(tileMap->getTile(row, col));

					// Create booleans to check position of node against boundaries
					bool topRow = row - 1 < 0;
					bool bottomRow = row + 1 >= numRows;
					bool leftmostColumn = col - 1 < 0;
					bool rightmostColumn = col + 1 >= numColumns;

					// Even row nodes (cyan) have a unique criteria for calculating edges
					if (row % 2 == 0) {
						if (!topRow)					   // North
							node->addEdge(nodeMap.at(tileMap->getTile(row - 1, col)));
						if (!topRow && !leftmostColumn)    // Northwest
							node->addEdge(nodeMap.at(tileMap->getTile(row - 1, col - 1)));
						if (!leftmostColumn)			   // West
							node->addEdge(nodeMap.at(tileMap->getTile(row, col - 1)));
						if (!leftmostColumn && !bottomRow) // Southwest
							node->addEdge(nodeMap.at(tileMap->getTile(row + 1, col - 1)));
						if (!bottomRow)					   // South
							node->addEdge(nodeMap.at(tileMap->getTile(row + 1, col)));
						if (!rightmostColumn)			   // East
							node->addEdge(nodeMap.at(tileMap->getTile(row, col + 1)));
					} 
					// ..as do odd row nodes (magenta)
					else {
						if (!topRow)						// North
							node->addEdge(nodeMap.at(tileMap->getTile(row - 1, col)));
						if (!topRow && !rightmostColumn)	// Northeast
							node->addEdge(nodeMap.at(tileMap->getTile(row - 1, col + 1)));
						if (!rightmostColumn)				// East
							node->addEdge(nodeMap.at(tileMap->getTile(row, col + 1)));
						if (!rightmostColumn && !bottomRow) // Southeast
							node->addEdge(nodeMap.at(tileMap->getTile(row + 1, col + 1)));
						if (!bottomRow)						// South
							node->addEdge(nodeMap.at(tileMap->getTile(row + 1, col)));
						if (!leftmostColumn)				// West
							node->addEdge(nodeMap.at(tileMap->getTile(row, col - 1)));
					}
				}
			}

		}

		// Called before any update of the path planner; should prepare for search to be performed between the tiles at
		// the coordinates indicated.
		void PathSearch::initialize(int startRow, int startCol, int goalRow, int goalCol) {
			done = false;
			start = nodeMap.at(tileMap->getTile(startRow, startCol));
			goal = nodeMap.at(tileMap->getTile(goalRow, goalCol));
			start->visited = true;
			open.push(make_pair(0, start));
		}


		// Called to allow the path planner to execute for the specified timeslice (in milliseconds). Within this method 
		// the search should be performed until the time expires or the solution is found. If timeslice is zero (0), this 
		// method should only do a single iteration of the algorithm. Otherwise the update should only iterate for the 
		// indicated number of milliseconds.
		void PathSearch::update(long _timeslice) {
			// Initialize timer variables
			milliseconds timeslice = milliseconds((long long)_timeslice);
			high_resolution_clock::time_point start = high_resolution_clock::now();
			duration<double, std::milli> elapsedTime(0);

			while (elapsedTime.count() <= timeslice.count()) {
				// Get the top MapNode from the pqueue
				MapNode* current = open.top().second;
				open.pop();

				// Mark it for debugging and display its coordinates
				current->vertex->setFill(0xFF7F0000);

				// Check if the current MapNode is the goal MapNode
				if (current == goal) {
					done = true;
					return;
				}

				// Place all of the current MapNode's unvisited edge MapNodes into the queue
				for (MapNode* edge : current->edges) {
					double edgeCost = (double)edge->terrainWeight * 2 * tileRadius;						 // Calculate edge cost
					double newCost = current->givenCost + edgeCost;										 // Calculate given cost for this edge
					if (!edge->visited) {																 // If we haven't visited this MapNode before
						edge->visited = true;															 // Mark MapNode as visited
						//edge->vertex->setFill(0xff2e00fa);											 // Fill the vertex with color to indicate edginess
						edge->givenCost = newCost;														 // Initialize the edge's givenCost
						edge->heuristicCost = edge->getHeuristicCost(goal);								 // Initialize edge with its unchanging heuristic cost
						edge->finalCost = edge->givenCost + (edge->heuristicCost * heuristicWeight);	 // Calculate the final cost of the edge
						edge->parent = current;															 // Update this edge MapNode's parent to be the current node
						open.push(make_pair(edge->finalCost, edge));									 // Place it in the priority queue for later iterations
					}
					else {																				 // Check if new path is better than old path
						if (newCost < edge->givenCost) {												 // Update edge and reinsert it into the queue
							edge->givenCost = newCost;
							edge->finalCost = edge->givenCost + (edge->heuristicCost * heuristicWeight); // Recalculate the final cost of the edge
							edge->parent = current;
 							open.push(make_pair(edge->finalCost, edge));
						}
					}
				}

				// Update time and check if we should stop
				elapsedTime = high_resolution_clock::now() - start;
			}
		}

		// Called when the current search data is no longer needed. It should clean up any memory allocated for this 
		// search. Note that this is not the same as the destructor, as the search object may be reset to perform another 
		// search on the same map.
		void PathSearch::shutdown() {
			for (std::pair<Tile const*, MapNode*> mapping : nodeMap) mapping.second->reset();
			while (!open.empty()) open.pop();
		}

		// Called when the tile map is unloaded.It should clean up any memory allocated for this tile map.Note that
		// this is not the same as the destructor, as the search object may be reinitialized with a new map.
		void PathSearch::unload() {
			shutdown();
			for (std::pair<Tile const*, MapNode*> mapping : nodeMap) delete mapping.second;
			nodeMap.clear();
		}

		// Returns true if the update function has finished because it found a solution, and false otherwise
		bool PathSearch::isDone() const {
			return done;
		}

		// Return a vector containing the solution path as an ordered series of Tile pointers from finish to start.
		vector<Tile const*> const PathSearch::getSolution() const {
			vector<Tile const*> finalPath;
			MapNode* current = goal;
			finalPath.push_back(goal->vertex);
			while (current->parent != nullptr) {
				finalPath.push_back(current->parent->vertex);
				current = current->parent;
			}
			return finalPath;
		}
	}
} 