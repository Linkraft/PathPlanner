#pragma once
#include "PathSearch.h"

namespace ufl_cap4053 {
	namespace searches {

		// The constructor; takes no arguments.
		PathSearch::PathSearch() {
			boundaryX = 0;
			boundaryY = 0;
		}

		// The destructor should perform any final cleanup required before deletion of the object.
		PathSearch::~PathSearch() {
			tileMap.clear();  // All of the MapNode destructors should be called here
		}
		
		// Called after the tile map is loaded. This is usually where the search graph is generated.
		void PathSearch::load(TileMap* _tileMap) {
			// First, add the tile locations into the search map
			for (int col = 0; col < _tileMap->getColumnCount(); col++) {
				for (int row = 0; row < _tileMap->getRowCount(); row++) {
					MapNode* node = new MapNode(_tileMap->getTile(row, col));
					tileMap.emplace(_tileMap->getTile(row, col), node);
				}
			}
			// Then calculate the edges for all the search map nodes
			for (int col = 0; col < _tileMap->getColumnCount(); col++) {
				for (int row = 0; row < _tileMap->getRowCount(); row++) {
					MapNode* node = tileMap.at(_tileMap->getTile(row, col));

					// Create booleans to check boundaries of node
					bool topRow = row - 1 < 0;
					bool bottomRow = row + 1 > _tileMap->getRowCount();
					bool leftColumn = col - 1 < 0;
					bool rightColumn = col + 1 > _tileMap->getColumnCount();

					// Even row nodes have a unique criteria for calculating edges
					if (row % 2 == 0) {
						// do magenta evaluation
					} 
					// ..as do odd row nodes
					else {
						// do purple evaluation
					}
				}
			}
		}

		// Called before any update of the path planner; should prepare for search to be performed between the tiles at
		// the coordinates indicated.
		void PathSearch::initialize(int startRow, int startCol, int goalRow, int goalCol) {
		
		}

		// Called to allow the path planner to execute for the specified timeslice (in milliseconds). Within this method 
		// the search should be performed until the time expires or the solution is found. If timeslice is zero (0), this 
		// method should only do a single iteration of the algorithm. Otherwise the update should only iterate for the 
		// indicated number of milliseconds.
		void PathSearch::update(long timeslice) {
			
		}

		// Called when the current search data is no longer needed. It should clean up any memory allocated for this 
		// search. Note that this is not the same as the destructor, as the search object may be reset to perform another 
		// search on the same map.
		void PathSearch::shutdown() {
			
		}

		// Called when the tile map is unloaded.It should clean up any memory allocated for this tile map.Note that
		// this is not the same as the destructor, as the search object may be reinitialized with a new map.
		void PathSearch::unload() {
			
		}

		// Returns true if the update function has finished because it found a solution, and false otherwise
		bool PathSearch::isDone() const {
			return true;
		}

		// Return a vector containing the solution path as an ordered series of Tile pointers from finish to start.
		vector<Tile const*> const PathSearch::getSolution() const {
			vector<Tile const*> finalPath;
			return finalPath;
		}
		
		bool PathSearch::areAdjacent(const Tile* lhs, const Tile* rhs) {
			return false;
		}

	}
}