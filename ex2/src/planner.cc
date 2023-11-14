/*
 * File name: planner.cc
 * Date:      2016-11-01
 * Author:    Miroslav Kulich, Lukáš Bertl
 */

#include "planner.h"

#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>
#include <queue>

#include "imr-h/logging.h"

using namespace imr;
using namespace std;

CPlanner::CPlanner() {}

/**
    Non-member method. Swaps two integer values.
*/
void SWAP(int &x, int &y) {
  int p;
  p = x;
  x = y;
  y = p;
}

bool CPlanner::bresenham(int x0, int y0, int x1, int y1) {
  int dx = x1 - x0;
  int dy = y1 - y0;
  int steep = (abs(dy) >= abs(dx));
  if (steep) {
    SWAP(x0, y0);
    SWAP(x1, y1);
    // recompute Dx, Dy after swap
    dx = x1 - x0;
    dy = y1 - y0;
  }
  int xstep = 1;
  if (dx < 0) {
    xstep = -1;
    dx = -dx;
  }
  int ystep = 1;
  if (dy < 0) {
    ystep = -1;
    dy = -dy;
  }
  int twoDy = 2 * dy;
  int twoDyTwoDx = twoDy - 2 * dx;  // 2*Dy - 2*Dx
  int e = twoDy - dx;               // 2*Dy - Dx
  int y = y0;
  int xDraw, yDraw;
  for (int x = x0; x != x1; x += xstep) {
    if (steep) {
      xDraw = y;
      yDraw = x;
    } else {
      xDraw = x;
      yDraw = y;
    }
    //TODO: do stuff with (xDraw, yDraw) here
    // moje implementace, nevim jestli je to spravne
    if(inf_map->getCell(xDraw,yDraw) == inf_map->WALL){
      return false;
    }

    if (e > 0) {
      e += twoDyTwoDx;  // E += 2*Dy - 2*Dx;
      y = y + ystep;
    } else {
      e += twoDy;  // E += 2*Dy;
    }
  }
  return true;
}

void CPlanner::setMap(CMapGrid &map) {
  this->map = &map;
  this->inf_map = new CMapGrid(map);
  this->map_height = map.getHeight();
  this->map_width = map.getWidth();
}

void CPlanner::inflateMap(void) {
  int gridSize = 4;
  vector<pair<int, int>> gridPairs; // (y, x)
  vector<pair<int, int>> cornersToRemove = {
    {-4,-4},{-4,-3},{-3,-4},
    {-4,3},{-4,4},{-3,4},
    {3,-4},{4,-4},{4,-3},
    {3,4},{4,3},{4,4}};

  for (int x = -gridSize; x <= gridSize; x++) {
    for (int y = -gridSize; y <= gridSize; y++) {        
      gridPairs.push_back(make_pair(x, y));            
    }
  }

  for (const auto& corner : cornersToRemove) {
    for (size_t i = 0; i < gridPairs.size(); i++) {
      if (gridPairs[i] == corner) {
        gridPairs.erase(gridPairs.begin() + i);
        break; // Move to the next corner
        }
      }
  }

  for (size_t y=0; y<map_height; y++){
    for (size_t x=0; x<map_width; x++){
      if(map->getCell(x, y)==map->WALL){
        for (size_t k=0; k<gridPairs.size(); k++){
          if ((gridPairs[k].second+x>=0 && gridPairs[k].second+x<map_width) && (gridPairs[k].first+y>=0 && gridPairs[k].first+y<map_height)){
            inf_map->setCell(gridPairs[k].second+x, gridPairs[k].first+y, map->WALL);
          }
        }
      }
    }
  }
}

bool CPlanner::dijkstra(int x0, int y0, int x1, int y1) {  
  unordered_map<int,float> visited; // map of cells to their costs
  unordered_map<int,int> trace; // map of cells to their neighbours
  priority_queue<               // priority queue using min heap
    pair<int, pair<int, int>>,    //cost of node, pair<x,y>
    vector<pair<int, pair<int, int>>>,
    greater<pair<int, pair<int, int>>>> pq;
  int cost = 0;
  pair<int,int> position = {x0,y0};
  pair<int,int> goal = {x1,y1};
  int start_index = get_index(x0, y0);
  pq.push({cost, position});
  visited[get_index(x0, y0)] = 0;
  trace[get_index(x0, y0)] = get_index(x0, y0);
  while (!pq.empty()){
    if (position == goal) {
    // Destination reached
      generatePath(trace, goal, start_index);
      return true;
    }
    pq.pop();
    vector<pair<pair<int,int>,float>> new_positions = expand(position.first,position.second);
    for (auto& pos : new_positions){
      pos.second = pos.second + cost;
      auto it = visited.find(get_index(pos.first.first,pos.first.second));
      if(it !=visited.end()){
        if(visited[get_index(pos.first.first,pos.first.second)] > pos.second){
          visited[get_index(pos.first.first,pos.first.second)] = pos.second;
          trace[get_index(pos.first.first,pos.first.second)] = get_index(position.first,position.second);
          pq.push({pos.second, pos.first});
        }
      }
      else{
        visited[get_index(pos.first.first,pos.first.second)] = pos.second;
        trace[get_index(pos.first.first,pos.first.second)] = get_index(position.first,position.second);
        pq.push({pos.second, pos.first});
      }
    }
    if(!pq.empty()){
      position = pq.top().second;
    }
    cost = visited[get_index(position.first,position.second)];
  }
  cout << "Path not found" << endl;
  return false;
}

void CPlanner::smooth_path(void) {
  int front_index = 0;
  int back_index = path.size()-1;
  spath.push_back(path[0]);
  while (front_index != back_index){
    if (bresenham(path[front_index].first,path[front_index].second,path[back_index].first,path[back_index].second)){
      spath.push_back(path[back_index]);
      SWAP(front_index,back_index);
      back_index = path.size()-1;
    }
    else{
      back_index--;
    }
  }
  
}

bool CPlanner::plan(int x0, int y0, int x1, int y1) {
  bool result = true;
  inflateMap();
  dijkstra( x0,  y0,  x1,  y1);
  if (path.size() > 0){
    smooth_path();
  }  
  return result;
}

vector<pair<pair<int,int>,float>> CPlanner::expand(size_t x0, size_t y0){
  vector<pair<pair<int,int>,float>> neighbours;
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {        
      if (x==0 && y==0){
        continue;
      }   
      else if ((x0+x>=0 && x0+x<map_width) && (y0+y>=0 && y0+y<map_height)) {
        if (inf_map->getCell(x0+x, y0+y)!=inf_map->WALL){ 
          if(x*y == -1 || x*y == 1) {            // all corners of the 8-neighbourhood
            neighbours.push_back(make_pair(make_pair(x0+x,y0+y),diagonal_weight));
          }
          else {
            neighbours.push_back(make_pair(make_pair(x0+x,y0+y),straight_weight));
          }
        }
      }       
    }
  }
  return neighbours;
}

int CPlanner::get_index(size_t x, size_t y){
  return y * map_width + x;
}

void CPlanner::generatePath(const unordered_map<int, int>& trace,
const std::pair<int, int>& goal,int start_index) {
//std::stack<std::pair<int, int>> path;
  int current_index = get_index(goal.first, goal.second);

  while (current_index != start_index) {
    path.push_back({current_index % map_width, current_index / map_width});
    auto it = trace.find(current_index);
      if (it != trace.end()) {
      current_index = it->second;
      } 
      else {
            // Key not found, handle appropriately (e.g., print an error message)
            // You might want to break out of the loop or handle the situation in a way that makes sense for your program.
        break;
      }
  }

  // Push the start position
  path.push_back({start_index % map_width, start_index / map_width});
  // for (auto it = path.rbegin(); it != path.rend(); ++it) {
  //       std::pair<int, int> p = *it;
  //       std::cout << "(" << p.first << ", " << p.second << ") ";
  //   }
  // std::cout << std::endl;
}
