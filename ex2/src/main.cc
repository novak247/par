/*
 * File name: main.cc
 * Date:      2019/09/30
 * Author:    Miroslav Kulich
 */

#include <gtkmm/application.h>
#include <gtkmm/window.h>
#include <fstream>
#include <thread>

#include "canvas.h"
#include "map_grid.h"
#include "planner.h"
#include "window.h"

const std::string PROGRAM_VERSION = "0.1";

void planning() {
  imr::CMapGrid gridMap("../maps/arena.txt");
  imr::CPlanner planner;
  imr::CCanvas &canvas = imr::CCanvas::getInstance();
  planner.setMap(gridMap);
  planner.plan(26,13,160,175);
  // draw a map
  //canvas.draw(gridMap);
  canvas.draw(*planner.inf_map);
  
  for (auto it = planner.path.rbegin(); it != planner.path.rend(); ++it) {
        std::pair<int, int> p = *it;
        canvas.drawPixel(p.first ,p.second, imr::Marron);
    }
  for (auto i = planner.spath.rbegin(); i != planner.spath.rend(); ++i) {
        std::pair<int, int> pt = *i;
        if(i+1 != planner.spath.rend()){
          std::pair<int, int> pp = *(i+1);
          canvas.drawLine(pt.first ,pt.second,pp.first,pp.second, imr::Yellow);
        }
          
    }
  // draw pixels with a given color
  // for (int i = 50; i <= 150; i++) {
  //   canvas.drawPixel(i, i, imr::Marron);
  // }

  //draw lines
  //canvas.drawLine(50,50,150,150,imr::Yellow);
  //canvas.drawLine(10,20,100,200,imr::Red);
  //canvas.drawLine(15,20,115,40,imr::Green);
  canvas.redraw();
}

/// ----------------------------------------------------------------------------
/// Main PROGRAM
/// ----------------------------------------------------------------------------
int main(int argc, char **argv) {
  auto app = Gtk::Application::create("cz.cvut.ciirc.imr.par.ex2");

  imr::CWindow win;
  imr::CCanvas &canvas = imr::CCanvas::getInstance();

  win.set_title("PAR - path planner");
  win.set_default_size(1600, 1400);
  win.add(canvas);
  canvas.show();

  std::thread t1(planning);
  app->run(win, argc, argv);
  t1.join();
}

/* end of main.cc */
