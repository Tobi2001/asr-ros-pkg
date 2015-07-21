#ifndef VIEW_H_
#define VIEW_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <set>
#include <utility>
#include <string>

namespace ASR {

  struct View {
  
      View() : center(0,0), fov(1,1), score(0), valid(false) {}

    Eigen::Vector2f center; //PTU coords
    Eigen::Vector2f fov;

    std::set<std::pair<std::string, std::string> > objects; //potentially visible objects
    
    double score;
    bool valid;
  
  };

}

#endif /* VIEW_H_ */
