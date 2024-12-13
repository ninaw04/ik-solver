//
//  joint.hpp
//  Skeleton
//
//  Created by Nina Wang on 11/13/24.
//

#ifndef joint_hpp
#define joint_hpp

#include <stdio.h>
#include "Primitives.h"
//#include "fwd.hpp"
#include "ofxAssimpModelLoader.h"


struct constraints3R {
    float minx;
    float maxx;
    float miny;
    float maxy;
    float minz;
    float maxz;
};


class Joint : public Sphere {
private:
    static int jointInstances;
    
public:
    Joint(glm::vec3 p, string n) { this->setPosition(p); name = n; }
    Joint(glm::vec3 p) { this->setPosition(p); name = "joint" + to_string(jointInstances++); }
    Joint() { position = glm::vec3(0,0,0); name = "joint" + to_string(jointInstances++); }

    void draw() override;
//    void update();
    void addModel(std::string fileName, glm::vec3 position);
  
    // delete function
    // add a range for x, y, z constraints per joint
  std::pair<int, int> xrange = {-360, 360};
  std::pair<int, int> yrange = {-360, 360};
  std::pair<int, int> zrange = {-360, 360};
  

    // x, y, z plane
    char plane = 'y';
    glm::vec3 bonePos;
    bool xConstraint = false;
    bool yConstraint = false;
    bool zConstraint = false;
    ofxAssimpModelLoader boneModel;
    float radius = 0.2;
  
    constraints3R constraints;

};

#endif /* joint_hpp */
