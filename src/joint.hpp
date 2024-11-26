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

class Joint : public Sphere {
private:
    static int jointInstances;
    
public:
    Joint(glm::vec3 p, string n) { position = p; name = n; }
    Joint(glm::vec3 p) { position = p; name = "joint" + to_string(jointInstances++); }
    Joint() {}
    
    void draw() override;
    // delete function
    
    float radius = 0.2;
};

#endif /* joint_hpp */
