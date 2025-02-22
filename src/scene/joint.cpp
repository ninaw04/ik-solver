//
//  joint.cpp
//  Skeleton
//
//  Created by Nina Wang on 11/13/24.
//

#include "joint.hpp"
#include "ofxAssimpModelLoader.h"

int Joint::jointInstances = 0;

void Joint::draw()
{
  glm::mat4 m = getMatrix();

  //   push the current stack matrix and multiply by this object's
  //   matrix. now all vertices dran will be transformed by this matrix
  //
  ofPushMatrix();
  ofMultMatrix(m);
  ofDrawSphere(radius);
  ofPopMatrix();

  for (auto child : childList) {
    glm::mat4 m1 = child->getMatrix();

    // positions of the joints (spheres) in world space
    glm::vec3 p1 = glm::vec3(m * glm::vec4(0, 0, 0, 1));
    glm::vec3 p2 = glm::vec3(m1 * glm::vec4(0, 0, 0, 1));
    float height = glm::distance(p1, p2) - 2 * radius;

    ofSetColor(ofColor::white);

    glm::vec3 coneDir = glm::vec3(0, 1, 0);
    glm::vec3 direction = glm::normalize(p1 - p2);
    glm::mat4 rotationMatrix = rotateToVector(coneDir, direction);
    glm::mat4 coneTransform =
      glm::translate(glm::mat4(1.0f), (p1 + p2) / 2) * rotationMatrix;

    ofPushMatrix();
    ofMultMatrix(coneTransform);
    if (boneModel.getNumMeshes() == 0) {
      ofDrawCone(glm::vec3(0, 0, 0), radius, height);
    } else {
      boneModel.drawFaces();
    }

    ofPopMatrix();
  }
}

void Joint::addModel(std::string file, glm::vec3 position, float scale)
{
  boneModel.load(file);
  bonePos = position;
  boneModel.setPosition(position.x, position.y, position.z);
  boneModel.setScale(scale, scale, scale);
  cout << "created " << file << endl;
}
