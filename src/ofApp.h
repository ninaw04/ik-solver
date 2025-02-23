
//
//  Starter file for Project 3 - Skeleton Builder
//
//  This file includes functionality that supports selection and
//  translate/rotation of scene objects using the mouse.
//
//  Modifer keys for rotatation are x, y and z keys (for each axis of rotation)
//
//  (c) Kevin M. Smith  - 24 September 2018

//  This project builds off of the Skeleton Builder for the arm
//  and uses interpolation for animating key frames
//
//  Nina Wang and Viha Shah - December 2024
//

#include "scene/Primitives.h"
#include "scene/box.h"
#include "scene/joint.hpp"
#include "KeyFrame.h"
#include "ofMain.h"
#include "ofxButton.h"
#include "ofxGui.h"
#include "ofxGuiUtils.h"
#include "ofxInputField.h"
#include "ofxSlider.h"

#include "ofxAssimpModelLoader.h"

class ofApp : public ofBaseApp
{

public:
  void setup();
  void update();
  void draw();

  void keyPressed(int key);
  void keyReleased(int key);
  void mouseMoved(int x, int y);
  void mouseDragged(int x, int y, int button);
  void mousePressed(int x, int y, int button);
  void mouseReleased(int x, int y, int button);
  void mouseEntered(int x, int y);
  void mouseExited(int x, int y);
  void windowResized(int w, int h);
  void dragEvent(ofDragInfo dragInfo);
  void gotMessage(ofMessage msg);
  static void drawAxis(glm::mat4 transform = glm::mat4(1.0), float len = 1.0);
  bool mouseToDragPlane(int x, int y, glm::vec3& point);
  void printChannels(shared_ptr<SceneObject> obj);
  bool objSelected()
  {
    return (selected.size() ? true : false);
  };
  void saveToFile();
  void readSkeleton(ofFile skeleton);

  void inverseKin2(glm::vec2 target,
                   Joint& joint1,
                   Joint& joint2,
                   Joint& joint3,
                   vector<pair<glm::vec2, glm::vec2>>& solutions);
  void inverseKin3(glm::vec3 target,
                   Joint& joint1,
                   Joint& joint2,
                   Joint& joint3);
  void handleSolutions(vector<jointDegrees3R>& solutions);

private:
  // Lights
  //
  ofLight light1;
  ofLight light2;

  // Cameras
  //
  ofEasyCam mainCam;

  // Materials
  //
  ofMaterial material;

  // scene components
  //
  vector<shared_ptr<SceneObject>> scene, selected;
  ofPlanePrimitive plane;

  // state
  bool bDrag = false;
  bool bHide = true;
  bool bAltKeyDown = false;
  bool bRotateX = false;
  bool bRotateY = false;
  bool bRotateZ = false;
  glm::vec3 lastPoint;

  // GUI
  ofxPanel gui;
  ofxLabel jointName;
  ofxLabel jointX;
  ofxLabel jointY;
  ofxLabel jointZ;

  ofxFloatSlider miny;
  ofxFloatSlider maxy;
  ofxFloatSlider minz;
  ofxFloatSlider maxz;

  ofTrueTypeFont font;
  int displaySolution;

  // ARM
  shared_ptr<Joint> j1 = make_shared<Joint>();
  shared_ptr<Joint> j2 = make_shared<Joint>();
  shared_ptr<Joint> j3 = make_shared<Joint>();
  shared_ptr<Joint> j4 = make_shared<Joint>();

  // OUTPUT
  int index = 0;
  vector<jointDegrees3R> solutions;
  KeyFrameManager keyFrameManager;
  glm::vec3 WORLDPOINT = glm::vec3(0, 0, 0);
};
