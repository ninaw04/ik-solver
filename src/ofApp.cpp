
//
//  Starter file for Project 3 - Skeleton Builder
//
//  This file includes functionality that supports selection and
//  translate/rotation of scene objects using the mouse.
//
//  Modifer keys for rotatation are x, y and z keys (for each axis of rotation)
//
//  (c) Kevin M. Smith  - 24 September 2018
//

//  Inverse Kinematics solver, implemented with Skeleton Builder
//  and animation frames.
//  Use the mouse to pick an end point for the arm, and it will
//  rotate to that position.
//  IK Reference https://motion.cs.illinois.edu/RoboticSystems/InverseKinematics.html#General-discussion
//
//  Nina Wang and Viha Shah - December 2024
//

#include "ofApp.h"
#include "ofColor.h"
#include "ofGraphics.h"

//--------------------------------------------------------------
//
void ofApp::setup()
{
  // scene and camera setup
  ofSetBackgroundColor(ofColor::black);
  ofEnableDepthTest();
  mainCam.setDistance(15);
  mainCam.setNearClip(.1);

  ofSetSmoothLighting(true);
  ofSetFrameRate(24);

  font.load("fonts/ArgakaFashion-Regular.otf", 24);
  displaySolution = -1;

  // lighting setup
  light1.enable();
  light1.setPosition(5, 5, 0);
  light1.setDiffuseColor(ofColor(255.f, 255.f, 255.f));
  light1.setSpecularColor(ofColor(255.f, 255.f, 255.f));

  light2.enable();
  light2.setPosition(0, 60, 40);
  light2.setDiffuseColor(ofColor(200.f, 200.f, 200.f));
  light2.setSpecularColor(ofColor(255.f, 255.f, 255.f));

  // GUI setup
  gui.setup();
  gui.setPosition(0, 0);
  gui.setSize(500, 20);
  gui.add(jointName.setup(std::string("No Label selected")));
  gui.add(jointX.setup(std::string("0")));
  gui.add(jointY.setup(std::string("0")));
  gui.add(jointZ.setup(std::string("0")));
  gui.add(miny.setup("Min y Angle", 0, 0, 180));
  gui.add(maxy.setup("Max y Angle", 180, -180, 180));
  gui.add(minz.setup("Min z Angle", 0, 0, 180));
  gui.add(maxz.setup("Max z Angle", 180, -180, 180));

  // ground plane
  //
  scene.reserve(4);
  scene.emplace_back(
    make_shared<Plane>(glm::vec3(0, -0.5, 0), glm::vec3(0, 1, 0), ofColor::darkOrchid));

  // Simple 2 R joint arm solution
  j1->addChild(j2);
  j2->addChild(j3);
  j3->addChild(j4);
  
  j1->setPosition(glm::vec3(0, 0, 0));
  j2->setPosition(glm::vec3(4, 0, 0));
  j3->setPosition(glm::vec3(8, 0, 0));
  j4->setPosition(glm::vec3(2, 0, 0));
  
  j1->addModel("arm-assets/shoulder.obj", glm::vec3(0, 2, 0));
  j2->addModel("arm-assets/elbow.obj", glm::vec3(0, 0, 0));
  j3->addModel("arm-assets/endeffector.obj", glm::vec3(0, 3, 0), 0.004);

  scene.emplace_back(j1);
  scene.emplace_back(j2);
  scene.emplace_back(j3);
}

void ofApp::update()
{
  keyFrameManager.update(j1, j2);
}

//--------------------------------------------------------------
void ofApp::draw()
{
  mainCam.begin();
  ofDrawGrid(1);
  ofNoFill();
  drawAxis();
  ofEnableLighting();

  //  User point
  ofDrawSphere(WORLDPOINT, 0.2);

  //  Text for displaying solutions on the screen
  ofSetColor(ofColor::white);
  ofPushMatrix();  // Save the current transformation state
  ofScale(0.07, 0.07);
  font.drawString("Displaying", -70, 160);
  font.drawString("Solution: " + to_string(displaySolution), -70, 130);
  ofPopMatrix();

  //  draw the objects in scene
  //
  material.begin();
  ofFill();
  for (int i = 0; i < scene.size(); i++) {
    if (objSelected() && scene[i] == selected[0])
      ofSetColor(ofColor::white);
    else
      ofSetColor(scene[i]->diffuseColor);
    scene[i]->draw();
  }

  material.end();
  ofDisableLighting();
  mainCam.end();

  //  Joint GUI
  ofDisableDepthTest();
  if (objSelected() && dynamic_pointer_cast<Joint>(selected[0]) != nullptr) {
    shared_ptr<Joint> selectedJoint = dynamic_pointer_cast<Joint>(selected[0]);
    
    auto rotationValue = selected[0]->rotation;
    jointName = "Joint Name: " + selectedJoint->name;
    jointX = "Rotation X: " + std::to_string(rotationValue.x);
    jointY = "Rotation Y: " + std::to_string(rotationValue.y);
    jointZ = "Rotation Z: " + std::to_string(rotationValue.z);

    selectedJoint->yrange.first = miny;
    selectedJoint->yrange.second = maxy;
    selectedJoint->zrange.first = minz;
    selectedJoint->zrange.second = maxz;

  } else {
    jointName = "Joint Name: No joint selected";
    jointX = "Rotation X: 0";
    jointY = "Rotation Y: 0";
    jointZ = "Rotation Z: 0";
  }
  gui.draw();

  ofEnableDepthTest();
}

//
// Draw an XYZ axis in RGB at transform
//
void ofApp::drawAxis(glm::mat4 m, float len)
{

  ofSetLineWidth(1.0);

  // X Axis
  ofSetColor(ofColor(255, 0, 0));
  ofDrawLine(glm::vec3(m * glm::vec4(0, 0, 0, 1)),
             glm::vec3(m * glm::vec4(len, 0, 0, 1)));

  // Y Axis
  ofSetColor(ofColor(0, 255, 0));
  ofDrawLine(glm::vec3(m * glm::vec4(0, 0, 0, 1)),
             glm::vec3(m * glm::vec4(0, len, 0, 1)));

  // Z Axis
  ofSetColor(ofColor(0, 0, 255));
  ofDrawLine(glm::vec3(m * glm::vec4(0, 0, 0, 1)),
             glm::vec3(m * glm::vec4(0, 0, len, 1)));
}

// print C++ code for obj tranformation channels. (for debugging);
//
void ofApp::printChannels(shared_ptr<SceneObject> obj)
{
  cout << "position = glm::vec3(" << obj->position.x << "," << obj->position.y
       << "," << obj->position.z << ");" << endl;
  cout << "rotation = glm::vec3(" << obj->rotation.x << "," << obj->rotation.y
       << "," << obj->rotation.z << ");" << endl;
  cout << "scale = glm::vec3(" << obj->scale.x << "," << obj->scale.y << ","
       << obj->scale.z << ");" << endl;
}

void ofApp::saveToFile()
{
  // traverse skeleton tree and save to file
  string filePath = "skeletons/skelly.txt";

  ofFile skelly;
  skelly.create(filePath);
  skelly.open(filePath, ofFile::WriteOnly);

  for (auto obj : scene) {
    // create -joint “joint name” -rotate “<x, y, z>” -translate “<x, y, z>”
    // -parent “name”;
    if(shared_ptr<Joint> jointObj = dynamic_pointer_cast<Joint>(obj)) {
      char jointBuf[256];
      // if is root, just don't include parent
      if (jointObj->parent == NULL) {
        snprintf(jointBuf,
                 sizeof(jointBuf),
                 "create -joint %s -rotate <%f, %f, %f> -translate <%f, %f, "
                 "%f> -parent none;",
                 jointObj->name.c_str(),
                 jointObj->rotation.x,
                 jointObj->rotation.y,
                 jointObj->rotation.z,
                 jointObj->position.x,
                 jointObj->position.y,
                 jointObj->position.z);
      }
      // translate and rotate values relative to parent
      // string jointBuf = "create -joint " + jointObj->name + " -rotate <%f,
      // %f, %f> -translate <%f, %f, %f> -parent " + jointObj->parent->name,
      // jointObj->rotation.x, jointObj->rotation.y, jointObj->rotation.z,
      // jointObj->position.x, jointObj->position.y, jointObj->position.z;
      else {
        snprintf(jointBuf,
                 sizeof(jointBuf),
                 "create -joint %s -rotate <%f, %f, %f> -translate <%f, %f, "
                 "%f> -parent %s;",
                 jointObj->name.c_str(),
                 jointObj->rotation.x,
                 jointObj->rotation.y,
                 jointObj->rotation.z,
                 jointObj->position.x,
                 jointObj->position.y,
                 jointObj->position.z,
                 jointObj->parent->name.c_str());
      }
      skelly << jointBuf << endl;
    }
  }
}

void ofApp::readSkeleton(ofFile skelly)
{
  if (skelly.exists()) {
    // read file into buffer
    ofBuffer buffer(skelly);
    skelly.close();
    string skeleton = buffer.getText();

    stringstream jointStream(skeleton);
    string jointLine;

    // one line from skelly gets stored in jointLine
    while (getline(jointStream, jointLine)) {
      // create -joint “joint name” -rotate “<x, y, z>” -translate “<x, y, z>”
      // -parent “name”
      vector<string> tokens;
      stringstream ss(jointLine);
      string token;
      char delimiter = ' ';

      string jointName;
      glm::vec3 rot;
      glm::vec3 tran;
      string parentName;

      // create
      getline(ss, token, delimiter);
      if (token == "create") {

        cout << "created" << endl;
        // joint
        getline(ss, token, delimiter);
        if (token == "-joint") {
          // joint name
          getline(ss, jointName, delimiter);

          cout << "jointName" << endl;

          // rotate
          getline(ss, token, delimiter);
          if (token == "-rotate") {
            // <x,y,z>
            getline(ss, token, '<');  // Read until '<'
            getline(ss, token, '>');  // Read until '>'
            sscanf(token.c_str(), "%f, %f, %f", &rot.x, &rot.y, &rot.z);

            cout << "Rotate: " << rot.x << ", " << rot.y << ", " << rot.z
                 << endl;

            // translate
            getline(ss, token, delimiter);  // gets space
            getline(ss, token, delimiter);
            if (token == "-translate") {
              getline(ss, token, '<');  // Read until '<'
              getline(ss, token, '>');  // Read until '>'
              sscanf(token.c_str(), "%f, %f, %f", &tran.x, &tran.y, &tran.z);

              cout << "Translate: " << tran.x << ", " << tran.y << ", "
                   << tran.z << endl;

              // parent
              getline(ss, token, delimiter);  // gets space
              getline(ss, token, delimiter);
              if (token == "-parent") {
                getline(ss, parentName, delimiter);
                cout << "Parent: " << parentName << endl;
              }
            }
          }
        }
      }

      // add rotation
      std::shared_ptr<Joint> curr = std::make_shared<Joint>(tran, jointName);
      curr->rotation = rot;

      // find parent
      // parent name has a ;
      parentName.pop_back();
      if (parentName != "none") {
        for (auto obj : scene) {
          if (obj->name == parentName) {
            obj->addChild(curr);
            break;
          }
        }
      }
      scene.push_back(curr);
    }
  } else {
    cout << "skeleton doesn't exist." << endl;
  }
}

void onLoadSkeletonPressed()
{
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{

  switch (key) {
    case OF_KEY_ALT:
      bAltKeyDown = false;
      mainCam.disableMouseInput();
      break;
    case 'x':
      bRotateX = false;
      break;
    case 'y':
      bRotateY = false;
      break;
    case 'z':
      bRotateZ = false;
      break;
    default:
      break;
  }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
  switch (key) {
    case 'C':
    case 'c':
      if (mainCam.getMouseInputEnabled())
        mainCam.disableMouseInput();
      else
        mainCam.enableMouseInput();
      break;
    case 'F':
    case 'b':
      break;
    case 'f':
      ofToggleFullscreen();
      break;
    case 'h':
      bHide = !bHide;
      break;
    case 'i':
      break;
    case 'j': {
      shared_ptr<Joint> joint = make_shared<Joint>(glm::vec3(1, 1, 1));

      // when selected
      if (objSelected()) {
        if (shared_ptr<Joint> selectedJoint = dynamic_pointer_cast<Joint>(selected[0])) {
          selectedJoint->addChild(joint);
        }
      }
      scene.push_back(joint);
      break;
    }
    case 'l': {
      ofFile file;
      file.open("skeletons/skelly.txt", ofFile::ReadOnly);
      readSkeleton(file);
      break;
    }
    case 's': {
      saveToFile();
      break;
    }
    case 'n':
      // goes to next frame starting from current position
      index++;
      keyFrameManager.resetKeyFrames();
      keyFrameManager.setFirstFrame();
      keyFrameManager.setKeyFrame(index, solutions, displaySolution);
      break;
    case ' ':
      keyFrameManager.bInPlayback = !keyFrameManager.bInPlayback;
      cout << "bInPlayback: " << keyFrameManager.bInPlayback << endl;
      break;
    case 'p':
      if (objSelected())
        printChannels(selected[0]);
      break;
    case 'r':
      break;
    case 'x':
      bRotateX = true;
      break;
    case 'y':
      bRotateY = true;
      break;
    case 'z':
      bRotateZ = true;
      break;
    case OF_KEY_ALT:
      bAltKeyDown = true;
      if (!mainCam.getMouseInputEnabled())
        mainCam.enableMouseInput();
      break;
    case OF_KEY_BACKSPACE: {
      if (objSelected()) {

        if (shared_ptr<Joint> selectedJoint = dynamic_pointer_cast<Joint>(selected[0])) {
          if (selectedJoint->parent == NULL) {
            // remove the whole tree?
          } else {
            for (auto child : selectedJoint->childList) {
              child->parent = selectedJoint->parent;
              selectedJoint->parent->addChild(child);
            }
            selectedJoint->childList.clear();

            auto& siblings = selectedJoint->parent->childList;

            // std::remove reorders the container to avoid invalidating the
            // iterators
            siblings.erase(
              std::remove(siblings.begin(), siblings.end(), selectedJoint),
              siblings.end());
            selectedJoint->parent = NULL;

            shared_ptr<SceneObject> tribute = selected[0];
            scene.erase(std::remove(scene.begin(), scene.end(), tribute),
                        scene.end());
            selected.erase(
              std::remove(selected.begin(), selected.end(), tribute),
              selected.end());
          }
        }
      }
    }
    default:
      break;
  }
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{

  if (objSelected() && bDrag) {
    glm::vec3 point;
    mouseToDragPlane(x, y, point);
  }
}

//  This projects the mouse point in screen space (x, y) to a 3D point on a
//  plane normal to the view axis of the camera passing through the point of the
//  selected object. If no object selected, the plane passing through the world
//  origin is used.
//
bool ofApp::mouseToDragPlane(int x, int y, glm::vec3& point)
{
  glm::vec3 p = mainCam.screenToWorld(glm::vec3(x, y, 0));
  glm::vec3 d = p - mainCam.getPosition();
  glm::vec3 dn = glm::normalize(d);

  float dist;
  glm::vec3 pos;
  if (objSelected()) {
    pos = selected[0]->position;
  } else
    pos = glm::vec3(0, 0, 0);
  if (glm::intersectRayPlane(
        p, dn, pos, glm::normalize(mainCam.getZAxis()), dist)) {
    point = p + dn * dist;
    return true;
  }
  return false;
}

//--------------------------------------------------------------
//
// Provides functionality of single selection and if something is already
// selected, sets up state for translation/rotation of object using mouse.
//
void ofApp::mousePressed(int x, int y, int button)
{
    // if we are moving the camera around, don't allow selection
    if (mainCam.getMouseInputEnabled())
        return;

    // clear selection list
    selected.clear();

    // list of intersected objects
    std::vector<std::shared_ptr<SceneObject>> hits;

    glm::vec3 p = mainCam.screenToWorld(glm::vec3(x, y, 0));
    glm::vec3 d = p - mainCam.getPosition();
    glm::vec3 dn = glm::normalize(d);

    // check for selection of scene objects
    glm::vec3 worldPoint;
    mouseToDragPlane(x, y, worldPoint);
    bool itemIntersected = false;

    for (const auto& obj : scene) {
        glm::vec3 point, norm;

        // We hit an object
        if (obj->isSelectable && obj->intersect(Ray(p, dn), point, norm)) {
            hits.push_back(obj);
            itemIntersected = true;
        }
    }

    if (!itemIntersected) {
        keyFrameManager.startConfig = {
            .rotunda = glm::vec3(0, j1->rotation.y, 0),
            .shoulder = glm::vec3(0, 0, j1->rotation.z),
            .elbow = glm::vec3(0, 0, j2->rotation.z)
        };
        keyFrameManager.resetKeyFrames();
        WORLDPOINT = worldPoint;
        solutions.clear();

        inverseKin3(worldPoint, *j1, *j2, *j3);

        // animation
        keyFrameManager.setFirstFrame();
        keyFrameManager.setKeyFrame(index, solutions, displaySolution);
        keyFrameManager.bInPlayback = true;
    }

    // if we selected more than one, pick nearest
    std::shared_ptr<SceneObject> selectedObj = nullptr;
    if (!hits.empty()) {
        selectedObj = hits[0];
        float nearestDist = std::numeric_limits<float>::infinity();

        for (const auto& obj : hits) {
            float dist = glm::length(obj->position - mainCam.getPosition());
            if (dist < nearestDist) {
                nearestDist = dist;
                selectedObj = obj;
            }
        }
    }

    if (selectedObj) {
        selected.push_back(selectedObj);
        bDrag = true;
        mouseToDragPlane(x, y, lastPoint);

        if (objSelected()) {
            auto selectedJoint = std::dynamic_pointer_cast<Joint>(selected[0]);
            if (selectedJoint) {
                // setting initial slider constraints to object constraints
                miny = selectedJoint->yrange.first;
                maxy = selectedJoint->yrange.second;
                minz = selectedJoint->zrange.first;
                maxz = selectedJoint->zrange.second;

                // now allowing user to set slider constraints
                selectedJoint->constraints.minx = selectedJoint->xrange.first;
                selectedJoint->constraints.maxx = selectedJoint->xrange.second;
                selectedJoint->constraints.miny = selectedJoint->yrange.first;
                selectedJoint->constraints.maxy = selectedJoint->yrange.second;
                selectedJoint->constraints.minz = selectedJoint->zrange.first;
                selectedJoint->constraints.maxz = selectedJoint->zrange.second;
            }
        }
    } else {
        selected.clear();
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
  bDrag = false;
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{
}

//--------------------------------------------------------------
// Computes 2D inverse kinematics for planar 2DOF arm
// returns elbow and shoulder rotation (joint 1 and 2)
//
void ofApp::inverseKin2(glm::vec2 target,
                        Joint& joint1,
                        Joint& joint2,
                        Joint& joint3,
                        vector<pair<glm::vec2, glm::vec2>>& solutions)
{
  // calculate total reach
  double reach = glm::distance(joint1.getPosition(), joint2.getPosition()) +
                 glm::distance(joint2.getPosition(), joint3.getPosition());

  double targetLen = glm::length(target);
  double bone1 = glm::distance(joint1.getPosition(), joint2.getPosition());
  double bone2 = glm::distance(joint2.getPosition(), joint3.getPosition());
  
  // law of cosines to find elbow angle c2
  double numerator =
    glm::pow(targetLen, 2) - glm::pow(bone1, 2) - glm::pow(bone2, 2);
  double denominator = 2 * bone1 * bone2;
  double c2 = 0;

  glm::vec2 rot1, rot2;

  if (denominator == 0) {
    printf("pattern says this is inappropriate\n");
    return;
  } else {
    c2 = numerator / denominator;
  }

  if (glm::abs(c2) > 1) { // target is unreachable
    printf("no bueno target location\n");
    return;
  } else if (c2 == 1) { // arm is fully extended {(𝑎𝑡𝑎𝑛2(𝑥𝑦,𝑥𝑥),0)}
    rot1 = glm::degrees(glm::vec2(0, atan2(target.y, target.x)));
    rot2 = glm::vec2(0, 0);
    solutions.push_back(pair(rot1, rot2));
  } else if (c2 == -1 and target != glm::vec2(0,0)) {  // arm is fully folded, if 𝑐2=−1 and 𝐱𝐷≠0 then return {(𝑎𝑡𝑎𝑛2(𝑥𝑦,𝑥𝑥),𝜋)}
    rot1 = glm::degrees(glm::vec2(0, atan2(target.y, target.x)));
    rot2 = glm::degrees(glm::vec2(0, PI));
    solutions.push_back(pair(rot1, rot2));
  } else if (c2 == -1 and target == glm::vec2(0, 0)) {  // return {(𝑞1,𝜋)|𝑞1∈[0,2𝜋)}
    // as long as q2 is pi, q1 can be anything, we are returning the original
    // joint rotation
    rot1 = joint1.rotation;                 // (q1)
    rot2 = glm::degrees(glm::vec2(0, PI));  // pi
    solutions.push_back(pair(rot1, rot2));
  } else {  // two possible elbow angles, let 𝑞(1)2←cos−1𝑐2 and 𝑞(2)2←−cos−1𝑐2
    
    double theta = glm::degrees(atan2(target.y, target.x));
    for (int k = 1; k <= 2; k++) {  // 𝑞(𝑘)1=𝜃−𝑎𝑡𝑎𝑛2(𝐿2sin𝑞(𝑘)2,𝐿1+𝐿2cos𝑞(𝑘)2)
      // positive and negative q2
      rot2 = (k == 1) ? glm::degrees(glm::vec2(0, glm::acos(c2)))
                      : glm::degrees(glm::vec2(0, -glm::acos(c2)));

      double endEffHeading =
        glm::degrees(atan2(bone2 * sin(glm::radians(rot2.y)),
                           bone1 + (bone2 * cos(glm::radians(rot2.y)))));
      double joint1RotationZ = theta - endEffHeading;

      rot1 = glm::vec2(0, joint1RotationZ);
      solutions.push_back(pair(rot1, rot2));
    }
  }
}

//--------------------------------------------------------------
// Extends inverseKin2 to 3D space
// First project 3D problem into 2D space
// Call inverseKin2 twice to get positive and negative solutions
// Compute rotunda rotation
//
void ofApp::inverseKin3(glm::vec3 target,
                        Joint& joint1,
                        Joint& joint2,
                        Joint& joint3)
{
  // calculate shoulder and elbow rotation about the z plane
  double shoulderOffset = 0;  // l1 is offset from rotunda to shoulder
  glm::vec2 targetVec2Pos = glm::vec2(sqrt(pow(target.x, 2) + pow(target.z, 2)),
                                      target.y + shoulderOffset);
  glm::vec2 targetVec2Neg = glm::vec2(
                                      -sqrt(pow(target.x, 2) + pow(target.z, 2)), target.y + shoulderOffset);
  
  vector<pair<glm::vec2, glm::vec2>> solutionPairs1, solutionPairs2;
  inverseKin2(targetVec2Pos, joint1, joint2, joint3, solutionPairs1);
  inverseKin2(targetVec2Neg, joint1, joint2, joint3, solutionPairs2);
  
  double joint1Angle = glm::degrees(
                                    glm::atan(target.z, target.x));  // rotunda rotation about the y
  
  // convert all solution pairs into solution triplets
  for (int i = 0; i < 2; i++) {
    float offset = (i == 0) ? 0 : 180;
    auto& solutionPairs = (i == 0) ? solutionPairs1 : solutionPairs2;
    
    for (const auto& sol : solutionPairs) {
      jointDegrees3R config;
      config.rotunda = glm::vec3(0, -(joint1Angle + offset), 0);
      config.shoulder = glm::vec3(sol.first[0], 0, sol.first[1]);  // y and z are flipped
      config.elbow = glm::vec3(sol.second[0], 0, sol.second[1]);  // y and z are flipped
      
      // check if valid within constraints
      if (config.rotunda.y >= j1->yrange.first &&
          config.rotunda.y <= j1->yrange.second &&
          config.shoulder.z >= j1->zrange.first &&
          config.shoulder.z <= j1->zrange.second &&
          config.elbow.z >= j2->zrange.first &&
          config.elbow.z <= j2->zrange.second) {
        solutions.push_back(config);
      }
    }
  }
}
