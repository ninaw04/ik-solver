
//
//  Starter file for Project 3 - Skeleton Builder
//
//  This file includes functionality that supports selection and translate/rotation
//  of scene objects using the mouse.
//
//  Modifer keys for rotatation are x, y and z keys (for each axis of rotation)
//
//  (c) Kevin M. Smith  - 24 September 2018
//


#include "ofApp.h"



//--------------------------------------------------------------
//
void ofApp::setup() {
    gui.setup();

    // setting up skeleton loader
//    loadSkeleton.setup("SkeletonLoader", "")
//    skeletonPath.addListener(this, &ofApp::onLoadSkeletonPressed);
    

    ofSetBackgroundColor(ofColor::black);
    ofEnableDepthTest();
    mainCam.setDistance(15);
    mainCam.setNearClip(.1);
    
    sideCam.setPosition(40, 0, 0);
    sideCam.lookAt(glm::vec3(0, 0, 0));
    topCam.setNearClip(.1);
    topCam.setPosition(0, 16, 0);
    topCam.lookAt(glm::vec3(0, 0, 0));
    ofSetSmoothLighting(true);


    // setup one point light
    //
    light1.enable();
    light1.setPosition(5, 5, 0);
    light1.setDiffuseColor(ofColor(255.f, 255.f, 255.f));
    light1.setSpecularColor(ofColor(255.f, 255.f, 255.f));

    theCam = &mainCam;

    //  create a scene consisting of a ground plane with 2x2 blocks
    //  arranged in semi-random positions, scales and rotations
    //
    // ground plane
    //
    scene.push_back(new Plane(glm::vec3(0, -2, 0), glm::vec3(0, 1, 0)));
    
    // joints
//    Joint *j1 = new Joint(glm::vec3(0,5,0), "j1");
//    Joint *j2 = new Joint(glm::vec3(0.1,2,0), "j2");
//    Joint *j3 = new Joint(glm::vec3(2,2,2), "j3");
//    
//    j1->addChild(j2);
//    j2->addChild(j3);
//    
//    scene.push_back(j1);
//    scene.push_back(j2);
//    scene.push_back(j3);
}

 
//--------------------------------------------------------------
void ofApp::update(){
}

//--------------------------------------------------------------
void ofApp::draw(){

    theCam->begin();
    ofNoFill();
    drawAxis();
    ofEnableLighting();

    //  draw the objects in scene
    //
    material.begin();
    ofFill();
    for (int i = 0; i < scene.size(); i++) {
        if (objSelected() && scene[i] == selected[0])
            ofSetColor(ofColor::white);
        else ofSetColor(scene[i]->diffuseColor);
        scene[i]->draw();
    }

    material.end();
    ofDisableLighting();
    theCam->end();
}

//
// Draw an XYZ axis in RGB at transform
//
void ofApp::drawAxis(glm::mat4 m, float len) {

    ofSetLineWidth(1.0);

    // X Axis
    ofSetColor(ofColor(255, 0, 0));
    ofDrawLine(glm::vec3(m*glm::vec4(0, 0, 0, 1)), glm::vec3(m*glm::vec4(len, 0, 0, 1)));


    // Y Axis
    ofSetColor(ofColor(0, 255, 0));
    ofDrawLine(glm::vec3(m*glm::vec4(0, 0, 0, 1)), glm::vec3(m*glm::vec4(0, len, 0, 1)));

    // Z Axis
    ofSetColor(ofColor(0, 0, 255));
    ofDrawLine(glm::vec3(m*glm::vec4(0, 0, 0, 1)), glm::vec3(m*glm::vec4(0, 0, len, 1)));
}

// print C++ code for obj tranformation channels. (for debugging);
//
void ofApp::printChannels(SceneObject *obj) {
    cout << "position = glm::vec3(" << obj->position.x << "," << obj->position.y << "," << obj->position.z << ");" << endl;
    cout << "rotation = glm::vec3(" << obj->rotation.x << "," << obj->rotation.y << "," << obj->rotation.z << ");" << endl;
    cout << "scale = glm::vec3(" << obj->scale.x << "," << obj->scale.y << "," << obj->scale.z << ");" << endl;
}

void ofApp::saveToFile() {
    // traverse skeleton tree and save to file
    string filePath = "skeletons/skelly.txt";
    
    ofFile skelly;
    skelly.create(filePath);
    skelly.open(filePath, ofFile::WriteOnly);
    
    for (auto obj : scene) {
        // create -joint “joint name” -rotate “<x, y, z>” -translate “<x, y, z>” -parent “name”;
        if (Joint* jointObj = dynamic_cast<Joint*>(obj)) {
            char jointBuf[256];
            // if is root, just don't include parent
            if (jointObj->parent == NULL) {
                snprintf(jointBuf, sizeof(jointBuf),
                     "create -joint %s -rotate <%f, %f, %f> -translate <%f, %f, %f> -parent none;",
                     jointObj->name.c_str(),
                     jointObj->rotation.x, jointObj->rotation.y, jointObj->rotation.z,
                     jointObj->position.x, jointObj->position.y, jointObj->position.z);
            }
            // translate and rotate values relative to parent
            // string jointBuf = "create -joint " + jointObj->name + " -rotate <%f, %f, %f> -translate <%f, %f, %f> -parent " + jointObj->parent->name, jointObj->rotation.x, jointObj->rotation.y, jointObj->rotation.z, jointObj->position.x, jointObj->position.y, jointObj->position.z;
            else {
                snprintf(jointBuf, sizeof(jointBuf),
                     "create -joint %s -rotate <%f, %f, %f> -translate <%f, %f, %f> -parent %s;",
                     jointObj->name.c_str(),
                     jointObj->rotation.x, jointObj->rotation.y, jointObj->rotation.z,
                     jointObj->position.x, jointObj->position.y, jointObj->position.z,
                     jointObj->parent->name.c_str());
            }
            skelly << jointBuf << endl;
        }
    }
    
}

void ofApp::readSkeleton(ofFile skelly) {
    if (skelly.exists()) {
        // read file into buffer
        ofBuffer buffer(skelly);
        skelly.close();
        string skeleton = buffer.getText();

        istringstream jointStream(skeleton);
        string jointLine;

        // one line from skelly gets stored in jointLine
        while (getline(jointStream, jointLine)) {
            // create -joint “joint name” -rotate “<x, y, z>” -translate “<x, y, z>” -parent “name”
            vector<string> tokens;
            istringstream ss(jointLine);
            string token;
            char delimiter = ' ';
            
            string jointName;
            glm::vec3 rot;
            glm::vec3 tran;
            string parentName;
            
            cout << "hello???" << endl;
            
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
                        getline(ss, token, '<'); // Read until '<'
                        getline(ss, token, '>'); // Read until '>'
                        sscanf(token.c_str(), "%f, %f, %f", &rot.x, &rot.y, &rot.z);
                        
                        cout << "Rotate: " << rot.x << ", " << rot.y << ", " << rot.z << endl;

                        // translate
                        getline(ss, token, delimiter); // gets space
                        getline(ss, token, delimiter);
                        if (token == "-translate") {
                            getline(ss, token, '<'); // Read until '<'
                            getline(ss, token, '>'); // Read until '>'
                            sscanf(token.c_str(), "%f, %f, %f", &tran.x, &tran.y, &tran.z);
                            
                            cout << "Translate: " << tran.x << ", " << tran.y << ", " << tran.z << endl;
                            
                            // parent
                            getline(ss, token, delimiter); // gets space
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
            Joint * curr = new Joint(tran, jointName);
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

void onLoadSkeletonPressed() {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

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
void ofApp::keyPressed(int key) {
    switch (key) {
    case 'C':
    case 'c':
        if (mainCam.getMouseInputEnabled()) mainCam.disableMouseInput();
        else mainCam.enableMouseInput();
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
    case 'j':
        {
            Joint* joint = new Joint(glm::vec3(1,1,1));

            // when selected
            if (objSelected()) {
                if (Joint* selectedJoint = dynamic_cast<Joint*>(selected[0])) {
                    selectedJoint->addChild(joint);
                }
            }
            scene.push_back(joint);
            break;
        }
    case 'l':
        {
            ofFile file;
            file.open("skeletons/skelly.txt", ofFile::ReadOnly);
            readSkeleton(file);
            break;
        }
    case 's':
        {
            saveToFile();
            break;
        }
    case 'n':
        break;
    case 'p':
        if (objSelected()) printChannels(selected[0]);
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
    case OF_KEY_F1:
        theCam = &mainCam;
        break;
    case OF_KEY_F2:
        theCam = &sideCam;
        break;
    case OF_KEY_F3:
        theCam = &topCam;
        break;
    case OF_KEY_ALT:
        bAltKeyDown = true;
        if (!mainCam.getMouseInputEnabled()) mainCam.enableMouseInput();
        break;
    case OF_KEY_BACKSPACE:
        {
            if (objSelected()) {
                
                if (Joint* selectedJoint = dynamic_cast<Joint*>(selected[0])) {
                    if (selectedJoint->parent == NULL) {
                        // remove the whole tree?
                    } else {
                        for (auto child : selectedJoint->childList) {
                            child->parent = selectedJoint->parent;
                            selectedJoint->parent->addChild(child);
                        }
                        selectedJoint->childList.clear();

                        auto& siblings = selectedJoint->parent->childList;
                        
                        // std::remove reorders the container to avoid invalidating the iterators
                        siblings.erase(std::remove(siblings.begin(), siblings.end(), selectedJoint), siblings.end());
                        selectedJoint->parent = NULL;
                        
                        SceneObject* tribute = selected[0];
                        scene.erase(std::remove(scene.begin(), scene.end(), tribute), scene.end());
                        selected.erase(std::remove(selected.begin(), selected.end(), tribute), selected.end());
                        delete tribute;
                    }
                }
            }
        }
    default:
        break;
    }
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

    if (objSelected() && bDrag) {
        glm::vec3 point;
        mouseToDragPlane(x, y, point);
        if (bRotateX) {
            selected[0]->rotation += glm::vec3((point.x - lastPoint.x) * 20.0, 0, 0);
        }
        else if (bRotateY) {
            selected[0]->rotation += glm::vec3(0, (point.x - lastPoint.x) * 20.0, 0);
        }
        else if (bRotateZ) {
            selected[0]->rotation += glm::vec3(0, 0, (point.x - lastPoint.x) * 20.0);
        }
        else {
            selected[0]->position += (point - lastPoint);
        }
        lastPoint = point;
    }

}

//  This projects the mouse point in screen space (x, y) to a 3D point on a plane
//  normal to the view axis of the camera passing through the point of the selected object.
//  If no object selected, the plane passing through the world origin is used.
//
bool ofApp::mouseToDragPlane(int x, int y, glm::vec3 &point) {
    glm::vec3 p = theCam->screenToWorld(glm::vec3(x, y, 0));
    glm::vec3 d = p - theCam->getPosition();
    glm::vec3 dn = glm::normalize(d);

    float dist;
    glm::vec3 pos;
    if (objSelected()) {
        pos = selected[0]->position;
    }
    else pos = glm::vec3(0, 0, 0);
    if (glm::intersectRayPlane(p, dn, pos, glm::normalize(theCam->getZAxis()), dist)) {
        point = p + dn * dist;
        return true;
    }
    return false;
}

//--------------------------------------------------------------
//
// Provides functionality of single selection and if something is already selected,
// sets up state for translation/rotation of object using mouse.
//
void ofApp::mousePressed(int x, int y, int button){

    // if we are moving the camera around, don't allow selection
    //
    if (mainCam.getMouseInputEnabled()) return;

    // clear selection list
    //
    selected.clear();

    //
    // test if something selected
    //
    vector<SceneObject *> hits;

    glm::vec3 p = theCam->screenToWorld(glm::vec3(x, y, 0));
    glm::vec3 d = p - theCam->getPosition();
    glm::vec3 dn = glm::normalize(d);

    // check for selection of scene objects
    //
    for (int i = 0; i < scene.size(); i++) {
        
        glm::vec3 point, norm;
        
        //  We hit an object
        //
        if (scene[i]->isSelectable && scene[i]->intersect(Ray(p, dn), point, norm)) {
            hits.push_back(scene[i]);
        }
    }


    // if we selected more than one, pick nearest
    //
    SceneObject *selectedObj = NULL;
    if (hits.size() > 0) {
        selectedObj = hits[0];
        float nearestDist = std::numeric_limits<float>::infinity();
        for (int n = 0; n < hits.size(); n++) {
            float dist = glm::length(hits[n]->position - theCam->getPosition());
            if (dist < nearestDist) {
                nearestDist = dist;
                selectedObj = hits[n];
            }
        }
    }
    if (selectedObj) {
        selected.push_back(selectedObj);
        bDrag = true;
        mouseToDragPlane(x, y, lastPoint);
    }
    else {
        selected.clear();
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    bDrag = false;

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}

