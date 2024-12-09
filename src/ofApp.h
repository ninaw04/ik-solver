
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

#include "ofMain.h"
#include "ofxButton.h"
#include "ofxGui.h"
#include "box.h"
#include "Primitives.h"
#include "joint.hpp"
#include "ofxGuiUtils.h"
#include "ofxInputField.h"
#include "ofxSlider.h"



class ofApp : public ofBaseApp{

    public:
        void setup();
        void update();
        void draw();

        void keyPressed(int key);
        void keyReleased(int key);
        void mouseMoved(int x, int y );
        void mouseDragged(int x, int y, int button);
        void mousePressed(int x, int y, int button);
        void mouseReleased(int x, int y, int button);
        void mouseEntered(int x, int y);
        void mouseExited(int x, int y);
        void windowResized(int w, int h);
        void dragEvent(ofDragInfo dragInfo);
        void gotMessage(ofMessage msg);
        static void drawAxis(glm::mat4 transform = glm::mat4(1.0), float len = 1.0);
        bool mouseToDragPlane(int x, int y, glm::vec3 &point);
        void printChannels(SceneObject *);
        bool objSelected() { return (selected.size() ? true : false ); };
        void saveToFile();
        void readSkeleton(ofFile skeleton);
        struct jointDegrees3R {
            glm::vec3 rotunda;
            glm::vec3 shoulder;
            glm::vec3 elbow;
        };
    
        void inverseKin2(glm::vec2 target, Joint& joint1, Joint& joint2, Joint& joint3, vector<pair<glm::vec2, glm::vec2>>& solutions);
        void inverseKin3(glm::vec3 target, Joint& joint1, Joint& joint2, Joint& joint3, vector<jointDegrees3R> &solutions);
        void handleSolutions(vector<jointDegrees3R> &solutions);
        // Lights
        //
        ofLight light1;
    
        // Cameras
        //
        ofEasyCam  mainCam;
        ofCamera sideCam;
        ofCamera topCam;
        ofCamera  *theCam;    // set to current camera either mainCam or sideCam

        // Materials
        //
        ofMaterial material;


        // scene components
        //
        vector<SceneObject *> scene;
        vector<SceneObject *> selected;
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

        ofxFloatSlider minAngle;
        ofxFloatSlider maxAngle;
        ofxToggle xAxis;
        ofxToggle yAxis;
        ofxToggle zAxis;
        
        // ARM
        Joint *j1 = new Joint(glm::vec3(0, 0, 0), "j1");
        Joint *j2 = new Joint(glm::vec3(0.1, 2, 0), "j2");
        Joint *j3 = new Joint(glm::vec3(2, 2, 0), "j3");
    
        // RAHHHH
        glm::vec3 WORLDPOINT = glm::vec3(0, 0, 0);

        

//        ofxButton loadSkeleton;
//        ofxTextField skeletonPath;
//
//        void onLoadSkeletonPressed();
        
//        ofxFloatSlider lightIntensity;
//        ofxIntSlider powerExponent;
};
