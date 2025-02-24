//
//  Primitives.h - Simple 3D Primitives with with Hierarchical Transformations
//
//  
//  (c) Kevin M. Smith  - 24 September 2018
//
#pragma once

#include "ofMain.h"
#include "box.h"
#include "glm/gtx/euler_angles.hpp"
#include "glm/gtx/intersect.hpp"


//  General Purpose Ray class 
//
class Ray {
public:
	Ray(glm::vec3 p, glm::vec3 d) { this->p = p; this->d = d; }
	void draw(float t) { ofDrawLine(p, p + t * d); }

	glm::vec3 evalPoint(float t) {
		return (p + t * d);
	}

	glm::vec3 p, d;
};

//  Base class for any renderable object in the scene
//
class SceneObject : public std::enable_shared_from_this<SceneObject> {
public:
    virtual ~SceneObject() = default;  // Virtual destructor for proper cleanup
    virtual void draw() = 0;    // pure virtual funcs - must be overloaded
    virtual bool intersect(const Ray &ray, glm::vec3 &point, glm::vec3 &normal) { return false; }

    glm::mat4 getRotateMatrix() {
        return (glm::eulerAngleYXZ(glm::radians(rotation.y), glm::radians(rotation.x), glm::radians(rotation.z))); // yaw, pitch, roll
    }

    glm::mat4 getTranslateMatrix() {
        return (glm::translate(glm::mat4(1.0), glm::vec3(position.x, position.y, position.z)));
    }

    glm::mat4 getScaleMatrix() {
        return (glm::scale(glm::mat4(1.0), glm::vec3(scale.x, scale.y, scale.z)));
    }

    glm::mat4 getLocalMatrix() {
        // get the local transformations + pivot
        glm::mat4 scale = getScaleMatrix();
        glm::mat4 rotate = getRotateMatrix();
        glm::mat4 trans = getTranslateMatrix();

        // handle pivot point (rotate around a point that is not the object's center)
        glm::mat4 pre = glm::translate(glm::mat4(1.0), glm::vec3(-pivot.x, -pivot.y, -pivot.z));
        glm::mat4 post = glm::translate(glm::mat4(1.0), glm::vec3(pivot.x, pivot.y, pivot.z));

        return (trans * post * rotate * pre * scale);
    }

    glm::vec3 getTotalRotation() {
        if (parent) {
            auto parentRotation = parent->getTotalRotation();
            return (parentRotation + rotation);
        } else {
            return (rotation);
        }
    }

    glm::mat4 getMatrix() {
        if (parent) {
            glm::mat4 M = parent->getMatrix();
            return (M * getLocalMatrix());
        }
        return getLocalMatrix();  // priority order is SRT
    }

    glm::vec3 getPosition() {
        return (getMatrix() * glm::vec4(0.0, 0.0, 0.0, 1.0));
    }

    void setPosition(glm::vec3 pos) {
        position = glm::inverse(getMatrix()) * glm::vec4(pos, 1.0);
    }
  
    // return a rotation  matrix that rotates one vector to another
    //
    glm::mat4 rotateToVector(glm::vec3 v1, glm::vec3 v2);

    // Hierarchy methods with shared_ptr
    void addChild(std::shared_ptr<SceneObject> child) {
        childList.push_back(child);
        child->parent = shared_from_this();  // Set parent using shared_ptr
    }

    std::shared_ptr<SceneObject> parent = nullptr;
    std::vector<std::shared_ptr<SceneObject>> childList;

    // Transformation properties
    glm::vec3 position = glm::vec3(0, 0, 0);  // translate
    glm::vec3 rotation = glm::vec3(0, 0, 0);  // rotate
    glm::vec3 scale = glm::vec3(1, 1, 1);     // scale

    glm::vec3 pivot = glm::vec3(0, 0, 0);  // rotate pivot

    // Material properties
    ofColor diffuseColor = ofColor::grey;
    ofColor specularColor = ofColor::lightGray;

    // UI parameters
    bool isSelectable = true;
    std::string name = "SceneObject";
};

class Cone : public SceneObject {
public:
	Cone(ofColor color = ofColor::blue) {
		diffuseColor = color;
	}
	Cone(glm::vec3 tran, glm::vec3 rot, glm::vec3 sc, ofColor color = ofColor::blue) {
		position = tran;
		rotation = rot;
		scale = sc;
		diffuseColor = color;
	}
	void draw();
	bool intersect(const Ray &ray, glm::vec3 &point, glm::vec3 &normal);

	float radius = 1.0;
	float height = 2.0;
};

class Cube : public SceneObject {
public:
	Cube(ofColor color = ofColor::blue) {
		diffuseColor = color;
	}
	Cube(glm::vec3 tran, glm::vec3 rot, glm::vec3 sc, ofColor color = ofColor::blue) {
		position = tran;
		rotation = rot;
		scale = sc;
		diffuseColor = color;
	}
	void draw();
	bool intersect(const Ray &ray, glm::vec3 &point, glm::vec3 &normal);

	float width = 2.0;
	float height = 2.0;
	float depth = 2.0;
};

//  General purpose sphere  (assume parametric)
//
class Sphere: public SceneObject {
public:
	Sphere(glm::vec3 p, float r, ofColor diffuse = ofColor::lightGray) { position = p; radius = r; diffuseColor = diffuse; }
	Sphere() {}
	bool intersect(const Ray &ray, glm::vec3 &point, glm::vec3 &normal);
	void draw();

	float radius = 1.0;
};


//  Mesh class (will complete later- this will be a refinement of Mesh from Project 1)
//
class Mesh : public SceneObject {
	bool intersect(const Ray &ray, glm::vec3 &point, glm::vec3 &normal) { return false; }
	void draw() { }
};


//  General purpose plane 
//
class Plane: public SceneObject {
public:
	Plane(glm::vec3 p, glm::vec3 n, ofColor diffuse = ofColor::darkGreen, float w = 20, float h = 20 ) {
		position = p; normal = n;
		width = w;
		height = h;
		diffuseColor = diffuse;
		isSelectable = false;
		plane.rotateDeg(-90, 1, 0, 0);
		plane.setPosition(position);
		plane.setWidth(width);
		plane.setHeight(height);
		plane.setResolution(4, 4);
		
	}
	Plane() { 
		plane.rotateDeg(-90, 1, 0, 0);
		isSelectable = false;
	}
	glm::vec3 normal = glm::vec3(0, 1, 0);
	bool intersect(const Ray &ray, glm::vec3 & point, glm::vec3 & normal);
	void draw() {
		material.begin();
		material.setDiffuseColor(diffuseColor);
		plane.drawFaces();
		material.end();
	}
	ofPlanePrimitive plane;
	ofMaterial material;
	
	float width = 20;
	float height = 20;
};
