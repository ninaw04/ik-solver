#ifndef KeyFrame_h
#define KeyFrame_h

#include "joint.hpp"
#include <vector>
#include <glm/glm.hpp>

struct jointDegrees3R
{
    glm::vec3 rotunda;
    glm::vec3 shoulder;
    glm::vec3 elbow;
};

struct KeyFrame
{
    int frame = -1;  // -1 => no key is set;
    jointDegrees3R configRotations;
};

class KeyFrameManager
{
public:
    KeyFrame key1, key2;       // This demo just has 2 key frames
    int frame = 1;             // Current frame
    int frameBegin = 1;        // First frame of playback range
    int frameEnd = 150;        // Last frame of playback range
    bool bInPlayback = false;  // True => we are in playback mode
    jointDegrees3R startConfig = { .rotunda = glm::vec3(0, 0, 0),
                                 .shoulder = glm::vec3(0, 0, 0),
                                 .elbow = glm::vec3(0, 0, 0) };

    void update(shared_ptr<Joint> j1, shared_ptr<Joint> j2);
    void setFirstFrame();
    void nextFrame();
    void prevFrame();
    void startPlayback();
    void stopPlayback();
    bool keyFramesSet();
    glm::vec3 linearInterp(int frame, int frameStart, int frameEnd, glm::vec3 const& start, glm::vec3 const& end);
    glm::vec3 easeInterp(int frame, int frameStart, int frameEnd, glm::vec3 const& start, glm::vec3 const& end);
    float ease(float x);
    glm::vec3 mapVec(float val, float start, float end, glm::vec3 const& outStart, glm::vec3 const& outEnd);
    glm::vec3 mapVec(glm::vec3 const& val, glm::vec3 const& start, glm::vec3 const& end, glm::vec3 const& outStart, glm::vec3 const& outEnd);
    void setKeyFrame(int index, std::vector<jointDegrees3R>& solutions, int& displaySolution);
    void resetKeyFrames();
};

#endif
