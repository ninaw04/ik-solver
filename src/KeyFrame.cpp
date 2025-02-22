#include "KeyFrame.h"
#include "ofMain.h"

void KeyFrameManager::update(shared_ptr<Joint> j1, shared_ptr<Joint> j2)
{
    if (bInPlayback) {
        nextFrame();
    }

    // Check if keyframes are set and the current frame is within the range
    if (keyFramesSet() && (frame >= key1.frame && frame <= key2.frame)) {
        j1->rotation = easeInterp(frame,
                                  key1.frame,
                                  key2.frame,
                                  key1.configRotations.rotunda,
                                  key2.configRotations.rotunda);
        j1->rotation += easeInterp(frame,
                                   key1.frame,
                                   key2.frame,
                                   key1.configRotations.shoulder,
                                   key2.configRotations.shoulder);
        j2->rotation = easeInterp(frame,
                                  key1.frame,
                                  key2.frame,
                                  key1.configRotations.elbow,
                                  key2.configRotations.elbow);
    }
}

void KeyFrameManager::setFirstFrame()
{
    frame = frameBegin;
}

void KeyFrameManager::nextFrame()
{
    if (frame != frameEnd)
    {
        frame = frame + 1;
    }
}

void KeyFrameManager::prevFrame()
{
    frame = (frame == frameBegin ? frame : frame - 1);
}

void KeyFrameManager::startPlayback()
{
    bInPlayback = true;
}

void KeyFrameManager::stopPlayback()
{
    bInPlayback = false;
}

bool KeyFrameManager::keyFramesSet()
{
    return (key1.frame != -1 && key2.frame != -1);
}

glm::vec3 KeyFrameManager::linearInterp(int frame, int frameStart, int frameEnd, glm::vec3 const& start, glm::vec3 const& end)
{
    return mapVec(frame, frameStart, frameEnd, start, end);
}

glm::vec3 KeyFrameManager::easeInterp(int frame, int frameStart, int frameEnd, glm::vec3 const& start, glm::vec3 const& end)
{
    float s = ease(ofMap(frame, frameStart, frameEnd, 0.0, 1.0));
    return mapVec(s, 0.0, 1.0, start, end);
}

float KeyFrameManager::ease(float x)
{
    return (x * x / (x * x + (1 - x) * (1 - x)));
}

glm::vec3 KeyFrameManager::mapVec(float val, float start, float end, glm::vec3 const& outStart, glm::vec3 const& outEnd)
{
    return glm::vec3(ofMap(val, start, end, outStart.x, outEnd.x),
                     ofMap(val, start, end, outStart.y, outEnd.y),
                     ofMap(val, start, end, outStart.z, outEnd.z));
}

glm::vec3 KeyFrameManager::mapVec(glm::vec3 const& val, glm::vec3 const& start, glm::vec3 const& end, glm::vec3 const& outStart, glm::vec3 const& outEnd)
{
    return glm::vec3(ofMap(val.x, start.x, end.x, outStart.x, outEnd.x),
                     ofMap(val.y, start.y, end.y, outStart.y, outEnd.y),
                     ofMap(val.z, start.z, end.z, outStart.z, outEnd.z));
}

void KeyFrameManager::setKeyFrame(int index, std::vector<jointDegrees3R>& solutions, int& displaySolution)
{
    key1.frame = frameBegin;
    key2.frame = frameEnd;

    if (solutions.size() > 0)
    {
        int solIndex = index % solutions.size();
        displaySolution = solIndex;

        key2.configRotations = { .rotunda = solutions[solIndex].rotunda,
                                 .shoulder = solutions[solIndex].shoulder,
                                 .elbow = solutions[solIndex].elbow };
    }
}

// Resets keyframes to initial state, startconfig may change based on previous solutions
void KeyFrameManager::resetKeyFrames()
{
    key1.frame = key2.frame = -1;
    key1.configRotations = startConfig;
}
