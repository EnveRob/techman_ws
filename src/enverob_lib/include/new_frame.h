#ifndef NEW_FRAME_H
#define NEW_FRAME_H

#include <vector>
#include <string>
namespace new_frame
{
    void fixedFrame_add(std::vector<double> frame_pose,
                        std::string reference_frame,
                        std::string target_frame);
}

#endif