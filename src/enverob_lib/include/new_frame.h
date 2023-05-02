#ifndef NEW_FRAME_H
#define NEW_FRAME_H

namespace new_frame
{
    void waitforTransform(std::string target_frame,
                          tf::StampedTransform target_transform);

    void fixedFrame_add(std::vector<double> frame_pose,
                        std::string reference_frame,
                        std::string target_frame);
}

#endif