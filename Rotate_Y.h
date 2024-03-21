//
// Created by ramik on 3/15/2024.
//

#ifndef UNTITLED_ROTATE_Y_H
#define UNTITLED_ROTATE_Y_H

class Rotate_Y : public Primitive {
private:
    Primitive* primitive_ptr;
    float sin_theta;
    float cos_theta;
    bool has_bbox;
    AABB bbox;
};
#endif //UNTITLED_ROTATE_Y_H
