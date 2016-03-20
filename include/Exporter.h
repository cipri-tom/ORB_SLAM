#ifndef EXPORTER_H
#define EXPORTER_H

#include "Map.h"

namespace ORB_SLAM
{

class Map;

class Exporter
{
private:
    Map *map_view;

    // string(path) or base_<date>.ext
    string GetPath(const char *path, const char *base, const char *ext = "txt") const;
public:
    Exporter(Map *map);
    void WriteKeyFrames   (const char *path = NULL) const;
    void WriteMapPointsTXT(const char *path = NULL) const; // X Y Z Nx Ny Nz R G B 1
    void WriteMapPointsPLY(const char *path = NULL) const;
};

}// namespace ORB_SLAM

#endif // EXPORTER_H
