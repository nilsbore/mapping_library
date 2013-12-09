#ifndef PRIMITIVE_LEAF_H
#define PRIMITIVE_LEAF_H

#include <pcl/octree/octree_container.h>

class primitive_leaf : public pcl::octree::OctreeContainerDataTVector<int>
{
public:
    bool remove_if_equal(int ind);
    primitive_leaf(const OctreeContainerDataTVector<int>& source);
    primitive_leaf();
};

#endif // PRIMITIVE_LEAF_H
