#ifndef PRIMITIVE_LEAF_H
#define PRIMITIVE_LEAF_H

#include <assert.h>
#include <pcl/octree/octree_container.h>

class primitive_leaf : public pcl::octree::OctreeContainerPointIndices
{
public:
    bool remove_if_equal(int ind);
    primitive_leaf(const OctreeContainerPointIndices& source);
    primitive_leaf();
};

#endif // PRIMITIVE_LEAF_H
